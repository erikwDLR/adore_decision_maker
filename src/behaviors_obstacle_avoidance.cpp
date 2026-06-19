/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "behaviors_obstacle_avoidance.hpp"
#include "behaviors.hpp"
#include "adore_map_conversions.hpp"
#include "planning/obstacle_avoidance.hpp"
#include "planning/active_avoidance.hpp"
#include <adore_dynamics_conversions.hpp>
#include <adore_math/distance.h>
#include <dynamics/comfort_settings.hpp>
#include <dynamics/trajectory.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <string>

namespace adore
{
namespace behavior
{
namespace
{

std::string
ids_to_string( const std::vector<int>& ids )
{
    std::string text = "[";
    for( std::size_t i = 0; i < ids.size(); ++i )
    {
        if( i > 0 )
        {
            text += ",";
        }
        text += std::to_string( ids[i] );
    }
    text += "]";
    return text;
}

void
append_unique_ids( std::vector<int>& target, const std::vector<int>& source )
{
    for( const int id : source )
    {
        if( std::find( target.begin(), target.end(), id ) == target.end() )
        {
            target.push_back( id );
        }
    }
}

// Gate for the directional turn-signal suffix. The indicator is derived
// downstream (trajectory_tracker) purely from the "... left)" / "... right)"
// label substring. We signal only over the window in which a human driver
// would: from blinker_lead_distance ahead of where the lateral shift begins
// (shift_start_s) until the maximum lateral shift is reached (max_shift_s, the
// avoided obstacle's leading edge). Before that the maneuver is only decided;
// after it the vehicle has finished moving over and is alongside the obstacle.
// Each bound is applied only when its geometry is finite, so the gate fails
// safe to signaling rather than moving over silently.
bool
should_indicate_avoidance_direction(
    double ego_s,
    double shift_start_s,
    double max_shift_s,
    const planner::ObstacleAvoidanceParams& params )
{
    if( !std::isfinite( ego_s ) )
    {
        return true;
    }
    if( std::isfinite( shift_start_s ) &&
        ego_s < shift_start_s - params.blinker_lead_distance )
    {
        return false;
    }
    if( std::isfinite( max_shift_s ) && ego_s >= max_shift_s )
    {
        return false;
    }
    return true;
}

std::string
obstacle_avoidance_label(
    planner::ObstacleAvoidanceMode mode,
    bool in_lane,
    double lateral_shift,
    bool active,
    bool indicate_direction )
{
    const std::string prefix =
        active
            ? "driving mission (active obstacle avoidance"
            : "driving mission (obstacle avoidance";

    if( in_lane )
    {
        return active
            ? "driving mission (active in-lane obstacle avoidance)"
            : "driving mission (in-lane obstacle avoidance)";
    }

    if( indicate_direction )
    {
        if( mode == planner::ObstacleAvoidanceMode::OvertakeLeft ||
            lateral_shift > 1e-6 )
        {
            return prefix + " left)";
        }

        if( mode == planner::ObstacleAvoidanceMode::OvertakeRight ||
            lateral_shift < -1e-6 )
        {
            return prefix + " right)";
        }
    }

    return active
        ? "driving mission (active obstacle avoidance)"
        : "driving mission (obstacle avoidance)";
}

// Single-sourced in planning/obstacle_avoidance.hpp; brought into this namespace
// so existing unqualified call sites keep working.
using planner::project_s_on_reference_line;
using planner::make_original_obstacle_envelope_from_participant;
using planner::update_obstacle_ghost_memory;
using planner::start_active_avoidance_state;
using planner::most_relevant_ghost_conflict;
using planner::find_unpassed_original_ghost;
using planner::RouteStopPlan;
using planner::make_oncoming_monitor_conflict;
using planner::static_or_slow_conflict_has_side_clearance;
using planner::compute_route_stop_plan;
using planner::should_stop_for_active_conflict;
using planner::compute_monotonic_ego_s_modified;

dynamics::Trajectory
make_immediate_hold_trajectory(
    const dynamics::VehicleStateDynamic& ego,
    double dt,
    const planner::ObstacleAvoidanceParams& params )
{
    dynamics::Trajectory trajectory;
    trajectory.label = "obstacle avoidance: immediate hold";

    const double step = std::max( params.trajectory_step_size, dt );
    const double start_speed = std::max( 0.0, ego.vx );
    const int samples = 6;

    for( int i = 0; i < samples; ++i )
    {
        auto state = ego;
        state.time = ego.time + i * step;
        state.vx =
            i == 0
                ? ego.vx
                : start_speed * std::max( 0.0, 1.0 - static_cast<double>( i ) / ( samples - 1 ) );
        trajectory.states.push_back( state );
    }

    trajectory.states.back().vx = 0.0;

    return trajectory;
}

dynamics::Trajectory
make_route_hold_trajectory(
    const map::Route& route,
    const dynamics::VehicleStateDynamic& ego,
    double dt,
    const planner::ObstacleAvoidanceParams& params )
{
    dynamics::Trajectory trajectory;
    trajectory.label = "obstacle avoidance: route hold";

    if( route.reference_line.empty() )
    {
        return trajectory;
    }

    double ego_s =
        project_s_on_reference_line(
            route,
            ego,
            std::numeric_limits<double>::quiet_NaN() );
    if( !std::isfinite( ego_s ) )
    {
        ego_s = route.reference_line.begin()->first;
    }

    const double step = std::max( params.trajectory_step_size, dt );
    const int samples = 6;

    for( int i = 0; i < samples; ++i )
    {
        const auto pose = route.get_pose_at_s( ego_s );

        auto state = ego;
        state.time = ego.time + i * step;
        state.x = pose.x;
        state.y = pose.y;
        state.yaw_angle = pose.yaw;
        state.vx = 0.0;
        state.vy = 0.0;
        state.ax = 0.0;
        state.ay = 0.0;
        state.yaw_rate = 0.0;
        trajectory.states.push_back( state );
    }

    return trajectory;
}

const char*
conflict_type_name( const planner::RouteCorridorConflict& conflict )
{
    if( conflict.currently_overlaps_ego_footprint )
    {
        return "ego_footprint_overlap";
    }
    if( conflict.predicted_spatiotemporal_conflict )
    {
        return "predicted_spatiotemporal";
    }
    if( conflict.currently_overlaps_route_corridor )
    {
        return "route_corridor_ahead";
    }
    return "ghost";
}

map::Route
apply_active_avoidance_speed_profile(
    const map::Route& active_route,
    double ego_s,
    double shift_start_s,
    double shift_end_s,
    double release_s,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::ObstacleAvoidanceParams& params )
{
    double maneuver_end_s =
        std::isfinite( release_s )
            ? release_s
            : shift_end_s;

    return planner::RouteSpeedPolicy::apply_avoidance_speed_profile(
        active_route,
        ego_s,
        shift_start_s,
        maneuver_end_s,
        vehicle_params,
        params );
}

map::Route
make_max_braking_route_from_s( const map::Route& route, double ego_s )
{
    map::Route max_braking_route = route;
    planner::set_route_points_from_s_to_zero( max_braking_route, ego_s );
    return max_braking_route;
}

map::Route
make_stop_route_on_active_route(
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    double requested_stop_s,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::ObstacleAvoidanceParams& params,
    const std::string& reason,
    const std::string& active_route_name )
{
    map::Route stop_route = active_route;

    if( stop_route.reference_line.empty() )
    {
        return stop_route;
    }

    double ego_s =
        project_s_on_reference_line(
            stop_route,
            ego,
            requested_stop_s );
    if( !std::isfinite( ego_s ) )
    {
        ego_s = stop_route.reference_line.begin()->first;
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][STOP_ROUTE] invalid ego projection on active_route=%s; stopping from route start reason=%s",
            active_route_name.c_str(),
            reason.c_str() );
    }

    const double a_abs =
        std::max( params.min_braking_deceleration, std::fabs( vehicle_params.acceleration_min ) );
    const double v0 = std::max( 0.0, ego.vx );
    const double braking_distance = v0 * v0 / ( 2.0 * a_abs );
    const double safety_margin =
        std::max( 0.0, params.modified_route_braking_safety_margin ) +
        std::max( 0.0, params.min_valid_stop_margin );
    const double close_stop_hold_margin =
        std::max( 0.25, std::min( 1.0, std::max( 0.0, params.min_valid_stop_margin ) ) );
    const double reachability_margin =
        v0 > 0.5
            ? safety_margin
            : close_stop_hold_margin;
    double target_stop_s = requested_stop_s;

    if( !std::isfinite( target_stop_s ) ||
        target_stop_s <= ego_s ||
        target_stop_s - ego_s < braking_distance + reachability_margin )
    {
        RCLCPP_WARN(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][STOP_ROUTE] requested stop unreachable; applying maximum route-based braking active_route=%s ego_s=%.2f requested_stop_s=%.2f v=%.2f braking_distance=%.2f safety_margin=%.2f reachability_margin=%.2f reason=%s",
            active_route_name.c_str(),
            ego_s,
            requested_stop_s,
            v0,
            braking_distance,
            safety_margin,
            reachability_margin,
            reason.c_str() );
        const auto stop_plan =
            planner::RouteStopPolicy::plan_stop_on_route(
                stop_route,
                ego_s,
                ego.vx,
                ego_s,
                planner::StopReason::ModifiedRouteConflict,
                vehicle_params,
                params );
        return stop_plan.route;
    }

    const auto stop_plan =
        planner::RouteStopPolicy::plan_stop_on_route(
            stop_route,
            ego_s,
            ego.vx,
            target_stop_s,
            planner::StopReason::ModifiedRouteConflict,
            vehicle_params,
            params );
    return stop_plan.route;
}

Behavior
make_emergency_hold_behavior(
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    const planner::ObstacleAvoidanceParams& params,
    const std::string& reason )
{
    auto trajectory =
        make_route_hold_trajectory( active_route, ego, params.stop_time_step, params );
    if( trajectory.states.empty() )
    {
        trajectory =
            make_immediate_hold_trajectory( ego, params.stop_time_step, params );
    }

    // The trajectory tracker treats this label as a hard braking command and
    // bypasses lateral trajectory tracking. Do not track a fabricated hold path
    // when all route-based planning attempts have already failed.
    trajectory.label = "emergency stop";
    trajectory.adjust_start_time( ego.time );

    Behavior behavior;
    behavior.trajectory = dynamics::conversions::to_ros_msg( trajectory );
    behavior.modified_route = map::conversions::to_ros_msg( active_route );

    RCLCPP_ERROR(
        rclcpp::get_logger( "Behaviors" ),
        "[OA][EMERGENCY_HOLD] reason=%s ego_x=%.3f ego_y=%.3f yaw=%.3f v=%.3f",
        reason.c_str(),
        ego.x,
        ego.y,
        ego.yaw_angle,
        ego.vx );

    return behavior;
}

Behavior
make_stop_on_route_behavior(
    planner::TrajectoryPlanner& planner,
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params,
    const std::string& reason,
    const std::string& active_route_name,
    map::Route* accepted_stop_route = nullptr )
{
    auto stop_route =
        make_stop_route_on_active_route(
            active_route,
            ego,
            std::numeric_limits<double>::quiet_NaN(),
            planner.get_physical_vehicle_parameters(),
            params,
            reason,
            active_route_name );

    dynamics::Trajectory trajectory;
    try
    {
        trajectory =
            planner.plan_route_trajectory(
                stop_route,
                ego,
                traffic_participants );
    }
    catch( const std::exception& e )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][STOP_ROUTE] planner exception on route-only stop: %s",
            e.what() );
    }

    if( trajectory.states.empty() )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][STOP_ROUTE] invalid stop trajectory; retrying with maximum route-based braking active_route=%s reason=%s",
            active_route_name.c_str(),
            reason.c_str() );

        double ego_s =
            project_s_on_reference_line(
                active_route,
                ego,
                std::numeric_limits<double>::quiet_NaN() );
        if( !std::isfinite( ego_s ) && !active_route.reference_line.empty() )
        {
            ego_s = active_route.reference_line.begin()->first;
        }

        auto max_braking_route =
            make_max_braking_route_from_s( active_route, ego_s );

        try
        {
            trajectory =
                planner.plan_route_trajectory(
                    max_braking_route,
                    ego,
                    traffic_participants );
        }
        catch( const std::exception& e )
        {
            RCLCPP_ERROR(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][STOP_ROUTE] planner exception on maximum route-based braking: %s",
                e.what() );
        }

        if( !trajectory.states.empty() )
        {
            trajectory.adjust_start_time( ego.time );
            trajectory.label =
                "obstacle avoidance: maximum route-based braking on active " +
                active_route_name + "_route";

            Behavior behavior;
            behavior.trajectory = dynamics::conversions::to_ros_msg( trajectory );
            behavior.modified_route = map::conversions::to_ros_msg( max_braking_route );
            if( accepted_stop_route != nullptr )
            {
                *accepted_stop_route = max_braking_route;
            }
            return behavior;
        }

        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][STOP_ROUTE] maximum route-based braking failed; using last-resort emergency hold active_route=%s reason=%s",
            active_route_name.c_str(),
            reason.c_str() );
        return make_emergency_hold_behavior(
            active_route,
            ego,
            params,
            "maximum route-based braking failed: " + reason );
    }

    trajectory.adjust_start_time( ego.time );
    trajectory.label = "obstacle avoidance: stop on " + active_route_name + "_route";

    Behavior behavior;
    behavior.trajectory = dynamics::conversions::to_ros_msg( trajectory );
    behavior.modified_route = map::conversions::to_ros_msg( stop_route );
    if( accepted_stop_route != nullptr )
    {
        *accepted_stop_route = stop_route;
    }
    return behavior;
}

Behavior
make_stop_on_active_route_behavior(
    planner::TrajectoryPlanner& planner,
    const map::Route& active_route,
    const planner::RouteCorridorConflict& conflict,
    const dynamics::VehicleStateDynamic& ego,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params,
    const std::string& active_route_name,
    map::Route* accepted_stop_route = nullptr )
{
    const auto vehicle_params = planner.get_physical_vehicle_parameters();
    const auto stop_plan =
        compute_route_stop_plan(
            active_route,
            ego,
            vehicle_params,
            conflict,
            params );
    const double requested_stop_s =
        std::isfinite( stop_plan.stop_s )
            ? stop_plan.stop_s
            : conflict.object_s_min -
              std::max( 0.0, params.stop_before_obstacle ) -
              ( vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border );
    const double v0 = std::max( 0.0, ego.vx );
    const double safety_margin =
        std::max( 0.0, params.modified_route_braking_safety_margin ) +
        std::max( 0.0, params.min_valid_stop_margin );
    const double close_stop_hold_margin =
        std::max( 0.25, std::min( 1.0, std::max( 0.0, params.min_valid_stop_margin ) ) );
    const double reachability_margin =
        v0 > 0.5
            ? safety_margin
            : close_stop_hold_margin;
    const bool requested_stop_reachable =
        std::isfinite( stop_plan.ego_s ) &&
        std::isfinite( requested_stop_s ) &&
        requested_stop_s > stop_plan.ego_s &&
        requested_stop_s - stop_plan.ego_s >=
            stop_plan.required_braking_distance + reachability_margin;

    map::Route stop_route =
        make_stop_route_on_active_route(
            active_route,
            ego,
            requested_stop_s,
            vehicle_params,
            params,
            conflict.reason.empty() ? "active route conflict stop" : conflict.reason,
            active_route_name );

    RCLCPP_WARN(
        rclcpp::get_logger( "Behaviors" ),
        "[OA][ACTIVE_STATE][STOP_DECISION] id=%d type=%s ego_s=%.2f requested_stop_s=%.2f v=%.2f required_braking_distance=%.2f",
        conflict.participant_id,
        conflict_type_name( conflict ),
        stop_plan.ego_s,
        requested_stop_s,
        ego.vx,
        stop_plan.required_braking_distance );

    RCLCPP_WARN(
        rclcpp::get_logger( "Behaviors" ),
        "[OA][ACTIVE_STATE][STOP] stop_on_modified_route id=%d type=%s dist=%.2f ttc=%.2f stop_s=%.2f",
        conflict.participant_id,
        conflict_type_name( conflict ),
        conflict.distance_s,
        conflict.time_to_conflict,
        requested_stop_s );

    dynamics::Trajectory trajectory;
    try
    {
        trajectory =
            planner.plan_route_trajectory( stop_route, ego, traffic_participants );
    }
    catch( const std::exception& e )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][STOP_ROUTE] planner exception on OA stop route: %s",
            e.what() );
    }

    if( trajectory.states.empty() ||
        !::adore::planner::trajectory_stops_before_conflict(
            trajectory,
            stop_route,
            conflict,
            vehicle_params,
            params ) )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][STOP_ROUTE] invalid stop trajectory; retrying with maximum route-based braking active_route=%s id=%d",
            active_route_name.c_str(),
            conflict.participant_id );

        if( requested_stop_reachable )
        {
            RCLCPP_ERROR(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][STOP_ROUTE] reachable stop profile was invalid; holding instead of moving stop point to ego active_route=%s id=%d",
                active_route_name.c_str(),
                conflict.participant_id );
            return make_emergency_hold_behavior(
                active_route,
                ego,
                params,
                "reachable stop profile invalid on active " + active_route_name + "_route" );
        }

        double ego_s = stop_plan.ego_s;
        if( !std::isfinite( ego_s ) )
        {
            ego_s =
                project_s_on_reference_line(
                    active_route,
                    ego,
                    requested_stop_s );
        }
        if( !std::isfinite( ego_s ) && !active_route.reference_line.empty() )
        {
            ego_s = active_route.reference_line.begin()->first;
        }

        auto max_braking_route =
            make_max_braking_route_from_s( active_route, ego_s );

        try
        {
            trajectory =
                planner.plan_route_trajectory(
                    max_braking_route,
                    ego,
                    traffic_participants );
        }
        catch( const std::exception& e )
        {
            RCLCPP_ERROR(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][STOP_ROUTE] planner exception on maximum route-based braking: %s",
                e.what() );
        }

        if( !trajectory.states.empty() &&
            ::adore::planner::trajectory_stops_before_conflict(
                trajectory,
                max_braking_route,
                conflict,
                vehicle_params,
                params ) )
        {
            trajectory.adjust_start_time( ego.time );
            trajectory.label =
                "obstacle avoidance: maximum route-based braking on active " +
                active_route_name + "_route";

            if( accepted_stop_route != nullptr )
            {
                *accepted_stop_route = max_braking_route;
            }

            Behavior behavior;
            behavior.trajectory = dynamics::conversions::to_ros_msg( trajectory );
            behavior.modified_route = map::conversions::to_ros_msg( max_braking_route );
            return behavior;
        }

        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][STOP_ROUTE] maximum route-based braking failed or did not stop before conflict; using last-resort emergency hold active_route=%s id=%d",
            active_route_name.c_str(),
            conflict.participant_id );
        return make_emergency_hold_behavior(
            active_route,
            ego,
            params,
            "maximum route-based braking failed on active " + active_route_name + "_route" );
    }

    trajectory.adjust_start_time( ego.time );
    trajectory.label = "obstacle avoidance: stop on active " + active_route_name + "_route";

    if( accepted_stop_route != nullptr )
    {
        *accepted_stop_route = stop_route;
    }

    Behavior behavior;
    behavior.trajectory = dynamics::conversions::to_ros_msg( trajectory );
    behavior.modified_route = map::conversions::to_ros_msg( stop_route );
    return behavior;
}

// Brake on the exact route ego is already tracking by zeroing the route-point
// speeds from ego onward and re-planning with the pinned start-s. Geometry and
// start-s stay identical to the normal active cycle, so the braking path is
// continuous with the previous trajectory and there is no lateral jump. This
// avoids the erratic steering that comes from building a separate stop route
// and letting the planner re-project ego onto the curved modified route and
// re-optimize a fresh deceleration profile. Used as the on-route stop for an
// active maneuver once avoidance is no longer possible.
Behavior
make_route_brake_in_place_behavior(
    planner::TrajectoryPlanner& planner,
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    double pinned_ego_s,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params,
    const std::string& label )
{
    double brake_from_s = pinned_ego_s;
    if( !std::isfinite( brake_from_s ) )
    {
        brake_from_s =
            project_s_on_reference_line(
                active_route,
                ego,
                std::numeric_limits<double>::quiet_NaN() );
    }
    if( !std::isfinite( brake_from_s ) && !active_route.reference_line.empty() )
    {
        brake_from_s = active_route.reference_line.begin()->first;
    }

    map::Route brake_route = active_route;
    ::adore::planner::set_route_points_from_s_to_zero( brake_route, brake_from_s );

    dynamics::Trajectory trajectory;
    try
    {
        trajectory =
            planner.plan_route_trajectory_from_s(
                brake_route,
                ego,
                traffic_participants,
                brake_from_s );
    }
    catch( const std::exception& e )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][BRAKE_IN_PLACE] planner exception while braking on zeroed route; falling back to emergency hold: %s",
            e.what() );
    }

    if( trajectory.states.empty() )
    {
        return make_emergency_hold_behavior(
            active_route,
            ego,
            params,
            "route brake-in-place produced no trajectory: " + label );
    }

    trajectory.adjust_start_time( ego.time );
    trajectory.label = label;

    Behavior behavior;
    behavior.trajectory = dynamics::conversions::to_ros_msg( trajectory );
    behavior.modified_route = map::conversions::to_ros_msg( brake_route );
    return behavior;
}

} // namespace


namespace
{

// Opposite-lane safety monitor for an active avoidance maneuver. Returns the
// conflict to stop for, or nullopt to keep following the route. With
// planned_trajectory == nullptr this runs the pre-commitment check; with a
// planned trajectory it runs the post-replan ("planned") check. The only
// differences between the two are the trajectory argument, the projection hint
// and the log/source tag prefix.
std::optional<::adore::planner::RouteCorridorConflict>
check_opposite_lane_monitor(
    const map::Route& route,
    const dynamics::VehicleStateDynamic& ego,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceManeuver& maneuver,
    int obstacle_id,
    double conflict_hint_s,
    const planner::ObstacleAvoidanceParams& params,
    const dynamics::Trajectory* planned_trajectory )
{
    if( !( maneuver.active && maneuver.uses_opposite_lane ) )
    {
        return std::nullopt;
    }

    const char* const tag_prefix = planned_trajectory != nullptr ? "PLANNED_" : "";

    const auto monitor_result =
        ::adore::planner::monitor_active_obstacle_avoidance_maneuver(
            route,
            ego,
            traffic_participants,
            maneuver,
            params,
            planned_trajectory );

    if( monitor_result.should_abort_before_commitment )
    {
        RCLCPP_WARN(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][MONITOR][%sABORT] obstacle_id=%d reason=%s; using route-based stop policy",
            tag_prefix,
            obstacle_id,
            monitor_result.reason.c_str() );

        return make_oncoming_monitor_conflict(
            monitor_result,
            conflict_hint_s,
            planned_trajectory != nullptr
                ? "planned_opposite_lane_monitor"
                : "opposite_lane_monitor" );
    }

    if( !monitor_result.safe_to_continue )
    {
        // Past commitment, stopping inside the opposite lane is usually more
        // dangerous than finishing the maneuver. Stop only if the conflicting
        // participant is already inside the conflict interval; for a predicted
        // future arrival, keep following the active route to clear the opposite
        // lane as quickly as possible.
        const bool conflict_already_in_interval =
            monitor_result.oncoming.oncoming_arrival_time <= 1e-6;

        if( conflict_already_in_interval )
        {
            RCLCPP_WARN(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][MONITOR][%sCOMMITTED] obstacle_id=%d reason=%s; already in interval, using route-based stop policy",
                tag_prefix,
                obstacle_id,
                monitor_result.reason.c_str() );

            return make_oncoming_monitor_conflict(
                monitor_result,
                conflict_hint_s,
                planned_trajectory != nullptr
                    ? "planned_opposite_lane_monitor_committed"
                    : "opposite_lane_monitor_committed" );
        }

        RCLCPP_WARN(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][MONITOR][%sCOMMITTED] obstacle_id=%d reason=%s; continuing to clear the opposite lane instead of stopping in it",
            tag_prefix,
            obstacle_id,
            monitor_result.reason.c_str() );
    }

    return std::nullopt;
}

// Try to reshape the active maneuver from a base route (active modified or
// mission). On success starts/refreshes the avoidance state and returns the
// resulting Behavior; nullopt means keep the previous active route this cycle.
std::optional<Behavior>
try_dynamic_replan_from_route(
    planner::TrajectoryPlanner& planner,
    const map::Route& replan_base_route,
    const char* replan_base_name,
    double projection_hint_s,
    const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance,
    planner::ActiveAvoidanceState& active_avoidance_state,
    const std::vector<int>* additional_ignored_participant_ids = nullptr )
{

    const auto replan_result =
        ::adore::planner::try_plan_obstacle_avoidance(
            planner,
            replan_base_route,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance,
            additional_ignored_participant_ids );

    if( !replan_result.success ||
        !replan_result.has_maneuver_bounds )
    {
        return std::nullopt;
    }

    dynamics::Trajectory trajectory;
    double replan_ego_s =
        get_s_on_reference_line_segments(
            replan_result.modified_route,
            vehicle_state_dynamic,
            projection_hint_s,
            params_for_obstacle_avoidance.route_window_min,
            params_for_obstacle_avoidance.max_projection_distance_from_route );
    if( !std::isfinite( replan_ego_s ) )
    {
        replan_ego_s = projection_hint_s;
        RCLCPP_WARN(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][ROUTE_ONLY][REPLAN] modified-route projection failed; using %s-route s=%.2f",
            replan_base_name,
            replan_ego_s );
    }

    auto replan_route_for_planning =
        apply_active_avoidance_speed_profile(
            replan_result.modified_route,
            replan_ego_s,
            replan_result.shift_start_s,
            replan_result.shift_end_s,
            replan_result.maneuver.release_s,
            planner.get_physical_vehicle_parameters(),
            params_for_obstacle_avoidance );

    try
    {
        trajectory =
            planner.plan_route_trajectory_from_s(
                replan_route_for_planning,
                vehicle_state_dynamic,
                traffic_participants,
                replan_ego_s );
    }
    catch( const std::exception& e )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][ROUTE_ONLY][REPLAN] planner exception on accepted modified_route; keeping previous active route for now: %s",
            e.what() );
        return std::nullopt;
    }

    if( trajectory.states.empty() )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][ROUTE_ONLY][REPLAN] planner returned empty trajectory; keeping previous active route for now" );
        return std::nullopt;
    }

    start_active_avoidance_state(
        active_avoidance_state,
        replan_result,
        replan_base_route,
        traffic_participants,
        vehicle_state_dynamic,
        params_for_obstacle_avoidance,
        true );
    if( additional_ignored_participant_ids != nullptr )
    {
        append_unique_ids(
            active_avoidance_state.obstacle_ids,
            *additional_ignored_participant_ids );
        append_unique_ids(
            active_avoidance_state.maneuver.obstacle_ids,
            *additional_ignored_participant_ids );
    }
    active_avoidance_state.modified_route =
        replan_route_for_planning;

    // Seed the monotonic-s ratchet on the freshly committed route instead of
    // leaving it at NaN (start_active_avoidance_state resets it). replan_ego_s
    // is the ego projection on this same modified route, so next cycle bounds
    // its projection by odometry from a consistent value rather than latching
    // an unguarded jump -> no lateral start discontinuity across the reshape.
    if( std::isfinite( replan_ego_s ) )
    {
        active_avoidance_state.last_modified_s = replan_ego_s;
        active_avoidance_state.last_modified_time = vehicle_state_dynamic.time;
    }

    trajectory.adjust_start_time( vehicle_state_dynamic.time );
    trajectory.label =
        obstacle_avoidance_label(
            replan_result.mode,
            replan_result.in_lane,
            replan_result.lateral_shift,
            true,
            should_indicate_avoidance_direction(
                replan_ego_s,
                replan_result.shift_start_s,
                replan_result.obstacle_s_min,
                params_for_obstacle_avoidance ) );

    Behavior trajectory_and_signal;
    trajectory_and_signal.trajectory =
        dynamics::conversions::to_ros_msg( trajectory );
    trajectory_and_signal.modified_route =
        map::conversions::to_ros_msg(
            replan_route_for_planning );

    RCLCPP_INFO(
        rclcpp::get_logger( "Behaviors" ),
        "[OA][ACTIVE_ROUTE][REPLAN] accepted dynamic modified_route from %s_route obstacle_ids=%s obstacle_id=%d cluster_s=[%.2f,%.2f] shift_start_s=%.2f shift_end_s=%.2f release_s=%.2f shift=%.2f in_lane=%s opposite=%s ghosts=%zu",
        replan_base_name,
        ids_to_string( active_avoidance_state.obstacle_ids ).c_str(),
        active_avoidance_state.obstacle_id,
        replan_result.obstacle_s_min,
        replan_result.obstacle_s_max,
        active_avoidance_state.shift_start_s,
        active_avoidance_state.shift_end_s,
        active_avoidance_state.release_s,
        active_avoidance_state.lateral_shift,
        active_avoidance_state.in_lane ? "true" : "false",
        active_avoidance_state.maneuver.uses_opposite_lane ? "true" : "false",
        active_avoidance_state.ghost_memory.size() );

    return trajectory_and_signal;
}

} // namespace

// Active obstacle-avoidance maneuver handling: keep following the stored
// modified route, monitor it for new/oncoming conflicts, replan or stop as
// needed, and release back to the mission route once the obstacle is passed.
Behavior
continue_active_avoidance(
    planner::TrajectoryPlanner& planner,
    const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
    const map::Route& route_with_signal,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const dynamics::TrafficSignalSet& traffic_signals,
    const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance,
    bool use_weather_comfort_settings,
    const dynamics::ComfortSettings& weather_comfort_settings,
    const std::string& weather_label,
    planner::ActiveAvoidanceState& active_avoidance_state )
{
    auto active_route =
        !active_avoidance_state.base_modified_route.reference_line.empty()
            ? active_avoidance_state.base_modified_route
            : active_avoidance_state.modified_route;

    // The stored modified route was created from a signal snapshot
    // at maneuver start. Re-apply the live traffic-signal state each
    // cycle so a signal that turned red during the maneuver still
    // produces a stop on the route ego is actually following.
    active_route = planner.compute_traffic_light_behavior( vehicle_state_dynamic, active_route, traffic_signals );

    // Pinned monotonic modified-route s, set once it is computed below. The
    // active-maneuver stop builders brake on the route ego is already tracking
    // from this s, so the braking path stays geometrically continuous with the
    // normal active cycle (no steering jerk). Stays NaN for the few stop paths
    // that fire before the projection is available; the helper then re-projects.
    double pinned_active_stop_s = std::numeric_limits<double>::quiet_NaN();

    auto make_active_stop_behavior =
        [&]( const std::string& label ) -> Behavior
        {
            return make_route_brake_in_place_behavior(
                planner,
                active_route,
                vehicle_state_dynamic,
                pinned_active_stop_s,
                traffic_participants,
                params_for_obstacle_avoidance,
                label );
        };

    auto make_active_conflict_stop_behavior =
        [&]( const ::adore::planner::RouteCorridorConflict& conflict ) -> Behavior
        {
            RCLCPP_WARN(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][ACTIVE_STATE][STOP] braking on active modified_route id=%d type=%s dist=%.2f ttc=%.2f",
                conflict.participant_id,
                conflict_type_name( conflict ),
                conflict.distance_s,
                conflict.time_to_conflict );
            return make_route_brake_in_place_behavior(
                planner,
                active_route,
                vehicle_state_dynamic,
                pinned_active_stop_s,
                traffic_participants,
                params_for_obstacle_avoidance,
                "obstacle avoidance: route brake on active modified_route" );
        };

    if( auto monitor_conflict =
            check_opposite_lane_monitor(
                active_route,
                vehicle_state_dynamic,
                traffic_participants,
                active_avoidance_state.maneuver,
                active_avoidance_state.obstacle_id,
                active_avoidance_state.last_modified_s,
                params_for_obstacle_avoidance,
                nullptr );
        monitor_conflict.has_value() )
    {
        return make_active_conflict_stop_behavior( monitor_conflict.value() );
    }

    const double ego_s_original =
        route_with_signal.get_s( vehicle_state_dynamic );

    const double ego_s_modified_raw =
        get_s_on_reference_line_segments(
            active_route,
            vehicle_state_dynamic,
            ego_s_original,
            params_for_obstacle_avoidance.route_window_min,
            params_for_obstacle_avoidance.max_projection_distance_from_route );

    const auto ego_s_modified_opt =
        compute_monotonic_ego_s_modified(
            ego_s_modified_raw,
            vehicle_state_dynamic,
            active_avoidance_state );

    if( !ego_s_modified_opt.has_value() )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][ERROR] ego_s_modified invalid while OA active; braking on modified_route speed profile" );

        return make_active_stop_behavior(
            "driving mission (active OA route stop: invalid modified-route projection)" );
    }

    const double ego_s_modified = ego_s_modified_opt.value();
    pinned_active_stop_s = ego_s_modified;

    const auto active_route_safety =
        ::adore::planner::check_route_corridor_safety(
            active_route,
            vehicle_state_dynamic,
            traffic_participants,
            planner.get_physical_vehicle_parameters(),
            params_for_obstacle_avoidance,
            nullptr,
            &active_avoidance_state.obstacle_ids );

    update_obstacle_ghost_memory(
        active_avoidance_state,
        active_route_safety,
        ego_s_modified,
        vehicle_state_dynamic.time,
        params_for_obstacle_avoidance );

    if( params_for_obstacle_avoidance.modified_route_safety_check_enabled &&
        active_route_safety.has_conflict )
    {
        RCLCPP_WARN(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][ROUTE_MONITOR] conflict on active route id=%d active_route=modified reason=%s",
            active_route_safety.conflict.participant_id,
            active_route_safety.conflict.reason.c_str() );
    }

    auto active_conflict =
        active_route_safety.has_conflict
            ? std::optional<::adore::planner::RouteCorridorConflict>( active_route_safety.conflict )
            : most_relevant_ghost_conflict( active_avoidance_state, ego_s_modified );

    if( active_conflict.has_value() &&
        active_avoidance_state.in_lane &&
        ::adore::planner::is_oncoming_other_lane_conflict(
            active_conflict.value(),
            planner.get_physical_vehicle_parameters(),
            params_for_obstacle_avoidance ) )
    {
        active_conflict.reset();
    }

    if( active_conflict.has_value() &&
        static_or_slow_conflict_has_side_clearance(
            active_conflict.value(),
            planner.get_physical_vehicle_parameters(),
            params_for_obstacle_avoidance ) )
    {
        active_conflict.reset();
    }

    if( active_conflict.has_value() &&
        active_conflict->object_class ==
            ::adore::planner::RouteCorridorObjectClass::StaticOrSlow &&
        std::isfinite( active_conflict->object_s_max ) &&
        std::isfinite( ego_s_modified ) &&
        active_conflict->object_s_max < ego_s_modified - 0.25 &&
        !active_conflict->currently_overlaps_ego_footprint )
    {
        active_conflict.reset();
    }

    if( active_conflict.has_value() )
    {
        // A real conflict on the active route can be a newly visible
        // obstacle. Try to reshape the modified route immediately;
        // waiting until braking is mandatory makes the maneuver look
        // artificially static.
        const bool is_new_conflict = active_route_safety.has_conflict;

        const bool stop_required =
            should_stop_for_active_conflict(
                active_route,
                vehicle_state_dynamic,
                planner.get_physical_vehicle_parameters(),
                active_conflict.value(),
                params_for_obstacle_avoidance );

        if( is_new_conflict )
        {
            if( auto replanned_behavior =
                    try_dynamic_replan_from_route(
                        planner,
                        active_route,
                        "active",
                        ego_s_modified,
                        vehicle_state_dynamic,
                        traffic_participants,
                        params_for_obstacle_avoidance,
                        active_avoidance_state,
                        &active_avoidance_state.obstacle_ids );
                replanned_behavior.has_value() )
            {
                return replanned_behavior.value();
            }

            if( auto replanned_behavior =
                    try_dynamic_replan_from_route(
                        planner,
                        route_with_signal,
                        "mission",
                        ego_s_original,
                        vehicle_state_dynamic,
                        traffic_participants,
                        params_for_obstacle_avoidance,
                        active_avoidance_state );
                replanned_behavior.has_value() )
            {
                return replanned_behavior.value();
            }
        }

        if( stop_required )
        {
            if( is_new_conflict )
            {
                RCLCPP_WARN(
                    rclcpp::get_logger( "Behaviors" ),
                    "[OA][ACTIVE_ROUTE][REPLAN] no validated dynamic replan; stopping on active modified_route" );
            }

            return make_active_conflict_stop_behavior( active_conflict.value() );
        }
    }


    const bool projection_valid =
        std::isfinite( ego_s_modified ) &&
        std::isfinite( ego_s_original );

    const double projection_error =
        projection_valid
            ? std::abs( ego_s_modified - ego_s_original )
            : std::numeric_limits<double>::infinity();

    const bool projection_consistent =
        projection_error < 2.0;

    const bool release_by_modified =
        std::isfinite( ego_s_modified ) &&
        ego_s_modified >= active_avoidance_state.release_s;

    const bool release_by_original_fallback =
        std::isfinite( ego_s_original ) &&
        ego_s_original >= active_avoidance_state.release_s &&
        projection_consistent;

    const auto retained_original_ghost =
        find_unpassed_original_ghost(
            active_avoidance_state,
            ego_s_modified );

    if( ( release_by_modified || release_by_original_fallback ) &&
        !retained_original_ghost.has_value() )
    {
        RCLCPP_INFO(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][RELEASE] modified_route completed ego_s=%.2f release_s=%.2f obstacle_id=%d ego_s_orig=%.2f by=%s returning_to_original_route",
            ego_s_modified,
            active_avoidance_state.release_s,
            active_avoidance_state.obstacle_id,
            ego_s_original,
            release_by_modified ? "modified" : "original_fallback" );

        if( projection_valid && projection_error > 2.0 )
        {
            RCLCPP_WARN(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][WARN] modified-route projection diverged: ego_s_mod=%.2f ego_s_orig=%.2f diff=%.2f",
                ego_s_modified,
                ego_s_original,
                projection_error );
        }

        active_avoidance_state.reset();

        // now normal route
        dynamics::Trajectory trajectory;
        try
        {
            trajectory =
                planner.plan_route_trajectory(
                    route_with_signal,
                    vehicle_state_dynamic,
                    traffic_participants );
        }
        catch( const std::exception& e )
        {
            RCLCPP_ERROR(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][STOP_ROUTE] planner exception after OA release; braking on active route instead: %s",
                e.what() );
            return make_active_stop_behavior(
                "driving mission (planner exception after OA release)" );
        }

        trajectory.adjust_start_time( vehicle_state_dynamic.time );
        trajectory.label = "driving mission";

        Behavior trajectory_and_signal;
        trajectory_and_signal.trajectory =
            dynamics::conversions::to_ros_msg( trajectory );

        return trajectory_and_signal;
    }
    else
    {
        dynamics::Trajectory trajectory;
        auto active_route_for_planning =
            apply_active_avoidance_speed_profile(
                active_route,
                ego_s_modified,
                active_avoidance_state.shift_start_s,
                active_avoidance_state.shift_end_s,
                active_avoidance_state.release_s,
                planner.get_physical_vehicle_parameters(),
                params_for_obstacle_avoidance );

        if( use_weather_comfort_settings )
        {
            try
            {
                trajectory =
                    planner.plan_route_trajectory_with_custom_comfort_settings_from_s(
                        active_route_for_planning,
                        vehicle_state_dynamic,
                        traffic_participants,
                        weather_comfort_settings,
                        ego_s_modified );
            }
            catch( const std::exception& e )
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger( "Behaviors" ),
                    "[OA][STOP_ROUTE] planner exception on active modified_route; braking on active route instead: %s",
                    e.what() );
                return make_active_stop_behavior(
                    "driving mission (active OA planner exception)" );
            }

            trajectory.label =
                obstacle_avoidance_label(
                    active_avoidance_state.maneuver.mode,
                    active_avoidance_state.in_lane,
                    active_avoidance_state.lateral_shift,
                    true,
                    should_indicate_avoidance_direction(
                        ego_s_modified,
                        active_avoidance_state.shift_start_s,
                        active_avoidance_state.obstacle_s_min,
                        params_for_obstacle_avoidance ) ) +
                " (" + weather_label + ")";
        }
        else
        {
            try
            {
                trajectory =
                    planner.plan_route_trajectory_from_s(
                        active_route_for_planning,
                        vehicle_state_dynamic,
                        traffic_participants,
                        ego_s_modified );
            }
            catch( const std::exception& e )
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger( "Behaviors" ),
                    "[OA][STOP_ROUTE] planner exception on active modified_route; braking on active route instead: %s",
                    e.what() );
                return make_active_stop_behavior(
                    "driving mission (active OA planner exception)" );
            }

            trajectory.label =
                obstacle_avoidance_label(
                    active_avoidance_state.maneuver.mode,
                    active_avoidance_state.in_lane,
                    active_avoidance_state.lateral_shift,
                    true,
                    should_indicate_avoidance_direction(
                        ego_s_modified,
                        active_avoidance_state.shift_start_s,
                        active_avoidance_state.obstacle_s_min,
                        params_for_obstacle_avoidance ) );
        }

        if( trajectory.states.empty() )
        {
            RCLCPP_ERROR(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][ERROR] planner returned empty trajectory on active modified_route; commanding controlled stop" );

            return make_active_stop_behavior(
                "driving mission (active OA controlled stop: planning failed)" );
        }

        if( auto monitor_conflict =
                check_opposite_lane_monitor(
                    active_route_for_planning,
                    vehicle_state_dynamic,
                    traffic_participants,
                    active_avoidance_state.maneuver,
                    active_avoidance_state.obstacle_id,
                    ego_s_modified,
                    params_for_obstacle_avoidance,
                    &trajectory );
            monitor_conflict.has_value() )
        {
            return make_active_conflict_stop_behavior( monitor_conflict.value() );
        }

        trajectory.adjust_start_time( vehicle_state_dynamic.time );

        Behavior trajectory_and_signal;
        trajectory_and_signal.trajectory =
            dynamics::conversions::to_ros_msg( trajectory );

        trajectory_and_signal.modified_route =
            map::conversions::to_ros_msg( active_route_for_planning );

        if( !std::isfinite( ego_s_modified ) )
        {
            RCLCPP_WARN(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][CONTINUE] ego_s_modified invalid while OA active; continuing modified_route" );
        }

        return trajectory_and_signal;
    }
}

// Defensive stop for oncoming traffic temporarily using the ego lane. Returns a
// stop behavior if such a participant is found, otherwise nullopt to fall
// through to weather/normal/OA planning.
std::optional<Behavior>
try_ego_lane_oncoming_stop_behavior(
    planner::TrajectoryPlanner& planner,
    const map::Route& route_with_signal,
    const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance )
{

    const auto ego_lane_oncoming_result =
        ::adore::planner::try_plan_ego_lane_oncoming_stop(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance );

    if( ego_lane_oncoming_result.success )
    {
        RCLCPP_INFO(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][EGO_LANE_ONCOMING] participant_id=%d distance=%.2f ttc=%.2f v_route=%.2f stop_s=%.2f reason=%s",
            ego_lane_oncoming_result.participant_id,
            ego_lane_oncoming_result.participant_distance_s,
            ego_lane_oncoming_result.time_to_conflict,
            ego_lane_oncoming_result.participant_route_speed,
            ego_lane_oncoming_result.stop_s,
            ego_lane_oncoming_result.reason.c_str() );

        auto trajectory = ego_lane_oncoming_result.trajectory;
        if( trajectory.states.empty() )
        {
            RCLCPP_ERROR(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][STOP_ROUTE] ego-lane oncoming stop returned empty trajectory; retrying route-based stop" );
            return make_stop_on_route_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance,
                "ego-lane oncoming stop trajectory empty",
                "original" );
        }
        trajectory.adjust_start_time( vehicle_state_dynamic.time );

        Behavior trajectory_and_signal;
        trajectory_and_signal.trajectory =
            dynamics::conversions::to_ros_msg( trajectory );

        trajectory_and_signal.modified_route =
            map::conversions::to_ros_msg( ego_lane_oncoming_result.modified_route );

        return trajectory_and_signal;
    }

    return std::nullopt;
}

// Reduced-speed route trajectory for adverse weather on the original route.
Behavior
plan_weather_behavior(
    planner::TrajectoryPlanner& planner,
    const map::Route& route_with_signal,
    const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance,
    const dynamics::ComfortSettings& weather_comfort_settings,
    const std::string& weather_label )
{
    dynamics::Trajectory trajectory;
    try
    {
        trajectory =
            planner.plan_route_trajectory_with_custom_comfort_settings(
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                weather_comfort_settings );
    }
    catch( const std::exception& e )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][STOP_ROUTE] weather planner exception; retrying route-based stop: %s",
            e.what() );
        return make_stop_on_route_behavior(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance,
            "weather route planner exception",
            "original" );
    }

    if( trajectory.states.empty() )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][STOP_ROUTE] weather planner returned empty trajectory; retrying route-based stop" );
        return make_stop_on_route_behavior(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance,
            "weather route planner returned empty trajectory",
            "original" );
    }

    trajectory.adjust_start_time( vehicle_state_dynamic.time );
    trajectory.label = "driving mission (" + weather_label + ")";

    Behavior trajectory_and_signal;
    trajectory_and_signal.trajectory =
        dynamics::conversions::to_ros_msg( trajectory );

    return trajectory_and_signal;
}

// No active maneuver: run corridor safety, then try to start a new obstacle
// avoidance maneuver; fall back to a route stop, stop-hold, or normal driving.
Behavior
plan_obstacle_avoidance_behavior(
    planner::TrajectoryPlanner& planner,
    const map::Route& route_with_signal,
    const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance,
    planner::ActiveAvoidanceState& active_avoidance_state )
{
    const auto original_route_safety =
        ::adore::planner::check_route_corridor_safety(
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            planner.get_physical_vehicle_parameters(),
            params_for_obstacle_avoidance );

    if( original_route_safety.has_conflict )
    {
        RCLCPP_WARN(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][ROUTE_MONITOR] conflict on active route id=%d active_route=original reason=%s",
            original_route_safety.conflict.participant_id,
            original_route_safety.conflict.reason.c_str() );
    }

    if( !params_for_obstacle_avoidance.enabled )
    {

        if( original_route_safety.has_conflict )
        {
            return make_stop_on_active_route_behavior(
                planner,
                route_with_signal,
                original_route_safety.conflict,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance,
                "original" );
        }

        dynamics::Trajectory trajectory;
        try
        {
            trajectory =
                planner.plan_route_trajectory(
                    route_with_signal,
                    vehicle_state_dynamic,
                    traffic_participants );
        }
        catch( const std::exception& e )
        {
            RCLCPP_ERROR(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][STOP_ROUTE] planner exception in OA-disabled mode; retrying route-based stop: %s",
                e.what() );
            return make_stop_on_route_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance,
                "driving mission (planner exception with OA disabled)",
                "original" );
        }

        if( trajectory.states.empty() )
        {
            RCLCPP_ERROR(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][STOP_ROUTE] OA-disabled route planner returned empty trajectory; retrying route-based stop" );
            return make_stop_on_route_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance,
                "OA-disabled route planner returned empty trajectory",
                "original" );
        }

        trajectory.adjust_start_time( vehicle_state_dynamic.time );
        trajectory.label = "driving mission (OA disabled)";

        Behavior trajectory_and_signal;
        trajectory_and_signal.trajectory =
            dynamics::conversions::to_ros_msg( trajectory );

        return trajectory_and_signal;
    }

    // =========================================================================
    const auto oa_result =
        ::adore::planner::try_plan_obstacle_avoidance(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance );

    if( oa_result.success )
    {
        // Only persist real avoidance maneuvers.
        // StopBeforeObstacle / WaitForOncoming may also return success=true,
        // but they must not activate a persistent avoidance state.
        if( oa_result.has_maneuver_bounds )
        {
            start_active_avoidance_state(
                active_avoidance_state,
                oa_result,
                route_with_signal,
                traffic_participants,
                vehicle_state_dynamic,
                params_for_obstacle_avoidance );
            // The active ghost memory takes over obstacle persistence.
            active_avoidance_state.stop_hold.reset();

            const auto memory_seed_obstacle_ids =
                oa_result.obstacle_ids.empty()
                    ? std::vector<int>{ oa_result.obstacle_id }
                    : oa_result.obstacle_ids;

            RCLCPP_INFO(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][START] active modified_route obstacle_ids=%s obstacle_id=%d cluster_s=[%.2f,%.2f] shift_start_s=%.2f shift_end_s=%.2f release_s=%.2f shift=%.2f in_lane=%s opposite=%s",
                ids_to_string( memory_seed_obstacle_ids ).c_str(),
                active_avoidance_state.obstacle_id,
                oa_result.obstacle_s_min,
                oa_result.obstacle_s_max,
                active_avoidance_state.shift_start_s,
                active_avoidance_state.shift_end_s,
                active_avoidance_state.release_s,
                active_avoidance_state.lateral_shift,
                active_avoidance_state.in_lane ? "true" : "false",
                active_avoidance_state.maneuver.uses_opposite_lane ? "true" : "false" );
        }
        else
        {

            // Remember the obstacle behind this stop/wait so short
            // perception dropouts do not toggle between stopping and
            // resuming.
            const auto stop_hold_participant_it =
                traffic_participants.participants.find( oa_result.obstacle_id );
            if( stop_hold_participant_it != traffic_participants.participants.end() )
            {
                const auto hold_envelope =
                    make_original_obstacle_envelope_from_participant(
                        route_with_signal,
                        stop_hold_participant_it->second,
                        vehicle_state_dynamic.time,
                        params_for_obstacle_avoidance );
                if( hold_envelope.has_value() )
                {
                    active_avoidance_state.stop_hold = hold_envelope;
                }
            }

            ::adore::planner::RouteCorridorConflict stop_conflict;
            stop_conflict.participant_id = oa_result.obstacle_id;
            stop_conflict.object_class =
                ::adore::planner::RouteCorridorObjectClass::StaticOrSlow;
            stop_conflict.object_s_min = oa_result.obstacle_s_min;
            stop_conflict.object_s_max = oa_result.obstacle_s_max;
            if( !std::isfinite( stop_conflict.object_s_min ) )
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger( "Behaviors" ),
                    "[OA][STOP_ROUTE] OA stop result has no finite obstacle_s_min; using last-resort route stop behavior reason=%s",
                    oa_result.reason.c_str() );
                return make_stop_on_route_behavior(
                    planner,
                    route_with_signal,
                    vehicle_state_dynamic,
                    traffic_participants,
                    params_for_obstacle_avoidance,
                    "OA stop result missing obstacle_s_min",
                    "original" );
            }
            stop_conflict.distance_s =
                std::max(
                    0.0,
                    oa_result.obstacle_s_min -
                    route_with_signal.get_s( vehicle_state_dynamic ) );
            stop_conflict.currently_overlaps_route_corridor = true;
            stop_conflict.requires_stop = true;
            stop_conflict.reason = oa_result.reason;

            return make_stop_on_active_route_behavior(
                planner,
                route_with_signal,
                stop_conflict,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance,
                "original" );
        }

        dynamics::Trajectory trajectory;
        const double ego_s_original =
            route_with_signal.get_s( vehicle_state_dynamic );
        double ego_s_modified =
            get_s_on_reference_line_segments(
                oa_result.modified_route,
                vehicle_state_dynamic,
                ego_s_original,
                params_for_obstacle_avoidance.route_window_min,
                params_for_obstacle_avoidance.max_projection_distance_from_route );

        if( !std::isfinite( ego_s_modified ) )
        {
            ego_s_modified = ego_s_original;
            RCLCPP_WARN(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][ROUTE_ONLY] modified-route projection failed; using original-route s=%.2f",
                ego_s_modified );
        }

        auto modified_route_for_planning =
            apply_active_avoidance_speed_profile(
                oa_result.modified_route,
                ego_s_modified,
                oa_result.shift_start_s,
                oa_result.shift_end_s,
                oa_result.maneuver.release_s,
                planner.get_physical_vehicle_parameters(),
                params_for_obstacle_avoidance );
        active_avoidance_state.modified_route =
            modified_route_for_planning;

        try
        {
            trajectory =
                planner.plan_route_trajectory_from_s(
                    modified_route_for_planning,
                    vehicle_state_dynamic,
                    traffic_participants,
                    ego_s_modified );
        }
        catch( const std::exception& e )
        {
            RCLCPP_ERROR(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][ROUTE_ONLY] planner exception on accepted modified_route; stopping on original route: %s",
                e.what() );
            return make_stop_on_route_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance,
                "OA route-only planner exception",
                "original" );
        }

        if( trajectory.states.empty() )
        {
            RCLCPP_ERROR(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][ROUTE_ONLY] planner returned empty trajectory on accepted modified_route; stopping on original route" );
            return make_stop_on_route_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance,
                "OA route-only planning failed",
                "original" );
        }

        trajectory.adjust_start_time( vehicle_state_dynamic.time );
        trajectory.label =
            obstacle_avoidance_label(
                oa_result.mode,
                oa_result.in_lane,
                oa_result.lateral_shift,
                false,
                should_indicate_avoidance_direction(
                    ego_s_modified,
                    oa_result.shift_start_s,
                    oa_result.obstacle_s_min,
                    params_for_obstacle_avoidance ) );

        Behavior trajectory_and_signal;
        trajectory_and_signal.trajectory =
            dynamics::conversions::to_ros_msg( trajectory );

        trajectory_and_signal.modified_route =
            map::conversions::to_ros_msg( modified_route_for_planning );

        return trajectory_and_signal;
    }

    if( oa_result.mode != ::adore::planner::ObstacleAvoidanceMode::None )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][FAIL_SAFE] OA returned mode=%d but no valid trajectory; holding instead of normal driving reason=%s",
            static_cast<int>( oa_result.mode ),
            oa_result.reason.c_str() );
        return make_emergency_hold_behavior(
            route_with_signal,
            vehicle_state_dynamic,
            params_for_obstacle_avoidance,
            "OA failed after detecting an obstacle: " + oa_result.reason );
    }

    // -------------------------------------------------------------------------
    // No OA result this cycle. If a stop/wait obstacle was seen recently,
    // keep stopping for its remembered envelope instead of resuming and
    // re-braking on every perception dropout.
    // -------------------------------------------------------------------------
    if( active_avoidance_state.stop_hold.has_value() )
    {
        const auto& hold = active_avoidance_state.stop_hold.value();
        const double now = vehicle_state_dynamic.time;
        const double ego_s_hold =
            route_with_signal.get_s( vehicle_state_dynamic );

        const bool hold_expired =
            !std::isfinite( hold.last_seen_time ) ||
            now > hold.last_seen_time +
                      std::max( 0.0, params_for_obstacle_avoidance.ghost_obstacle_hold_time );
        const bool hold_passed =
            std::isfinite( ego_s_hold ) &&
            ego_s_hold > hold.hold_until_s;

        if( hold_expired || hold_passed )
        {
            RCLCPP_INFO(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][STOP_HOLD] release id=%d reason=%s ego_s=%.2f hold_until_s=%.2f",
                hold.last_participant_id,
                hold_passed ? "ego_passed_hold_until_s" : "timeout_after_missing_detection",
                ego_s_hold,
                hold.hold_until_s );
            active_avoidance_state.stop_hold.reset();
        }
        else
        {
            ::adore::planner::RouteCorridorConflict hold_conflict;
            hold_conflict.participant_id = hold.last_participant_id;
            hold_conflict.object_class =
                ::adore::planner::RouteCorridorObjectClass::StaticOrSlow;
            hold_conflict.object_s_min = hold.object_s_min;
            hold_conflict.object_s_max = hold.object_s_max;
            hold_conflict.object_l_min = hold.object_l_min;
            hold_conflict.object_l_max = hold.object_l_max;
            hold_conflict.inflated_s_min = hold.inflated_s_min;
            hold_conflict.inflated_s_max = hold.inflated_s_max;
            hold_conflict.inflated_l_min = hold.inflated_l_min;
            hold_conflict.inflated_l_max = hold.inflated_l_max;
            hold_conflict.distance_s =
                std::isfinite( ego_s_hold )
                    ? std::max( 0.0, hold.object_s_min - ego_s_hold )
                    : std::numeric_limits<double>::infinity();
            hold_conflict.time_to_conflict = 0.0;
            hold_conflict.currently_overlaps_route_corridor = true;
            hold_conflict.requires_stop = true;
            hold_conflict.allows_replan = true;
            hold_conflict.ttc_source = "stop_hold";
            hold_conflict.currently_inside_corridor = true;
            hold_conflict.object_center_x = hold.object_center_x;
            hold_conflict.object_center_y = hold.object_center_y;
            hold_conflict.object_yaw = hold.object_yaw;
            hold_conflict.object_length = hold.object_length;
            hold_conflict.object_width = hold.object_width;
            hold_conflict.footprint_x = hold.footprint_x;
            hold_conflict.footprint_y = hold.footprint_y;
            hold_conflict.has_world_footprint = hold.has_world_footprint;
            hold_conflict.reason =
                "stop-hold envelope retained after temporary detection loss";

            RCLCPP_WARN(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][STOP_HOLD] obstacle id=%d not detected this cycle; keeping stop before remembered envelope s=[%.2f,%.2f] ego_s=%.2f",
                hold.last_participant_id,
                hold.object_s_min,
                hold.object_s_max,
                ego_s_hold );

            return make_stop_on_active_route_behavior(
                planner,
                route_with_signal,
                hold_conflict,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance,
                "original" );
        }
    }

    // -------------------------------------------------------------------------
    // Normal driving mission if no OA result is available.
    // -------------------------------------------------------------------------
    dynamics::Trajectory trajectory;
    try
    {
        trajectory =
            planner.plan_route_trajectory(
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants );
    }
    catch( const std::exception& e )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][STOP_ROUTE] route planner exception; retrying route-based stop: %s",
            e.what() );
        return make_stop_on_route_behavior(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance,
            "route planner exception",
            "original" );
    }

    if( trajectory.states.empty() )
    {
        RCLCPP_ERROR(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][STOP_ROUTE] route planner returned empty trajectory; retrying route-based stop" );
        return make_stop_on_route_behavior(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance,
            "route planner returned empty trajectory",
            "original" );
    }

    trajectory.adjust_start_time( vehicle_state_dynamic.time );
    trajectory.label = "driving mission";

    Behavior trajectory_and_signal;
    trajectory_and_signal.trajectory =
        dynamics::conversions::to_ros_msg( trajectory );

    return trajectory_and_signal;
}

} // namespace behavior
} // namespace adore
