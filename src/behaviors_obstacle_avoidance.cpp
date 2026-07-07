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
#include <dynamics/comfort_settings.hpp>
#include <dynamics/trajectory.hpp>
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <optional>
#include <string>

namespace adore
{
namespace behavior
{
namespace
{

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
using planner::start_active_avoidance_state;
using planner::make_oncoming_monitor_conflict;
using planner::static_or_slow_conflict_has_side_clearance;
using planner::should_stop_for_active_conflict;
using planner::compute_monotonic_ego_s_modified;

double
ego_front_offset( const dynamics::PhysicalVehicleParameters& vehicle_params )
{
    return vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
}

// Route-s at which ego's front comes to rest stop_before_obstacle ahead of an
// obstacle/conflict reference edge. Non-finite reference_s -> NaN.
double
stop_s_before_obstacle(
    double reference_s,
    const planner::ObstacleAvoidanceParams& params,
    const dynamics::PhysicalVehicleParameters& vehicle_params )
{
    if( !std::isfinite( reference_s ) )
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return reference_s
         - std::max( 0.0, params.stop_before_obstacle )
         - ego_front_offset( vehicle_params );
}

map::Route
apply_route_max_speed_cap( const map::Route& route, double max_speed )
{
    map::Route capped_route = route;
    if( !std::isfinite( max_speed ) || max_speed < 0.0 )
    {
        return capped_route;
    }

    for( auto& [s, point] : capped_route.reference_line )
    {
        static_cast<void>( s );
        point.max_speed =
            std::min(
                point.max_speed.value_or(
                    std::numeric_limits<double>::infinity() ),
                max_speed );
    }

    return capped_route;
}

map::Route
apply_active_avoidance_speed_profile(
    const map::Route& active_route,
    double ego_s,
    double shift_start_s,
    double shift_end_s,
    double release_s,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::ObstacleAvoidanceParams& params,
    double ego_v = std::numeric_limits<double>::infinity() )
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
        params,
        ego_v );
}

// Copy of params with max_speed_during_avoidance sized (physics) to the maneuver
// shift and its FIXED entry-ramp length (obstacle_s_min - shift_start_s), so a
// re-applied avoidance speed profile matches what try_plan validated instead of
// overwriting it with the fixed cap (see planner::avoidance_speed_for_shift).
// Must use the maneuver ramp, NOT the shrinking ego->obstacle distance, otherwise
// the sized speed would keep dropping as ego approaches and ego would crawl.
static planner::ObstacleAvoidanceParams
params_with_sized_avoidance_speed(
    const planner::ObstacleAvoidanceParams& params,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    double entry_ramp_length,
    double lateral_shift )
{
    auto sized = params;
    const double ego_front_offset =
        vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
    const double ramp =
        ::adore::planner::avoidance_ramp_length(
            entry_ramp_length, ego_front_offset, params );
    sized.max_speed_during_avoidance =
        ::adore::planner::avoidance_speed_for_shift(
            ramp, lateral_shift, params );
    return sized;
}

map::Route
make_max_braking_route_from_s(
    const map::Route& route,
    double ego_s,
    double ego_v,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::ObstacleAvoidanceParams& params )
{
    // Maximum-deceleration braking profile from ego onward (|acceleration_min|),
    // not a hard zero-from-ego, so the fallback still reduces speed smoothly.
    return planner::RouteSpeedPolicy::apply_max_braking_profile(
        route, ego_s, ego_v, vehicle_params, params );
}

map::Route
make_stop_route_on_active_route(
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    double requested_stop_s,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::ObstacleAvoidanceParams& params )
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
    }

    const double a_abs =
        planner::planned_braking_deceleration( vehicle_params, params );
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
        const auto stop_plan =
            planner::RouteStopPolicy::plan_stop_on_route(
                stop_route,
                ego_s,
                ego.vx,
                ego_s,
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
            vehicle_params,
            params );
    return stop_plan.route;
}

Behavior
make_route_failsafe_behavior(
    planner::TrajectoryPlanner& planner,
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params )
{
    double ego_s =
        project_s_on_reference_line(
            active_route,
            ego,
            std::numeric_limits<double>::quiet_NaN() );
    if( !std::isfinite( ego_s ) && !active_route.reference_line.empty() )
    {
        ego_s = active_route.reference_line.begin()->first;
    }

    map::Route failsafe_route =
        make_max_braking_route_from_s(
            active_route,
            ego_s,
            ego.vx,
            planner.get_physical_vehicle_parameters(),
            params );

    dynamics::Trajectory trajectory;
    try
    {
        trajectory =
            std::isfinite( ego_s )
                ? planner.plan_route_trajectory_from_s(
                    failsafe_route,
                    ego,
                    traffic_participants,
                    ego_s )
                : planner.plan_route_trajectory(
                    failsafe_route,
                    ego,
                    traffic_participants );
    }
    catch( const std::exception& )
    {
    }

    // A zero-speed route is the final fallback. Even here no trajectory states
    // are fabricated: the regular route planner either derives them from this
    // route or returns an empty result while the route is still published.
    if( trajectory.states.empty() )
    {
        failsafe_route = active_route;
        planner::set_route_points_from_s_to_zero( failsafe_route, ego_s );
        try
        {
            trajectory =
                std::isfinite( ego_s )
                    ? planner.plan_route_trajectory_from_s(
                        failsafe_route,
                        ego,
                        traffic_participants,
                        ego_s )
                    : planner.plan_route_trajectory(
                        failsafe_route,
                        ego,
                        traffic_participants );
        }
        catch( const std::exception& )
        {
        }
    }

    if( !trajectory.states.empty() )
    {
        trajectory.label = "obstacle avoidance: route failsafe";
        trajectory.adjust_start_time( ego.time );
    }

    Behavior behavior;
    behavior.trajectory = dynamics::conversions::to_ros_msg( trajectory );
    behavior.modified_route = map::conversions::to_ros_msg( failsafe_route );

    return behavior;
}

Behavior
make_stop_on_route_behavior(
    planner::TrajectoryPlanner& planner,
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params,
    const std::string& active_route_name,
    map::Route* accepted_stop_route = nullptr )
{
    auto stop_route =
        make_stop_route_on_active_route(
            active_route,
            ego,
            std::numeric_limits<double>::quiet_NaN(),
            planner.get_physical_vehicle_parameters(),
            params );

    dynamics::Trajectory trajectory;
    try
    {
        trajectory =
            planner.plan_route_trajectory(
                stop_route,
                ego,
                traffic_participants );
    }
    catch( const std::exception& )
    {
    }

    if( trajectory.states.empty() )
    {
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
            make_max_braking_route_from_s(
                active_route,
                ego_s,
                ego.vx,
                planner.get_physical_vehicle_parameters(),
                params );

        try
        {
            trajectory =
                planner.plan_route_trajectory(
                    max_braking_route,
                    ego,
                    traffic_participants );
        }
        catch( const std::exception& )
        {
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

        return make_route_failsafe_behavior(
            planner,
            active_route,
            ego,
            traffic_participants,
            params );
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
    double conflict_stop_s,
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

    const auto vehicle_params = planner.get_physical_vehicle_parameters();

    // With an explicit stop target, brake toward exactly that point:
    // apply_brake_envelope keeps full speed until braking is needed, then brakes
    // gently when the target is far enough and at the physical maximum only when
    // too close (so ego stops AT the target, not early). Without a target (generic
    // active stop) fall back to the nearest comfortable in-place stop, which is
    // zero distance when ego is already stopped, so it holds instead of creeping.
    double target_stop_s = conflict_stop_s;
    if( !std::isfinite( target_stop_s ) )
    {
        const double comfort_decel =
            ::adore::planner::planned_braking_deceleration( vehicle_params, params );
        const double v = std::max( 0.0, ego.vx );
        target_stop_s = brake_from_s + ( v * v ) / ( 2.0 * comfort_decel );
    }

    map::Route brake_route =
        ::adore::planner::RouteSpeedPolicy::apply_brake_envelope(
            active_route,
            brake_from_s,
            ego.vx,
            target_stop_s,
            vehicle_params,
            params );

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
    catch( const std::exception& )
    {
    }

    if( trajectory.states.empty() )
    {
        return make_route_failsafe_behavior(
            planner,
            active_route,
            ego,
            traffic_participants,
            params );
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
    double conflict_hint_s,
    const planner::ObstacleAvoidanceParams& params,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const dynamics::Trajectory* planned_trajectory )
{
    if( !( maneuver.active && maneuver.uses_opposite_lane ) )
    {
        return std::nullopt;
    }

    const auto monitor_result =
        ::adore::planner::monitor_active_obstacle_avoidance_maneuver(
            route,
            ego,
            traffic_participants,
            maneuver,
            vehicle_params,
            params,
            planned_trajectory );

    if( monitor_result.should_abort_before_commitment )
    {
        return make_oncoming_monitor_conflict(
            monitor_result,
            conflict_hint_s );
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
            return make_oncoming_monitor_conflict(
                monitor_result,
                conflict_hint_s );
        }

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
    double projection_hint_s,
    const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance,
    planner::ActiveAvoidanceState& active_avoidance_state,
    const std::vector<int>* additional_ignored_participant_ids = nullptr,
    const std::vector<int>* committed_obstacle_ids = nullptr,
    double min_shift_magnitude = 0.0 )
{
    // Mid-maneuver replan for a new obstacle. try_plan sizes the entry ramp to the
    // available distance and the maneuver speed to the shift (physics), so a late
    // obstacle where ego is close still gets a fitting short ramp + matching slow
    // speed. The committed obstacle stays in the group but its clearance is not
    // re-validated from ego's transient turn-in pose (committed_obstacle_ids).
    const auto replan_result =
        ::adore::planner::try_plan_obstacle_avoidance(
            planner,
            replan_base_route,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance,
            additional_ignored_participant_ids,
            committed_obstacle_ids );

    if( !replan_result.success ||
        !replan_result.has_maneuver_bounds )
    {
        // Surface why the reshape produced no maneuver (drivable-area /
        // validation / projection / oncoming ...). Otherwise this failure is
        // invisible: the caller just falls through to a route brake. Diagnostic
        // for the late-appearing second-obstacle case where ego brakes instead
        // of extending the avoidance.
        static rclcpp::Clock replan_fail_log_clock{ RCL_STEADY_TIME };
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger( "Behaviors" ),
            replan_fail_log_clock,
            1000,
            "[OA][REPLAN_FAIL] no maneuver from base route (hint_s=%.1f, mode=%d): %s",
            projection_hint_s,
            static_cast<int>( replan_result.mode ),
            replan_result.reason.c_str() );
        return std::nullopt;
    }

    // Strict monotonic widen: a mid-maneuver reshape must never NARROW the committed
    // shift (that would pull ego back toward the object and let fragment flicker
    // oscillate the path). Reject a smaller shift; the caller then keeps the current
    // route or brakes. Shrinking back out of the shift is a separate, deferred idea.
    if( std::fabs( replan_result.lateral_shift ) + 1e-3 <
        std::max( 0.0, min_shift_magnitude ) )
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
    }

    // Match the re-applied speed profile to try_plan's physics-sized maneuver:
    // the avoidance speed depends on the chosen shift and the available ramp, so
    // recompute it (same helper) instead of letting the re-apply overwrite the
    // slow validated speed with the fixed cap.
    const auto replan_params =
        params_with_sized_avoidance_speed(
            params_for_obstacle_avoidance,
            planner.get_physical_vehicle_parameters(),
            replan_result.obstacle_s_min - replan_result.shift_start_s,
            replan_result.lateral_shift );

    auto replan_route_for_planning =
        apply_active_avoidance_speed_profile(
            replan_result.modified_route,
            replan_ego_s,
            replan_result.shift_start_s,
            replan_result.shift_end_s,
            replan_result.maneuver.release_s,
            planner.get_physical_vehicle_parameters(),
            replan_params,
            vehicle_state_dynamic.vx );

    try
    {
        trajectory =
            planner.plan_route_trajectory_from_s(
                replan_route_for_planning,
                vehicle_state_dynamic,
                traffic_participants,
                replan_ego_s );
    }
    catch( const std::exception& )
    {
        return std::nullopt;
    }

    if( trajectory.states.empty() )
    {
        return std::nullopt;
    }

    start_active_avoidance_state(
        active_avoidance_state,
        replan_result );
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
                replan_params ) );

    Behavior trajectory_and_signal;
    trajectory_and_signal.trajectory =
        dynamics::conversions::to_ros_msg( trajectory );
    trajectory_and_signal.modified_route =
        map::conversions::to_ros_msg(
            replan_route_for_planning );

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
                std::numeric_limits<double>::quiet_NaN(),
                traffic_participants,
                params_for_obstacle_avoidance,
                label );
        };

    auto make_active_conflict_stop_behavior =
        [&]( const ::adore::planner::RouteCorridorConflict& conflict ) -> Behavior
        {
            // Stop point before the conflict: brake gently if it is far enough,
            // otherwise apply_brake_envelope falls back to maximum braking.
            const auto vehicle_params = planner.get_physical_vehicle_parameters();

            // For an oncoming conflict, wait pulled up at the avoided obstacle
            // (still in the ego lane) - exactly like the opposite-lane-disabled
            // case. The opposite-lane conflict line already carries the oncoming
            // rear safety margin, so stopping stop_before_obstacle ahead of it
            // would double-count the margin and stop ego far too early.
            const bool oncoming_conflict =
                conflict.object_class ==
                    ::adore::planner::RouteCorridorObjectClass::Oncoming;
            const double reference_s =
                ( oncoming_conflict &&
                  std::isfinite( active_avoidance_state.obstacle_s_min ) )
                    ? active_avoidance_state.obstacle_s_min
                    : conflict.object_s_min;

            const double conflict_stop_s =
                stop_s_before_obstacle(
                    reference_s, params_for_obstacle_avoidance, vehicle_params );

            return make_route_brake_in_place_behavior(
                planner,
                active_route,
                vehicle_state_dynamic,
                pinned_active_stop_s,
                conflict_stop_s,
                traffic_participants,
                params_for_obstacle_avoidance,
                "obstacle avoidance: route brake on active modified_route" );
        };

    // Update the ego progress on the modified route before any active-stop or
    // oncoming-wait path can return. The vehicle can travel a substantial
    // distance while braking before it reaches the wait position. If
    // last_modified_s remains frozen at the first braking cycle, the projection
    // after release looks like an implausible forward jump and the monotonic
    // filter restarts the trajectory tens of metres behind ego, on the still
    // unshifted part of the route.
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
        return make_active_stop_behavior(
            "driving mission (active OA route stop: invalid modified-route projection)" );
    }

    const double ego_s_modified = ego_s_modified_opt.value();
    pinned_active_stop_s = ego_s_modified;

    // fix(oa) branch: commit as soon as a maneuver is active, not only once ego has
    // physically begun the lateral shift. Once an avoidance is planned for a static
    // object (v < max_static_object_speed), ego drives the modified route and is
    // driven to release_s; it never flickers back to the mission line pre-commit and
    // is replanned only from the modified route. A static object cannot leave the
    // corridor, so a lost detection is a perception glitch, never a real departure --
    // holding the maneuver is the safe response. The only return to the mission line
    // is via release_s (ego has passed the obstacle).
    active_avoidance_state.committed = true;

    // Oncoming-wait latch. Once we stop for an oncoming participant, keep holding
    // (pulled up at the avoided obstacle) until that participant has cleared the
    // opposite-lane conflict interval, instead of re-running the per-cycle go/stop
    // decision. Re-deciding each cycle near the decision boundary makes ego
    // oscillate between braking and creeping for the whole wait; latching the stop
    // turns that limit cycle into a single clean hold.
    if( active_avoidance_state.oncoming_wait_active )
    {
        bool release_wait = false;

        const auto wait_participant_it =
            traffic_participants.participants.find(
                active_avoidance_state.oncoming_wait_participant_id );
        if( wait_participant_it != traffic_participants.participants.end() )
        {
            active_avoidance_state.oncoming_wait_last_seen_time =
                vehicle_state_dynamic.time;

            const double participant_s =
                project_s_on_reference_line(
                    active_route,
                    wait_participant_it->second.state,
                    active_avoidance_state.oncoming_wait_release_s );

            // The oncoming travels against the route direction, so its route-s
            // decreases as it approaches; it has cleared the conflict interval once
            // it drops below the near edge (conflict_start_s).
            if( std::isfinite( participant_s ) &&
                std::isfinite( active_avoidance_state.oncoming_wait_release_s ) &&
                participant_s < active_avoidance_state.oncoming_wait_release_s )
            {
                release_wait = true;
            }

            // Also release if the participant has come to a stop and no longer
            // reaches into the driven corridor (e.g. it pulled aside to let ego
            // through). The monitor's corridor gate is bypassed while latched, so
            // mirror it here: a stopped participant that leaves side clearance to the
            // route-centered ego corridor is not blocking and the hold can end even
            // though it never crossed the release edge longitudinally.
            if( !release_wait &&
                std::fabs( wait_participant_it->second.state.vx ) <=
                    params_for_obstacle_avoidance.max_static_object_speed &&
                ::adore::planner::participant_has_side_clearance_to_route_corridor(
                    active_route,
                    wait_participant_it->second,
                    planner.get_physical_vehicle_parameters(),
                    params_for_obstacle_avoidance ) )
            {
                release_wait = true;
            }
        }
        else
        {
            // A participant that is no longer perceived is gone: release the latch
            // immediately. The vehicle detects present objects directly, so an
            // absent detection is treated as an absent object, not bridged.
            release_wait = true;
        }

        if( !release_wait )
        {
            // Hold via a synthetic oncoming conflict so the stop target (pull-up at
            // the avoided obstacle) is identical every cycle regardless of monitor
            // flicker -> no creep, no re-decision.
            ::adore::planner::RouteCorridorConflict held_conflict;
            held_conflict.object_class =
                ::adore::planner::RouteCorridorObjectClass::Oncoming;
            held_conflict.participant_id =
                active_avoidance_state.oncoming_wait_participant_id;
            held_conflict.object_s_min =
                active_avoidance_state.oncoming_wait_release_s;
            held_conflict.object_s_max =
                active_avoidance_state.oncoming_wait_release_s;
            held_conflict.predicted_spatiotemporal_conflict = true;
            held_conflict.requires_stop = true;
            held_conflict.reason =
                "oncoming-wait latch: holding until oncoming clears the conflict interval";
            return make_active_conflict_stop_behavior( held_conflict );
        }

        active_avoidance_state.clear_oncoming_wait();
    }

    if( auto monitor_conflict =
            check_opposite_lane_monitor(
                active_route,
                vehicle_state_dynamic,
                traffic_participants,
                active_avoidance_state.maneuver,
                active_avoidance_state.last_modified_s,
                params_for_obstacle_avoidance,
                planner.get_physical_vehicle_parameters(),
                nullptr );
        monitor_conflict.has_value() )
    {
        // Engage the wait latch for a genuine oncoming participant so this stop
        // sticks until it has cleared the conflict interval. The conflict carries
        // conflict_start_s as object_s_min (the release edge).
        if( monitor_conflict->object_class ==
                ::adore::planner::RouteCorridorObjectClass::Oncoming &&
            monitor_conflict->participant_id >= 0 &&
            std::isfinite( monitor_conflict->object_s_min ) )
        {
            active_avoidance_state.oncoming_wait_active = true;
            active_avoidance_state.oncoming_wait_participant_id =
                monitor_conflict->participant_id;
            active_avoidance_state.oncoming_wait_release_s =
                monitor_conflict->object_s_min;
            active_avoidance_state.oncoming_wait_last_seen_time =
                vehicle_state_dynamic.time;
        }
        return make_active_conflict_stop_behavior( monitor_conflict.value() );
    }

    // Stage 1: a single id-independent corridor pass on the DRIVEN (modified) route,
    // with NO ignore list. A fragment the committed shift has cleared projects
    // laterally out of the shifted corridor and never appears; only fragments that
    // still intrude are returned, regardless of tracking id. This one pass replaces
    // the former active_route_safety (id-ignore) + driven-corridor growth guard
    // (max-hull) pair. Membership is decided by position each cycle, so re-id /
    // fragmentation is handled by location, not identity.
    const auto driven_safety =
        ::adore::planner::check_route_corridor_safety(
            active_route,
            vehicle_state_dynamic,
            traffic_participants,
            planner.get_physical_vehicle_parameters(),
            params_for_obstacle_avoidance );

    const double ego_front_reach =
        planner.get_physical_vehicle_parameters().wheelbase +
        planner.get_physical_vehicle_parameters().front_axle_to_front_border;
    const double ego_front_s = ego_s_modified + ego_front_reach;

    // Classify every intruding conflict by its EDGES (consistent with the front-bumper
    // scan, which gates on the far edge s_max). A Stopping condition wins over a widen.
    //  - static, straddling (near edge s_min <= ego_front_s -> ego alongside it, no
    //    room to develop a shift) -> stop;
    //  - static, fully ahead (s_min > ego_front_s) -> widen (replan);
    //  - moving object in the corridor -> stop if braking is warranted (OA cannot shift
    //    around it). Opposite-lane oncoming timing is handled by the monitor above.
    // Non-blocking conflicts (a static object still keeping side_clearance to the
    // shifted ego, or an oncoming other-lane object during an in-lane shift) are skipped.
    std::optional<::adore::planner::RouteCorridorConflict> stop_conflict;
    std::optional<::adore::planner::RouteCorridorConflict> ahead_conflict;
    std::vector<int> passed_ids; // obstacles ego is currently alongside / passing

    const auto prefer_more_urgent =
        [&]( const std::optional<::adore::planner::RouteCorridorConflict>& current,
             const ::adore::planner::RouteCorridorConflict& candidate )
        {
            return !current.has_value() ||
                   ( candidate.currently_overlaps_ego_footprint &&
                     !current->currently_overlaps_ego_footprint ) ||
                   ( candidate.currently_overlaps_ego_footprint ==
                         current->currently_overlaps_ego_footprint &&
                     candidate.distance_s < current->distance_s );
        };

    for( const auto& conflict : driven_safety.conflicts )
    {
        if( !std::isfinite( conflict.object_s_min ) ||
            !std::isfinite( conflict.object_s_max ) )
        {
            continue;
        }

        // Oncoming other-lane traffic that keeps clear during an in-lane shift does
        // not block (mirrors the previous active-conflict filter).
        if( active_avoidance_state.in_lane &&
            ::adore::planner::is_oncoming_other_lane_conflict(
                conflict,
                planner.get_physical_vehicle_parameters(),
                params_for_obstacle_avoidance ) )
        {
            continue;
        }

        const bool is_static =
            conflict.object_class ==
            ::adore::planner::RouteCorridorObjectClass::StaticOrSlow;

        if( is_static )
        {
            // A static object still keeping side_clearance to the shifted ego is
            // already cleared by the maneuver -> not blocking.
            if( static_or_slow_conflict_has_side_clearance(
                    conflict,
                    planner.get_physical_vehicle_parameters(),
                    params_for_obstacle_avoidance ) )
            {
                continue;
            }

            if( conflict.object_s_min <= ego_front_s )
            {
                // Straddling: ego is alongside the near edge, cannot widen -> stop.
                passed_ids.push_back( conflict.participant_id );
                if( prefer_more_urgent( stop_conflict, conflict ) )
                {
                    stop_conflict = conflict;
                }
            }
            else if( !ahead_conflict.has_value() ||
                     conflict.distance_s < ahead_conflict->distance_s )
            {
                // Fully ahead: room to widen the shift for it.
                ahead_conflict = conflict;
            }
        }
        else
        {
            // Moving object in the driven corridor: OA cannot shift around it. Stop if
            // braking is warranted (a crossing / entering vehicle or pedestrian during
            // an in-lane or adjacent shift; opposite-lane oncoming is handled above).
            if( should_stop_for_active_conflict(
                    active_route,
                    vehicle_state_dynamic,
                    planner.get_physical_vehicle_parameters(),
                    conflict,
                    params_for_obstacle_avoidance ) &&
                prefer_more_urgent( stop_conflict, conflict ) )
            {
                stop_conflict = conflict;
            }
        }
    }

    // A Stopping condition wins: ego is alongside an un-cleared static object, or a
    // moving object requires braking. Brake on the driven route (no snap).
    if( stop_conflict.has_value() )
    {
        static rclcpp::Clock driven_stop_log_clock{ RCL_STEADY_TIME };
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger( "Behaviors" ), driven_stop_log_clock, 1000,
            "[OA][DRIVEN] blocking conflict id=%d class=%d s=[%.1f,%.1f] -> stop",
            stop_conflict->participant_id,
            static_cast<int>( stop_conflict->object_class ),
            stop_conflict->object_s_min,
            stop_conflict->object_s_max );
        return make_active_conflict_stop_behavior( stop_conflict.value() );
    }

    // A static fragment fully ahead reveals the object is wider / longer than the
    // current shift. Widen: replan from the driven route. The ignore + skip-clearance
    // lists are computed fresh from position (passed_ids) so obstacles ego is passing
    // are not re-triggered and not clearance-failed while ego turns in for the one
    // ahead. Strict monotonic: adopt only a shift at least as large as the current one,
    // so fragment flicker cannot narrow / oscillate the path.
    if( ahead_conflict.has_value() )
    {
        static rclcpp::Clock driven_widen_log_clock{ RCL_STEADY_TIME };
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger( "Behaviors" ), driven_widen_log_clock, 1000,
            "[OA][DRIVEN] ahead intrusion id=%d s=[%.1f,%.1f] -> widen",
            ahead_conflict->participant_id,
            ahead_conflict->object_s_min,
            ahead_conflict->object_s_max );

        const double current_shift_magnitude =
            std::fabs( active_avoidance_state.lateral_shift );

        if( auto replanned_behavior =
                try_dynamic_replan_from_route(
                    planner,
                    active_route,
                    ego_s_modified,
                    vehicle_state_dynamic,
                    traffic_participants,
                    params_for_obstacle_avoidance,
                    active_avoidance_state,
                    &passed_ids,
                    &passed_ids,
                    current_shift_magnitude );
            replanned_behavior.has_value() )
        {
            return replanned_behavior.value();
        }

        // No wider maneuver validated (or it would narrow the shift) -> brake on the
        // driven route rather than snapping toward the mission line.
        return make_active_conflict_stop_behavior( ahead_conflict.value() );
    }

    // fix(oa): the maneuver commits immediately (committed = true above), so there
    // is no pre-commit return-to-mission path. A static object (v <
    // max_static_object_speed) that drops out of perception is treated as a glitch
    // and the shift is held; the only return to the mission line is via release_s
    // (obstacle passed) below, or a full stop when avoidance is no longer possible.
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

    if( release_by_modified || release_by_original_fallback )
    {
        active_avoidance_state.reset();

        // Resume planning on the mission route after releasing the maneuver.
        dynamics::Trajectory trajectory;
        try
        {
            trajectory =
                planner.plan_route_trajectory(
                    route_with_signal,
                    vehicle_state_dynamic,
                    traffic_participants );
        }
        catch( const std::exception& )
        {
            return make_active_stop_behavior(
                "driving mission (planner exception after OA release)" );
        }

        trajectory.adjust_start_time( vehicle_state_dynamic.time );
        trajectory.label = "driving mission";

        Behavior trajectory_and_signal;
        trajectory_and_signal.trajectory =
            dynamics::conversions::to_ros_msg( trajectory );
        trajectory_and_signal.modified_route =
            map::conversions::to_ros_msg( route_with_signal );

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
                params_with_sized_avoidance_speed(
                    params_for_obstacle_avoidance,
                    planner.get_physical_vehicle_parameters(),
                    active_avoidance_state.obstacle_s_min - active_avoidance_state.shift_start_s,
                    active_avoidance_state.lateral_shift ),
                vehicle_state_dynamic.vx );

        if( use_weather_comfort_settings )
        {
            active_route_for_planning =
                apply_route_max_speed_cap(
                    active_route_for_planning,
                    weather_comfort_settings.max_speed );
        }

        try
        {
            trajectory =
                planner.plan_route_trajectory_from_s(
                    active_route_for_planning,
                    vehicle_state_dynamic,
                    traffic_participants,
                    ego_s_modified );
        }
        catch( const std::exception& )
        {
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
        if( use_weather_comfort_settings )
        {
            trajectory.label += " (" + weather_label + ")";
        }

        if( trajectory.states.empty() )
        {

            return make_active_stop_behavior(
                "driving mission (active OA controlled stop: planning failed)" );
        }

        if( auto monitor_conflict =
                check_opposite_lane_monitor(
                    active_route_for_planning,
                    vehicle_state_dynamic,
                    traffic_participants,
                    active_avoidance_state.maneuver,
                    ego_s_modified,
                    params_for_obstacle_avoidance,
                    planner.get_physical_vehicle_parameters(),
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

        auto trajectory = ego_lane_oncoming_result.trajectory;
        if( trajectory.states.empty() )
        {
            return make_stop_on_route_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance,
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
    const auto weather_route =
        apply_route_max_speed_cap(
            route_with_signal,
            weather_comfort_settings.max_speed );

    dynamics::Trajectory trajectory;
    try
    {
        trajectory =
            planner.plan_route_trajectory(
                weather_route,
                vehicle_state_dynamic,
                traffic_participants );
    }
    catch( const std::exception& )
    {
        return make_stop_on_route_behavior(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance,
            "original" );
    }

    if( trajectory.states.empty() )
    {
        return make_stop_on_route_behavior(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance,
            "original" );
    }

    trajectory.adjust_start_time( vehicle_state_dynamic.time );
    trajectory.label = "driving mission (" + weather_label + ")";

    Behavior trajectory_and_signal;
    trajectory_and_signal.trajectory =
        dynamics::conversions::to_ros_msg( trajectory );
    trajectory_and_signal.modified_route =
        map::conversions::to_ros_msg( weather_route );

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
    if( !params_for_obstacle_avoidance.enabled )
    {
        active_avoidance_state.reset();

        dynamics::Trajectory trajectory;
        try
        {
            trajectory =
                planner.plan_route_trajectory(
                    route_with_signal,
                    vehicle_state_dynamic,
                    traffic_participants );
        }
        catch( const std::exception& )
        {
            // Route-only invariant: even when OA is disabled the node never emits
            // a fabricated routeless trajectory. Brake to a stop on the published
            // route instead of the free-space emergency() hold.
            return make_route_failsafe_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance );
        }

        if( trajectory.states.empty() )
        {
            return make_route_failsafe_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance );
        }

        trajectory.adjust_start_time( vehicle_state_dynamic.time );
        trajectory.label = "driving mission";

        Behavior behavior;
        behavior.trajectory =
            dynamics::conversions::to_ros_msg( trajectory );
        behavior.modified_route =
            map::conversions::to_ros_msg( route_with_signal );
        return behavior;
    }

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
            // Fresh maneuver: recorded uncommitted here, but on fix(oa) the next
            // continue_active_avoidance cycle commits it immediately (static
            // object). The flag is kept only for consistency with the shared
            // ActiveAvoidanceState; it drives no pre-commit behaviour on this branch.
            start_active_avoidance_state(
                active_avoidance_state,
                oa_result );
            active_avoidance_state.committed = false;
        }
        else
        {
            // While the obstacle is still beyond braking range, do not force a stop
            // trajectory: building a route-stop-policy stop here can
            // false-fail its in-horizon stop proof when the stop point is far and
            // fall back to a last-resort hold (a v=0 freeze with no usable route
            // trajectory). try_plan_obstacle_avoidance already returned a valid
            // route-following trajectory for this far case, so publish that and keep
            // ego moving on its route. The maneuver is re-evaluated next cycle as ego
            // approaches; this upholds the invariant that ego always follows a route
            // with a valid trajectory.
            const auto far_vehicle_params =
                planner.get_physical_vehicle_parameters();
            const double far_ego_s =
                route_with_signal.get_s( vehicle_state_dynamic );
            const double far_v0 = std::max( 0.0, vehicle_state_dynamic.vx );
            const double far_comfort_braking_distance =
                far_v0 * far_v0 /
                ( 2.0 * ::adore::planner::planned_braking_deceleration(
                            far_vehicle_params, params_for_obstacle_avoidance ) );
            const double far_stop_s =
                stop_s_before_obstacle(
                    oa_result.obstacle_s_min,
                    params_for_obstacle_avoidance,
                    far_vehicle_params );
            const bool obstacle_still_far =
                !oa_result.trajectory.states.empty() &&
                std::isfinite( far_ego_s ) &&
                std::isfinite( oa_result.obstacle_s_min ) &&
                ( far_stop_s - far_ego_s ) >
                    far_comfort_braking_distance +
                    std::max( 0.0,
                              params_for_obstacle_avoidance
                                  .modified_route_braking_safety_margin );

            if( obstacle_still_far )
            {
                auto far_trajectory = oa_result.trajectory;
                far_trajectory.adjust_start_time( vehicle_state_dynamic.time );

                Behavior far_behavior;
                far_behavior.trajectory =
                    dynamics::conversions::to_ros_msg( far_trajectory );
                far_behavior.modified_route =
                    map::conversions::to_ros_msg( oa_result.modified_route );
                return far_behavior;
            }

            if( !std::isfinite( far_stop_s ) )
            {
                return make_stop_on_route_behavior(
                    planner,
                    route_with_signal,
                    vehicle_state_dynamic,
                    traffic_participants,
                    params_for_obstacle_avoidance,
                    "original" );
            }

            // Brake to a clean stop before the obstacle on the same unified
            // brake-envelope used for the oncoming wait: coast at the current speed
            // (no acceleration, via the coast cap in apply_brake_envelope) then brake
            // comfortably to far_stop_s. This replaces the old route-stop-policy stop,
            // whose hard reachable/unreachable switch produced the creep-vs-brake
            // limit cycle around v0=0.5 seen on the final approach.
            return make_route_brake_in_place_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                std::numeric_limits<double>::quiet_NaN(),
                far_stop_s,
                traffic_participants,
                params_for_obstacle_avoidance,
                "obstacle avoidance: stop before obstacle on original_route" );
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
        }

        auto modified_route_for_planning =
            apply_active_avoidance_speed_profile(
                oa_result.modified_route,
                ego_s_modified,
                oa_result.shift_start_s,
                oa_result.shift_end_s,
                oa_result.maneuver.release_s,
                planner.get_physical_vehicle_parameters(),
                params_with_sized_avoidance_speed(
                    params_for_obstacle_avoidance,
                    planner.get_physical_vehicle_parameters(),
                    oa_result.obstacle_s_min - oa_result.shift_start_s,
                    oa_result.lateral_shift ),
                vehicle_state_dynamic.vx );
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
        catch( const std::exception& )
        {
            return make_stop_on_route_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance,
                "original" );
        }

        if( trajectory.states.empty() )
        {
            return make_stop_on_route_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance,
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
            "[OA][FAIL_SAFE] OA returned mode=%d but no valid route-derived trajectory; braking on route reason=%s",
            static_cast<int>( oa_result.mode ),
            oa_result.reason.c_str() );
        return make_route_failsafe_behavior(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance );
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
    catch( const std::exception& )
    {
        return make_stop_on_route_behavior(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance,
            "original" );
    }

    if( trajectory.states.empty() )
    {
        return make_stop_on_route_behavior(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance,
            "original" );
    }

    trajectory.adjust_start_time( vehicle_state_dynamic.time );
    trajectory.label = "driving mission";

    Behavior trajectory_and_signal;
    trajectory_and_signal.trajectory =
        dynamics::conversions::to_ros_msg( trajectory );
    trajectory_and_signal.modified_route =
        map::conversions::to_ros_msg( route_with_signal );

    return trajectory_and_signal;
}

} // namespace behavior
} // namespace adore
