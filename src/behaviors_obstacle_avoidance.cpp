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
#include <iterator>
#include <limits>
#include <optional>
#include <string>

namespace adore
{
namespace behavior
{
namespace
{

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
    bool indicate_direction )
{
    // Reduced, colleague-facing label: "driving mission (avoiding obstacle[ left/right])".
    // An in-lane nudge never signals a direction (no lane change), and the direction
    // suffix is only added inside the signalling window (indicate_direction). The turn
    // indicator downstream keys on the " left)" / " right)" substring.
    if( !in_lane && indicate_direction )
    {
        if( mode == planner::ObstacleAvoidanceMode::OvertakeLeft ||
            lateral_shift > 1e-6 )
        {
            return "driving mission (avoiding obstacle left)";
        }

        if( mode == planner::ObstacleAvoidanceMode::OvertakeRight ||
            lateral_shift < -1e-6 )
        {
            return "driving mission (avoiding obstacle right)";
        }
    }

    return "driving mission (avoiding obstacle)";
}

// Single-sourced in planning/obstacle_avoidance.hpp; brought into this namespace
// so existing unqualified call sites keep working.
using planner::project_s_on_reference_line;
using planner::start_active_avoidance_state;
using planner::make_oncoming_monitor_conflict;
using planner::should_stop_for_active_conflict;
using planner::compute_monotonic_ego_s_modified;

double
ego_front_offset( const dynamics::PhysicalVehicleParameters& vehicle_params )
{
    return vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
}

dynamics::TrafficParticipantSet
only_participants_with_valid_dimensions(
    const dynamics::TrafficParticipantSet& traffic_participants )
{
    dynamics::TrafficParticipantSet valid;
    valid.validity_area = traffic_participants.validity_area;
    for( const auto& [id, participant] : traffic_participants.participants )
    {
        if( planner::participant_has_valid_dimensions( participant ) )
        {
            valid.participants.emplace( id, participant );
        }
    }
    return valid;
}

void
log_invalid_participant_dimensions()
{
    static rclcpp::Clock log_clock{ RCL_STEADY_TIME };
    RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger( "Behaviors" ),
        log_clock,
        1000,
        "[OA][INVALID_PARTICIPANT_DIMENSIONS] participant length/width must "
        "be finite and > 0; stopping on the driven route" );
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
apply_live_speed_overlays(
    const map::Route& geometry_route,
    const map::Route& live_route )
{
    map::Route overlaid = geometry_route;
    if( live_route.reference_line.empty() )
    {
        return overlaid;
    }

    for( auto& [s, point] : overlaid.reference_line )
    {
        auto live_it = live_route.reference_line.lower_bound( s );
        if( live_it == live_route.reference_line.end() )
        {
            live_it = std::prev( live_route.reference_line.end() );
        }
        else if( live_it != live_route.reference_line.begin() )
        {
            const auto previous = std::prev( live_it );
            if( std::fabs( previous->first - s ) <
                std::fabs( live_it->first - s ) )
            {
                live_it = previous;
            }
        }

        point.max_speed = live_it->second.max_speed;
    }

    return overlaid;
}

map::Route
apply_active_avoidance_speed_profile(
    const map::Route& active_route,
    double ego_s,
    const std::vector<planner::AvoidanceShiftContribution>& contributions,
    double shift_start_s,
    double shift_end_s,
    double release_s,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::ObstacleAvoidanceParams& params,
    double ego_v = std::numeric_limits<double>::infinity() )
{
    std::vector<planner::AvoidanceSpeedSegment> speed_segments;
    speed_segments.reserve( contributions.size() );
    for( const auto& contribution : contributions )
    {
        if( !std::isfinite( contribution.ramp_start_s ) ||
            !std::isfinite( contribution.ramp_end_s ) )
        {
            continue;
        }

        speed_segments.push_back(
            planner::AvoidanceSpeedSegment{
                contribution.ramp_start_s,
                contribution.ramp_end_s } );
    }

    if( !speed_segments.empty() )
    {
        return planner::RouteSpeedPolicy::
            apply_segmented_avoidance_speed_profile(
                active_route,
                ego_s,
                speed_segments,
                vehicle_params,
                params );
    }

    // Compatibility fallback for a legacy/incomplete active state without
    // per-object ramp bounds.
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

// Copy of params carrying the speed that was validated together with the
// selected persistent geometry. Re-deriving it from a group's earliest obstacle
// after a later extension would couple the new maneuver back to the old ramp.
static planner::ObstacleAvoidanceParams
params_with_selected_avoidance_speed(
    const planner::ObstacleAvoidanceParams& params,
    double selected_speed )
{
    auto sized = params;
    if( std::isfinite( selected_speed ) && selected_speed >= 0.0 )
    {
        sized.max_speed_during_avoidance = selected_speed;
    }
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
    if( active_route.reference_line.empty() )
    {
        return active_route;
    }

    double ego_s =
        project_s_on_reference_line(
            active_route,
            ego,
            requested_stop_s );
    if( !std::isfinite( ego_s ) )
    {
        ego_s = active_route.reference_line.begin()->first;
    }

    double target_stop_s = requested_stop_s;
    if( !std::isfinite( target_stop_s ) )
    {
        const double comfort_deceleration =
            planner::planned_braking_deceleration( vehicle_params, params );
        const double speed = std::max( 0.0, ego.vx );
        target_stop_s =
            ego_s + speed * speed / ( 2.0 * comfort_deceleration );
    }

    return planner::RouteSpeedPolicy::apply_brake_envelope(
            active_route,
            ego_s,
            ego.vx,
            target_stop_s,
            vehicle_params,
            params );
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
        trajectory.label = "driving mission (fallback stop)";
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
    const planner::ObstacleAvoidanceParams& params )
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
            trajectory.label = "driving mission (fallback stop)";

            Behavior behavior;
            behavior.trajectory = dynamics::conversions::to_ros_msg( trajectory );
            behavior.modified_route = map::conversions::to_ros_msg( max_braking_route );
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
    trajectory.label = "driving mission (stopping for obstacle)";

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

    const auto plan_brake_route =
        [&]( const map::Route& route_to_plan ) -> dynamics::Trajectory
        {
            dynamics::Trajectory planned;
            try
            {
                planned =
                    planner.plan_route_trajectory_from_s(
                        route_to_plan,
                        ego,
                        traffic_participants,
                        brake_from_s );
            }
            catch( const std::exception& )
            {
            }
            return planned;
        };

    dynamics::Trajectory trajectory = plan_brake_route( brake_route );

    // A concrete conflict stop is safety-relevant: a non-empty trajectory is
    // insufficient if it still crosses the requested stop point with speed.
    //
    // The route envelope above already selects comfort, intermediate or maximum
    // deceleration according to the available distance. The trajectory check
    // must therefore prove physical reachability with maximum deceleration; a
    // comfort-only proof would reject the intentionally selected intermediate
    // regime and replace a reachable stop at stop_before_obstacle with immediate
    // maximum braking. Only when the target is no longer physically reachable do
    // we publish the earliest achievable maximum-braking trajectory instead.
    const bool verify_explicit_stop = std::isfinite( conflict_stop_s );
    if( verify_explicit_stop &&
        !::adore::planner::trajectory_stops_by_route_s(
            trajectory,
            brake_route,
            target_stop_s,
            vehicle_params,
            params,
            true ) )
    {
        static rclcpp::Clock brake_fallback_log_clock{ RCL_STEADY_TIME };
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger( "Behaviors" ),
            brake_fallback_log_clock,
            1000,
            "[OA][BRAKE_VERIFY] planned active stop at s=%.2f was not proven; "
            "using maximum braking",
            target_stop_s );

        brake_route =
            make_max_braking_route_from_s(
                active_route,
                brake_from_s,
                ego.vx,
                vehicle_params,
                params );
        trajectory = plan_brake_route( brake_route );

        if( !::adore::planner::trajectory_stops_by_route_s(
                trajectory,
                brake_route,
                target_stop_s,
                vehicle_params,
                params,
                true ) )
        {
            static rclcpp::Clock brake_limit_log_clock{ RCL_STEADY_TIME };
            RCLCPP_ERROR_THROTTLE(
                rclcpp::get_logger( "Behaviors" ),
                brake_limit_log_clock,
                1000,
                "[OA][BRAKE_LIMIT] maximum braking cannot guarantee requested "
                "active stop at s=%.2f",
                target_stop_s );
        }
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
// conflict to stop for, or nullopt to keep following the route. It evaluates
// every current participant against the route-derived ego trajectory each cycle;
// participant IDs are diagnostic only and never persistent state.
std::optional<::adore::planner::RouteCorridorConflict>
check_opposite_lane_monitor(
    const map::Route& route,
    const dynamics::VehicleStateDynamic& ego,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceManeuver& maneuver,
    double conflict_hint_s,
    const planner::ObstacleAvoidanceParams& params,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const dynamics::Trajectory& planned_trajectory )
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
            &planned_trajectory );

    if( ::adore::planner::should_stop_for_oncoming_monitor_result(
            monitor_result ) )
    {
        return make_oncoming_monitor_conflict(
            monitor_result,
            conflict_hint_s );
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
    const std::vector<int>* forced_participant_ids = nullptr,
    double shift_direction_sign = 0.0 )
{
    // Rebuild from the fixed mission-route baseline. Previously accepted object
    // hulls stay frozen in that frame; participants selected by the corridor check
    // on the driven route are forced into the group even when they sit outside the
    // original mission-route trigger corridor.
    const auto* committed_contributions =
        active_avoidance_state.committed_contributions.empty()
            ? nullptr
            : &active_avoidance_state.committed_contributions;
    const auto replan_result =
        ::adore::planner::try_plan_obstacle_avoidance(
            planner,
            replan_base_route,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance,
            shift_direction_sign,
            committed_contributions,
            forced_participant_ids );

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

    // A mid-maneuver reshape never narrows an accepted object hull: committed
    // contributions are rebuilt from the mission frame and observations only
    // expand their bounds. Per-object shifts are composed by max magnitude.
    double replan_ego_s =
        get_s_on_reference_line_segments(
            replan_result.modified_route,
            vehicle_state_dynamic,
            projection_hint_s,
            params_for_obstacle_avoidance.route_window_min );
    if( !std::isfinite( replan_ego_s ) )
    {
        replan_ego_s = projection_hint_s;
    }

    // try_plan_obstacle_avoidance has already applied the physics-sized speed
    // profile, planned the trajectory and validated that exact route/trajectory
    // pair. Publish that pair verbatim; planning it again here would discard the
    // validation result and could produce a different trajectory.
    const auto replan_params =
        params_with_selected_avoidance_speed(
            params_for_obstacle_avoidance,
            replan_result.avoidance_speed );
    auto trajectory = replan_result.trajectory;
    if( trajectory.states.empty() )
    {
        return std::nullopt;
    }

    const auto mission_route_baseline =
        active_avoidance_state.mission_route_baseline;
    start_active_avoidance_state(
        active_avoidance_state,
        replan_result,
        mission_route_baseline );

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
            replan_result.modified_route );

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
    const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance,
    bool use_weather_comfort_settings,
    const std::string& weather_label,
    planner::ActiveAvoidanceState& active_avoidance_state )
{
    const bool participant_dimensions_valid =
        ::adore::planner::traffic_participants_have_valid_dimensions(
            traffic_participants );
    dynamics::TrafficParticipantSet valid_traffic_participants;
    const dynamics::TrafficParticipantSet* traffic_participants_for_stop =
        &traffic_participants;
    if( !participant_dimensions_valid )
    {
        valid_traffic_participants =
            only_participants_with_valid_dimensions( traffic_participants );
        traffic_participants_for_stop = &valid_traffic_participants;
    }

    // Geometry is persistent, speed overlays are not. Copy current mission,
    // signal and weather speed values onto the stored avoidance geometry.
    auto active_route =
        apply_live_speed_overlays(
            active_avoidance_state.base_modified_route,
            route_with_signal );

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
                *traffic_participants_for_stop,
                params_for_obstacle_avoidance,
                label );
        };

    auto make_active_conflict_stop_behavior =
        [&]( const ::adore::planner::RouteCorridorConflict& conflict,
             double ego_s_on_active_route ) -> Behavior
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

            // Waiting for oncoming traffic before pulling out -> "waiting for oncoming";
            // any other active-route brake (obstacle blocks, no room) -> "stopping for
            // obstacle". Keep the turn indicator on through the oncoming wait (a human
            // holds the blinker while waiting, not only while moving over): append the
            // " left)" / " right)" suffix the indicator downstream keys on, over the same
            // window the shift behavior uses. An in-lane maneuver or a non-oncoming brake
            // stays direction-less, so it never signals a pull-out that is not happening.
            std::string brake_label;
            if( oncoming_conflict )
            {
                brake_label = "driving mission (waiting for oncoming";
                const bool indicate =
                    !active_avoidance_state.in_lane &&
                    should_indicate_avoidance_direction(
                        ego_s_on_active_route,
                        active_avoidance_state.shift_start_s,
                        active_avoidance_state.obstacle_s_min,
                        params_for_obstacle_avoidance );
                if( indicate && active_avoidance_state.lateral_shift > 1e-6 )
                {
                    brake_label += " left)";
                }
                else if( indicate && active_avoidance_state.lateral_shift < -1e-6 )
                {
                    brake_label += " right)";
                }
                else
                {
                    brake_label += ")";
                }
            }
            else
            {
                brake_label = "driving mission (stopping for obstacle)";
            }

            return make_route_brake_in_place_behavior(
                planner,
                active_route,
                vehicle_state_dynamic,
                pinned_active_stop_s,
                conflict_stop_s,
                *traffic_participants_for_stop,
                params_for_obstacle_avoidance,
                brake_label );
        };

    if( !participant_dimensions_valid )
    {
        log_invalid_participant_dimensions();
        return make_active_stop_behavior(
            "driving mission (stopping: invalid participant dimensions)" );
    }

    // Update the ego progress on the modified route before any active-stop path
    // can return. The vehicle can travel a substantial
    // distance while braking before it reaches the wait position. If
    // last_modified_s remains frozen at the first braking cycle, the projection
    // after release looks like an implausible forward jump and the monotonic
    // filter restarts the trajectory tens of metres behind ego, on the still
    // unshifted part of the route.
    const double baseline_projection_hint_s =
        std::isfinite( active_avoidance_state.last_modified_s )
            ? active_avoidance_state.last_modified_s
            : active_avoidance_state.mission_route_baseline.get_s(
                  vehicle_state_dynamic );

    const double ego_s_modified_raw =
        get_s_on_reference_line_segments(
            active_route,
            vehicle_state_dynamic,
            baseline_projection_hint_s,
            params_for_obstacle_avoidance.route_window_min );

    const auto ego_s_modified_opt =
        compute_monotonic_ego_s_modified(
            ego_s_modified_raw,
            vehicle_state_dynamic,
            active_avoidance_state,
            params_for_obstacle_avoidance );

    if( !ego_s_modified_opt.has_value() )
    {
        return make_active_stop_behavior(
            "driving mission (fallback stop)" );
    }

    const double ego_s_modified = ego_s_modified_opt.value();
    pinned_active_stop_s = ego_s_modified;

    // Stage 1: a single id-independent corridor pass on the DRIVEN (modified)
    // route, with NO ignore list. It reports hard corridor conflicts separately
    // from static objects that remain safe but fall short of side_clearance.
    // Membership is decided by position each cycle, so re-id / fragmentation is
    // handled by location, not identity.
    const auto driven_safety =
        ::adore::planner::check_route_corridor_safety(
            active_route,
            vehicle_state_dynamic,
            traffic_participants,
            planner.get_physical_vehicle_parameters(),
            params_for_obstacle_avoidance );

    // The corridor check above has already selected objects overlapping the
    // currently driven ego corridor. The static-object rule is deliberately simple:
    // every static conflict triggers a validated replan, independent of participant
    // id and of its longitudinal position relative to ego. This includes the object
    // already being avoided when perception refines its dimensions.
    //
    // Moving objects in the corridor still stop immediately when braking is warranted:
    // OA never swerves around moving/crossing traffic. Opposite-lane oncoming timing is
    // handled by the dedicated monitor after the current trajectory is available.
    std::optional<::adore::planner::RouteCorridorConflict> stop_conflict;
    std::optional<::adore::planner::RouteCorridorConflict> hard_static_replan_conflict;
    std::optional<::adore::planner::RouteCorridorConflict> best_effort_replan_conflict;
    std::vector<int> static_replan_conflict_ids;

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

    const auto append_static_replan_id =
        [&]( int participant_id )
        {
            if( participant_id >= 0 &&
                std::find(
                    static_replan_conflict_ids.begin(),
                    static_replan_conflict_ids.end(),
                    participant_id ) ==
                    static_replan_conflict_ids.end() )
            {
                static_replan_conflict_ids.push_back( participant_id );
            }
        };

    for( const auto& conflict : driven_safety.conflicts )
    {
        if( !std::isfinite( conflict.object_s_min ) ||
            !std::isfinite( conflict.object_s_max ) )
        {
            continue;
        }

        const bool is_static =
            conflict.object_class ==
            ::adore::planner::RouteCorridorObjectClass::StaticOrSlow;

        if( is_static )
        {
            if( prefer_more_urgent( hard_static_replan_conflict, conflict ) )
            {
                hard_static_replan_conflict = conflict;
            }

            append_static_replan_id( conflict.participant_id );
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

        // The dedicated opposite-lane monitor later in this cycle is the sole authority for
        // oncoming traffic during an opposite-lane maneuver. Reprocessing the
        // same participant here could command a stop inside the opposite lane
        // after the monitor deliberately chose to continue clearing it.
        if( active_avoidance_state.maneuver.uses_opposite_lane &&
            conflict.object_class ==
                ::adore::planner::RouteCorridorObjectClass::Oncoming )
        {
            continue;
        }

        // Moving object in the driven corridor: OA cannot shift around it. Stop if
        // braking is warranted (a crossing / entering vehicle or pedestrian during
        // an in-lane or adjacent shift; opposite-lane oncoming is handled separately).
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

    for( const auto& candidate :
         driven_safety.static_clearance_improvements )
    {
        if( candidate.object_class !=
                ::adore::planner::RouteCorridorObjectClass::StaticOrSlow ||
            !std::isfinite( candidate.object_s_min ) ||
            !std::isfinite( candidate.object_s_max ) )
        {
            continue;
        }

        if( prefer_more_urgent( best_effort_replan_conflict, candidate ) )
        {
            best_effort_replan_conflict = candidate;
        }
        append_static_replan_id( candidate.participant_id );
    }

    // Moving/crossing traffic is not an avoidance-replan target. It keeps priority
    // over a simultaneous static-object extension and is braked for on the driven
    // route (no snap).
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
        return make_active_conflict_stop_behavior( stop_conflict.value(), ego_s_modified );
    }

    // Any uncleared static hard intrusion requires a reshape. A static object
    // between ego_corridor_safety_margin and side_clearance requests the same
    // reshape only as a best-effort improvement: if no wider continuous route
    // validates, the current route remains safe and must stay usable.
    //
    // Replan the whole maneuver from the mission route in one pass regardless of
    // whether the object's near edge is ahead of or already beside ego. Frozen
    // object hulls are carried in the mission frame and composed with every
    // selected static object. For a same-id length refinement, the monotone hull
    // merge extends the full-shift plateau and release point; for a width
    // refinement, candidate validation decides whether a larger lateral shift can
    // still be reached safely from the live ego pose.
    //
    // Lock the replan to the current shift direction (shift_direction_sign) so it only
    // ever widens the side ego is already going, never the opposite (which would pull
    // ego back toward the object it is passing).
    const bool hard_static_replan_required =
        hard_static_replan_conflict.has_value();
    const auto static_replan_conflict =
        hard_static_replan_required
            ? hard_static_replan_conflict
            : best_effort_replan_conflict;

    if( static_replan_conflict.has_value() )
    {
        if( hard_static_replan_required )
        {
            static rclcpp::Clock driven_widen_log_clock{ RCL_STEADY_TIME };
            RCLCPP_INFO_THROTTLE(
                rclcpp::get_logger( "Behaviors" ), driven_widen_log_clock, 1000,
                "[OA][DRIVEN] static intrusion id=%d s=[%.1f,%.1f] "
                "-> required replan (mission baseline)",
                static_replan_conflict->participant_id,
                static_replan_conflict->object_s_min,
                static_replan_conflict->object_s_max );
        }
        else
        {
            static rclcpp::Clock clearance_replan_log_clock{ RCL_STEADY_TIME };
            RCLCPP_INFO_THROTTLE(
                rclcpp::get_logger( "Behaviors" ),
                clearance_replan_log_clock,
                1000,
                "[OA][DRIVEN] static clearance id=%d actual=%.2f target=%.2f "
                "s=[%.1f,%.1f] -> best-effort replan",
                static_replan_conflict->participant_id,
                static_replan_conflict->actual_lateral_clearance,
                params_for_obstacle_avoidance.side_clearance,
                static_replan_conflict->object_s_min,
                static_replan_conflict->object_s_max );
        }

        const double shift_direction_sign =
            active_avoidance_state.lateral_shift >= 0.0 ? 1.0 : -1.0;

        auto replan_baseline =
            apply_live_speed_overlays(
                active_avoidance_state.mission_route_baseline,
                route_with_signal );
        double ego_s_replan_baseline =
            get_s_on_reference_line_segments(
                replan_baseline,
                vehicle_state_dynamic,
                ego_s_modified,
                params_for_obstacle_avoidance.route_window_min );
        if( !std::isfinite( ego_s_replan_baseline ) )
        {
            ego_s_replan_baseline = ego_s_modified;
        }

        if( auto replanned_behavior =
                try_dynamic_replan_from_route(
                    planner,
                    replan_baseline,
                    ego_s_replan_baseline,
                    vehicle_state_dynamic,
                    traffic_participants,
                    params_for_obstacle_avoidance,
                    active_avoidance_state,
                    &static_replan_conflict_ids,
                    shift_direction_sign );
            replanned_behavior.has_value() )
        {
            return replanned_behavior.value();
        }

        if( hard_static_replan_required )
        {
            // No continuous, drivable extension validated for a hard intrusion:
            // brake on the driven route rather than snapping toward the mission
            // line.
            return make_active_conflict_stop_behavior(
                static_replan_conflict.value(), ego_s_modified );
        }

        // The desired side-clearance target could not be restored, but the
        // object remains outside the hard corridor. Keep following the already
        // safe active route; the same best-effort attempt is repeated on later
        // cycles so newly available space can still be used.
        static rclcpp::Clock clearance_keep_log_clock{ RCL_STEADY_TIME };
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger( "Behaviors" ),
            clearance_keep_log_clock,
            1000,
            "[OA][DRIVEN] side-clearance replan unavailable "
            "(actual=%.2f target=%.2f hard=%.2f) -> keep safe active route",
            static_replan_conflict->actual_lateral_clearance,
            params_for_obstacle_avoidance.side_clearance,
            params_for_obstacle_avoidance.ego_corridor_safety_margin );
    }

    // Persistent obstacle hulls keep the accepted route stable when a static
    // detection disappears. The maneuver returns to the mission route only at
    // release_s below, or stops when avoidance is no longer possible.
    if( std::isfinite( ego_s_modified ) &&
        ego_s_modified >= active_avoidance_state.release_s )
    {
        // Resume on the latest mission route only after ego has reached
        // release_s in the active modified-route frame.
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
                "driving mission (fallback stop)" );
        }

        if( trajectory.states.empty() )
        {
            return make_active_stop_behavior(
                "driving mission (fallback stop)" );
        }

        // Commit the lifecycle transition only after a usable mission-route
        // trajectory exists. Otherwise the next cycle must retain the active
        // geometry on which ego is safely braking.
        active_avoidance_state.reset();

        trajectory.adjust_start_time( vehicle_state_dynamic.time );
        trajectory.label = "driving mission";
        if( use_weather_comfort_settings )
        {
            trajectory.label += " (" + weather_label + ")";
        }

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
                active_avoidance_state.committed_contributions,
                active_avoidance_state.shift_start_s,
                active_avoidance_state.shift_end_s,
                active_avoidance_state.release_s,
                planner.get_physical_vehicle_parameters(),
                params_with_selected_avoidance_speed(
                    params_for_obstacle_avoidance,
                    active_avoidance_state.avoidance_speed ),
                vehicle_state_dynamic.vx );

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
                "driving mission (fallback stop)" );
        }

        trajectory.label =
            obstacle_avoidance_label(
                active_avoidance_state.maneuver.mode,
                active_avoidance_state.in_lane,
                active_avoidance_state.lateral_shift,
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
                "driving mission (fallback stop)" );
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
                    trajectory );
            monitor_conflict.has_value() )
        {
            return make_active_conflict_stop_behavior(
                monitor_conflict.value(), ego_s_modified );
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
    if( !::adore::planner::traffic_participants_have_valid_dimensions(
          traffic_participants ) )
    {
        log_invalid_participant_dimensions();
        const auto valid_traffic_participants =
            only_participants_with_valid_dimensions( traffic_participants );
        return make_stop_on_route_behavior(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            valid_traffic_participants,
            params_for_obstacle_avoidance );
    }

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
                params_for_obstacle_avoidance );
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
    if( !::adore::planner::traffic_participants_have_valid_dimensions(
          traffic_participants ) )
    {
        active_avoidance_state.reset();
        log_invalid_participant_dimensions();
        const auto valid_traffic_participants =
            only_participants_with_valid_dimensions( traffic_participants );
        return make_stop_on_route_behavior(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            valid_traffic_participants,
            params_for_obstacle_avoidance );
    }

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
            start_active_avoidance_state(
                active_avoidance_state,
                oa_result,
                route_with_signal );
        }
        else
        {
            // The OA core has already planned and validated this exact
            // route/trajectory pair, whether it is still approaching the
            // obstacle or braking for it. Publishing it unchanged avoids a
            // second stop policy with different reachability thresholds.
            auto trajectory = oa_result.trajectory;
            if( trajectory.states.empty() )
            {
                return make_stop_on_route_behavior(
                    planner,
                    route_with_signal,
                    vehicle_state_dynamic,
                    traffic_participants,
                    params_for_obstacle_avoidance );
            }

            trajectory.adjust_start_time( vehicle_state_dynamic.time );

            Behavior behavior;
            behavior.trajectory =
                dynamics::conversions::to_ros_msg( trajectory );
            behavior.modified_route =
                map::conversions::to_ros_msg( oa_result.modified_route );
            return behavior;
        }

        const double ego_s_original =
            route_with_signal.get_s( vehicle_state_dynamic );
        double ego_s_modified =
            get_s_on_reference_line_segments(
                oa_result.modified_route,
                vehicle_state_dynamic,
                ego_s_original,
                params_for_obstacle_avoidance.route_window_min );

        if( !std::isfinite( ego_s_modified ) )
        {
            ego_s_modified = ego_s_original;
        }

        // The OA core returns the exact trajectory it validated against this
        // modified route. Do not plan a second, unvalidated trajectory here.
        auto trajectory = oa_result.trajectory;
        if( trajectory.states.empty() )
        {
            return make_stop_on_route_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance );
        }

        trajectory.adjust_start_time( vehicle_state_dynamic.time );
        trajectory.label =
            obstacle_avoidance_label(
                oa_result.mode,
                oa_result.in_lane,
                oa_result.lateral_shift,
                should_indicate_avoidance_direction(
                    ego_s_modified,
                    oa_result.shift_start_s,
                    oa_result.obstacle_s_min,
                    params_for_obstacle_avoidance ) );

        Behavior trajectory_and_signal;
        trajectory_and_signal.trajectory =
            dynamics::conversions::to_ros_msg( trajectory );

        trajectory_and_signal.modified_route =
            map::conversions::to_ros_msg( oa_result.modified_route );

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
            params_for_obstacle_avoidance );
    }

    if( trajectory.states.empty() )
    {
        return make_stop_on_route_behavior(
            planner,
            route_with_signal,
            vehicle_state_dynamic,
            traffic_participants,
            params_for_obstacle_avoidance );
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
