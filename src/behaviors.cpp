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

#include "behaviors.hpp"
#include "adore_map_conversions.hpp"
#include "planning/obstacle_avoidance.hpp"
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

// Tolerance added on top of the odometry-based advance when judging whether a
// new projection onto the active modified route is plausible. Covers
// projection refinement noise between cycles.
constexpr double MAX_PLAUSIBLE_MODIFIED_S_JUMP = 2.0;

const char*
object_class_name( planner::RouteCorridorObjectClass object_class )
{
    switch( object_class )
    {
        case planner::RouteCorridorObjectClass::StaticOrSlow:
            return "static_or_slow";
        case planner::RouteCorridorObjectClass::Oncoming:
            return "oncoming";
        case planner::RouteCorridorObjectClass::SameDirection:
            return "same_direction";
        case planner::RouteCorridorObjectClass::CrossingOrUnknown:
        default:
            return "crossing_or_unknown";
    }
}

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

std::string
obstacle_avoidance_label(
    planner::ObstacleAvoidanceMode mode,
    bool in_lane,
    double lateral_shift,
    bool active )
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

    return active
        ? "driving mission (active obstacle avoidance)"
        : "driving mission (obstacle avoidance)";
}

// extra_l_margin widens the lateral match band. Ghost envelopes can stem from
// projections onto different reference lines (original vs. laterally shifted
// modified route); their l values then differ by up to the lateral shift, so
// matching must tolerate that offset.
bool
envelopes_overlap_with_margin(
    const planner::ObstacleGhostEnvelope& a,
    const planner::RouteCorridorConflict& b,
    const planner::ObstacleAvoidanceParams& params,
    double extra_l_margin = 0.0 )
{
    const double l_margin =
        params.ghost_obstacle_match_l_margin + std::max( 0.0, extra_l_margin );
    return a.object_s_max + params.ghost_obstacle_match_s_margin >= b.object_s_min &&
           a.object_s_min - params.ghost_obstacle_match_s_margin <= b.object_s_max &&
           a.object_l_max + l_margin >= b.object_l_min &&
           a.object_l_min - l_margin <= b.object_l_max;
}

bool
ghost_envelopes_overlap_with_margin(
    const planner::ObstacleGhostEnvelope& a,
    const planner::ObstacleGhostEnvelope& b,
    const planner::ObstacleAvoidanceParams& params,
    double extra_l_margin = 0.0 )
{
    const double l_margin =
        params.ghost_obstacle_match_l_margin + std::max( 0.0, extra_l_margin );
    return a.object_s_max + params.ghost_obstacle_match_s_margin >= b.object_s_min &&
           a.object_s_min - params.ghost_obstacle_match_s_margin <= b.object_s_max &&
           a.object_l_max + l_margin >= b.object_l_min &&
           a.object_l_min - l_margin <= b.object_l_max;
}

planner::ObstacleGhostEnvelope
make_ghost_envelope(
    const planner::RouteCorridorConflict& conflict,
    double now,
    const planner::ObstacleAvoidanceParams& params,
    bool created_from_active_route_conflict )
{
    planner::ObstacleGhostEnvelope envelope;
    envelope.object_s_min = conflict.object_s_min;
    envelope.object_s_max = conflict.object_s_max;
    envelope.object_l_min = conflict.object_l_min;
    envelope.object_l_max = conflict.object_l_max;
    envelope.inflated_s_min = conflict.inflated_s_min;
    envelope.inflated_s_max = conflict.inflated_s_max;
    envelope.inflated_l_min = conflict.inflated_l_min;
    envelope.inflated_l_max = conflict.inflated_l_max;
    envelope.object_class = conflict.object_class;
    envelope.first_seen_time = now;
    envelope.last_seen_time = now;
    envelope.seen_count = 1;
    envelope.hold_until_s =
        conflict.object_s_max +
        std::max( 0.0, params.rear_clearance ) +
        std::max( 0.0, params.ghost_obstacle_release_extra_s );
    envelope.hold_until_passed =
        conflict.object_class == planner::RouteCorridorObjectClass::StaticOrSlow;
    envelope.created_from_active_route_conflict = created_from_active_route_conflict;
    envelope.is_ghost = false;
    envelope.last_participant_id = conflict.participant_id;
    envelope.object_center_x = conflict.object_center_x;
    envelope.object_center_y = conflict.object_center_y;
    envelope.object_yaw = conflict.object_yaw;
    envelope.object_length = conflict.object_length;
    envelope.object_width = conflict.object_width;
    envelope.footprint_x = conflict.footprint_x;
    envelope.footprint_y = conflict.footprint_y;
    envelope.has_world_footprint = conflict.has_world_footprint;
    return envelope;
}

std::optional<planner::ObstacleGhostEnvelope>
make_original_obstacle_envelope_from_participant(
    const map::Route& route,
    const dynamics::TrafficParticipant& participant,
    double now,
    const planner::ObstacleAvoidanceParams& params )
{
    planner::RouteCorridorConflict conflict;
    conflict.participant_id = participant.id;
    conflict.object_class = planner::RouteCorridorObjectClass::StaticOrSlow;
    conflict.object_center_x = participant.state.x;
    conflict.object_center_y = participant.state.y;
    conflict.object_yaw = participant.state.yaw_angle;
    conflict.object_length =
        std::max( params.min_vehicle_dimension, participant.physical_parameters.body_length );
    conflict.object_width =
        std::max( params.min_vehicle_dimension, participant.physical_parameters.body_width );
    conflict.has_world_footprint = true;

    const double half_length = 0.5 * conflict.object_length;
    const double half_width = 0.5 * conflict.object_width;
    const double cos_yaw = std::cos( conflict.object_yaw );
    const double sin_yaw = std::sin( conflict.object_yaw );
    const std::array<std::pair<double, double>, 4> local_corners = {{
        { -half_length, -half_width },
        { -half_length,  half_width },
        {  half_length,  half_width },
        {  half_length, -half_width }
    }};

    conflict.object_s_min = std::numeric_limits<double>::infinity();
    conflict.object_s_max = -std::numeric_limits<double>::infinity();
    conflict.object_l_min = std::numeric_limits<double>::infinity();
    conflict.object_l_max = -std::numeric_limits<double>::infinity();

    for( std::size_t i = 0; i < local_corners.size(); ++i )
    {
        const auto& [local_x, local_y] = local_corners[i];
        const double x =
            conflict.object_center_x + local_x * cos_yaw - local_y * sin_yaw;
        const double y =
            conflict.object_center_y + local_x * sin_yaw + local_y * cos_yaw;
        conflict.footprint_x[i] = x;
        conflict.footprint_y[i] = y;

        dynamics::VehicleStateDynamic corner_state;
        corner_state.x = x;
        corner_state.y = y;
        const double corner_s = route.get_s( corner_state );
        if( !std::isfinite( corner_s ) )
        {
            continue;
        }

        const auto route_pose = route.get_pose_at_s( corner_s );
        const double dx = x - route_pose.x;
        const double dy = y - route_pose.y;
        const double corner_l =
            -dx * std::sin( route_pose.yaw ) + dy * std::cos( route_pose.yaw );

        conflict.object_s_min = std::min( conflict.object_s_min, corner_s );
        conflict.object_s_max = std::max( conflict.object_s_max, corner_s );
        conflict.object_l_min = std::min( conflict.object_l_min, corner_l );
        conflict.object_l_max = std::max( conflict.object_l_max, corner_l );
    }

    if( !std::isfinite( conflict.object_s_min ) ||
        !std::isfinite( conflict.object_s_max ) )
    {
        return std::nullopt;
    }

    conflict.inflated_s_min = conflict.object_s_min - std::max( 0.0, params.rear_clearance );
    conflict.inflated_s_max = conflict.object_s_max + std::max( 0.0, params.front_clearance );
    conflict.inflated_l_min = conflict.object_l_min - std::max( 0.0, params.side_clearance );
    conflict.inflated_l_max = conflict.object_l_max + std::max( 0.0, params.side_clearance );

    auto envelope = make_ghost_envelope( conflict, now, params, false );
    envelope.created_from_original_avoidance_obstacle = true;
    envelope.hold_until_passed = true;
    return envelope;
}

template<typename State>
double
project_s_on_reference_line(
    const map::Route& route,
    const State& state,
    double hint_s = std::numeric_limits<double>::quiet_NaN(),
    double search_window = std::numeric_limits<double>::infinity() )
{
    if( route.reference_line.size() < 2 )
    {
        return std::numeric_limits<double>::infinity();
    }

    const double first_s = route.reference_line.begin()->first;
    const double last_s = route.reference_line.rbegin()->first;
    const double route_window =
        std::max( planner::ObstacleAvoidanceParams{}.route_window_min, last_s - first_s );
    const double coarse_s =
        std::isfinite( hint_s )
            ? hint_s
            : 0.5 * ( first_s + last_s );
    const double window =
        std::isfinite( search_window )
            ? search_window
            : route_window;

    static bool logged_segment_projection = false;
    if( !logged_segment_projection )
    {
        RCLCPP_INFO(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][S_PROJECTION] using reference-line segment projection for modified route" );
        logged_segment_projection = true;
    }

    return get_s_on_reference_line_segments(
        route,
        state,
        coarse_s,
        window );
}

void
update_obstacle_ghost_memory(
    ActiveAvoidanceState& state,
    const planner::RouteCorridorCheckResult& safety,
    double ego_s,
    double now,
    const planner::ObstacleAvoidanceParams& params )
{
    for( auto& envelope : state.ghost_memory )
    {
        envelope.is_ghost = true;
        ++envelope.consecutive_missing_cycles;
        RCLCPP_DEBUG(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][ghost] marked missing id=%d missing_cycles=%d",
            envelope.last_participant_id,
            envelope.consecutive_missing_cycles );
    }

    // Process every conflict seen this cycle, not only the most relevant one,
    // so simultaneously visible obstacles do not age out as "missing".
    const auto detected_conflicts =
        !safety.conflicts.empty()
            ? safety.conflicts
            : ( safety.has_conflict
                    ? std::vector<planner::RouteCorridorConflict>{ safety.conflict }
                    : std::vector<planner::RouteCorridorConflict>{} );

    // Envelopes may have been created on a differently shifted reference line;
    // widen the lateral match band by the active shift.
    const double match_extra_l_margin = std::fabs( state.lateral_shift );

    for( const auto& detected_conflict : detected_conflicts )
    {
        auto match_it =
            std::find_if(
                state.ghost_memory.begin(),
                state.ghost_memory.end(),
                [&]( const planner::ObstacleGhostEnvelope& envelope )
                {
                    return envelopes_overlap_with_margin(
                        envelope,
                        detected_conflict,
                        params,
                        match_extra_l_margin );
                } );

        const auto updated =
            make_ghost_envelope( detected_conflict, now, params, true );

        if( match_it != state.ghost_memory.end() )
        {
            const int old_id = match_it->last_participant_id;
            const double first_seen_time = match_it->first_seen_time;
            const int seen_count = match_it->seen_count + 1;
            const bool original_obstacle =
                match_it->created_from_original_avoidance_obstacle;
            *match_it = updated;
            match_it->first_seen_time = first_seen_time;
            match_it->seen_count = seen_count;
            match_it->consecutive_missing_cycles = 0;
            match_it->created_from_original_avoidance_obstacle = original_obstacle;
            match_it->hold_until_passed =
                original_obstacle ||
                ( match_it->object_class == planner::RouteCorridorObjectClass::StaticOrSlow &&
                  match_it->seen_count >= 2 );
            RCLCPP_INFO(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][ghost] update matched old envelope with new participant id=%d old_id=%d hold_until_s=%.2f seen_count=%d",
                detected_conflict.participant_id,
                old_id,
                updated.hold_until_s,
                match_it->seen_count );
        }
        else
        {
            state.ghost_memory.push_back( updated );
            RCLCPP_INFO(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][ghost] create participant id=%d class=%s s=[%.2f,%.2f] l=[%.2f,%.2f] hold_until_s=%.2f",
                detected_conflict.participant_id,
                object_class_name( updated.object_class ),
                updated.object_s_min,
                updated.object_s_max,
                updated.object_l_min,
                updated.object_l_max,
                updated.hold_until_s );
        }
    }

    state.ghost_memory.erase(
        std::remove_if(
            state.ghost_memory.begin(),
            state.ghost_memory.end(),
            [&]( const planner::ObstacleGhostEnvelope& envelope )
            {
                const bool passed_release =
                    envelope.hold_until_passed &&
                    std::isfinite( ego_s ) &&
                    ego_s > envelope.hold_until_s;
                const bool timeout_release =
                    envelope.is_ghost &&
                    !envelope.created_from_original_avoidance_obstacle &&
                    now >= envelope.last_seen_time + std::max( 0.0, params.ghost_obstacle_hold_time ) &&
                    ( envelope.object_class != planner::RouteCorridorObjectClass::StaticOrSlow ||
                      envelope.seen_count < 2 ||
                      envelope.consecutive_missing_cycles >=
                          std::max( 1, params.ghost_dynamic_max_missing_cycles ) );
                const bool lifetime_release =
                    envelope.is_ghost &&
                    !envelope.created_from_original_avoidance_obstacle &&
                    std::isfinite( envelope.first_seen_time ) &&
                    now >= envelope.first_seen_time + std::max( params.ghost_obstacle_hold_time, params.ghost_obstacle_max_lifetime );

                if( passed_release || timeout_release || lifetime_release )
                {
                    RCLCPP_INFO(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][ghost] release %s ghost id=%d reason=%s ego_s=%.2f hold_until_s=%.2f missing_cycles=%d",
                        object_class_name( envelope.object_class ),
                        envelope.last_participant_id,
                        passed_release
                            ? "ego_passed_hold_until_s"
                            : ( lifetime_release
                                  ? "max_lifetime"
                                  : "timeout_after_missing_detection" ),
                        ego_s,
                        envelope.hold_until_s,
                        envelope.consecutive_missing_cycles );
                }
                return passed_release || timeout_release || lifetime_release;
            } ),
        state.ghost_memory.end() );
}

void
start_active_avoidance_state(
    ActiveAvoidanceState& state,
    const planner::ObstacleAvoidanceResult& oa_result,
    const map::Route& original_route,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const dynamics::VehicleStateDynamic& ego,
    const planner::ObstacleAvoidanceParams& params,
    bool preserve_existing_ghost_memory = false )
{
    const auto previous_ghost_memory =
        preserve_existing_ghost_memory
            ? state.ghost_memory
            : std::vector<planner::ObstacleGhostEnvelope>{};

    state.active = true;
    state.base_modified_route = oa_result.modified_route;
    state.modified_route = oa_result.modified_route;

    state.obstacle_id = oa_result.obstacle_id;
    state.obstacle_ids = oa_result.obstacle_ids;

    state.shift_start_s = oa_result.shift_start_s;
    state.shift_end_s = oa_result.shift_end_s;
    state.release_s = oa_result.shift_end_s;

    state.lateral_shift = oa_result.lateral_shift;
    state.in_lane = oa_result.in_lane;
    state.maneuver = oa_result.maneuver;

    state.last_modified_s = std::numeric_limits<double>::quiet_NaN();
    state.last_modified_time = std::numeric_limits<double>::quiet_NaN();
    state.ghost_memory.clear();

    const auto memory_seed_obstacle_ids =
        oa_result.obstacle_ids.empty()
            ? std::vector<int>{ oa_result.obstacle_id }
            : oa_result.obstacle_ids;

    for( const int obstacle_id : memory_seed_obstacle_ids )
    {
        const auto original_participant_it =
            traffic_participants.participants.find( obstacle_id );
        if( original_participant_it == traffic_participants.participants.end() )
        {
            continue;
        }

        const auto original_envelope =
            make_original_obstacle_envelope_from_participant(
                original_route,
                original_participant_it->second,
                ego.time,
                params );
        if( original_envelope.has_value() )
        {
            state.ghost_memory.push_back( original_envelope.value() );
            RCLCPP_INFO(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][ghost] create original obstacle id=%d s=[%.2f,%.2f] hold_until_s=%.2f",
                obstacle_id,
                original_envelope->object_s_min,
                original_envelope->object_s_max,
                original_envelope->hold_until_s );
        }
    }

    for( const auto& previous_envelope : previous_ghost_memory )
    {
        const auto duplicate_it =
            std::find_if(
                state.ghost_memory.begin(),
                state.ghost_memory.end(),
                [&]( const planner::ObstacleGhostEnvelope& current_envelope )
                {
                    return ghost_envelopes_overlap_with_margin(
                        current_envelope,
                        previous_envelope,
                        params,
                        std::fabs( oa_result.lateral_shift ) );
                } );

        if( duplicate_it == state.ghost_memory.end() )
        {
            state.ghost_memory.push_back( previous_envelope );
        }
    }
}

std::optional<planner::RouteCorridorConflict>
most_relevant_ghost_conflict(
    const ActiveAvoidanceState& state,
    double ego_s )
{
    std::optional<planner::RouteCorridorConflict> best;

    for( const auto& envelope : state.ghost_memory )
    {
        if( !envelope.is_ghost )
        {
            continue;
        }
        if( envelope.created_from_original_avoidance_obstacle )
        {
            continue;
        }
        if( std::isfinite( ego_s ) && ego_s > envelope.hold_until_s )
        {
            continue;
        }

        planner::RouteCorridorConflict conflict;
        conflict.participant_id = envelope.last_participant_id;
        conflict.object_class = envelope.object_class;
        conflict.object_s_min = envelope.object_s_min;
        conflict.object_s_max = envelope.object_s_max;
        conflict.object_l_min = envelope.object_l_min;
        conflict.object_l_max = envelope.object_l_max;
        conflict.inflated_s_min = envelope.inflated_s_min;
        conflict.inflated_s_max = envelope.inflated_s_max;
        conflict.inflated_l_min = envelope.inflated_l_min;
        conflict.inflated_l_max = envelope.inflated_l_max;
        conflict.distance_s = std::isfinite( ego_s )
            ? std::max( 0.0, envelope.object_s_min - ego_s )
            : std::numeric_limits<double>::infinity();
        conflict.time_to_conflict = 0.0;
        conflict.currently_overlaps_route_corridor = true;
        conflict.requires_stop = true;
        conflict.allows_replan = true;
        conflict.ttc_source = "ghost";
        conflict.currently_inside_corridor = true;
        conflict.object_center_x = envelope.object_center_x;
        conflict.object_center_y = envelope.object_center_y;
        conflict.object_yaw = envelope.object_yaw;
        conflict.object_length = envelope.object_length;
        conflict.object_width = envelope.object_width;
        conflict.footprint_x = envelope.footprint_x;
        conflict.footprint_y = envelope.footprint_y;
        conflict.has_world_footprint = envelope.has_world_footprint;
        conflict.reason = "ghost envelope retained after temporary detection loss";

        if( !best.has_value() || conflict.distance_s < best->distance_s )
        {
            best = conflict;
        }
    }

    return best;
}

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

std::optional<planner::ObstacleGhostEnvelope>
find_unpassed_original_ghost(
    const ActiveAvoidanceState& state,
    double ego_s )
{
    for( const auto& envelope : state.ghost_memory )
    {
        if( envelope.created_from_original_avoidance_obstacle &&
            envelope.hold_until_passed &&
            std::isfinite( ego_s ) &&
            ego_s <= envelope.hold_until_s )
        {
            return envelope;
        }
    }

    return std::nullopt;
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

bool
is_oncoming_other_lane_conflict_for_in_lane_shift(
    const planner::RouteCorridorConflict& conflict,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::ObstacleAvoidanceParams& params )
{
    if( conflict.object_class != planner::RouteCorridorObjectClass::Oncoming ||
        conflict.currently_overlaps_ego_footprint )
    {
        return false;
    }

    const double ego_half_width =
        0.5 * std::max( params.min_vehicle_dimension, vehicle_params.body_width );
    const double left_clearance = -ego_half_width - conflict.object_l_max;
    const double right_clearance = conflict.object_l_min - ego_half_width;
    const double actual_clearance =
        std::max( left_clearance, right_clearance );

    return actual_clearance >= std::max( 0.0, params.side_clearance );
}

bool
static_or_slow_conflict_has_side_clearance(
    const planner::RouteCorridorConflict& conflict,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::ObstacleAvoidanceParams& params )
{
    if( conflict.object_class != planner::RouteCorridorObjectClass::StaticOrSlow ||
        conflict.currently_overlaps_ego_footprint ||
        !std::isfinite( conflict.object_l_min ) ||
        !std::isfinite( conflict.object_l_max ) )
    {
        return false;
    }

    const double ego_half_width =
        0.5 * std::max( params.min_vehicle_dimension, vehicle_params.body_width );
    const double left_clearance = -ego_half_width - conflict.object_l_max;
    const double right_clearance = conflict.object_l_min - ego_half_width;
    const double actual_clearance =
        std::max( left_clearance, right_clearance );
    const double required_clearance = std::max( 0.0, params.side_clearance );

    return actual_clearance + 0.02 >= required_clearance;
}

struct RouteStopPlan
{
    bool valid = false;
    double ego_s = std::numeric_limits<double>::quiet_NaN();
    double stop_s = std::numeric_limits<double>::quiet_NaN();
    double brake_start_s = std::numeric_limits<double>::quiet_NaN();
    double required_braking_distance = 0.0;
    double braking_deceleration = 0.0;
    std::string reason;
};

RouteStopPlan
compute_route_stop_plan(
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::RouteCorridorConflict& conflict,
    const planner::ObstacleAvoidanceParams& params )
{
    RouteStopPlan plan;
    const double conflict_hint =
        std::isfinite( conflict.object_s_min )
            ? conflict.object_s_min
            : std::numeric_limits<double>::quiet_NaN();
    plan.ego_s =
        project_s_on_reference_line(
            active_route,
            ego,
            conflict_hint );

    if( !std::isfinite( plan.ego_s ) )
    {
        plan.reason = "invalid ego projection";
        return plan;
    }

    const double conflict_s =
        std::isfinite( conflict.object_s_min )
            ? conflict.object_s_min
            : plan.ego_s + params.stop_before_obstacle;
    const double ego_front_offset =
        vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;

    plan.stop_s =
        conflict_s -
        std::max( 0.0, params.stop_before_obstacle ) -
        ego_front_offset;

    plan.braking_deceleration =
        std::max( params.min_braking_deceleration, std::fabs( vehicle_params.acceleration_min ) );
    plan.required_braking_distance =
        std::max( 0.0, ego.vx ) * std::max( 0.0, ego.vx ) /
        ( 2.0 * plan.braking_deceleration );
    plan.brake_start_s =
        plan.stop_s -
        plan.required_braking_distance -
        std::max( 0.0, params.modified_route_braking_safety_margin );
    plan.valid =
        std::isfinite( plan.stop_s ) &&
        std::isfinite( plan.brake_start_s );
    plan.reason = plan.valid ? "valid" : "invalid stop geometry";

    return plan;
}

bool
should_stop_for_active_conflict(
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::RouteCorridorConflict& conflict,
    const planner::ObstacleAvoidanceParams& params )
{
    if( conflict.currently_overlaps_ego_footprint )
    {
        return true;
    }

    const auto stop_plan =
        compute_route_stop_plan(
            active_route,
            ego,
            vehicle_params,
            conflict,
            params );

    if( !stop_plan.valid )
    {
        return true;
    }

    if( conflict.predicted_spatiotemporal_conflict &&
        conflict.time_to_conflict <= params.modified_route_stop_ttc_threshold )
    {
        if( conflict.object_class != planner::RouteCorridorObjectClass::StaticOrSlow )
        {
            return true;
        }

        return stop_plan.ego_s >= stop_plan.brake_start_s;
    }

    if( conflict.currently_overlaps_route_corridor &&
        stop_plan.ego_s >= stop_plan.brake_start_s )
    {
        return true;
    }

    return false;
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

class PreviousTrajectoryFallbackGuard
{
public:
    explicit PreviousTrajectoryFallbackGuard(
        planner::TrajectoryPlanner& planner,
        const std::string& context = "OA stop" )
        : planner_( planner ),
          previous_( planner.get_allow_previous_trajectory_fallback() )
    {
        planner_.set_allow_previous_trajectory_fallback( false );
        RCLCPP_INFO(
            rclcpp::get_logger( "Behaviors" ),
            "[OA][ROUTE_GUARD] previous trajectory fallback disabled for %s",
            context.c_str() );
    }

    ~PreviousTrajectoryFallbackGuard()
    {
        planner_.set_allow_previous_trajectory_fallback( previous_ );
    }

private:
    planner::TrajectoryPlanner& planner_;
    bool previous_;
};

void
set_route_points_from_s_to_zero( map::Route& route, double ego_s )
{
    if( route.reference_line.empty() )
    {
        return;
    }

    if( !std::isfinite( ego_s ) )
    {
        ego_s = route.reference_line.begin()->first;
    }

    for( auto& [s, point] : route.reference_line )
    {
        if( s >= ego_s )
        {
            point.max_speed = 0.0;
        }
    }
}

map::Route
make_max_braking_route_from_s( const map::Route& route, double ego_s )
{
    map::Route max_braking_route = route;
    set_route_points_from_s_to_zero( max_braking_route, ego_s );
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

    RCLCPP_INFO(
        rclcpp::get_logger( "Behaviors" ),
        "[OA][STOP_ROUTE] building reachable stop profile active_route=%s ego_s=%.2f requested_stop_s=%.2f target_stop_s=%.2f v=%.2f braking_distance=%.2f safety_margin=%.2f reachability_margin=%.2f reason=%s",
        active_route_name.c_str(),
        ego_s,
        requested_stop_s,
        target_stop_s,
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
        PreviousTrajectoryFallbackGuard fallback_guard( planner );
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
            PreviousTrajectoryFallbackGuard fallback_guard( planner );
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
            RCLCPP_INFO(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][STOP_ROUTE] maximum route-based braking trajectory accepted active_route=%s ego_s=%.2f",
                active_route_name.c_str(),
                ego_s );
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
        PreviousTrajectoryFallbackGuard fallback_guard( planner );
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
            PreviousTrajectoryFallbackGuard fallback_guard( planner );
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
            RCLCPP_INFO(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][STOP_ROUTE] maximum route-based braking trajectory accepted active_route=%s ego_s=%.2f id=%d",
                active_route_name.c_str(),
                ego_s,
                conflict.participant_id );
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

} // namespace

            Behavior driving_mission(
            planner::TrajectoryPlanner& planner,
            const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
            const map::Route& route,
            const dynamics::TrafficParticipantSet& traffic_participants,
            const std::map<size_t, adore_ros2_msgs::msg::TrafficSignal>& traffic_signals,
            const std::optional<adore_ros2_msgs::msg::Weather>& weather,
            const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance,
            ActiveAvoidanceState& active_avoidance_state )
        {
            // Go through route, and update speed at points based on traffic signal positions.
            auto route_with_signal = route;

            for( auto& p : route_with_signal.reference_line )
            {
                if( std::any_of(
                        traffic_signals.begin(),
                        traffic_signals.end(),
                        [&]( const auto& s )
                        {
                            return adore::math::distance_2d( s.second, p.second ) < 3.0 &&
                                s.second.state != adore_ros2_msgs::msg::TrafficSignal::GREEN;
                        } ) )
                {
                    p.second.max_speed = 0;
                }
            }

            // Determine weather-based speed limitation once.
            bool use_weather_comfort_settings = false;
            std::string weather_label;

            dynamics::ComfortSettings weather_comfort_settings;
            weather_comfort_settings.max_speed = 5.5; // 20 km/h

            if( weather.has_value() )
            {
                if( weather.value().wind_intensity > 2 )
                {
                    use_weather_comfort_settings = true;
                    weather_label = "carefully due to wind";
                }
                else if( weather.value().wetness > 20 )
                {
                    use_weather_comfort_settings = true;
                    weather_label = "carefully due to rain";
                }
            }

            // -------------------------------------------------------------------------
            // OA invariant:
            // The decision maker never publishes free-space emergency trajectories.
            // During obstacle avoidance, ego always follows either the original route
            // or the active modified route.
            // Braking and waiting are represented by reducing max_speed on the active route.
            //
            // Persistent obstacle avoidance maneuver.
            //
            // If an OA maneuver is active, keep driving the stored modified route until
            // the ego vehicle has passed the release point. Do not use disappearing
            // obstacle detections to end a successful avoidance early. If a new
            // obstacle conflicts with the active modified route through the generic
            // route-corridor safety check, first try to replan a new modified route;
            // fall back to a route speed-profile stop only if no validated replan
            // exists.
            //
            // Opposite-lane monitor conflicts are handled more conservatively: they
            // do not trigger active replanning. Before commitment, an oncoming
            // conflict aborts to a controlled stop on the active route. After
            // commitment, the vehicle keeps the active route and brakes on that route
            // instead of abruptly abandoning the return path.
            // -------------------------------------------------------------------------
            if( active_avoidance_state.active )
            {
                auto active_route =
                    !active_avoidance_state.base_modified_route.reference_line.empty()
                        ? active_avoidance_state.base_modified_route
                        : active_avoidance_state.modified_route;

                // The stored modified route was created from a signal snapshot
                // at maneuver start. Re-apply the live traffic-signal state each
                // cycle so a signal that turned red during the maneuver still
                // produces a stop on the route ego is actually following.
                for( auto& p : active_route.reference_line )
                {
                    if( std::any_of(
                            traffic_signals.begin(),
                            traffic_signals.end(),
                            [&]( const auto& s )
                            {
                                return adore::math::distance_2d( s.second, p.second ) < 3.0 &&
                                    s.second.state != adore_ros2_msgs::msg::TrafficSignal::GREEN;
                            } ) )
                    {
                        p.second.max_speed = 0;
                    }
                }

                RCLCPP_INFO(
                    rclcpp::get_logger( "Behaviors" ),
                    "[OA][ACTIVE_ROUTE] following modified_route" );

                auto make_active_stop_behavior =
                    [&]( const std::string& label ) -> Behavior
                    {
                        return make_stop_on_route_behavior(
                            planner,
                            active_route,
                            vehicle_state_dynamic,
                            traffic_participants,
                            params_for_obstacle_avoidance,
                            label,
                            "modified" );
                    };

                auto make_active_conflict_stop_behavior =
                    [&]( const ::adore::planner::RouteCorridorConflict& conflict ) -> Behavior
                    {
                        return make_stop_on_active_route_behavior(
                            planner,
                            active_route,
                            conflict,
                            vehicle_state_dynamic,
                            traffic_participants,
                            params_for_obstacle_avoidance,
                            "modified" );
                    };

                if( active_avoidance_state.maneuver.active &&
                    active_avoidance_state.maneuver.uses_opposite_lane )
                {
                    const auto monitor_result =
                        ::adore::planner::monitor_active_obstacle_avoidance_maneuver(
                            active_route,
                            vehicle_state_dynamic,
                            traffic_participants,
                            active_avoidance_state.maneuver,
                            params_for_obstacle_avoidance );

                    if( monitor_result.should_abort_before_commitment )
                    {
                        RCLCPP_WARN(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][MONITOR][ABORT] obstacle_id=%d reason=%s",
                            active_avoidance_state.obstacle_id,
                            monitor_result.reason.c_str() );

                        ::adore::planner::RouteCorridorConflict monitor_conflict;
                        monitor_conflict.participant_id =
                            monitor_result.oncoming.participant_id;
                        monitor_conflict.object_class =
                            ::adore::planner::RouteCorridorObjectClass::Oncoming;
                        monitor_conflict.object_s_min =
                            monitor_result.oncoming.conflict_start_s;
                        monitor_conflict.object_s_max =
                            monitor_result.oncoming.conflict_end_s;
                        monitor_conflict.distance_s =
                            std::max(
                                0.0,
                                monitor_result.oncoming.conflict_start_s -
                                active_avoidance_state.last_modified_s );
                        monitor_conflict.time_to_conflict =
                            monitor_result.oncoming.oncoming_arrival_time;
                        monitor_conflict.predicted_spatiotemporal_conflict = true;
                        monitor_conflict.requires_stop = true;
                        monitor_conflict.ttc_source = "opposite_lane_monitor";
                        monitor_conflict.reason = monitor_result.reason;

                        RCLCPP_WARN(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][ACTIVE_STATE] opposite-lane monitor conflict; using route-based stop policy" );

                        return make_active_conflict_stop_behavior( monitor_conflict );
                    }

                    if( !monitor_result.safe_to_continue )
                    {
                        // Past commitment, stopping inside the opposite lane is
                        // usually more dangerous than finishing the maneuver.
                        // Stop only if the conflicting participant is already
                        // inside the conflict interval; for a predicted future
                        // arrival, keep following the active route to clear the
                        // opposite lane as quickly as possible.
                        const bool conflict_already_in_interval =
                            monitor_result.oncoming.oncoming_arrival_time <= 1e-6;

                        if( conflict_already_in_interval )
                        {
                            RCLCPP_WARN(
                                rclcpp::get_logger( "Behaviors" ),
                                "[OA][MONITOR][COMMITTED] obstacle_id=%d reason=%s",
                                active_avoidance_state.obstacle_id,
                                monitor_result.reason.c_str() );

                            ::adore::planner::RouteCorridorConflict monitor_conflict;
                            monitor_conflict.participant_id =
                                monitor_result.oncoming.participant_id;
                            monitor_conflict.object_class =
                                ::adore::planner::RouteCorridorObjectClass::Oncoming;
                            monitor_conflict.object_s_min =
                                monitor_result.oncoming.conflict_start_s;
                            monitor_conflict.object_s_max =
                                monitor_result.oncoming.conflict_end_s;
                            monitor_conflict.distance_s =
                                std::max(
                                    0.0,
                                    monitor_result.oncoming.conflict_start_s -
                                    active_avoidance_state.last_modified_s );
                            monitor_conflict.time_to_conflict =
                                monitor_result.oncoming.oncoming_arrival_time;
                            monitor_conflict.predicted_spatiotemporal_conflict = true;
                            monitor_conflict.requires_stop = true;
                            monitor_conflict.ttc_source = "opposite_lane_monitor_committed";
                            monitor_conflict.reason = monitor_result.reason;

                            RCLCPP_WARN(
                                rclcpp::get_logger( "Behaviors" ),
                                "[OA][ACTIVE_STATE] committed opposite-lane monitor conflict already in interval; using route-based stop policy" );

                            return make_active_conflict_stop_behavior( monitor_conflict );
                        }

                        RCLCPP_WARN(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][MONITOR][COMMITTED] obstacle_id=%d reason=%s; continuing to clear the opposite lane instead of stopping in it",
                            active_avoidance_state.obstacle_id,
                            monitor_result.reason.c_str() );
                    }
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

                // Make ego_s_modified monotonic during active OA
                double ego_s_modified = ego_s_modified_raw;

                if( std::isfinite( ego_s_modified_raw ) &&
                    std::isfinite( active_avoidance_state.last_modified_s ) )
                {
                    // Enforce monotonic progression: never go backward. Forward
                    // jumps beyond what the vehicle can have travelled since the
                    // last cycle are projection artifacts (e.g. matches at the
                    // search-window edge); advance by odometry instead of
                    // latching the jump permanently.
                    const double dt =
                        std::isfinite( active_avoidance_state.last_modified_time )
                            ? std::max(
                                  0.0,
                                  vehicle_state_dynamic.time -
                                      active_avoidance_state.last_modified_time )
                            : 0.0;
                    const double odometry_advance =
                        std::max( 0.0, vehicle_state_dynamic.vx ) * dt;
                    const double max_plausible_advance =
                        odometry_advance + MAX_PLAUSIBLE_MODIFIED_S_JUMP;

                    if( ego_s_modified_raw < active_avoidance_state.last_modified_s )
                    {
                        ego_s_modified = active_avoidance_state.last_modified_s;
                    }
                    else if( ego_s_modified_raw - active_avoidance_state.last_modified_s >
                             max_plausible_advance )
                    {
                        ego_s_modified =
                            active_avoidance_state.last_modified_s + odometry_advance;
                        RCLCPP_WARN(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][WARN] implausible ego_s_modified jump raw=%.2f last=%.2f max_advance=%.2f; using odometry advance to %.2f",
                            ego_s_modified_raw,
                            active_avoidance_state.last_modified_s,
                            max_plausible_advance,
                            ego_s_modified );
                    }
                }
                else if( !std::isfinite( ego_s_modified_raw ) &&
                         std::isfinite( active_avoidance_state.last_modified_s ) )
                {
                    // Fallback: projection lost, use last valid value
                    ego_s_modified = active_avoidance_state.last_modified_s;
                    RCLCPP_WARN(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][WARN] ego_s_modified projection lost; using last_modified_s=%.2f",
                        active_avoidance_state.last_modified_s );
                }

                if( std::isfinite( ego_s_modified ) )
                {
                    active_avoidance_state.last_modified_s = ego_s_modified;
                    active_avoidance_state.last_modified_time = vehicle_state_dynamic.time;
                }
                else
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][ERROR] ego_s_modified invalid while OA active; braking on modified_route speed profile" );

                    return make_active_stop_behavior(
                        "driving mission (active OA route stop: invalid modified-route projection)" );
                }

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

                if( params_for_obstacle_avoidance.modified_route_safety_check_enabled )
                {
                    static bool active_route_safety_logged = false;
                    if( !active_route_safety_logged )
                    {
                        RCLCPP_INFO(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][ROUTE_MONITOR] modified-route safety check active" );
                        active_route_safety_logged = true;
                    }

                    if( active_route_safety.has_conflict )
                    {
                        RCLCPP_WARN(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][ROUTE_MONITOR] conflict on active route id=%d active_route=modified reason=%s",
                            active_route_safety.conflict.participant_id,
                            active_route_safety.conflict.reason.c_str() );
                    }
                    else
                    {
                        RCLCPP_DEBUG(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][ROUTE_MONITOR] active_route=modified conflict=false" );
                    }
                }

                // Validate that we're ignoring the expected obstacles from the original avoidance
                for( int ignored_id : active_avoidance_state.obstacle_ids )
                {
                    RCLCPP_DEBUG(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][MONITOR] ignoring active avoidance obstacle id=%d as expected obstacle",
                        ignored_id );
                }

                auto active_conflict =
                    active_route_safety.has_conflict
                        ? std::optional<::adore::planner::RouteCorridorConflict>( active_route_safety.conflict )
                        : most_relevant_ghost_conflict( active_avoidance_state, ego_s_modified );

                if( active_conflict.has_value() &&
                    active_avoidance_state.in_lane &&
                    is_oncoming_other_lane_conflict_for_in_lane_shift(
                        active_conflict.value(),
                        planner.get_physical_vehicle_parameters(),
                        params_for_obstacle_avoidance ) )
                {
                    RCLCPP_INFO(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][ACTIVE_ROUTE] ignoring oncoming other-lane conflict during in-lane shift id=%d",
                        active_conflict->participant_id );
                    active_conflict.reset();
                }

                if( active_conflict.has_value() &&
                    static_or_slow_conflict_has_side_clearance(
                        active_conflict.value(),
                        planner.get_physical_vehicle_parameters(),
                        params_for_obstacle_avoidance ) )
                {
                    RCLCPP_INFO(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][ACTIVE_ROUTE] ignoring static_or_slow side-clearance conflict id=%d type=%s l=[%.2f,%.2f] required_clearance=%.2f",
                        active_conflict->participant_id,
                        conflict_type_name( active_conflict.value() ),
                        active_conflict->object_l_min,
                        active_conflict->object_l_max,
                        std::max( 0.0, params_for_obstacle_avoidance.side_clearance ) );
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
                    RCLCPP_INFO(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][ACTIVE_ROUTE] ignoring static_or_slow conflict behind ego id=%d object_s_max=%.2f ego_s=%.2f",
                        active_conflict->participant_id,
                        active_conflict->object_s_max,
                        ego_s_modified );
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

                    auto try_dynamic_replan_from_route =
                        [&]( const map::Route& replan_base_route,
                             const char* replan_base_name,
                             double projection_hint_s,
                             const std::vector<int>* additional_ignored_participant_ids = nullptr ) -> std::optional<Behavior>
                    {
                        RCLCPP_WARN(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][ACTIVE_ROUTE] new conflict id=%d; attempting dynamic replan from %s_route",
                            active_conflict->participant_id,
                            replan_base_name );

                        if( additional_ignored_participant_ids != nullptr &&
                            !additional_ignored_participant_ids->empty() )
                        {
                            RCLCPP_INFO(
                                rclcpp::get_logger( "Behaviors" ),
                                "[OA][ACTIVE_ROUTE][REPLAN] treating current active obstacle_ids=%s as expected during %s_route replan",
                                ids_to_string( *additional_ignored_participant_ids ).c_str(),
                                replan_base_name );
                        }

                        const auto replan_result =
                            ::adore::planner::try_plan_obstacle_avoidance(
                                planner,
                                replan_base_route,
                                vehicle_state_dynamic,
                                traffic_participants,
                                params_for_obstacle_avoidance,
                                additional_ignored_participant_ids );

                        RCLCPP_INFO(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][ACTIVE_ROUTE][REPLAN] result success=%s mode=%d bounds=%s reason=%s",
                            replan_result.success ? "true" : "false",
                            static_cast<int>( replan_result.mode ),
                            replan_result.has_maneuver_bounds ? "true" : "false",
                            replan_result.reason.c_str() );

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
                            PreviousTrajectoryFallbackGuard fallback_guard(
                                planner,
                                "dynamic modified-route replan" );
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

                        trajectory.adjust_start_time( vehicle_state_dynamic.time );
                        trajectory.label =
                            obstacle_avoidance_label(
                                replan_result.mode,
                                replan_result.in_lane,
                                replan_result.lateral_shift,
                                true );

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
                    };

                    if( is_new_conflict )
                    {
                        if( auto replanned_behavior =
                                try_dynamic_replan_from_route(
                                    active_route,
                                    "active",
                                    ego_s_modified,
                                    &active_avoidance_state.obstacle_ids );
                            replanned_behavior.has_value() )
                        {
                            return replanned_behavior.value();
                        }

                        if( auto replanned_behavior =
                                try_dynamic_replan_from_route(
                                    route_with_signal,
                                    "mission",
                                    ego_s_original );
                            replanned_behavior.has_value() )
                        {
                            return replanned_behavior.value();
                        }
                    }

                    if( !stop_required )
                    {
                        const auto stop_plan =
                            compute_route_stop_plan(
                                active_route,
                                vehicle_state_dynamic,
                                planner.get_physical_vehicle_parameters(),
                                active_conflict.value(),
                                params_for_obstacle_avoidance );

                        RCLCPP_INFO(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][ACTIVE_STATE] conflict ahead id=%d type=%s dist=%.2f ttc=%.2f brake_start_s=%.2f ego_s=%.2f action=continue_active_route",
                            active_conflict->participant_id,
                            conflict_type_name( active_conflict.value() ),
                            active_conflict->distance_s,
                            active_conflict->time_to_conflict,
                            stop_plan.brake_start_s,
                            stop_plan.ego_s );
                    }
                    else
                    {
                        if( is_new_conflict )
                        {
                            RCLCPP_WARN(
                                rclcpp::get_logger( "Behaviors" ),
                                "[OA][ACTIVE_ROUTE][REPLAN] no validated dynamic replan; stopping on active modified_route" );
                        }
                        else
                        {
                            RCLCPP_INFO(
                                rclcpp::get_logger( "Behaviors" ),
                                "[OA][ACTIVE_STATE] monitored conflict (ghost) id=%d type=%s dist=%.2f ttc=%.2f action=stop_on_modified_route",
                                active_conflict->participant_id,
                                conflict_type_name( active_conflict.value() ),
                                active_conflict->distance_s,
                                active_conflict->time_to_conflict );
                        }

                        RCLCPP_INFO(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][ACTIVE_STATE] using route-based stop policy after dynamic replan check" );

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
                    retained_original_ghost.has_value() )
                {
                    RCLCPP_INFO(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][ghost] retained until pass id=%d ego_s=%.2f hold_until_s=%.2f",
                        retained_original_ghost->last_participant_id,
                        ego_s_modified,
                        retained_original_ghost->hold_until_s );
                }

                if( ( release_by_modified || release_by_original_fallback ) &&
                    !retained_original_ghost.has_value() )
                {
                    std::size_t active_route_zero_speed_points = 0;
                    double first_zero_speed_s =
                        std::numeric_limits<double>::quiet_NaN();
                    double last_zero_speed_s =
                        std::numeric_limits<double>::quiet_NaN();
                    for( const auto& [s, point] : active_avoidance_state.modified_route.reference_line )
                    {
                        if( point.max_speed.has_value() && *point.max_speed <= 0.01 )
                        {
                            if( active_route_zero_speed_points == 0 )
                            {
                                first_zero_speed_s = s;
                            }
                            last_zero_speed_s = s;
                            ++active_route_zero_speed_points;
                        }
                    }

                    RCLCPP_INFO(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][RELEASE] modified_route completed ego_s=%.2f release_s=%.2f obstacle_id=%d ego_s_orig=%.2f by=%s returning_to_original_route",
                        ego_s_modified,
                        active_avoidance_state.release_s,
                        active_avoidance_state.obstacle_id,
                        ego_s_original,
                        release_by_modified ? "modified" : "original_fallback" );

                    RCLCPP_WARN(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][RELEASE_CHECK] before reset obstacle_id=%d ego_s_mod=%.2f ego_s_orig=%.2f release_s=%.2f projection_diff=%.2f active_route_points=%zu stored_modified_route_points=%zu zero_speed_points=%zu first_zero_speed_s=%.2f last_zero_speed_s=%.2f",
                        active_avoidance_state.obstacle_id,
                        ego_s_modified,
                        ego_s_original,
                        active_avoidance_state.release_s,
                        projection_error,
                        active_route.reference_line.size(),
                        active_avoidance_state.modified_route.reference_line.size(),
                        active_route_zero_speed_points,
                        first_zero_speed_s,
                        last_zero_speed_s );

                    if( projection_valid && projection_error > 2.0 )
                    {
                        RCLCPP_WARN(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][WARN] modified-route projection diverged: ego_s_mod=%.2f ego_s_orig=%.2f diff=%.2f",
                            ego_s_modified,
                            ego_s_original,
                            projection_error );
                    }

                    RCLCPP_WARN(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][RELEASE_CHECK] resetting active avoidance state and planning driving mission on original route" );

                    active_avoidance_state.reset();

                    // now normal route
                    dynamics::Trajectory trajectory;
                    try
                    {
                        PreviousTrajectoryFallbackGuard fallback_guard(
                            planner,
                            "return from modified route to original route" );
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
                            PreviousTrajectoryFallbackGuard fallback_guard(
                                planner,
                                "active modified-route tracking with weather comfort" );
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
                                true ) +
                            " (" + weather_label + ")";
                    }
                    else
                    {
                        try
                        {
                            PreviousTrajectoryFallbackGuard fallback_guard(
                                planner,
                                "active modified-route tracking" );
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
                                true );
                    }

                    if( trajectory.states.empty() )
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][ERROR] planner returned empty trajectory on active modified_route; commanding controlled stop" );

                        return make_active_stop_behavior(
                            "driving mission (active OA controlled stop: planning failed)" );
                    }

                    if( active_avoidance_state.maneuver.active &&
                        active_avoidance_state.maneuver.uses_opposite_lane )
                    {
                        const auto planned_monitor_result =
                            ::adore::planner::monitor_active_obstacle_avoidance_maneuver(
                                active_route_for_planning,
                                vehicle_state_dynamic,
                                traffic_participants,
                                active_avoidance_state.maneuver,
                                params_for_obstacle_avoidance,
                                &trajectory );

                        if( planned_monitor_result.should_abort_before_commitment )
                        {
                            RCLCPP_WARN(
                                rclcpp::get_logger( "Behaviors" ),
                                "[OA][MONITOR][PLANNED_ABORT] obstacle_id=%d reason=%s",
                                active_avoidance_state.obstacle_id,
                                planned_monitor_result.reason.c_str() );

                            ::adore::planner::RouteCorridorConflict monitor_conflict;
                            monitor_conflict.participant_id =
                                planned_monitor_result.oncoming.participant_id;
                            monitor_conflict.object_class =
                                ::adore::planner::RouteCorridorObjectClass::Oncoming;
                            monitor_conflict.object_s_min =
                                planned_monitor_result.oncoming.conflict_start_s;
                            monitor_conflict.object_s_max =
                                planned_monitor_result.oncoming.conflict_end_s;
                            monitor_conflict.distance_s =
                                std::max(
                                    0.0,
                                    planned_monitor_result.oncoming.conflict_start_s -
                                    ego_s_modified );
                            monitor_conflict.time_to_conflict =
                                planned_monitor_result.oncoming.oncoming_arrival_time;
                            monitor_conflict.predicted_spatiotemporal_conflict = true;
                            monitor_conflict.requires_stop = true;
                            monitor_conflict.ttc_source = "planned_opposite_lane_monitor";
                            monitor_conflict.reason = planned_monitor_result.reason;

                            RCLCPP_WARN(
                                rclcpp::get_logger( "Behaviors" ),
                                "[OA][ACTIVE_STATE] planned opposite-lane monitor conflict; using route-based stop policy" );

                            return make_active_conflict_stop_behavior( monitor_conflict );
                        }

                        if( !planned_monitor_result.safe_to_continue )
                        {
                            // Same committed policy as the early monitor: stop
                            // only if the conflict is already inside the
                            // interval, otherwise clear the opposite lane.
                            const bool conflict_already_in_interval =
                                planned_monitor_result.oncoming.oncoming_arrival_time <= 1e-6;

                            if( conflict_already_in_interval )
                            {
                                RCLCPP_WARN(
                                    rclcpp::get_logger( "Behaviors" ),
                                    "[OA][MONITOR][PLANNED_COMMITTED] obstacle_id=%d reason=%s",
                                    active_avoidance_state.obstacle_id,
                                    planned_monitor_result.reason.c_str() );

                                ::adore::planner::RouteCorridorConflict monitor_conflict;
                                monitor_conflict.participant_id =
                                    planned_monitor_result.oncoming.participant_id;
                                monitor_conflict.object_class =
                                    ::adore::planner::RouteCorridorObjectClass::Oncoming;
                                monitor_conflict.object_s_min =
                                    planned_monitor_result.oncoming.conflict_start_s;
                                monitor_conflict.object_s_max =
                                    planned_monitor_result.oncoming.conflict_end_s;
                                monitor_conflict.distance_s =
                                    std::max(
                                        0.0,
                                        planned_monitor_result.oncoming.conflict_start_s -
                                        ego_s_modified );
                                monitor_conflict.time_to_conflict =
                                    planned_monitor_result.oncoming.oncoming_arrival_time;
                                monitor_conflict.predicted_spatiotemporal_conflict = true;
                                monitor_conflict.requires_stop = true;
                                monitor_conflict.ttc_source = "planned_opposite_lane_monitor_committed";
                                monitor_conflict.reason = planned_monitor_result.reason;

                                RCLCPP_WARN(
                                    rclcpp::get_logger( "Behaviors" ),
                                    "[OA][ACTIVE_STATE] planned committed opposite-lane monitor conflict already in interval; using route-based stop policy" );

                                return make_active_conflict_stop_behavior( monitor_conflict );
                            }

                            RCLCPP_WARN(
                                rclcpp::get_logger( "Behaviors" ),
                                "[OA][MONITOR][PLANNED_COMMITTED] obstacle_id=%d reason=%s; continuing to clear the opposite lane instead of stopping in it",
                                active_avoidance_state.obstacle_id,
                                planned_monitor_result.reason.c_str() );
                        }
                    }

                    trajectory.adjust_start_time( vehicle_state_dynamic.time );

                    Behavior trajectory_and_signal;
                    trajectory_and_signal.trajectory =
                        dynamics::conversions::to_ros_msg( trajectory );

                    trajectory_and_signal.modified_route =
                        map::conversions::to_ros_msg( active_route_for_planning );

                    if( std::isfinite( ego_s_modified ) )
                    {
                        RCLCPP_INFO(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][CONTINUE] obstacle_id=%d ego_xy=(%.3f, %.3f) v=%.3f "
                            "ego_s_mod=%.2f ego_s_orig=%.2f release_s=%.2f "
                            "remaining_mod=%.2f remaining_orig=%.2f",
                            active_avoidance_state.obstacle_id,
                            vehicle_state_dynamic.x,
                            vehicle_state_dynamic.y,
                            vehicle_state_dynamic.vx,
                            ego_s_modified,
                            ego_s_original,
                            active_avoidance_state.release_s,
                            active_avoidance_state.release_s - ego_s_modified,
                            active_avoidance_state.release_s - ego_s_original );
                    }
                    else
                    {
                        RCLCPP_WARN(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][CONTINUE] ego_s_modified invalid while OA active; continuing modified_route" );
                    }

                    RCLCPP_INFO(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][ACTIVE_STATE] continue active modified_route ego_s=%.2f release_s=%.2f",
                        ego_s_modified,
                        active_avoidance_state.release_s );
                    RCLCPP_INFO(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][ACTIVE_STATE] follow modified_route ego_s=%.2f release_s=%.2f v=%.2f",
                        ego_s_modified,
                        active_avoidance_state.release_s,
                        vehicle_state_dynamic.vx );

                    return trajectory_and_signal;
                }
            }

            // -------------------------------------------------------------------------
            // Defensive stop for oncoming traffic on the ego lane.
            //
            // This is intentionally separate from obstacle avoidance. It covers the
            // case where another vehicle temporarily drives on the ego lane, e.g.
            // because it is avoiding a parked vehicle on its own side. A human driver
            // would slow down or stop and let the other vehicle finish its maneuver.
            //
            // Run this before weather/normal/OA planning when no active OA state is active.
            // -------------------------------------------------------------------------
            RCLCPP_INFO(
                rclcpp::get_logger( "Behaviors" ),
                "[OA][STATE_NORMAL] follow original_route" );

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

            // -------------------------------------------------------------------------
            // Weather behavior only applies directly if no OA maneuver is currently
            // active. If OA is active, weather speed reduction is applied to the active
            // modified route above.
            // -------------------------------------------------------------------------
            if( use_weather_comfort_settings )
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

            // -------------------------------------------------------------------------
            // Try obstacle avoidance only if no maneuver is currently active.
            // =========================================================================
            // Check master OA enable switch. If OA is disabled, follow normal route.
            // =========================================================================
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
                RCLCPP_INFO(
                    rclcpp::get_logger( "Behaviors" ),
                    "[OA][DISABLED] obstacle avoidance disabled by parameter; following original route" );

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

            RCLCPP_INFO(
                rclcpp::get_logger( "Behaviors" ),
                "Obstacle avoidance result: success=%s, mode=%d, reason=%s",
                oa_result.success ? "true" : "false",
                static_cast<int>( oa_result.mode ),
                oa_result.reason.c_str() );

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
                    RCLCPP_INFO(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][NO_ACTIVE_STATE] using OA result without maneuver bounds: mode=%d reason=%s",
                        static_cast<int>( oa_result.mode ),
                        oa_result.reason.c_str() );

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
                    PreviousTrajectoryFallbackGuard fallback_guard(
                        planner,
                        "new modified-route start" );
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
                        false );

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

    Behavior driving_mission_following_managed(
                            planner::TrajectoryPlanner& planner,
                            const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                            const dynamics::Trajectory& managed_trajectory, 
                            const math::Polygon2d managed_zone
    )
    {
        Behavior trajectory_and_signals;
        trajectory_and_signals.trajectory = dynamics::conversions::to_ros_msg( managed_trajectory );
        trajectory_and_signals.trajectory.label = "driving mission (following managed)";

        return trajectory_and_signals;
    }

    Behavior waiting_for_mission(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const dynamics::TrafficParticipantSet& traffic_participants 
                            )
    {
        dynamics::Trajectory trajectory;
        trajectory.label = "waiting for mission";

        trajectory.states.push_back( vehicle_state_dynamic );

        Behavior trajectory_and_signal;
        trajectory_and_signal.trajectory = dynamics::conversions::to_ros_msg( trajectory );

        return trajectory_and_signal;
    }

    Behavior remote_operations(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants,
                                bool& approved_to_drive_suggested_remote_operations,
                                std::optional<dynamics::Trajectory>& suggested_remote_operator_trajectory
    )
    {
        Behavior trajectory_and_signals;

        if ( suggested_remote_operator_trajectory.has_value() && approved_to_drive_suggested_remote_operations )
        {
            auto trajectory = suggested_remote_operator_trajectory.value();
            
            if ( !trajectory.states.empty() ) 
            {
                if ( 
                    adore::math::distance_2d(trajectory.states.back(), vehicle_state_dynamic) > MAX_DISTANCE_TO_LAST_TRAJECTORY_POINT_BEFORE_RETURNING_TO_REMOTE_OPERATIONS_DRIVING
                )
                {
                    trajectory.label              = "remote operations (driving remote operator instructions)";
                    trajectory_and_signals.trajectory = dynamics::conversions::to_ros_msg(trajectory);

                    return trajectory_and_signals;
                }

                // Return the vehicle to waiting for intructions if it doesn't pass the above checks
                suggested_remote_operator_trajectory.reset();
                approved_to_drive_suggested_remote_operations = false;
            }
        }

        trajectory_and_signals = minimum_risk(
            planner,
            vehicle_state_dynamic,
            route,
            traffic_participants,
            {}
        );
        trajectory_and_signals.trajectory.label = "remote operations (waiting for remote operator instructions)";

        if ( vehicle_state_dynamic.vx < 0.5 ) // Should first send alternative trajectories when standing still
        {
            trajectory_and_signals.alternative_trajectory = dynamics::conversions::to_ros_msg(
                                                                                              get_alternative_trajectory_in_remote_operations(
                                                                                                  planner, 
                                                                                                  vehicle_state_dynamic, 
                                                                                                  route, 
                                                                                                  traffic_participants));
        }

        return trajectory_and_signals;
    }

    dynamics::Trajectory get_alternative_trajectory_in_remote_operations(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants 
    )
    {
        // @TODO, make this alternative trajectory take the area with problems into account
        
        double state_s = route.get_s( vehicle_state_dynamic );
        auto   cut_route          = route.get_shortened_route( state_s, 100.0 );
        auto   alternative_trajectory = planner::waypoints_to_trajectory( 
                                                                        vehicle_state_dynamic, cut_route, traffic_participants,
                                                                        dynamics::PhysicalVehicleModel(planner.get_physical_vehicle_parameters()), 
                                                                        5.5 /* target_speed 20 km/h */ );
        return alternative_trajectory;
    }

    Behavior minimum_risk(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants, 
                                const std::optional<adore_ros2_msgs::msg::Odd>& odd
    )
    {

        double state_s = route.get_s( vehicle_state_dynamic );
        auto   cut_route          = route.get_shortened_route( state_s, 100.0 );
        auto   planned_trajectory = planner::waypoints_to_trajectory( vehicle_state_dynamic, cut_route, traffic_participants,
                                                                        dynamics::PhysicalVehicleModel(planner.get_physical_vehicle_parameters()), 0.0 /* target_speed */ );

        // planned_trajectory = planning_tools.planner.optimize_trajectory( *domain.vehicle_state, planned_trajectory );
        if( planned_trajectory.states.size() < 2 )
        {
            planned_trajectory.states.push_back( vehicle_state_dynamic );
        }

        planned_trajectory.label = "Minimum Risk Maneuver - due to missing odd topic";
        if ( odd.has_value() )
        {
            if ( !odd.value().matching )
            {
                planned_trajectory.label = "Minimum Risk Maneuver - " + odd.value().status;
            }
        }


        Behavior trajectory_and_signals;
        trajectory_and_signals.trajectory = dynamics::conversions::to_ros_msg(planned_trajectory);

        return trajectory_and_signals;
    }

    Behavior avoiding_safety_corridor(
                            planner::TrajectoryPlanner& planner,
                            const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                            const dynamics::TrafficParticipantSet& traffic_participants, 
                            const adore_ros2_msgs::msg::SafetyCorridor& safety_corridor
    )
    {
        auto     right_forward_points = planner::filter_points_in_front( safety_corridor.right_border, vehicle_state_dynamic );
        auto     safety_waypoints     = planner::shift_points_right( right_forward_points, planner.get_physical_vehicle_parameters().body_width );
        double   target_speed         = planner::is_point_to_right_of_line( vehicle_state_dynamic, right_forward_points ) ? 0 : 2.0;

        auto planned_trajectory = planner::waypoints_to_trajectory( vehicle_state_dynamic, safety_waypoints, traffic_participants,
                                                                    dynamics::PhysicalVehicleModel(planner.get_physical_vehicle_parameters()), target_speed );

        planned_trajectory = planner.optimize_trajectory( vehicle_state_dynamic, planned_trajectory );

        planned_trajectory.label = "avoiding safety corridor";

        Behavior trajectory_and_signals;
        trajectory_and_signals.trajectory = dynamics::conversions::to_ros_msg(planned_trajectory);

        return trajectory_and_signals;
    }

    Behavior emergency(
                            planner::TrajectoryPlanner& planner,
                            const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic  
    )
    {
        dynamics::Trajectory emergency_stop_trajectory;
        if( vehicle_state_dynamic.has_value() )
            emergency_stop_trajectory.states.push_back( vehicle_state_dynamic.value() );

        emergency_stop_trajectory.label = "emergency stop";

        Behavior trajectory_and_signals;
        trajectory_and_signals.trajectory = dynamics::conversions::to_ros_msg(emergency_stop_trajectory);

        return trajectory_and_signals;
    }
} // namespace behavior
} // namespace adore
