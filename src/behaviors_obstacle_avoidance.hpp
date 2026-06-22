/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
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

#pragma once

#include <optional>
#include <string>

#include <dynamics/comfort_settings.hpp>

#include "behaviors.hpp"

namespace adore
{
namespace behavior
{

// Obstacle-avoidance behavior dispatch, factored out of behaviors.cpp.
// driving_mission() dispatches to these; all of their internal helpers
// (hold/stop behavior builders, ghost-memory orchestration, monitors) are
// file-local in behaviors_obstacle_avoidance.cpp.

// Active obstacle-avoidance maneuver handling: keep following the stored
// modified route, monitor it, replan or stop on new/oncoming conflicts, and
// release back to the mission route once the obstacle is passed.
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
    planner::ActiveAvoidanceState& active_avoidance_state );

// Defensive stop for a participant moving against the ego route direction on
// the ego lane. nullopt if no such threat requires a stop.
std::optional<Behavior>
try_ego_lane_oncoming_stop_behavior(
    planner::TrajectoryPlanner& planner,
    const map::Route& route_with_signal,
    const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance );

// Reduced-speed driving under adverse weather.
Behavior
plan_weather_behavior(
    planner::TrajectoryPlanner& planner,
    const map::Route& route_with_signal,
    const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance,
    const dynamics::ComfortSettings& weather_comfort_settings,
    const std::string& weather_label );

// Plan a new obstacle-avoidance maneuver (shift or stop) when none is active.
Behavior
plan_obstacle_avoidance_behavior(
    planner::TrajectoryPlanner& planner,
    const map::Route& route_with_signal,
    const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance,
    planner::ActiveAvoidanceState& active_avoidance_state );

} // namespace behavior
} // namespace adore
