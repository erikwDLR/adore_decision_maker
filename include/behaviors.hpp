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
#include <cmath>

#include "dynamics/trajectory.hpp"
#include "adore_map/route.hpp"

#include "adore_ros2_msgs/msg/trajectory.hpp"
#include "adore_ros2_msgs/msg/route.hpp"
#include "adore_ros2_msgs/msg/vehicle_signals.hpp"
#include "adore_ros2_msgs/msg/traffic_signal.hpp"
#include "adore_ros2_msgs/msg/safety_corridor.hpp"

#include "dynamics/traffic_participant.hpp"
#include "planning/trajectory_planner.hpp"

#include "adore_dynamics_conversions.hpp"
#include "adore_ros2_msgs/msg/weather.hpp"
#include <adore_math/polygon.h>

#include "planning/trajectory_planner.hpp"
#include "planning/planning_helpers.hpp"
#include "adore_ros2_msgs/msg/odd.hpp"

#include "planning/obstacle_avoidance.hpp"
#include "oa_maneuver_lock.hpp"

namespace adore
{
namespace behavior
{    
    struct Behavior
    {
        adore_ros2_msgs::msg::Trajectory trajectory;
        std::optional<adore_ros2_msgs::msg::Trajectory> alternative_trajectory;
    std::optional<adore_ros2_msgs::msg::Route> modified_route;
        adore_ros2_msgs::msg::VehicleSignals signals;
    };

    const double MAX_DISTANCE_TO_LAST_TRAJECTORY_POINT_BEFORE_RETURNING_TO_REMOTE_OPERATIONS_DRIVING = 1.0;

    Behavior driving_mission(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants,
                                const std::map<size_t, adore_ros2_msgs::msg::TrafficSignal>& traffic_signals,
                                const std::optional<adore_ros2_msgs::msg::Weather>& weather,
                                const planner::ObstacleAvoidanceParams& obstacle_avoidance_params,
                                ObstacleAvoidanceLock& oa_maneuver_lock
    );

    Behavior driving_mission_following_managed(
                            planner::TrajectoryPlanner& planner,
                            const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                            const dynamics::Trajectory& managed_trajectory, 
                            const math::Polygon2d managed_zone
    );

    Behavior waiting_for_mission(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const dynamics::TrafficParticipantSet& traffic_participants 
    );
    
    Behavior remote_operations(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants,
                                bool& approved_to_drive_suggested_remote_operations,
                                std::optional<dynamics::Trajectory>& suggested_remote_operator_trajectory
    );

    dynamics::Trajectory get_alternative_trajectory_in_remote_operations(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants 
    );

    Behavior minimum_risk(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants,
                                const std::optional<adore_ros2_msgs::msg::Odd>& odd
    );

    Behavior avoiding_safety_corridor(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const dynamics::TrafficParticipantSet& traffic_participants, 
                                const adore_ros2_msgs::msg::SafetyCorridor& safety_corridor
    );

    Behavior emergency(
                                planner::TrajectoryPlanner& planner,
                                const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic
    );
} // namespace behavior
} // namespace adore
