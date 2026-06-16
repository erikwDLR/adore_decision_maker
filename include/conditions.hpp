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
#include <array>
#include <cstdint>
#include "adore_dynamics_conversions.hpp"
#include "adore_ros2_msgs/msg/safety_corridor.hpp"
#include "adore_ros2_msgs/msg/odd.hpp"
#include <adore_math/polygon.h>
#include <dynamics/trajectory.hpp>

namespace adore
{
    namespace conditions
    {
        const size_t MININUM_REFERENCE_TRAJECTORY_SIZE = 5;
        const double MAXIMUM_VEHICLE_STATE_DYNAMIC_AGE_SECONDS = 1.0;
        const double MAXIMUM_REFERENCE_TRAJECTORY_AGE_SECONDS = 1.0; 
        const size_t MINIMUM_ROUTE_LENGHTH_METERS = 1;
        const double MAXIMUM_ODD_AGE_SECONDS = 2.0;

        // @TODO, improve these two functions, they are too simple currently
        bool has_localization( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, const double& time_now);
        bool has_mission( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, const std::optional<map::Route>& route );
        bool needs_remote_operator_assitance( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, const std::map<std::string, math::Polygon2d>& caution_zones);
        bool needs_to_avoid_safety_corridor( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, const std::optional<adore_ros2_msgs::msg::SafetyCorridor>& safety_corridor );
        bool can_drive_managed( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, const double& time_now, const std::optional<math::Polygon2d>& managed_zone, const std::optional<dynamics::Trajectory>& managed_trajectory);
        bool odd_conditions_satisfied( const std::optional<adore_ros2_msgs::msg::Odd>& odd, const double& time_now );

    } // namespace conditions
} // namespace adore


