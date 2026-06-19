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

#include "conditions.hpp"
#include "adore_ros2_msgs/msg/odd.hpp"
#include <dynamics/vehicle_state.hpp>



namespace adore
{
namespace conditions
{


bool has_localization( 
                const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                const double& time_now )
{
    if ( !vehicle_state_dynamic.has_value() )
        return false;

    if ( time_now - vehicle_state_dynamic.value().time > MAXIMUM_VEHICLE_STATE_DYNAMIC_AGE_SECONDS ) // If the message is more than one second old 
        return false;

    // @TODO, add covarianve of estimate

    return true;
}

bool has_mission( 
                const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                const std::optional<map::Route>& route 
            )
{
    if( !route.has_value() || !vehicle_state_dynamic.has_value() )
        return false;

    double remaining = route->get_length() - route->get_s( *vehicle_state_dynamic );
    return remaining > MINIMUM_ROUTE_LENGHTH_METERS;
}

bool needs_remote_operator_assitance( 
                            const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                            const std::map<std::string, math::Polygon2d>& caution_zones
                        )
{
    if ( !vehicle_state_dynamic.has_value() )
        return false;

    // check if in a caution zone
    return std::any_of( caution_zones.begin(), caution_zones.end(),
                        [&]( const auto& zone ) { return zone.second.point_inside( *vehicle_state_dynamic ); } );
}

bool needs_to_avoid_safety_corridor(
                                        const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                                        const std::optional<adore_ros2_msgs::msg::SafetyCorridor>& safety_corridor 
)
{
    if ( !vehicle_state_dynamic.has_value() || !safety_corridor.has_value() )
        return false;

    // @TODO, needs to do a check if it is inside of the safety corridor

    return true;
}

bool can_drive_managed( 
                        const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                        const double& time_now, 
                        const std::optional<math::Polygon2d>& managed_zone, 
                        const std::optional<dynamics::Trajectory>& managed_trajectory)
{
    if (
        !vehicle_state_dynamic.has_value() ||
        !managed_zone.has_value() ||
        !managed_trajectory.has_value()
    )
    {
        return false;
    }

    if ( !managed_zone.value().point_inside(vehicle_state_dynamic.value()) )
        return false;

    if( managed_trajectory.value().states.size() < MININUM_REFERENCE_TRAJECTORY_SIZE )
        return false;

    double age = vehicle_state_dynamic.value().time - managed_trajectory.value().states.front().time;
    return age <= MAXIMUM_REFERENCE_TRAJECTORY_AGE_SECONDS;
}

bool odd_conditions_satisfied( 
                               const std::optional<adore_ros2_msgs::msg::Odd>& odd, 
                               const double& time_now )
{
    if ( !odd.has_value() )
        return false;

    if ( time_now - odd.value().time > MAXIMUM_ODD_AGE_SECONDS )
        return false;
    
    return odd.value().matching;
}

bool must_drive_unstructured( const std::optional<dynamics::VehicleStateDynamic>& vehicle_state_dynamic, 
                             const math::Polygon2d& unstructured_drivable_area )
{
    if( !vehicle_state_dynamic.has_value() )
        return false;

    if( vehicle_state_dynamic.value().vx < 0.1 && !unstructured_drivable_area.point_inside( vehicle_state_dynamic.value() ) )
        return false;
    
    return unstructured_drivable_area.points.size() > 2;
}

} // namespace conditions
} // namespace adore
