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

namespace adore
{
namespace behavior
{
    Behavior driving_mission(
                                planner::TrajectoryPlanner& planner,
                                const dynamics::VehicleStateDynamic& vehicle_state_dynamic,  
                                const map::Route& route,
                                const dynamics::TrafficParticipantSet& traffic_participants,
                                const std::map<size_t, adore_ros2_msgs::msg::TrafficSignal>& traffic_signals,
                                const std::optional<adore_ros2_msgs::msg::Weather>& weather,
                                const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance
                           )
    {
        // Go through route, and update speed at points based on traffic signal positions
        auto route_with_signal = route;
        for( auto& p : route_with_signal.reference_line )
        {
            if( std::any_of( traffic_signals.begin(), traffic_signals.end(), [&]( const auto& s ) {
                return adore::math::distance_2d( s.second, p.second ) < 3.0 && s.second.state != adore_ros2_msgs::msg::TrafficSignal::GREEN;
                } ) )
            p.second.max_speed = 0;
        }

        if ( weather.has_value() )
        {
            // //Debug
            //     RCLCPP_INFO(
            //         rclcpp::get_logger( "Behaviors" ),
            //         "Current weather: wind_intensity=%.2f, wetness=%.2f",
            //         weather.value().wind_intensity,
            //         weather.value().wetness );

            if ( weather.value().wind_intensity > 2 )
            {
                dynamics::ComfortSettings custom_comfort_settings;
                custom_comfort_settings.max_speed = 5.5; // 20 km/h
            
                dynamics::Trajectory trajectory = planner.plan_route_trajectory_with_custom_comfort_settings( route_with_signal, vehicle_state_dynamic, traffic_participants, custom_comfort_settings );
                trajectory.adjust_start_time( vehicle_state_dynamic.time );
                trajectory.label              = "driving mission (carefully due to wind)";

                Behavior trajectory_and_signal;
                trajectory_and_signal.trajectory = dynamics::conversions::to_ros_msg( trajectory );

                return trajectory_and_signal;
            }

            if ( weather.value().wetness > 20 )
            {
                dynamics::ComfortSettings custom_comfort_settings;
                custom_comfort_settings.max_speed = 5.5; // 20 km/h
            
                dynamics::Trajectory trajectory = planner.plan_route_trajectory_with_custom_comfort_settings( route_with_signal, vehicle_state_dynamic, traffic_participants, custom_comfort_settings );
                trajectory.adjust_start_time( vehicle_state_dynamic.time );
                trajectory.label              = "driving mission (carefully due to rain)";

                Behavior trajectory_and_signal;
                trajectory_and_signal.trajectory = dynamics::conversions::to_ros_msg( trajectory );

                return trajectory_and_signal;
            }
        }


    
            
        const auto oa_result = ::adore::planner::try_plan_obstacle_avoidance(
        planner,
        route_with_signal,
        vehicle_state_dynamic,
        traffic_participants,
        params_for_obstacle_avoidance );

        //Debug oa_result
        RCLCPP_INFO(
            rclcpp::get_logger( "Behaviors" ),
            "Obstacle avoidance result: success=%s, mode=%d, reason=%s",
            oa_result.success ? "true" : "false",
            static_cast<int>( oa_result.mode ),
            oa_result.reason.c_str() );

        if( oa_result.success )
        {
        auto trajectory = oa_result.trajectory;
        trajectory.adjust_start_time( vehicle_state_dynamic.time );

        Behavior trajectory_and_signal;
        trajectory_and_signal.trajectory = dynamics::conversions::to_ros_msg( trajectory );
        trajectory_and_signal.modified_route = map::conversions::to_ros_msg( oa_result.modified_route );
        return trajectory_and_signal;
        }
    

    dynamics::Trajectory trajectory = planner.plan_route_trajectory( route_with_signal, vehicle_state_dynamic, traffic_participants );
            trajectory.adjust_start_time( vehicle_state_dynamic.time );
            trajectory.label              = "driving mission";

        Behavior trajectory_and_signal;
        trajectory_and_signal.trajectory = dynamics::conversions::to_ros_msg( trajectory );

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
}
}

