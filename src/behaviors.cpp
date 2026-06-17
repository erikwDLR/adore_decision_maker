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
#include "behaviors_obstacle_avoidance.hpp"
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

            Behavior driving_mission(
            planner::TrajectoryPlanner& planner,
            const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
            const map::Route& route,
            const dynamics::TrafficParticipantSet& traffic_participants,
            const adore_ros2_msgs::msg::TrafficSignals& traffic_signals,
            const std::optional<adore_ros2_msgs::msg::Weather>& weather,
            const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance,
            planner::ActiveAvoidanceState& active_avoidance_state )
        {
            // Convert the live traffic signals and let the planner bake the
            // resulting stop behaviour into the route once; the modified route
            // is then threaded through the obstacle-avoidance helpers below.
            dynamics::TrafficSignalSet traffic_signal_set = dynamics::conversions::to_cpp_type( traffic_signals );
            auto route_with_signal = planner.compute_traffic_light_behavior( vehicle_state_dynamic, route, traffic_signal_set );

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
                return continue_active_avoidance(
                    planner,
                    vehicle_state_dynamic,
                    route_with_signal,
                    traffic_participants,
                    traffic_signal_set,
                    params_for_obstacle_avoidance,
                    use_weather_comfort_settings,
                    weather_comfort_settings,
                    weather_label,
                    active_avoidance_state );
            }

            if( auto ego_lane_behavior =
                    try_ego_lane_oncoming_stop_behavior(
                        planner,
                        route_with_signal,
                        vehicle_state_dynamic,
                        traffic_participants,
                        params_for_obstacle_avoidance );
                ego_lane_behavior.has_value() )
            {
                return ego_lane_behavior.value();
            }

            if( use_weather_comfort_settings )
            {
                return plan_weather_behavior(
                    planner,
                    route_with_signal,
                    vehicle_state_dynamic,
                    traffic_participants,
                    params_for_obstacle_avoidance,
                    weather_comfort_settings,
                    weather_label );
            }

            return plan_obstacle_avoidance_behavior(
                planner,
                route_with_signal,
                vehicle_state_dynamic,
                traffic_participants,
                params_for_obstacle_avoidance,
                active_avoidance_state );
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
