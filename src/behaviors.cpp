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
            const planner::ObstacleAvoidanceParams& params_for_obstacle_avoidance,
            ObstacleAvoidanceLock& oa_maneuver_lock )
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
            // Persistent obstacle avoidance maneuver.
            //
            // If an OA maneuver is active, keep driving the stored modified route until
            // the ego vehicle has passed the release point. Do NOT call
            // try_plan_obstacle_avoidance() to decide whether an active maneuver should
            // continue. Obstacle detection may disappear during a successful avoidance.
            //
            // Opposite-lane maneuvers are monitored until the commitment point.
            // Before commitment, a new oncoming conflict aborts to a controlled
            // stop. After commitment, the vehicle keeps the locked route rather
            // than abruptly abandoning the return path.
            // -------------------------------------------------------------------------
            if( oa_maneuver_lock.active )
            {
                auto make_locked_stop_behavior =
                    [&]( const std::string& label ) -> Behavior
                    {
                        dynamics::Trajectory stop_trajectory;
                        stop_trajectory.label = label;

                        auto stop_state = vehicle_state_dynamic;
                        stop_state.vx = 0.0;
                        stop_state.time = vehicle_state_dynamic.time + 0.1;

                        stop_trajectory.states.push_back( vehicle_state_dynamic );
                        stop_trajectory.states.push_back( stop_state );
                        stop_trajectory.adjust_start_time( vehicle_state_dynamic.time );

                        Behavior trajectory_and_signal;
                        trajectory_and_signal.trajectory =
                            dynamics::conversions::to_ros_msg( stop_trajectory );
                        trajectory_and_signal.modified_route =
                            map::conversions::to_ros_msg( oa_maneuver_lock.modified_route );

                        return trajectory_and_signal;
                    };

                if( oa_maneuver_lock.maneuver.active &&
                    oa_maneuver_lock.maneuver.uses_opposite_lane )
                {
                    const auto monitor_result =
                        ::adore::planner::monitor_active_obstacle_avoidance_maneuver(
                            route_with_signal,
                            vehicle_state_dynamic,
                            traffic_participants,
                            oa_maneuver_lock.maneuver,
                            params_for_obstacle_avoidance );

                    if( monitor_result.should_abort_before_commitment )
                    {
                        RCLCPP_WARN(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][MONITOR][ABORT] obstacle_id=%d reason=%s",
                            oa_maneuver_lock.obstacle_id,
                            monitor_result.reason.c_str() );

                        auto stop_behavior = make_locked_stop_behavior(
                            "driving mission (OA abort before opposite-lane commitment: oncoming conflict)" );

                        oa_maneuver_lock.reset();
                        return stop_behavior;
                    }

                    if( !monitor_result.safe_to_continue )
                    {
                        RCLCPP_WARN(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][MONITOR][COMMITTED] obstacle_id=%d reason=%s",
                            oa_maneuver_lock.obstacle_id,
                            monitor_result.reason.c_str() );
                    }
                }

                // If the active OA maneuver is still an in-lane maneuver, the
                // ego vehicle remains in its own lane. An oncoming vehicle
                // intruding into that lane is therefore still a direct head-on
                // threat. Use the locked modified route for the stop profile so
                // we do not jump back to the original route.
                if( oa_maneuver_lock.in_lane )
                {
                    const auto ego_lane_oncoming_result =
                        ::adore::planner::try_plan_ego_lane_oncoming_stop(
                            planner,
                            oa_maneuver_lock.modified_route,
                            vehicle_state_dynamic,
                            traffic_participants,
                            params_for_obstacle_avoidance );

                    if( ego_lane_oncoming_result.success )
                    {
                        RCLCPP_INFO(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][LOCKED][EGO_LANE_ONCOMING] participant_id=%d distance=%.2f ttc=%.2f v_route=%.2f stop_s=%.2f reason=%s",
                            ego_lane_oncoming_result.participant_id,
                            ego_lane_oncoming_result.participant_distance_s,
                            ego_lane_oncoming_result.time_to_conflict,
                            ego_lane_oncoming_result.participant_route_speed,
                            ego_lane_oncoming_result.stop_s,
                            ego_lane_oncoming_result.reason.c_str() );

                        auto trajectory = ego_lane_oncoming_result.trajectory;
                        trajectory.adjust_start_time( vehicle_state_dynamic.time );

                        Behavior trajectory_and_signal;
                        trajectory_and_signal.trajectory =
                            dynamics::conversions::to_ros_msg( trajectory );

                        trajectory_and_signal.modified_route =
                            map::conversions::to_ros_msg( ego_lane_oncoming_result.modified_route );

                        return trajectory_and_signal;
                    }
                }

                const double ego_s_original =
                    route_with_signal.get_s( vehicle_state_dynamic );

                const double ego_s_modified_raw =
                    get_s_on_reference_line_segments(
                        oa_maneuver_lock.modified_route,
                        vehicle_state_dynamic,
                        ego_s_original,
                        20.0 );

                // Make ego_s_modified monotonic during active OA
                double ego_s_modified = ego_s_modified_raw;

                if( std::isfinite( ego_s_modified_raw ) &&
                    std::isfinite( oa_maneuver_lock.last_modified_s ) )
                {
                    // Enforce monotonic progression: never go backward
                    ego_s_modified =
                        std::max( ego_s_modified_raw, oa_maneuver_lock.last_modified_s );
                }
                else if( !std::isfinite( ego_s_modified_raw ) &&
                         std::isfinite( oa_maneuver_lock.last_modified_s ) )
                {
                    // Fallback: projection lost, use last valid value
                    ego_s_modified = oa_maneuver_lock.last_modified_s;
                    RCLCPP_WARN(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][WARN] ego_s_modified projection lost; using last_modified_s=%.2f",
                        oa_maneuver_lock.last_modified_s );
                }

                if( std::isfinite( ego_s_modified ) )
                {
                    oa_maneuver_lock.last_modified_s = ego_s_modified;
                }
                else
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][ERROR] ego_s_modified invalid and no valid fallback exists while OA active; commanding controlled stop" );

                    return make_locked_stop_behavior(
                        "driving mission (OA locked controlled stop: invalid modified-route projection)" );
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
                    ego_s_modified >= oa_maneuver_lock.release_s;

                const bool release_by_original_fallback =
                    std::isfinite( ego_s_original ) &&
                    ego_s_original >= oa_maneuver_lock.release_s &&
                    projection_consistent;

                if( release_by_modified || release_by_original_fallback )
                {
                    RCLCPP_INFO(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][FINISH] obstacle_id=%d ego_s_mod=%.2f ego_s_orig=%.2f release_s=%.2f by=%s returning_to_original_route",
                        oa_maneuver_lock.obstacle_id,
                        ego_s_modified,
                        ego_s_original,
                        oa_maneuver_lock.release_s,
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

                    oa_maneuver_lock.reset();

                    // now normal route
                    dynamics::Trajectory trajectory =
                        planner.plan_route_trajectory(
                            route_with_signal,
                            vehicle_state_dynamic,
                            traffic_participants );

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

                    if( use_weather_comfort_settings )
                    {
                        trajectory =
                            planner.plan_route_trajectory_with_custom_comfort_settings_from_s(
                                oa_maneuver_lock.modified_route,
                                vehicle_state_dynamic,
                                traffic_participants,
                                weather_comfort_settings,
                                ego_s_modified );

                        trajectory.label =
                            "driving mission (OA locked, " + weather_label + ")";
                    }
                    else
                    {
                        trajectory =
                            planner.plan_route_trajectory_from_s(
                                oa_maneuver_lock.modified_route,
                                vehicle_state_dynamic,
                                traffic_participants,
                                ego_s_modified );

                        trajectory.label = "driving mission (OA locked)";
                    }

                    if( trajectory.states.empty() )
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][ERROR] planner returned empty trajectory on locked modified_route; commanding controlled stop" );

                        return make_locked_stop_behavior(
                            "driving mission (OA locked controlled stop: planning failed)" );
                    }

                    trajectory.adjust_start_time( vehicle_state_dynamic.time );

                    Behavior trajectory_and_signal;
                    trajectory_and_signal.trajectory =
                        dynamics::conversions::to_ros_msg( trajectory );

                    trajectory_and_signal.modified_route =
                        map::conversions::to_ros_msg( oa_maneuver_lock.modified_route );

                    if( std::isfinite( ego_s_modified ) )
                    {
                        RCLCPP_INFO(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][CONTINUE] obstacle_id=%d ego_xy=(%.3f, %.3f) v=%.3f "
                            "ego_s_mod=%.2f ego_s_orig=%.2f release_s=%.2f "
                            "remaining_mod=%.2f remaining_orig=%.2f",
                            oa_maneuver_lock.obstacle_id,
                            vehicle_state_dynamic.x,
                            vehicle_state_dynamic.y,
                            vehicle_state_dynamic.vx,
                            ego_s_modified,
                            ego_s_original,
                            oa_maneuver_lock.release_s,
                            oa_maneuver_lock.release_s - ego_s_modified,
                            oa_maneuver_lock.release_s - ego_s_original );
                    }
                    else
                    {
                        RCLCPP_WARN(
                            rclcpp::get_logger( "Behaviors" ),
                            "[OA][CONTINUE] ego_s_modified invalid while OA active; continuing modified_route" );
                    }

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
            // Run this before weather/normal/OA planning when no OA lock is active.
            // -------------------------------------------------------------------------
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
            // active. If OA is active, weather speed reduction is applied to the locked
            // modified route above.
            // -------------------------------------------------------------------------
            if( use_weather_comfort_settings )
            {
                dynamics::Trajectory trajectory =
                    planner.plan_route_trajectory_with_custom_comfort_settings(
                        route_with_signal,
                        vehicle_state_dynamic,
                        traffic_participants,
                        weather_comfort_settings );

                trajectory.adjust_start_time( vehicle_state_dynamic.time );
                trajectory.label = "driving mission (" + weather_label + ")";

                Behavior trajectory_and_signal;
                trajectory_and_signal.trajectory =
                    dynamics::conversions::to_ros_msg( trajectory );

                return trajectory_and_signal;
            }

            // -------------------------------------------------------------------------
            // Try obstacle avoidance only if no maneuver is currently locked.
            // -------------------------------------------------------------------------
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
                // Only lock real avoidance maneuvers.
                // StopBeforeObstacle / WaitForOncoming may also return success=true,
                // but they must not activate a persistent route lock.
                if( oa_result.has_maneuver_bounds )
                {
                    oa_maneuver_lock.active = true;
                    oa_maneuver_lock.modified_route = oa_result.modified_route;

                    oa_maneuver_lock.obstacle_id = oa_result.obstacle_id;

                    oa_maneuver_lock.shift_start_s = oa_result.shift_start_s;
                    oa_maneuver_lock.shift_end_s = oa_result.shift_end_s;
                    oa_maneuver_lock.release_s = oa_result.shift_end_s;

                    oa_maneuver_lock.lateral_shift = oa_result.lateral_shift;
                    oa_maneuver_lock.in_lane = oa_result.in_lane;
                    oa_maneuver_lock.maneuver = oa_result.maneuver;

                    // Reset last_modified_s when starting a new OA maneuver
                    oa_maneuver_lock.last_modified_s = std::numeric_limits<double>::quiet_NaN();

                    RCLCPP_INFO(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][START] obstacle_id=%d shift_start_s=%.2f shift_end_s=%.2f release_s=%.2f shift=%.2f in_lane=%s",
                        oa_maneuver_lock.obstacle_id,
                        oa_maneuver_lock.shift_start_s,
                        oa_maneuver_lock.shift_end_s,
                        oa_maneuver_lock.release_s,
                        oa_maneuver_lock.lateral_shift,
                        oa_maneuver_lock.in_lane ? "true" : "false" );
                }
                else
                {
                    RCLCPP_INFO(
                        rclcpp::get_logger( "Behaviors" ),
                        "[OA][NO_LOCK] using OA result without maneuver bounds: mode=%d reason=%s",
                        static_cast<int>( oa_result.mode ),
                        oa_result.reason.c_str() );
                }

                auto trajectory = oa_result.trajectory;
                trajectory.adjust_start_time( vehicle_state_dynamic.time );

                Behavior trajectory_and_signal;
                trajectory_and_signal.trajectory =
                    dynamics::conversions::to_ros_msg( trajectory );

                trajectory_and_signal.modified_route =
                    map::conversions::to_ros_msg( oa_result.modified_route );

                return trajectory_and_signal;
            }

            // -------------------------------------------------------------------------
            // Normal driving mission if no OA result is available.
            // -------------------------------------------------------------------------
            dynamics::Trajectory trajectory =
                planner.plan_route_trajectory(
                    route_with_signal,
                    vehicle_state_dynamic,
                    traffic_participants );

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
}
}
