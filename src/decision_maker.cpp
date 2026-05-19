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

#include "decision_maker.hpp"
#include "adore_ros2_msgs/msg/odd.hpp"
#include "adore_ros2_msgs/msg/traffic_participant.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "behaviors.hpp"
#include "conditions.hpp"

namespace adore
{

adore::planner::ObstacleAvoidanceParams
load_obstacle_avoidance_params( rclcpp::Node& node )
{
  adore::planner::ObstacleAvoidanceParams params;

  params.max_static_object_speed = node.declare_parameter<double>( "obstacle_avoidance.max_static_object_speed", params.max_static_object_speed );
  params.max_object_ahead = node.declare_parameter<double>( "obstacle_avoidance.max_object_ahead", params.max_object_ahead );
  params.min_obstacle_route_overlap = node.declare_parameter<double>( "obstacle_avoidance.min_obstacle_route_overlap", params.min_obstacle_route_overlap );
  params.max_projection_distance_from_route = node.declare_parameter<double>( "obstacle_avoidance.max_projection_distance_from_route", params.max_projection_distance_from_route );
  params.max_object_lateral_distance = node.declare_parameter<double>( "obstacle_avoidance.max_object_lateral_distance", params.max_object_lateral_distance );
  params.ego_corridor_safety_margin = node.declare_parameter<double>( "obstacle_avoidance.ego_corridor_safety_margin", params.ego_corridor_safety_margin );
  params.side_clearance = node.declare_parameter<double>( "obstacle_avoidance.side_clearance", params.side_clearance );
  params.front_clearance = node.declare_parameter<double>( "obstacle_avoidance.front_clearance", params.front_clearance );
  params.rear_clearance = node.declare_parameter<double>( "obstacle_avoidance.rear_clearance", params.rear_clearance );

  params.entry_extra_distance = node.declare_parameter<double>( "obstacle_avoidance.entry_extra_distance", params.entry_extra_distance );
  params.return_extra_distance = node.declare_parameter<double>( "obstacle_avoidance.return_extra_distance", params.return_extra_distance );
  params.max_speed_during_avoidance = node.declare_parameter<double>( "obstacle_avoidance.max_speed_during_avoidance", params.max_speed_during_avoidance );
  params.stop_distance_before_obstacle = node.declare_parameter<double>( "obstacle_avoidance.stop_distance_before_obstacle", params.stop_distance_before_obstacle );
  params.stop_time_step = node.declare_parameter<double>( "obstacle_avoidance.stop_time_step", params.stop_time_step );
  params.oncoming_lookahead_after_obj = node.declare_parameter<double>( "obstacle_avoidance.oncoming_lookahead_after_obj", params.oncoming_lookahead_after_obj );
  params.min_oncoming_heading_diff = node.declare_parameter<double>( "obstacle_avoidance.min_oncoming_heading_diff", params.min_oncoming_heading_diff );

  params.prefer_left_shift = node.declare_parameter<bool>( "obstacle_avoidance.prefer_left_shift", params.prefer_left_shift );
  params.overtaking_allowed = node.declare_parameter<bool>( "obstacle_avoidance.overtaking_allowed", params.overtaking_allowed );
  params.enforce_drivable_area = node.declare_parameter<bool>( "obstacle_avoidance.enforce_drivable_area", params.enforce_drivable_area );

  params.drivable_area_margin = node.declare_parameter<double>( "obstacle_avoidance.drivable_area_margin", params.drivable_area_margin );
  params.lane_boundary_join_slack = node.declare_parameter<double>( "obstacle_avoidance.lane_boundary_join_slack", params.lane_boundary_join_slack );
  params.lane_s_overlap_slack = node.declare_parameter<double>( "obstacle_avoidance.lane_s_overlap_slack", params.lane_s_overlap_slack );

  params.allow_opposite_direction_lanes = node.declare_parameter<bool>( "obstacle_avoidance.allow_opposite_direction_lanes", params.allow_opposite_direction_lanes );
  params.obstacle_cluster_join_gap_s = node.declare_parameter<double>( "obstacle_avoidance.obstacle_cluster_join_gap_s", params.obstacle_cluster_join_gap_s );
  params.enable_multi_candidate_route_shift = node.declare_parameter<bool>( "obstacle_avoidance.enable_multi_candidate_route_shift", params.enable_multi_candidate_route_shift );
  params.lateral_candidate_extra_steps = node.declare_parameter<int>( "obstacle_avoidance.lateral_candidate_extra_steps", params.lateral_candidate_extra_steps );
  params.lateral_candidate_extra_step = node.declare_parameter<double>( "obstacle_avoidance.lateral_candidate_extra_step", params.lateral_candidate_extra_step );
  params.candidate_longitudinal_stretch_factor = node.declare_parameter<double>( "obstacle_avoidance.candidate_longitudinal_stretch_factor", params.candidate_longitudinal_stretch_factor );
  params.validate_shifted_trajectory = node.declare_parameter<bool>( "obstacle_avoidance.validate_shifted_trajectory", params.validate_shifted_trajectory );
  params.trajectory_validation_lateral_margin = node.declare_parameter<double>( "obstacle_avoidance.trajectory_validation_lateral_margin", params.trajectory_validation_lateral_margin );

  // Oncoming traffic gap-acceptance parameters (improved time-based check).
  params.oncoming_time_margin = node.declare_parameter<double>( "obstacle_avoidance.oncoming_time_margin", params.oncoming_time_margin );
  params.min_ego_speed_for_gap_check = node.declare_parameter<double>( "obstacle_avoidance.min_ego_speed_for_gap_check", params.min_ego_speed_for_gap_check );
  params.min_oncoming_speed_for_gap_check = node.declare_parameter<double>( "obstacle_avoidance.min_oncoming_speed_for_gap_check", params.min_oncoming_speed_for_gap_check );
  params.min_oncoming_route_speed = node.declare_parameter<double>( "obstacle_avoidance.min_oncoming_route_speed", params.min_oncoming_route_speed );
  params.prediction_time_horizon = node.declare_parameter<double>( "obstacle_avoidance.prediction_time_horizon", params.prediction_time_horizon );
  params.prediction_time_step = node.declare_parameter<double>( "obstacle_avoidance.prediction_time_step", params.prediction_time_step );
  params.oncoming_safety_distance_front = node.declare_parameter<double>( "obstacle_avoidance.oncoming_safety_distance_front", params.oncoming_safety_distance_front );
  params.oncoming_safety_distance_rear = node.declare_parameter<double>( "obstacle_avoidance.oncoming_safety_distance_rear", params.oncoming_safety_distance_rear );
  params.debug_oncoming_check = node.declare_parameter<bool>( "obstacle_avoidance.debug_oncoming_check", params.debug_oncoming_check );

  // Defensive stop behavior for vehicles moving against the ego route direction
  // on the ego lane/corridor.
  params.ego_lane_oncoming_stop_enabled = node.declare_parameter<bool>( "obstacle_avoidance.ego_lane_oncoming_stop_enabled", params.ego_lane_oncoming_stop_enabled );
  params.ego_lane_oncoming_max_distance = node.declare_parameter<double>( "obstacle_avoidance.ego_lane_oncoming_max_distance", params.ego_lane_oncoming_max_distance );
  params.ego_lane_oncoming_time_horizon = node.declare_parameter<double>( "obstacle_avoidance.ego_lane_oncoming_time_horizon", params.ego_lane_oncoming_time_horizon );
  params.ego_lane_oncoming_min_route_speed = node.declare_parameter<double>( "obstacle_avoidance.ego_lane_oncoming_min_route_speed", params.ego_lane_oncoming_min_route_speed );
  params.ego_lane_oncoming_lateral_margin = node.declare_parameter<double>( "obstacle_avoidance.ego_lane_oncoming_lateral_margin", params.ego_lane_oncoming_lateral_margin );
  params.ego_lane_oncoming_stop_distance = node.declare_parameter<double>( "obstacle_avoidance.ego_lane_oncoming_stop_distance", params.ego_lane_oncoming_stop_distance );

  RCLCPP_INFO(
    node.get_logger(),
    "[OA][params] max_speed=%.3f side_clearance=%.3f front=%.3f rear=%.3f "
    "entry_extra=%.3f return_extra=%.3f corridor_margin=%.3f "
    "overtaking_allowed=%s enforce_drivable_area=%s drivable_margin=%.3f boundary_slack=%.3f s_slack=%.3f opposite_lanes=%s "
    "obstacle_cluster_join_gap_s=%.3f",
    params.max_speed_during_avoidance,
    params.side_clearance,
    params.front_clearance,
    params.rear_clearance,
    params.entry_extra_distance,
    params.return_extra_distance,
    params.ego_corridor_safety_margin,
    params.overtaking_allowed ? "true" : "false",
    params.enforce_drivable_area ? "true" : "false",
    params.drivable_area_margin,
    params.lane_boundary_join_slack,
    params.lane_s_overlap_slack,
    params.allow_opposite_direction_lanes ? "true" : "false",
    params.obstacle_cluster_join_gap_s);

  return params;
}

DecisionMaker::DecisionMaker( const rclcpp::NodeOptions& opts ) :
  rclcpp::Node{ "decision_maker", opts }

{
  load_parameters();
  setup_subscribers();
  setup_publishers();
  obstacle_avoidance_params = load_obstacle_avoidance_params( *this );
}

void DecisionMaker::load_parameters()
{  
  v2x_id = declare_parameter( "v2x_id", v2x_id );

  std::vector<std::string> keys   = declare_parameter( "planner_settings_keys", std::vector<std::string>{} );
  std::vector<double>      values = declare_parameter( "planner_settings_values", std::vector<double>{} );

  std::map<std::string, double> planner_settings;
  if( keys.size() == values.size() )
  {
    for( size_t i = 0; i < keys.size(); ++i )
    {
      planner_settings[keys[i]] = values[i];
    }
  }

  std::string vehicle_model_file = declare_parameter( "vehicle_model_file", "" );

  auto vehicle_model    = std::make_shared<dynamics::PhysicalVehicleModel>( vehicle_model_file, false );
  auto comfort_settings = dynamics::ComfortSettings(); // default value comfort settings

  planner.set_vehicle_parameters( vehicle_model->params );
  planner.set_comfort_settings( comfort_settings );
  planner.set_parameters( planner_settings );
}

void DecisionMaker::setup_subscribers()
{
  timer = create_wall_timer( std::chrono::milliseconds( static_cast<int>( 100 ) ), // 10 Hz
                             std::bind( &DecisionMaker::timer_callback, this ) );

  subscriber_vehicle_state_dynamic = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>( "vehicle_state_dynamic", 1,
                                      [this](const adore_ros2_msgs::msg::VehicleStateDynamic& msg) {  latest_vehicle_state_dynamic = dynamics::conversions::to_cpp_type(msg); });

  subscriber_route = create_subscription<adore_ros2_msgs::msg::Route>( "route", 1,
                                      [this](const adore_ros2_msgs::msg::Route& msg) {  latest_route = map::conversions::to_cpp_type(msg); });

  subscriber_odd = create_subscription<adore_ros2_msgs::msg::Odd>( "odd", 1,
                                      [this](const adore_ros2_msgs::msg::Odd& msg) {  latest_odd = msg; });

  subscriber_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>( "traffic_participants", 1,
                                      [this](const adore_ros2_msgs::msg::TrafficParticipantSet& msg) 
                                      {  
                                        auto participants = dynamics::conversions::to_cpp_type(msg);
                                        for( const auto& [id, participant] : participants.participants )
                                        {
                                            traffic_participants.update_traffic_participants( participant );
                                        }

                                        double max_participant_age = 1.0;
                                        traffic_participants.remove_old_participants( max_participant_age, now().seconds() );
                                      });

  subscriber_v2x_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>( "/infrastructure/planned_traffic", 1,
                                      [this](const adore_ros2_msgs::msg::TrafficParticipantSet& msg) 
                                      {  
                                        auto participants = dynamics::conversions::to_cpp_type(msg);

                                        if ( participants.validity_area.has_value() )
                                        {
                                          latest_managed_zone = participants.validity_area;
                                        }
                                        
                                        for( const auto& [id, participant] : participants.participants )
                                        {
                                          if ( !participant.v2x_id.has_value() || v2x_id != participant.v2x_id.value())
                                          {
                                            traffic_participants.update_traffic_participants( participant );
                                            continue;
                                          }

                                          if ( participant.trajectory.has_value() )
                                          {
                                            latest_managed_trajectory = participant.trajectory; 
                                          }
                                        }

                                        double max_participant_age = 1.0;
                                        traffic_participants.remove_old_participants( max_participant_age, now().seconds() );
                                      });

  subscriber_reference_trajectory = create_subscription<adore_ros2_msgs::msg::Trajectory>( "reference_trajectory", 1,
                                      [this](const adore_ros2_msgs::msg::Trajectory& msg) { latest_reference_trajectory = dynamics::conversions::to_cpp_type(msg); });

  subscriber_traffic_signal = create_subscription<adore_ros2_msgs::msg::TrafficSignal>( "traffic_signals", 1,
                                      [this](const adore_ros2_msgs::msg::TrafficSignal& msg) { traffic_signals[msg.signal_group_id] = msg; });

  subscriber_safety_corridor = create_subscription<adore_ros2_msgs::msg::SafetyCorridor>( "safety_corridor", 1,
                                      [this](const adore_ros2_msgs::msg::SafetyCorridor& msg) { latest_safety_corridor = msg; });

  subscriber_caution_zones = create_subscription<adore_ros2_msgs::msg::CautionZone>( "caution_zones", 1,
                                      [this](const adore_ros2_msgs::msg::CautionZone& msg) {  caution_zones[msg.label] = math::conversions::to_cpp_type(msg.polygon); });

  subscriber_weather = create_subscription<adore_ros2_msgs::msg::Weather>( "weather", 1,
                                      [this](const adore_ros2_msgs::msg::Weather& msg) {  latest_weather = msg; });

  subscriber_remote_operator_drive_approval = create_subscription<std_msgs::msg::Bool>( "suggested_trajectory_accepted", 1,
                                      [this](const std_msgs::msg::Bool& msg) {  remote_operator_drive_approval = msg.data; });

  subscriber_suggested_remote_operator_trajectory = create_subscription<adore_ros2_msgs::msg::Trajectory>( "suggested_remote_operator_trajectory", 1,
                                      [this](const adore_ros2_msgs::msg::Trajectory& msg) { 

                                        if ( !latest_vehicle_state_dynamic.has_value() )
                                          return;

                                        suggested_remote_operator_trajectory = dynamics::conversions::to_cpp_type(msg); 
                                        suggested_remote_operator_trajectory.value().adjust_start_time( latest_vehicle_state_dynamic.value().time );
                                       });
}

void DecisionMaker::setup_publishers()
{
  publisher_trajectory_decision = create_publisher<adore_ros2_msgs::msg::Trajectory>( "trajectory_decision", 1 );
  publisher_alternative_trajectory_decision = create_publisher<adore_ros2_msgs::msg::Trajectory>( "alternative_trajectory_decision", 1 );
    publisher_modified_route = create_publisher<adore_ros2_msgs::msg::Route>( "modified_route", 1 );
  publisher_v2x_traffic_participant = create_publisher<adore_ros2_msgs::msg::TrafficParticipant>( "v2x_traffic_participant", 1 );
}

void DecisionMaker::timer_callback()
{
  auto behavior = choose_and_plan_driving_behavior();
  publisher_trajectory_decision->publish(behavior.trajectory);
    if ( behavior.modified_route.has_value() )
    {
        publisher_modified_route->publish( behavior.modified_route.value() );
    }

  if ( behavior.alternative_trajectory.has_value() )
  {
    publisher_alternative_trajectory_decision->publish(behavior.alternative_trajectory.value());
  }

  // @TODO, add publisher and behavior for signals

  publisher_v2x_traffic_participant->publish( make_default_participant() );

  // @TODO, add a cleanup step, that removes old caution zones and old suggested trajectories, old safety corridors
}

behavior::Behavior DecisionMaker::choose_and_plan_driving_behavior()
{
  double time_now = now().seconds();

  bool has_localization = conditions::has_localization(latest_vehicle_state_dynamic, time_now);
  bool has_mission = conditions::has_mission(latest_vehicle_state_dynamic, latest_route);
  bool needs_remote_operator_assitance = conditions::needs_remote_operator_assitance( latest_vehicle_state_dynamic, caution_zones ); 
  bool needs_to_avoid_safety_corridor = conditions::needs_to_avoid_safety_corridor(latest_vehicle_state_dynamic, latest_safety_corridor);
  bool can_drive_managed = conditions::can_drive_managed(latest_vehicle_state_dynamic, time_now, latest_managed_zone, latest_managed_trajectory);
  bool odd_conditions_satisfied = conditions::odd_conditions_satisfied(latest_odd, time_now);

  if (
    has_localization &&
    needs_to_avoid_safety_corridor
  )
  {
    return behavior::avoiding_safety_corridor(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                traffic_participants,
                                latest_safety_corridor.value()
    );
  }

  if (
      has_localization &&
      has_mission &&
      needs_remote_operator_assitance
    )
  {
    return behavior::remote_operations(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                latest_route.value(),
                                traffic_participants,
                                remote_operator_drive_approval,
                                suggested_remote_operator_trajectory
    );
  }

  if (
    has_localization &&
    has_mission &&
    odd_conditions_satisfied && 
    can_drive_managed

  )
  {
    return behavior::driving_mission_following_managed(
                  planner,
                  latest_vehicle_state_dynamic.value(),
                  latest_managed_trajectory.value(),
                  latest_managed_zone.value()
    );
  }

  if (
      has_localization &&
      has_mission &&
      odd_conditions_satisfied
  )
  {
    return behavior::driving_mission(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                latest_route.value(),
                                traffic_participants,
                                traffic_signals,
                                latest_weather,
                                obstacle_avoidance_params,
                                oa_maneuver_lock
                              );
  }

  if ( 
      has_localization &&
      odd_conditions_satisfied
  )
  {
    return behavior::waiting_for_mission(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                traffic_participants
                              );
  }

  if (
    has_localization &&
    has_mission
  )
  {
    return behavior::minimum_risk(
                                  planner, 
                                  latest_vehicle_state_dynamic.value(), 
                                  latest_route.value(), 
                                  traffic_participants,
                                  latest_odd
                              );
  }

  return behavior::emergency(planner, latest_vehicle_state_dynamic);
}

adore_ros2_msgs::msg::TrafficParticipant DecisionMaker::make_default_participant()
{
  dynamics::TrafficParticipant participant;
  if( latest_vehicle_state_dynamic.has_value() )
    participant.state = latest_vehicle_state_dynamic.value();

  if( latest_route.has_value() )
  {
    participant.goal_point = latest_route->destination;
    participant.route      = latest_route.value();
  }

  participant.id                  = v2x_id;
  participant.v2x_id              = v2x_id;
  participant.classification      = dynamics::CAR;
  participant.physical_parameters = planner.get_physical_vehicle_parameters();

  return dynamics::conversions::to_ros_msg( participant );
}

} // namespace adore

/* Register as component --------------------------------------------- */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::DecisionMaker )
