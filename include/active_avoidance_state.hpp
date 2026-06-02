#pragma once
#include <limits>
#include <cmath>
#include <vector>

#include <adore_map/route.hpp>
#include <planning/obstacle_avoidance.hpp>

namespace adore
{
namespace behavior
{

struct ActiveAvoidanceState
{
  bool active = false;

  // The avoidance route selected for the active maneuver. Temporary
  // braking/waiting stop profiles are built on copies so the maneuver can
  // continue once a transient conflict clears.
  adore::map::Route base_modified_route;
  adore::map::Route modified_route;

  int obstacle_id = -1;
  std::vector<int> obstacle_ids;

  double shift_start_s = 0.0;
  double shift_end_s = 0.0;
  double release_s = 0.0;

  double lateral_shift = 0.0;
  bool in_lane = false;

  adore::planner::ObstacleAvoidanceManeuver maneuver;

  // Geometry/time memory for obstacles that may disappear from perception while
  // ego is still passing them or while they conflict with the active route.
  std::vector<adore::planner::ObstacleGhostEnvelope> ghost_memory;

  // Last valid projection on the active modified route. This keeps progress
  // monotonic while the route is laterally offset from the mission route.
  double last_modified_s = std::numeric_limits<double>::quiet_NaN();

  void reset()
  {
    active = false;
    base_modified_route = adore::map::Route{};
    modified_route = adore::map::Route{};

    obstacle_id = -1;
    obstacle_ids.clear();

    shift_start_s = 0.0;
    shift_end_s = 0.0;
    release_s = 0.0;

    lateral_shift = 0.0;
    in_lane = false;

    maneuver = adore::planner::ObstacleAvoidanceManeuver{};
    ghost_memory.clear();

    last_modified_s = std::numeric_limits<double>::quiet_NaN();
  }
};

} // namespace behavior
} // namespace adore
