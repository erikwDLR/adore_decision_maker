#pragma once
#include <limits>
#include <cmath>

#include <adore_map/route.hpp>

namespace adore
{
namespace behavior
{

struct ObstacleAvoidanceLock
{
  bool active = false;

  adore::map::Route modified_route;

  int obstacle_id = -1;

  double shift_start_s = 0.0;
  double shift_end_s = 0.0;
  double release_s = 0.0;

  double lateral_shift = 0.0;
  bool in_lane = false;

  double last_modified_s = std::numeric_limits<double>::quiet_NaN();

  void reset()
  {
    active = false;
    modified_route = adore::map::Route{};

    obstacle_id = -1;

    shift_start_s = 0.0;
    shift_end_s = 0.0;
    release_s = 0.0;

    lateral_shift = 0.0;
    in_lane = false;

    last_modified_s = std::numeric_limits<double>::quiet_NaN();
  }
};

} // namespace behavior
} // namespace adore