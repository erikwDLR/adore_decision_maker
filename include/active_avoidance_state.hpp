#pragma once

// ActiveAvoidanceState now lives in the adore_planning library so the
// obstacle-avoidance maneuver state and its pure operations can be shared and
// unit-tested without ROS. This shim preserves the historical
// adore::behavior::ActiveAvoidanceState name used across the decision_maker node.

#include <planning/active_avoidance_state.hpp>

namespace adore
{
namespace behavior
{

using planner::ActiveAvoidanceState;

} // namespace behavior
} // namespace adore
