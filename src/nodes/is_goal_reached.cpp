/**
 * @file is_goal_reached.cpp
 * @brief Implementation of the IsGoalReached condition node
 */

#include "nodes/is_goal_reached.h"
#include "config.h"

#include <cmath>

IsGoalReached::IsGoalReached(
    const std::string&    name,
    const BT::NodeConfig& config,
    Simulation&           simulation)
    : BT::ConditionNode(name, config)
    , simulation_(simulation)
{}

BT::PortsList IsGoalReached::providedPorts()
{
    return {};
}

BT::NodeStatus IsGoalReached::tick()
{
    Vector2 pos  = simulation_.getRobot().position;
    Vector2 goal = simulation_.getWorld().goal;

    float dx   = pos.x - goal.x;
    float dy   = pos.y - goal.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= Config::GOAL_REACH_DIST)
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}