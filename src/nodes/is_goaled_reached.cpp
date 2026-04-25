/**
 * @file is_goal_reached.cpp
 * @brief Implementation of the IsGoalReached condition node
 */

#include "nodes/is_goal_reached.h"
#include "config.h"

#include <cmath>

IsGoalReached::IsGoalReached(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{}

BT::PortsList IsGoalReached::providedPorts()
{
    return
    {
        BT::InputPort<Vector2>("robot_position"),
        BT::InputPort<Vector2>("goal_position")
    };
}

BT::NodeStatus IsGoalReached::tick()
{
    Vector2 pos  = getInput<Vector2>("robot_position").value();
    Vector2 goal = getInput<Vector2>("goal_position").value();

    float dx   = pos.x - goal.x;
    float dy   = pos.y - goal.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= Config::GOAL_REACH_DIST)
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}