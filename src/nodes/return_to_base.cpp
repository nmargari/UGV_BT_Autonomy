/**
 * @file return_to_base.cpp
 * @brief Implementation of the ReturnToBase action node
 */

#include "nodes/return_to_base.h"
#include "config.h"

#include <cmath>

ReturnToBase::ReturnToBase(
    const std::string&    name,
    const BT::NodeConfig& config,
    Simulation&           simulation)
    : BT::StatefulActionNode(name, config)
    , simulation_(simulation)
{}

BT::PortsList ReturnToBase::providedPorts()
{
    return
    {
        BT::InputPort<Vector2>("robot_position"),
        BT::InputPort<Vector2>("base_position")
    };
}

BT::NodeStatus ReturnToBase::onStart()
{
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReturnToBase::onRunning()
{
    Vector2 pos  = getInput<Vector2>("robot_position").value();
    Vector2 base = getInput<Vector2>("base_position").value();

    float dx   = pos.x - base.x;
    float dy   = pos.y - base.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    // Check if base is reached
    if (dist <= Config::GOAL_REACH_DIST)
    {
        return BT::NodeStatus::SUCCESS;
    }

    // Compute attractive force toward base
    Vector2 f_att =
    {
        Config::K_ATT * (base.x - pos.x),
        Config::K_ATT * (base.y - pos.y)
    };

    // Snap force to nearest compass direction and move
    Direction dir = snapToDirection(f_att);
    simulation_.getRobot().move(dir, 1.0f / Config::FPS_TARGET);

    return BT::NodeStatus::RUNNING;
}

void ReturnToBase::onHalted()
{
    // Nothing to clean up
}