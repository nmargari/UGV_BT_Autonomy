/**
 * @file move_toward_goal.cpp
 * @brief Implementation of the MoveTowardGoal action node
 */

#include "nodes/move_toward_goal.h"
#include "config.h"

MoveTowardGoal::MoveTowardGoal(
    const std::string&    name,
    const BT::NodeConfig& config,
    Simulation&           simulation)
    : BT::StatefulActionNode(name, config)
    , simulation_(simulation)
{}

BT::PortsList MoveTowardGoal::providedPorts()
{
    return { BT::InputPort<Vector2>("resultant_force") };
}

BT::NodeStatus MoveTowardGoal::onStart()
{
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveTowardGoal::onRunning()
{
    Vector2   force = getInput<Vector2>("resultant_force").value();
    Direction dir   = snapToDirection(force);

    simulation_.getRobot().move(dir, 1.0f / Config::FPS_TARGET);

    // Stuck counter is managed by simulation.update() — not here
    return BT::NodeStatus::RUNNING;
}

void MoveTowardGoal::onHalted()
{
    // Nothing to clean up
}