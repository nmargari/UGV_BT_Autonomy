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
    return {};
}

BT::NodeStatus MoveTowardGoal::onStart()
{
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveTowardGoal::onRunning()
{
    Robot&       robot = simulation_.getRobot();
    const World& world = simulation_.getWorld();

    Vector2   force = simulation_.getResultantForce();
    Direction dir   = snapToDirection(force);

    int d  = static_cast<int>(dir);
    int nx = static_cast<int>(robot.getCell().x) + DIRECTION_DX[d];
    int ny = static_cast<int>(robot.getCell().y) + DIRECTION_DY[d];

    if (!world.isWalkable(nx, ny))
    {
        // Blocked — increment stuck counter and signal FAILURE
        // so the Fallback activates WallFollow
        robot.stuck_counter++;
        return BT::NodeStatus::FAILURE;
    }

    robot.move(dir, 1.0f / Config::FPS_TARGET);
    robot.stuck_counter = 0;
    return BT::NodeStatus::RUNNING;
}

void MoveTowardGoal::onHalted()
{
}