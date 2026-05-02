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
    return {};
}

BT::NodeStatus ReturnToBase::onStart()
{
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReturnToBase::onRunning()
{
    Robot&       robot = simulation_.getRobot();
    const World& world = simulation_.getWorld();

    float dx   = robot.position.x - world.start.x;
    float dy   = robot.position.y - world.start.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= Config::GOAL_REACH_DIST)
    {
        return BT::NodeStatus::SUCCESS;
    }

    // Attractive force toward base only
    Vector2 f_att =
    {
        Config::K_ATT * (world.start.x - robot.position.x),
        Config::K_ATT * (world.start.y - robot.position.y)
    };

    Direction dir = snapToDirection(f_att);

    int d  = static_cast<int>(dir);
    int nx = static_cast<int>(robot.getCell().x) + DIRECTION_DX[d];
    int ny = static_cast<int>(robot.getCell().y) + DIRECTION_DY[d];

    if (world.isWalkable(nx, ny))
    {
        robot.move(dir, 1.0f / Config::FPS_TARGET);
    }

    return BT::NodeStatus::RUNNING;
}

void ReturnToBase::onHalted()
{
}