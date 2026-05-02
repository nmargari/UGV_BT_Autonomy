/**
 * @file is_direction_clear.cpp
 * @brief Implementation of the IsDirectionClear condition node
 */

#include "nodes/is_direction_clear.h"

IsDirectionClear::IsDirectionClear(
    const std::string&    name,
    const BT::NodeConfig& config,
    Simulation&           simulation)
    : BT::ConditionNode(name, config)
    , simulation_(simulation)
{}

BT::PortsList IsDirectionClear::providedPorts()
{
    return {};
}

BT::NodeStatus IsDirectionClear::tick()
{
    const Robot& robot = simulation_.getRobot();
    const World& world = simulation_.getWorld();

    Vector2   force = simulation_.getResultantForce();
    Direction dir   = snapToDirection(force);

    int d  = static_cast<int>(dir);
    int nx = static_cast<int>(robot.getCell().x) + DIRECTION_DX[d];
    int ny = static_cast<int>(robot.getCell().y) + DIRECTION_DY[d];

    if (world.isWalkable(nx, ny))
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}