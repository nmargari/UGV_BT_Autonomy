/**
 * @file wall_follow.cpp
 * @brief Implementation of the WallFollow action node
 */

#include "nodes/wall_follow.h"
#include "config.h"

WallFollow::WallFollow(
    const std::string& name,
    const BT::NodeConfig& config,
    Simulation& simulation)
    : BT::StatefulActionNode(name, config)
    , simulation_(simulation)
    , wall_dir_(Direction::N)
{}

BT::PortsList WallFollow::providedPorts()
{
    return {};
}

BT::NodeStatus WallFollow::onStart()
{
    Robot& robot = simulation_.getRobot();
    robot.wall_following = true;
    wall_dir_ = robot.compass;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WallFollow::onRunning()
{
    Robot&       robot = simulation_.getRobot();
    const World& world = simulation_.getWorld();

    // Exit when the resultant force direction is clear again
    Vector2   force    = simulation_.getResultantForce();
    Direction goal_dir = snapToDirection(force);
    int       goal_d   = static_cast<int>(goal_dir);
    int       goal_nx  = static_cast<int>(robot.getCell().x) + DIRECTION_DX[goal_d];
    int       goal_ny  = static_cast<int>(robot.getCell().y) + DIRECTION_DY[goal_d];

    if (world.isWalkable(goal_nx, goal_ny))
    {
        robot.wall_following = false;
        return BT::NodeStatus::FAILURE;
    }

    // Right-hand rule — wall_dir_ stays FIXED
    Direction candidates[4] =
    {
        rotateClockwise90(wall_dir_),
        wall_dir_,
        rotateCounterClockwise90(wall_dir_),
        rotateClockwise90(rotateClockwise90(wall_dir_))
    };

    for (Direction dir : candidates)
    {
        int d  = static_cast<int>(dir);
        int nx = static_cast<int>(robot.getCell().x) + DIRECTION_DX[d];
        int ny = static_cast<int>(robot.getCell().y) + DIRECTION_DY[d];

        if (world.isWalkable(nx, ny))
        {
            robot.move(dir, 1.0f / Config::FPS_TARGET);
            return BT::NodeStatus::RUNNING;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void WallFollow::onHalted()
{
    simulation_.getRobot().wall_following = false;
}