/**
 * @file wall_follow.cpp
 * @brief Implementation of the WallFollow action node
 */

#include "nodes/wall_follow.h"
#include "config.h"

WallFollow::WallFollow(
    const std::string&    name,
    const BT::NodeConfig& config,
    Simulation&           simulation)
    : BT::StatefulActionNode(name, config)
    , simulation_(simulation)
{}

BT::PortsList WallFollow::providedPorts()
{
    return
    {
        BT::InputPort<std::array<bool, 8>>("neighbors_blocked"),
        BT::InputPort<Direction>("compass_heading"),
        BT::InputPort<bool>("is_stuck")
    };
}

BT::NodeStatus WallFollow::onStart()
{
    simulation_.getRobot().wall_following = true;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WallFollow::onRunning()
{
    bool is_stuck = getInput<bool>("is_stuck").value();

    // Return FAILURE to hand control back to MoveTowardGoal
    if (!is_stuck)
    {
        simulation_.getRobot().wall_following = false;
        simulation_.getRobot().stuck_counter  = 0;
        return BT::NodeStatus::FAILURE;
    }

    auto      blocked = getInput<std::array<bool, 8>>("neighbors_blocked").value();
    Direction heading = getInput<Direction>("compass_heading").value();

    // Determine wall side direction
    Direction side = Config::WALL_FOLLOW_RIGHT ? rightOf(heading) : leftOf(heading);
    int       side_index  = static_cast<int>(side);
    int       front_index = static_cast<int>(heading);

    Robot& robot = simulation_.getRobot();

    if (!blocked[side_index])
    {
        // Side is free — turn toward it and move
        robot.move(side, 1.0f / Config::FPS_TARGET);
    }
    else if (!blocked[front_index])
    {
        // Side is blocked but front is free — move forward
        robot.move(heading, 1.0f / Config::FPS_TARGET);
    }
    else
    {
        // Both side and front are blocked — turn away from wall
        Direction away = Config::WALL_FOLLOW_RIGHT ? leftOf(heading) : rightOf(heading);
        robot.setCompass(away);
    }

    return BT::NodeStatus::RUNNING;
}

void WallFollow::onHalted()
{
    simulation_.getRobot().wall_following = false;
}

Direction WallFollow::rightOf(Direction heading) const
{
    return rotateClockwise90(heading);
}

Direction WallFollow::leftOf(Direction heading) const
{
    return rotateCounterClockwise90(heading);
}