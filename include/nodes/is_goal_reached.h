/**
 * @file is_goal_reached.h
 * @brief Condition node that checks whether the robot has reached the goal
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <raylib.h>

/**
 * @brief Returns SUCCESS if the robot is within Config::GOAL_REACH_DIST of the goal
 *
 * Reads from Blackboard:
 * - "robot_position" : Vector2
 * - "goal_position"  : Vector2
 */
class IsGoalReached : public BT::ConditionNode
{
public:
    /**
     * @brief Constructor
     * @param name   Node name used in the BT tree
     * @param config Node configuration provided by BehaviorTree.CPP
     */
    IsGoalReached(const std::string& name, const BT::NodeConfig& config);

    /**
     * @brief Declares the Blackboard ports used by this node
     * @return List of input ports
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Checks the distance between robot and goal
     * @return SUCCESS if within Config::GOAL_REACH_DIST, FAILURE otherwise
     */
    BT::NodeStatus tick() override;
};