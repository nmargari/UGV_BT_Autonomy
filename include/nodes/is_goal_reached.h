/**
 * @file is_goal_reached.h
 * @brief Condition node that checks whether the robot has reached the goal
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "simulation.h"

/**
 * @brief Returns SUCCESS if robot is within Config::GOAL_REACH_DIST of the goal
 */
class IsGoalReached : public BT::ConditionNode
{
public:
    /**
     * @brief Constructor
     * @param name       Node name
     * @param config     Node configuration
     * @param simulation Reference to the simulation
     */
    IsGoalReached(const std::string& name, const BT::NodeConfig& config, Simulation& simulation);

    /**
     * @brief No ports — reads directly from simulation
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Checks distance between robot and goal
     * @return SUCCESS if goal is reached, FAILURE otherwise
     */
    BT::NodeStatus tick() override;

private:
    Simulation& simulation_; ///< Reference to the simulation
};