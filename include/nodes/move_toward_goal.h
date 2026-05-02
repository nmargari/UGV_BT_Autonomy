/**
 * @file move_toward_goal.h
 * @brief Action node that moves the robot toward the goal using Potential Fields
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "simulation.h"

/**
 * @brief Moves the robot one step in the resultant force direction
 *
 * Returns FAILURE if the direction is blocked so the Fallback
 * node can activate WallFollow.
 */
class MoveTowardGoal : public BT::StatefulActionNode
{
public:
    /**
     * @brief Constructor
     * @param name       Node name
     * @param config     Node configuration
     * @param simulation Reference to the simulation
     */
    MoveTowardGoal(const std::string& name, const BT::NodeConfig& config, Simulation& simulation);

    /**
     * @brief No ports — reads directly from simulation
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Called once when node activates
     * @return RUNNING
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Moves the robot one step in the resultant force direction
     * @return RUNNING on success, FAILURE if direction is blocked
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called when node is halted externally
     */
    void onHalted() override;

private:
    Simulation& simulation_; ///< Reference to the simulation
};