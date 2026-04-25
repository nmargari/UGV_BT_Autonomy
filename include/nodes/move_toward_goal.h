/**
 * @file move_toward_goal.h
 * @brief Action node that moves the robot toward the goal using Potential Fields
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <raylib.h>

#include "simulation.h"

/**
 * @brief Moves the robot one step in the direction of the resultant force
 *
 * Uses the precomputed Potential Fields resultant force from the Blackboard.
 * Always returns RUNNING — the BT tree decides when to stop via IsGoalReached.
 *
 * Reads from Blackboard:
 * - "resultant_force" : Vector2
 *
 * Writes to Robot directly via Simulation reference.
 */
class MoveTowardGoal : public BT::StatefulActionNode
{
public:
    /**
     * @brief Constructor
     * @param name       Node name used in the BT tree
     * @param config     Node configuration provided by BehaviorTree.CPP
     * @param simulation Reference to the simulation for robot movement
     */
    MoveTowardGoal(const std::string& name, const BT::NodeConfig& config, Simulation& simulation);

    /**
     * @brief Declares the Blackboard ports used by this node
     * @return List of input ports
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Called once when the node is first activated
     * @return RUNNING to begin movement
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Moves the robot one step in the resultant force direction
     * @return Always RUNNING
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called when the node is halted externally
     */
    void onHalted() override;

private:
    Simulation& simulation_; ///< Reference to the simulation for robot movement
};