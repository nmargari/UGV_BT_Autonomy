/**
 * @file return_to_base.h
 * @brief Action node that drives the robot back to the start position
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <raylib.h>

#include "simulation.h"

/**
 * @brief Moves the robot toward the start position using the resultant force
 *
 * Temporarily overrides the attractive force target from goal to start.
 * Returns SUCCESS when the robot reaches the start position,
 * RUNNING while still moving toward it.
 *
 * Reads from Blackboard:
 * - "robot_position" : Vector2
 * - "base_position"  : Vector2
 *
 * Writes to Robot directly via Simulation reference.
 */
class ReturnToBase : public BT::StatefulActionNode
{
public:
    /**
     * @brief Constructor
     * @param name Node name used in the BT tree
     * @param config Node configuration provided by BehaviorTree.CPP
     * @param simulation Reference to the simulation for robot movement
     */
    ReturnToBase(const std::string& name, const BT::NodeConfig& config, Simulation& simulation);

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
     * @brief Called every tick while the node is active
     * @return RUNNING while moving, SUCCESS when base is reached
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called when the node is halted externally
     */
    void onHalted() override;

private:
    Simulation& simulation_; ///< Reference to the simulation for robot movement
};