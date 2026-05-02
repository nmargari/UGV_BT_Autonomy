/**
 * @file return_to_base.h
 * @brief Action node that drives the robot back to the start position
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "simulation.h"

/**
 * @brief Moves the robot toward the start position
 *
 * Uses attractive force toward base only.
 * Returns SUCCESS when the robot reaches the start position.
 */
class ReturnToBase : public BT::StatefulActionNode
{
public:
    /**
     * @brief Constructor
     * @param name       Node name
     * @param config     Node configuration
     * @param simulation Reference to the simulation
     */
    ReturnToBase(const std::string& name, const BT::NodeConfig& config, Simulation& simulation);

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
     * @brief Moves robot one step toward base
     * @return RUNNING while moving, SUCCESS when base is reached
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called when node is halted externally
     */
    void onHalted() override;

private:
    Simulation& simulation_; ///< Reference to the simulation
};