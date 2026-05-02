/**
 * @file wall_follow.h
 * @brief Action node that follows a wall using the right-hand rule
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "simulation.h"

/**
 * @brief Follows the wall until the robot is no longer stuck
 *
 * Uses the right-hand rule: try right → forward → left → back.
 * Returns FAILURE when stuck_counter reaches zero, allowing
 * the BT to switch back to MoveTowardGoal.
 */
class WallFollow : public BT::StatefulActionNode
{
public:
    /**
     * @brief Constructor
     * @param name       Node name
     * @param config     Node configuration
     * @param simulation Reference to the simulation
     */
    WallFollow(const std::string& name, const BT::NodeConfig& config, Simulation& simulation);

    /**
     * @brief No ports — reads directly from simulation
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Called once when node activates — stores initial heading
     * @return RUNNING
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Executes one step of the wall following algorithm
     * @return RUNNING while following wall, FAILURE when no longer stuck
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called when node is halted externally
     */
    void onHalted() override;

private:
    Simulation& simulation_; ///< Reference to the simulation
    Direction   wall_dir_;   ///< Reference heading when wall following started
};