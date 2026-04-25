/**
 * @file wall_follow.h
 * @brief Action node that follows a wall using the right-hand (or left-hand) rule
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <array>

#include "simulation.h"

/**
 * @brief Follows the wall on the configured side until the robot is no longer stuck
 *
 * Uses the robot's compass heading to determine which cell is to the right
 * (or left, based on Config::WALL_FOLLOW_RIGHT). Implements the wall
 * following algorithm:
 * 1. If the side cell is free, turn toward it and move
 * 2. Else if the front cell is free, move forward
 * 3. Else turn away from the wall
 *
 * Returns RUNNING while following the wall.
 * Returns FAILURE when is_stuck becomes false, allowing the BT to switch
 * back to MoveTowardGoal.
 *
 * Reads from Blackboard:
 * - "neighbors_blocked" : std::array<bool, 8>
 * - "compass_heading"   : Direction
 * - "is_stuck"          : bool
 *
 * Writes to Robot directly via Simulation reference.
 */
class WallFollow : public BT::StatefulActionNode
{
public:
    /**
     * @brief Constructor
     * @param name       Node name used in the BT tree
     * @param config     Node configuration provided by BehaviorTree.CPP
     * @param simulation Reference to the simulation for robot movement
     */
    WallFollow(const std::string& name, const BT::NodeConfig& config, Simulation& simulation);

    /**
     * @brief Declares the Blackboard ports used by this node
     * @return List of input ports
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Called once when the node is first activated
     * @return RUNNING to begin wall following
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Executes one step of the wall following algorithm
     * @return RUNNING while following wall, FAILURE when no longer stuck
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called when the node is halted externally
     */
    void onHalted() override;

private:
    Simulation& simulation_; ///< Reference to the simulation for robot movement

    /**
     * @brief Returns the direction to the right of the given heading
     * @param heading Current facing direction
     * @return Direction 90 degrees clockwise from heading
     */
    Direction rightOf(Direction heading) const;

    /**
     * @brief Returns the direction to the left of the given heading
     * @param heading Current facing direction
     * @return Direction 90 degrees counter-clockwise from heading
     */
    Direction leftOf(Direction heading) const;
};