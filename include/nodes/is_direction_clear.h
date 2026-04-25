/**
 * @file is_direction_clear.h
 * @brief Condition node that checks whether the resultant force direction is clear
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <raylib.h>
#include <array>

#include "simulation.h"

/**
 * @brief Returns SUCCESS if the cell in the resultant force direction is not blocked
 *
 * Snaps the Potential Fields resultant force to the nearest compass direction
 * and checks whether that cell is free according to sensor readings.
 *
 * Reads from Blackboard:
 * - "resultant_force"   : Vector2
 * - "neighbors_blocked" : std::array<bool, 8>
 */
class IsDirectionClear : public BT::ConditionNode
{
public:
    /**
     * @brief Constructor
     * @param name   Node name used in the BT tree
     * @param config Node configuration provided by BehaviorTree.CPP
     */
    IsDirectionClear(const std::string& name, const BT::NodeConfig& config);

    /**
     * @brief Declares the Blackboard ports used by this node
     * @return List of input ports
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Checks whether the resultant force direction is unobstructed
     * @return SUCCESS if direction is clear, FAILURE otherwise
     */
    BT::NodeStatus tick() override;
};