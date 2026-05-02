/**
 * @file is_direction_clear.h
 * @brief Condition node that checks whether the resultant force direction is walkable
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "simulation.h"

/**
 * @brief Returns SUCCESS if the cell in the resultant force direction is walkable
 */
class IsDirectionClear : public BT::ConditionNode
{
public:
    /**
     * @brief Constructor
     * @param name       Node name
     * @param config     Node configuration
     * @param simulation Reference to the simulation
     */
    IsDirectionClear(const std::string& name, const BT::NodeConfig& config, Simulation& simulation);

    /**
     * @brief No ports — reads directly from simulation
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Checks whether the resultant force direction is unobstructed
     * @return SUCCESS if direction is clear, FAILURE otherwise
     */
    BT::NodeStatus tick() override;

private:
    Simulation& simulation_; ///< Reference to the simulation
};