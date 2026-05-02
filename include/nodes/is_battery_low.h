/**
 * @file is_battery_low.h
 * @brief Condition node that checks whether the robot battery is low
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "simulation.h"

/**
 * @brief Returns SUCCESS if battery is below Config::BATTERY_LOW
 */
class IsBatteryLow : public BT::ConditionNode
{
public:
    /**
     * @brief Constructor
     * @param name       Node name
     * @param config     Node configuration
     * @param simulation Reference to the simulation
     */
    IsBatteryLow(const std::string& name, const BT::NodeConfig& config, Simulation& simulation);

    /**
     * @brief No ports — reads directly from simulation
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Checks battery level
     * @return SUCCESS if battery is low, FAILURE otherwise
     */
    BT::NodeStatus tick() override;

private:
    Simulation& simulation_; ///< Reference to the simulation
};