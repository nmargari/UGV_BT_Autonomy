/**
 * @file is_battery_low.h
 * @brief Condition node that checks whether the robot battery is low
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

/**
 * @brief Returns SUCCESS if battery_level is below Config::BATTERY_LOW
 *
 * Reads from Blackboard:
 * - "battery_level" : float
 */
class IsBatteryLow : public BT::ConditionNode
{
public:
    /**
     * @brief Constructor
     * @param name Node name used in the BT tree
     * @param config Node configuration provided by BehaviorTree.CPP
     */
    IsBatteryLow(const std::string& name, const BT::NodeConfig& config);

    /**
     * @brief Declares the Blackboard ports used by this node
     * @return List of input ports
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Checks the battery level against Config::BATTERY_LOW
     * @return SUCCESS if battery is low, FAILURE otherwise
     */
    BT::NodeStatus tick() override;
};