/**
 * @file is_battery_low.cpp
 * @brief Implementation of the IsBatteryLow condition node
 */

#include "nodes/is_battery_low.h"
#include "config.h"

IsBatteryLow::IsBatteryLow(
    const std::string&    name,
    const BT::NodeConfig& config,
    Simulation&           simulation)
    : BT::ConditionNode(name, config)
    , simulation_(simulation)
{}

BT::PortsList IsBatteryLow::providedPorts()
{
    return {};
}

BT::NodeStatus IsBatteryLow::tick()
{
    if (simulation_.getRobot().battery < Config::BATTERY_LOW)
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}