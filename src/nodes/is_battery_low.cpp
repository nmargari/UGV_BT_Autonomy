/**
 * @file is_battery_low.cpp
 * @brief Implementation of the IsBatteryLow condition node
 */

#include "nodes/is_battery_low.h"
#include "config.h"

IsBatteryLow::IsBatteryLow(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{}

BT::PortsList IsBatteryLow::providedPorts()
{
    return { BT::InputPort<float>("battery_level") };
}

BT::NodeStatus IsBatteryLow::tick()
{
    float battery = getInput<float>("battery_level").value();

    if (battery < Config::BATTERY_LOW)
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}