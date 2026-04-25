/**
 * @file is_direction_clear.cpp
 * @brief Implementation of the IsDirectionClear condition node
 */

#include "nodes/is_direction_clear.h"

IsDirectionClear::IsDirectionClear(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{}

BT::PortsList IsDirectionClear::providedPorts()
{
    return
    {
        BT::InputPort<Vector2>("resultant_force"),
        BT::InputPort<std::array<bool, 8>>("neighbors_blocked")
    };
}

BT::NodeStatus IsDirectionClear::tick()
{
    Vector2 force   = getInput<Vector2>("resultant_force").value();
    auto    blocked = getInput<std::array<bool, 8>>("neighbors_blocked").value();

    Direction dir   = snapToDirection(force);
    int       index = static_cast<int>(dir);

    if (!blocked[index])
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}