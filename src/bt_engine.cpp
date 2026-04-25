/**
 * @file bt_engine.cpp
 * @brief Implementation of the BTEngine class
 */

#include "bt_engine.h"

#include "nodes/is_battery_low.h"
#include "nodes/is_goal_reached.h"
#include "nodes/is_direction_clear.h"
#include "nodes/return_to_base.h"
#include "nodes/move_toward_goal.h"
#include "nodes/wall_follow.h"

// ─────────────────────────────────────────────
//  XML tree definition
// ─────────────────────────────────────────────

static const char* BT_XML = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">

    <Selector>

      <!-- Emergency: return to base if battery is low -->
      <Sequence>
        <IsBatteryLow
          battery_level="{battery_level}"/>
        <ReturnToBase
          robot_position="{robot_position}"
          base_position="{base_position}"/>
      </Sequence>

      <!-- Navigate: move toward goal or follow wall if stuck -->
      <Sequence>
        <Selector>
          <Sequence>
            <IsDirectionClear
              resultant_force="{resultant_force}"
              neighbors_blocked="{neighbors_blocked}"/>
            <MoveTowardGoal
              resultant_force="{resultant_force}"/>
          </Sequence>
          <WallFollow
            neighbors_blocked="{neighbors_blocked}"
            compass_heading="{compass_heading}"
            is_stuck="{is_stuck}"/>
        </Selector>
        <IsGoalReached
          robot_position="{robot_position}"
          goal_position="{goal_position}"/>
      </Sequence>

    </Selector>
  </BehaviorTree>
</root>
)";

// ─────────────────────────────────────────────
//  BTEngine
// ─────────────────────────────────────────────

BTEngine::BTEngine(BT::Blackboard::Ptr blackboard, Simulation& simulation)
{
    registerNodes(simulation);
    buildTree(blackboard);
}

void BTEngine::tick()
{
    tree_.tickOnce();
}

void BTEngine::registerNodes(Simulation& simulation)
{
    // Condition nodes — simple registration
    factory_.registerNodeType<IsBatteryLow>("IsBatteryLow");
    factory_.registerNodeType<IsGoalReached>("IsGoalReached");
    factory_.registerNodeType<IsDirectionClear>("IsDirectionClear");

    // Action nodes — custom constructor requires lambda builder
    factory_.registerBuilder<ReturnToBase>(
        "ReturnToBase",
        [&simulation](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<ReturnToBase>(name, config, simulation);
        }
    );

    factory_.registerBuilder<MoveTowardGoal>(
        "MoveTowardGoal",
        [&simulation](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<MoveTowardGoal>(name, config, simulation);
        }
    );

    factory_.registerBuilder<WallFollow>(
        "WallFollow",
        [&simulation](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<WallFollow>(name, config, simulation);
        }
    );
}

void BTEngine::buildTree(BT::Blackboard::Ptr blackboard)
{
    tree_ = factory_.createTreeFromText(BT_XML, blackboard);
}