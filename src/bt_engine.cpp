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
    <Fallback>
      <Sequence>
        <IsBatteryLow/>
        <ReturnToBase/>
      </Sequence>
      <Sequence>
        <Fallback>
          <Sequence>
            <IsDirectionClear/>
            <MoveTowardGoal/>
          </Sequence>
          <WallFollow/>
        </Fallback>
        <IsGoalReached/>
      </Sequence>
    </Fallback>
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
    factory_.registerBuilder<IsBatteryLow>(
        "IsBatteryLow",
        [&simulation](const std::string& name, const BT::NodeConfig& config)
        { return std::make_unique<IsBatteryLow>(name, config, simulation); }
    );

    factory_.registerBuilder<IsGoalReached>(
        "IsGoalReached",
        [&simulation](const std::string& name, const BT::NodeConfig& config)
        { return std::make_unique<IsGoalReached>(name, config, simulation); }
    );

    factory_.registerBuilder<IsDirectionClear>(
        "IsDirectionClear",
        [&simulation](const std::string& name, const BT::NodeConfig& config)
        { return std::make_unique<IsDirectionClear>(name, config, simulation); }
    );

    factory_.registerBuilder<MoveTowardGoal>(
        "MoveTowardGoal",
        [&simulation](const std::string& name, const BT::NodeConfig& config)
        { return std::make_unique<MoveTowardGoal>(name, config, simulation); }
    );

    factory_.registerBuilder<WallFollow>(
        "WallFollow",
        [&simulation](const std::string& name, const BT::NodeConfig& config)
        { return std::make_unique<WallFollow>(name, config, simulation); }
    );

    factory_.registerBuilder<ReturnToBase>(
        "ReturnToBase",
        [&simulation](const std::string& name, const BT::NodeConfig& config)
        { return std::make_unique<ReturnToBase>(name, config, simulation); }
    );
}

void BTEngine::buildTree(BT::Blackboard::Ptr blackboard)
{
    tree_ = factory_.createTreeFromText(BT_XML, blackboard);
}
