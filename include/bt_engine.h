/**
 * @file bt_engine.h
 * @brief Behavior Tree engine that builds and ticks the UGV decision tree
 *
 * Registers all BT nodes, builds the tree from XML, and exposes
 * a single tick() method called by the main loop every simulation step.
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "simulation.h"

/**
 * @brief Owns the BehaviorTree factory and tree instance
 *
 * All node registration and tree construction happens in the constructor.
 * The main loop calls tick() once per simulation step.
 */
class BTEngine
{
public:
    /**
     * @brief Constructs the BT engine, registers all nodes and builds the tree
     * @param blackboard Shared Blackboard instance
     * @param simulation Reference to the simulation for action node movement
     */
    BTEngine(BT::Blackboard::Ptr blackboard, Simulation& simulation);

    /**
     * @brief Ticks the behavior tree once
     *
     * Called every simulation step after simulation.update().
     * Reads sensor data from the Blackboard and drives robot movement.
     */
    void tick();

private:
    BT::BehaviorTreeFactory factory_;  ///< Factory used to register and create nodes
    BT::Tree tree_; ///< The active behavior tree instance

    /**
     * @brief Registers all condition and action nodes with the factory
     * @param simulation Reference to simulation passed to action node constructors
     */
    void registerNodes(Simulation& simulation);

    /**
     * @brief Builds the tree from the XML definition
     * @param blackboard Shared Blackboard instance
     */
    void buildTree(BT::Blackboard::Ptr blackboard);
};