/**
 * @file main.cpp
 * @brief Entry point of the UGV Behavior Tree Simulator
 *
 * Initializes all components and runs the main simulation
 * loop using a fixed timestep for deterministic behavior.
 */

#include <raylib.h>
#include <behaviortree_cpp/bt_factory.h>
#include <array>

#include "config.h"
#include "simulation.h"
#include "bt_engine.h"
#include "renderer.h"

/**
 * @brief Application entry point
 *
 * Creates all components, initializes the Blackboard with
 * default values, and runs the main loop with a fixed timestep.
 *
 * @return 0 on normal termination
 */
int main()
{
    InitWindow(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, "UGV Behavior Tree Simulator");
    SetTargetFPS(Config::FPS_TARGET);

    auto blackboard = BT::Blackboard::create();

    Simulation simulation(blackboard);
    BTEngine bt_engine(blackboard, simulation);
    Renderer renderer;

    // Initialize Blackboard with default values to prevent
    // uninitialized reads on the first frame
    blackboard->set("neighbors_blocked", std::array<bool, 8>{});
    blackboard->set("battery_level", Config::BATTERY_MAX);
    blackboard->set("robot_position", Vector2{ 1.0f, 1.0f });
    blackboard->set("base_position", Vector2{ 1.0f, 1.0f });
    blackboard->set("goal_position", Vector2{ static_cast<float>(Config::GRID_WIDTH  - 2), static_cast<float>(Config::GRID_HEIGHT - 2)});
    blackboard->set("compass_heading", Direction::N);
    blackboard->set("goal_direction", Direction::N);
    blackboard->set("resultant_force", Vector2{ 0.0f, 0.0f });
    blackboard->set("is_stuck", false);

    // Fixed timestep ensures deterministic simulation
    // regardless of actual frame rate
    const float FIXED_DT  = 1.0f / Config::FPS_TARGET;
    float accumulator = 0.0f;  ///< Accumulated time between frames

    while (!WindowShouldClose())
    {
        accumulator += GetFrameTime();

        // Run simulation steps as many times as needed
        // to stay synchronized with real time
        while (accumulator >= FIXED_DT)
        {
            simulation.update(FIXED_DT);
            bt_engine.tick();
            accumulator -= FIXED_DT;
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);
        renderer.draw(simulation);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}