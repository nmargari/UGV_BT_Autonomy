/**
 * @file main.cpp
 * @brief Entry point to UGV Behavior Tree Simulator
 * 
 * Initializes all components and runs the simulation loop with fixed timestep.
 */

#include <raylib.h>
#include <behaviortree_cpp/bt_factory.h>

#include "config.h"
#include "simulation.h"
// #include "bt_engine.h"
// #include "renderer.h"

/**
 * @brief Entry point 
 * 
 * Create and initialize components.
 * 
 * @return 0 If normal program termination.
 */
int main()
{
    InitWindow(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, "UGV Behavior Tree Simulator");
    SetTargetFPS(Config::FPS_TARGET);

    auto blackboard = BT::Blackboard::create();
    Simulation simulation(blackboard); ///< World, robot, sensors
    // BTEngine bt_engine(blackboard); ///< BT tree and logic
    // Renderer renderer; ///< Raylib 2D rendering


    /// Initialize blackboard
    blackboard->set("neighbors_blocked", std::array<bool, 8>{});
    blackboard->set("battery_level", Config::BATTERY_MAX);
    blackboard->set("robot_position", Vector2{ 1.0f, 1.0f });
    blackboard->set("compass_heading", Direction::N);
    blackboard->set("goal_direction", Direction::N);
    blackboard->set("goal_angle_diff", 0);
    blackboard->set("is_stuck", false);

    /// Fixed timestep
    const float FIXED_DT = 1.0f / Config::FPS_TARGET;
    float accumulator = 0.0f;

    while(!WindowShouldClose())
    {
        accumulator += GetFrameTime();

        /// Synchronize with real time
        while(accumulator >= FIXED_DT)
        {
            simulation.update(FIXED_DT);
            // bt_engine.tick();
            accumulator -= FIXED_DT;
        }

        BeginDrawing();

        ClearBackground(RAYWHITE);

        // renderer.draw(simulation);

        EndDrawing();
    }
    
    CloseWindow();
    return 0;
}