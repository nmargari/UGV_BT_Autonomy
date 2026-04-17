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
    blackboard->set("target_velocity", 0.0f);
    blackboard->set("avoid", false);
    blackboard->set("return_to_base", false);

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