/**
 * @file renderer.h
 * @brief 2D rendering of the simulation world using Raylib
 *
 * Draws the grid, obstacles, robot, compass heading,
 * goal, and the Potential Fields resultant force vector.
 */

#pragma once

#include <raylib.h>

#include "simulation.h"
#include "config.h"

/**
 * @brief Handles all 2D rendering of the simulation state
 *
 * Reads exclusively from Simulation — never writes to it.
 * Called once per frame after the BT engine has ticked.
 */
class Renderer
{
public:
    /**
     * @brief Draws the complete simulation state for one frame
     * @param simulation Read-only reference to the current simulation state
     */
    void draw(const Simulation& simulation);

private:
    /**
     * @brief Draws the grid cells and obstacles
     * @param world The world grid to draw
     */
    void drawWorld(const World& world);

    /**
     * @brief Draws the robot as a colored circle with compass heading indicator
     * @param robot The robot state to draw
     */
    void drawRobot(const Robot& robot);

    /**
     * @brief Draws the goal cell
     * @param world The world containing the goal position
     */
    void drawGoal(const World& world);

    /**
     * @brief Draws the Potential Fields resultant force vector
     *
     * Visualizes the force as an arrow originating from the robot position.
     *
     * @param robot The robot whose position is used as origin
     * @param force The resultant force vector to draw
     */
    void drawForceVector(const Robot& robot, Vector2 force);

    /**
     * @brief Draws a HUD with battery level, compass heading and robot status
     * @param robot The robot state to display
     */
    void drawHUD(const Robot& robot);

    /**
     * @brief Converts a grid position to screen pixel coordinates
     * @param grid_pos Position in grid coordinates
     * @return Center pixel position of the cell on screen
     */
    Vector2 gridToScreen(Vector2 grid_pos) const;
};