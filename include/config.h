/**
 * @file config.h
 * @brief Central configuration file for the UGV Behavior Tree Simulator
 *
 * All simulation parameters are defined here as compile-time constants.
 * Modify this file to tune the behavior of the simulator without
 * touching the implementation files.
 */

#pragma once

namespace Config
{
    /// @defgroup World World configuration
    /// @{

    constexpr int GRID_WIDTH = 53;    ///< Number of columns in the grid
    constexpr int GRID_HEIGHT = 30;    ///< Number of rows in the grid
    constexpr float OBSTACLE_RATIO = 0.2f;  ///< Fraction of cells that are obstacles (0.0 - 1.0)
    constexpr int SAFETY_MARGIN = 1;     ///< Minimum distance in cells between robot path and obstacles

    /// @}

    /// @defgroup Robot Robot configuration
    /// @{

    constexpr float MOVE_SPEED = 5.0f;   ///< Robot movement speed in cells per second
    constexpr float BATTERY_MAX = 100.0f; ///< Maximum battery level
    constexpr float BATTERY_DRAIN = 0.1f;   ///< Battery drain per simulation step
    constexpr float BATTERY_LOW = 20.0f;  ///< Battery level threshold that triggers return to base

    /// @}

    /// @defgroup Compass Compass configuration
    /// @{

    constexpr bool COMPASS_8DIR = true;   ///< If true, compass uses 8 directions; otherwise 4 cardinal only

    /// @}

    /// @defgroup Sensors Sensor configuration
    /// @{

    constexpr int SENSOR_RANGE = 1;      ///< Sensor perception range in cells
    constexpr bool  SENSOR_DIAGONAL = true;   ///< Whether sensors detect diagonal neighbors

    /// @}

    /// @defgroup Navigation Navigation — Potential Fields configuration
    /// @{

    constexpr float K_ATT = 1.0f;   ///< Attractive force constant toward goal
    constexpr float K_REP = 2.0f;   ///< Repulsive force constant from obstacles
    constexpr int REP_RANGE = 3;      ///< Obstacle repulsion range in cells
    constexpr int STUCK_THRESHOLD = 10;     ///< Steps without progress before switching to wall following
    constexpr float GOAL_REACH_DIST = 0.5f;   ///< Distance in cells to consider goal reached
    constexpr bool WALL_FOLLOW_RIGHT = true;   ///< If true, follow wall on the right side; otherwise left

    /// @}

    /// @defgroup Renderer Renderer configuration
    /// @{

    constexpr int CELL_SIZE = 24;     ///< Size of each grid cell in pixels
    constexpr int SCREEN_WIDTH = 1280;   ///< Window width in pixels
    constexpr int SCREEN_HEIGHT = 720;    ///< Window height in pixels
    constexpr float FPS_TARGET = 60.0f;  ///< Target frames per second

    /// @}
}
