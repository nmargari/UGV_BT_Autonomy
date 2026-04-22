/**
 * @file simulation.h
 * @brief Simulation components: World, Robot, Sensors and Simulation
 *
 * Defines the core simulation structures. The Simulation class
 * is updated every tick before the BT engine runs, ensuring the
 * Blackboard always contains fresh sensor data.
 *
 * Navigation is handled reactively using Potential Fields as the
 * primary strategy and Wall Following as fallback when the robot
 * gets stuck in a local minimum.
 */

#pragma once

#include <raylib.h>
#include <behaviortree_cpp/bt_factory.h>
#include <array>

#include "config.h"

// ─────────────────────────────────────────────
//  Direction
// ─────────────────────────────────────────────

/**
 * @brief 8-directional compass rose used by robot orientation and sensors
 *
 * Values are ordered clockwise starting from North.
 * Cardinal directions are always available. Diagonal directions
 * are used only when Config::COMPASS_8DIR is true.
 */
enum class Direction
{
    N = 0,  ///< North
    NE = 1,  ///< North-East
    E = 2,  ///< East
    SE = 3,  ///< South-East
    S = 4,  ///< South
    SW = 5,  ///< South-West
    W = 6,  ///< West
    NW = 7   ///< North-West
};

/**
 * @brief Column offset for each Direction, indexed by Direction enum value
 */
constexpr int DIRECTION_DX[8] = { 0, 1, 1,  1,  0, -1, -1, -1 };

/**
 * @brief Row offset for each Direction, indexed by Direction enum value
 */
constexpr int DIRECTION_DY[8] = {-1,-1, 0,  1,  1,  1,  0, -1 };

/**
 * @brief Returns the direction rotated 90 degrees clockwise
 * @param dir Input direction
 * @return Direction rotated clockwise by 90 degrees
 */
Direction rotateClockwise90(Direction dir);

/**
 * @brief Returns the direction rotated 90 degrees counter-clockwise
 * @param dir Input direction
 * @return Direction rotated counter-clockwise by 90 degrees
 */
Direction rotateCounterClockwise90(Direction dir);

/**
 * @brief Computes the compass direction from one grid cell toward another
 * @param from Starting grid position
 * @param to   Target grid position
 * @return The closest Direction from from toward to
 */
Direction directionTo(Vector2 from, Vector2 to);

/**
 * @brief Snaps a continuous force vector to the nearest compass Direction
 *
 * Converts the angle of the vector to the closest of the 8 compass
 * directions. Used to translate the Potential Fields resultant force
 * into a discrete grid movement direction.
 *
 * @param force The resultant force vector
 * @return The nearest Direction
 */
Direction snapToDirection(Vector2 force);

// ─────────────────────────────────────────────
//  World
// ─────────────────────────────────────────────

/**
 * @brief 2D grid world containing obstacles, start and goal positions
 *
 * Holds two grids: the raw obstacle grid and an inflated version
 * used to enforce the safety margin around obstacles.
 * The world is randomly generated and guaranteed to have start
 * and goal cells always free.
 */
struct World
{
    bool grid[Config::GRID_HEIGHT][Config::GRID_WIDTH];           ///< True if cell contains an obstacle
    bool inflated_grid[Config::GRID_HEIGHT][Config::GRID_WIDTH];  ///< Obstacle grid inflated by Config::SAFETY_MARGIN

    Vector2 start;  ///< Robot spawn position in grid coordinates
    Vector2 goal;   ///< Goal position in grid coordinates

    /**
     * @brief Randomly generates the world with obstacles
     *
     * Places obstacles based on Config::OBSTACLE_RATIO.
     * Guarantees that start and goal cells are always free.
     */
    void generate();

    /**
     * @brief Inflates obstacle cells outward by Config::SAFETY_MARGIN
     *
     * Marks all cells within SAFETY_MARGIN distance of an obstacle
     * as blocked in inflated_grid. Used by sensors to determine
     * safe movement options.
     */
    void inflate();

    /**
     * @brief Checks whether a grid cell is within bounds
     * @param x Column index
     * @param y Row index
     * @return true if (x, y) is a valid grid coordinate
     */
    bool inBounds(int x, int y) const;

    /**
     * @brief Checks whether a grid cell is free in the inflated grid
     * @param x Column index
     * @param y Row index
     * @return true if the cell is within bounds and not inflated-blocked
     */
    bool isWalkable(int x, int y) const;
};

// ─────────────────────────────────────────────
//  Robot
// ─────────────────────────────────────────────

/**
 * @brief Represents the UGV state including position, compass and battery
 *
 * The robot navigates reactively using Potential Fields. It has no
 * global map knowledge — it only knows its current cell, orientation,
 * and what the sensors report.
 */
struct Robot
{
    Vector2 position;       ///< Current position in grid coordinates (continuous)
    Direction compass;        ///< Current facing direction
    float battery;        ///< Current battery level (0.0 to Config::BATTERY_MAX)
    float speed;          ///< Movement speed in cells per second
    bool wall_following; ///< True if robot is currently in wall following mode
    int stuck_counter;  ///< Number of steps without progress toward goal

    /**
     * @brief Initializes the robot at the given position with full battery
     * @param start Starting grid position
     */
    void init(Vector2 start);

    /**
     * @brief Moves the robot one step in the given direction
     * @param dir Direction to move
     * @param dt  Delta time in seconds
     */
    void move(Direction dir, float dt);

    /**
     * @brief Updates the compass to face the given direction
     * @param dir New facing direction
     */
    void setCompass(Direction dir);

    /**
     * @brief Reduces battery by Config::BATTERY_DRAIN per call
     */
    void drainBattery();

    /**
     * @brief Returns the current grid cell of the robot
     * @return Grid cell position as integer-snapped Vector2
     */
    Vector2 getCell() const;
};

// ─────────────────────────────────────────────
//  Sensors
// ─────────────────────────────────────────────

/**
 * @brief Reads the environment around the robot and writes results to the Blackboard
 *
 * Scans neighboring cells within Config::SENSOR_RANGE, computes the
 * Potential Fields resultant force, and writes all results to the
 * Blackboard so BT condition nodes can read them without accessing
 * the World or Robot directly.
 *
 * Blackboard keys written:
 * - "neighbors_blocked" : std::array<bool, 8> — blocked state per direction
 * - "battery_level"     : float               — current battery level
 * - "robot_position"    : Vector2             — current robot position
 * - "compass_heading"   : Direction           — current robot orientation
 * - "goal_direction"    : Direction           — closest direction toward goal
 * - "resultant_force"   : Vector2             — Potential Fields resultant force vector
 * - "is_stuck"          : bool               — true if stuck_counter >= Config::STUCK_THRESHOLD
 */
struct Sensors
{
    static constexpr int NEIGHBOR_COUNT = 8; ///< Total number of scanned directions (N NE E SE S SW W NW)

    /**
     * @brief Scans all neighboring cells within Config::SENSOR_RANGE
     * @param world The current world grid
     * @param robot The robot whose surroundings are scanned
     * @return Array of 8 booleans indexed by Direction, true if blocked
     */
    std::array<bool, NEIGHBOR_COUNT> scanNeighbors(const World& world, const Robot& robot) const;

    /**
     * @brief Computes the Potential Fields resultant force acting on the robot
     *
     * Combines:
     * - Attractive force from the goal: K_ATT * (goal - position)
     * - Repulsive forces from all obstacles within REP_RANGE: K_REP / d^2 * (position - obstacle)
     *
     * @param world The current world grid
     * @param robot The robot to compute forces for
     * @return Resultant force vector
     */
    Vector2 computeResultantForce(const World& world, const Robot& robot) const;

    /**
     * @brief Writes all sensor readings to the shared Blackboard
     * @param blackboard Shared Blackboard instance
     * @param world      The current world grid
     * @param robot      The robot to read sensor data from
     */
    void writeBlackboard(BT::Blackboard::Ptr blackboard, const World& world, const Robot& robot) const;
};

// ─────────────────────────────────────────────
//  Simulation
// ─────────────────────────────────────────────

/**
 * @brief Top-level simulation class that owns World, Robot and Sensors
 *
 * Updated every simulation step before the BT engine ticks,
 * ensuring the Blackboard always reflects the current world state.
 */
class Simulation
{
public:
    /**
     * @brief Constructs the simulation, generates the world and spawns the robot
     * @param blackboard Shared Blackboard instance used for sensor output
     */
    explicit Simulation(BT::Blackboard::Ptr blackboard);

    /**
     * @brief Advances the simulation by one fixed timestep
     *
     * Executes in order: battery drain -> sensor scan -> Blackboard write.
     * Robot movement is driven by BT action nodes, not by this method.
     *
     * @param dt Delta time in seconds
     */
    void update(float dt);

    /**
     * @brief Returns a read-only reference to the world
     * @return Const reference to the World struct
     */
    const World& getWorld() const;

    /**
     * @brief Returns a read-only reference to the robot
     * @return Const reference to the Robot struct
     */
    const Robot& getRobot() const;

    /**
     * @brief Returns a mutable reference to the robot
     *
     * Used by BT action nodes to apply movement commands.
     *
     * @return Reference to the Robot struct
     */
    Robot& getRobot();

private:
    World world_;      ///< The 2D grid world with obstacles
    Robot robot_;      ///< The UGV robot state
    Sensors sensors_;    ///< Sensor system for environment perception
    BT::Blackboard::Ptr blackboard_; ///< Shared Blackboard for BT communication
};
