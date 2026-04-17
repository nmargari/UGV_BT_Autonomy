/**
 * @file simulation.h
 * @brief Simulation components: World, Robot, Sensors and Simulation
 *
 * Defines the core simulation structures. The Simulation class
 * is updated every tick before the BT engine runs, ensuring the
 * Blackboard always contains fresh sensor data.
 */

#pragma once

#include <raylib.h>
#include <behaviortree_cpp/bt_factory.h>
#include <vector>
#include <array>

#include "config.h"

// ---------------------------------------------
//  World
// ---------------------------------------------

/**
 * @brief 2D grid world containing obstacles and goal position
 *
 * Holds two grids: the raw obstacle grid and an inflated version
 * used by the path planner to enforce the safety margin.
 */
struct World
{
    bool grid[Config::GRID_HEIGHT][Config::GRID_WIDTH]; ///< True if cell contains an obstacle
    bool inflated_grid[Config::GRID_HEIGHT][Config::GRID_WIDTH]; ///< Obstacle grid inflated by Config::SAFETY_MARGIN, used by A*

    Vector2 start; ///< Robot spawn position in grid coordinates
    Vector2 goal; ///< Goal position in grid coordinates

    /**
     * @brief Randomly generates the world with obstacles
     *
     * Places obstacles based on Config::OBSTACLE_RATIO, then verifies
     * that a valid path exists from start to goal using A*.
     * Regenerates if no path is found.
     */
    void generate();

    /**
     * @brief Inflates obstacle cells outward by Config::SAFETY_MARGIN
     *
     * Marks all cells within SAFETY_MARGIN distance of an obstacle
     * as blocked in inflated_grid. The path planner uses this grid
     * to keep the robot at a safe distance from obstacles.
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
     * @brief Checks whether a grid cell is walkable in the inflated grid
     * @param x Column index
     * @param y Row index
     * @return true if the cell is within bounds and not inflated-blocked
     */
    bool isWalkable(int x, int y) const;
};

// ---------------------------------------------
//  Robot
// ---------------------------------------------

/**
 * @brief Represents the UGV state including position, battery and planned path
 */
struct Robot
{
    Vector2 position; ///< Current position in grid coordinates
    Vector2 heading; ///< Normalized movement direction vector
    float battery; ///< Current battery level (0.0 to Config::BATTERY_MAX)
    float speed; ///< Movement speed in cells per second
    std::vector<Vector2> waypoints; ///< Ordered list of waypoints produced by A*
    int waypoint_index; ///< Index of the next waypoint to move toward

    /**
     * @brief Initializes the robot at the given grid position with full battery
     * @param start Starting grid position
     */
    void init(Vector2 start);

    /**
     * @brief Moves the robot toward the next waypoint
     * @param dt Delta time in seconds
     */
    void update(float dt);

    /**
     * @brief Reduces battery by Config::BATTERY_DRAIN per call
     */
    void drainBattery();

    /**
     * @brief Checks whether the robot has reached the current waypoint
     * @return true if the robot is within arrival threshold of the next waypoint
     */
    bool hasReachedWaypoint() const;
};

// ---------------------------------------------
//  Sensors
// ---------------------------------------------

/**
 * @brief Reads the environment around the robot and writes results to the Blackboard
 *
 * Scans up to 8 neighboring cells (cardinal + diagonal) within
 * Config::SENSOR_RANGE. Results are written to the Blackboard
 * so BT condition nodes can read them without accessing the World directly.
 */
struct Sensors
{
    static constexpr int NEIGHBOR_COUNT = 8; ///< Maximum number of scanned neighbors (N NE E SE S SW W NW)

    /// Column offsets for 8-directional neighbor scanning
    static constexpr int DX[NEIGHBOR_COUNT] = { 0, 1, 1,  1,  0, -1, -1, -1 };

    /// Row offsets for 8-directional neighbor scanning
    static constexpr int DY[NEIGHBOR_COUNT] = {-1,-1, 0,  1,  1,  1,  0, -1 };

    /**
     * @brief Scans cells around the robot within Config::SENSOR_RANGE
     * @param world The current world grid
     * @param robot The robot whose surroundings are scanned
     * @return Array of 8 booleans, true if the neighbor cell is blocked
     */
    std::array<bool, NEIGHBOR_COUNT> scanNeighbors(const World& world, const Robot& robot) const;

    /**
     * @brief Writes sensor readings to the shared Blackboard
     *
     * Keys written:
     * - "neighbors_blocked" : std::array<bool, 8>
     * - "battery_level"     : float
     * - "robot_position"    : Vector2
     *
     * @param blackboard Shared Blackboard instance
     * @param world      The current world grid
     * @param robot      The robot to read sensor data from
     */
    void writeBlackboard(BT::Blackboard::Ptr blackboard, const World& world, const Robot& robot) const;
};

// ---------------------------------------------
//  Simulation
// ---------------------------------------------

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
     * Executes in order: robot movement → battery drain → sensor scan
     * → Blackboard write.
     *
     * @param dt Delta time in seconds (typically Config::FIXED_DT)
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
    World world_; ///< The 2D grid world with obstacles
    Robot robot_; ///< The UGV robot state
    Sensors sensors_; ///< Sensor system for environment perception
    BT::Blackboard::Ptr blackboard_; ///< Shared Blackboard for BT communication
};