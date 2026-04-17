/**
 * @file path_planner.h
 * @brief A* path planning algorithm for UGV navigation
 *
 * Computes the shortest walkable path between two grid cells
 * using the A* algorithm on the inflated world grid, ensuring
 * the robot maintains a safe distance from obstacles.
 */

#pragma once

#include <vector>
#include <array>

#include <raylib.h>

#include "config.h"

// Forward declaration to avoid circular dependency with simulation.h
struct World;

/**
 * @brief Represents a single cell evaluated during A* search
 */
struct AStarNode
{
    int x;          ///< Column index in the grid
    int y;          ///< Row index in the grid
    float g;          ///< Actual cost from start to this node
    float f;          ///< Estimated total cost (g + heuristic)
    int parent_x;   ///< Column index of the parent node
    int parent_y;   ///< Row index of the parent node

    /**
     * @brief Comparison operator for priority queue ordering (min-heap by f)
     * @param other Node to compare against
     * @return true if this node has a higher f cost than other
     */
    bool operator>(const AStarNode& other) const;
};

/**
 * @brief Provides A* path planning on a 2D grid world
 *
 * Operates exclusively on World::inflated_grid to ensure the robot
 * path respects the configured safety margin around obstacles.
 */
class PathPlanner
{
public:

    /**
     * @brief Computes the shortest path from start to goal using A*
     *
     * Runs A* on World::inflated_grid. Returns an empty vector if no
     * valid path exists. Diagonal movement is controlled by
     * Config::ASTAR_DIAGONAL.
     *
     * @param start Starting cell position in grid coordinates
     * @param goal  Target cell position in grid coordinates
     * @param world The current world containing the inflated grid
     * @return Ordered list of waypoints from start to goal,
     *         empty if no path was found
     */
    std::vector<Vector2> findPath(Vector2 start, Vector2 goal, const World& world) const;

private:

    /**
     * @brief Computes the heuristic estimate from a cell to the goal
     *
     * Uses Euclidean distance, which is admissible for both cardinal
     * and diagonal movement.
     *
     * @param x    Column index of the current cell
     * @param y    Row index of the current cell
     * @param goal Target cell position
     * @return Estimated cost from (x, y) to goal
     */
    float heuristic(int x, int y, Vector2 goal) const;

    /**
     * @brief Reconstructs the path from the came_from map
     *
     * Traces back from goal to start using the parent references
     * stored during A* search, then reverses the result.
     *
     * @param came_from 2D array of parent cell coordinates
     * @param start     Starting cell position
     * @param goal      Target cell position
     * @return Ordered list of waypoints from start to goal
     */
    std::vector<Vector2> reconstructPath(
        const std::array<std::array<Vector2, Config::GRID_WIDTH>, Config::GRID_HEIGHT>& came_from,
        Vector2 start,
        Vector2 goal) const;

    /**
     * @brief Returns the valid neighbors of a given cell
     *
     * Returns up to 8 neighbors depending on Config::ASTAR_DIAGONAL.
     * Only walkable cells within bounds are included.
     *
     * @param x     Column index of the current cell
     * @param y     Row index of the current cell
     * @param world The current world for walkability checks
     * @return Vector of walkable neighbor positions
     */
    std::vector<Vector2> getNeighbors(int x, int y, const World& world) const;
};