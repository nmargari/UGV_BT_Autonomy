/**
 * @file path_planner.cpp
 * @brief Implementation of the A* path planning algorithm
 */

#include "path_planner.h"
#include "simulation.h"

#include <queue>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

// ---------------------------------------------
//  AStarNode
// ---------------------------------------------

bool AStarNode::operator>(const AStarNode& other) const
{
    return f > other.f;
}

// ---------------------------------------------
//  PathPlanner
// ---------------------------------------------

float PathPlanner::heuristic(int x, int y, Vector2 goal) const
{
    float dx = static_cast<float>(x) - goal.x;
    float dy = static_cast<float>(y) - goal.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Vector2> PathPlanner::getNeighbors(int x, int y, const World& world) const
{
    // Cardinal directions: N E S W
    static constexpr int CARDINAL_DX[] = { 0,  1,  0, -1 };
    static constexpr int CARDINAL_DY[] = {-1,  0,  1,  0 };

    // Diagonal directions: NE SE SW NW
    static constexpr int DIAGONAL_DX[] = { 1,  1, -1, -1 };
    static constexpr int DIAGONAL_DY[] = {-1,  1,  1, -1 };

    std::vector<Vector2> neighbors;

    // Always add cardinal neighbors
    for (int i = 0; i < 4; i++)
    {
        int nx = x + CARDINAL_DX[i];
        int ny = y + CARDINAL_DY[i];

        if (world.isWalkable(nx, ny))
        {
            neighbors.push_back({ static_cast<float>(nx), static_cast<float>(ny) });
        }
    }

    // Add diagonal neighbors only if enabled in config
    if (Config::ASTAR_DIAGONAL)
    {
        for (int i = 0; i < 4; i++)
        {
            int nx = x + DIAGONAL_DX[i];
            int ny = y + DIAGONAL_DY[i];

            if (world.isWalkable(nx, ny))
            {
                neighbors.push_back({ static_cast<float>(nx), static_cast<float>(ny) });
            }
        }
    }

    return neighbors;
}

std::vector<Vector2> PathPlanner::reconstructPath(
    const std::array<std::array<Vector2, Config::GRID_WIDTH>, Config::GRID_HEIGHT>& came_from,
    Vector2 start,
    Vector2 goal) const
{
    std::vector<Vector2> path;
    Vector2 current = goal;

    while (!(current.x == start.x && current.y == start.y))
    {
        path.push_back(current);
        Vector2 parent = came_from[static_cast<int>(current.y)][static_cast<int>(current.x)];
        current = parent;
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Vector2> PathPlanner::findPath(
    Vector2      start,
    Vector2      goal,
    const World& world) const
{
    // Min-heap ordered by f cost
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

    // Cost from start to each cell
    std::array<std::array<float, Config::GRID_WIDTH>, Config::GRID_HEIGHT> g_cost;
    for (auto& row : g_cost)
    {
        row.fill(std::numeric_limits<float>::infinity());
    }

    // Parent tracking for path reconstruction
    std::array<std::array<Vector2, Config::GRID_WIDTH>, Config::GRID_HEIGHT> came_from;

    // Visited cells
    std::array<std::array<bool, Config::GRID_WIDTH>, Config::GRID_HEIGHT> closed;
    for (auto& row : closed)
    {
        row.fill(false);
    }

    int sx = static_cast<int>(start.x);
    int sy = static_cast<int>(start.y);
    int gx = static_cast<int>(goal.x);
    int gy = static_cast<int>(goal.y);

    g_cost[sy][sx] = 0.0f;
    open_set.push({ sx, sy, 0.0f, heuristic(sx, sy, goal), sx, sy });

    while (!open_set.empty())
    {
        AStarNode current = open_set.top();
        open_set.pop();

        // Skip if already visited
        if (closed[current.y][current.x])
        {
            continue;
        }

        closed[current.y][current.x] = true;

        // Goal reached — reconstruct and return path
        if (current.x == gx && current.y == gy)
        {
            return reconstructPath(came_from, start, goal);
        }

        for (const Vector2& neighbor : getNeighbors(current.x, current.y, world))
        {
            int nx = static_cast<int>(neighbor.x);
            int ny = static_cast<int>(neighbor.y);

            if (closed[ny][nx])
            {
                continue;
            }

            // Diagonal movement costs more than cardinal
            float dx         = static_cast<float>(nx - current.x);
            float dy         = static_cast<float>(ny - current.y);
            float move_cost  = (dx != 0.0f && dy != 0.0f) ? 1.414f : 1.0f;
            float tentative_g = g_cost[current.y][current.x] + move_cost;

            if (tentative_g < g_cost[ny][nx])
            {
                g_cost[ny][nx]    = tentative_g;
                came_from[ny][nx] = { static_cast<float>(current.x),
                                      static_cast<float>(current.y) };

                float f = tentative_g + heuristic(nx, ny, goal);
                open_set.push({ nx, ny, tentative_g, f, current.x, current.y });
            }
        }
    }

    // No path found
    return {};
}