/**
 * @file simulation.cpp
 * @brief Implementation of World, Robot, Sensors and Simulation classes
 */

#include "simulation.h"
#include "path_planner.h"

#include <cstdlib>
#include <ctime>
#include <cmath>

// ---------------------------------------------
//  World
// ---------------------------------------------

void World::generate()
{
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    // Define fixed start and goal positions
    start = { 1, 1 };
    goal  = { static_cast<float>(Config::GRID_WIDTH - 2),
              static_cast<float>(Config::GRID_HEIGHT - 2) };

    bool valid = false;

    while (!valid)
    {
        // Clear both grids
        for (int y = 0; y < Config::GRID_HEIGHT; y++)
        {
            for (int x = 0; x < Config::GRID_WIDTH; x++)
            {
                grid[y][x]          = false;
                inflated_grid[y][x] = false;
            }
        }

        // Place obstacles randomly based on OBSTACLE_RATIO
        for (int y = 0; y < Config::GRID_HEIGHT; y++)
        {
            for (int x = 0; x < Config::GRID_WIDTH; x++)
            {
                // Never place obstacles on start or goal
                if ((x == static_cast<int>(start.x) && y == static_cast<int>(start.y)) ||
                    (x == static_cast<int>(goal.x)  && y == static_cast<int>(goal.y)))
                {
                    continue;
                }

                if ((std::rand() / static_cast<float>(RAND_MAX)) < Config::OBSTACLE_RATIO)
                {
                    grid[y][x] = true;
                }
            }
        }

        inflate();

        // Verify a valid path exists using A*
        // If no path found, regenerate
        PathPlanner planner;
        valid = !planner.findPath(start, goal, *this).empty();
    }
}

void World::inflate()
{
    for (int y = 0; y < Config::GRID_HEIGHT; y++)
    {
        for (int x = 0; x < Config::GRID_WIDTH; x++)
        {
            inflated_grid[y][x] = grid[y][x];
        }
    }

    for (int y = 0; y < Config::GRID_HEIGHT; y++)
    {
        for (int x = 0; x < Config::GRID_WIDTH; x++)
        {
            if (!grid[y][x])
            {
                continue;
            }

            // Mark all cells within SAFETY_MARGIN as blocked
            for (int dy = -Config::SAFETY_MARGIN; dy <= Config::SAFETY_MARGIN; dy++)
            {
                for (int dx = -Config::SAFETY_MARGIN; dx <= Config::SAFETY_MARGIN; dx++)
                {
                    int nx = x + dx;
                    int ny = y + dy;

                    if (inBounds(nx, ny))
                    {
                        inflated_grid[ny][nx] = true;
                    }
                }
            }
        }
    }
}

bool World::inBounds(int x, int y) const
{
    return x >= 0 && x < Config::GRID_WIDTH &&
           y >= 0 && y < Config::GRID_HEIGHT;
}

bool World::isWalkable(int x, int y) const
{
    return inBounds(x, y) && !inflated_grid[y][x];
}

// ---------------------------------------------
//  Robot
// ---------------------------------------------

void Robot::init(Vector2 start)
{
    position = start;
    heading = { 0.0f, 0.0f };
    battery = Config::BATTERY_MAX;
    speed = Config::MOVE_SPEED;
    waypoints = {};
    waypoint_index = 0;
}

void Robot::update(float dt)
{
    if (waypoints.empty() || waypoint_index >= static_cast<int>(waypoints.size()))
    {
        return;
    }

    Vector2 target = waypoints[waypoint_index];

    // Compute direction toward next waypoint
    float dx = target.x - position.x;
    float dy = target.y - position.y;
    float distance = std::sqrt(dx * dx + dy * dy);

    if (distance < 0.01f)
    {
        // Arrived at waypoint, advance to next
        waypoint_index++;
        return;
    }

    // Normalize direction and move
    heading = { dx / distance, dy / distance };
    position.x += heading.x * speed * dt;
    position.y += heading.y * speed * dt;
}

void Robot::drainBattery()
{
    battery -= Config::BATTERY_DRAIN;

    if (battery < 0.0f)
    {
        battery = 0.0f;
    }
}

bool Robot::hasReachedWaypoint() const
{
    if (waypoints.empty() || waypoint_index >= static_cast<int>(waypoints.size()))
    {
        return true;
    }

    Vector2 target = waypoints[waypoint_index];
    float dx = target.x - position.x;
    float dy = target.y - position.y;

    return std::sqrt(dx * dx + dy * dy) < 0.1f;
}

// ---------------------------------------------
//  Sensors
// ---------------------------------------------

std::array<bool, Sensors::NEIGHBOR_COUNT> Sensors::scanNeighbors(
    const World& world, const Robot& robot) const
{
    std::array<bool, NEIGHBOR_COUNT> blocked = {};
    int rx = static_cast<int>(robot.position.x);
    int ry = static_cast<int>(robot.position.y);

    for (int i = 0; i < NEIGHBOR_COUNT; i++)
    {
        // Skip diagonal neighbors if disabled in config
        bool is_diagonal = (DX[i] != 0 && DY[i] != 0);

        if (is_diagonal && !Config::SENSOR_DIAGONAL)
        {
            blocked[i] = false;
            continue;
        }

        // Check all cells within SENSOR_RANGE
        for (int r = 1; r <= Config::SENSOR_RANGE; r++)
        {
            int nx = rx + DX[i] * r;
            int ny = ry + DY[i] * r;

            if (!world.inBounds(nx, ny) || world.grid[ny][nx])
            {
                blocked[i] = true;
                break;
            }
        }
    }

    return blocked;
}

void Sensors::writeBlackboard(
    BT::Blackboard::Ptr blackboard,
    const World&        world,
    const Robot&        robot) const
{
    blackboard->set("neighbors_blocked", scanNeighbors(world, robot));
    blackboard->set("battery_level",     robot.battery);
    blackboard->set("robot_position",    robot.position);
}

// ---------------------------------------------
//  Simulation
// ---------------------------------------------

Simulation::Simulation(BT::Blackboard::Ptr blackboard)
    : blackboard_(blackboard)
{
    world_.generate();
    robot_.init(world_.start);

    // Write initial sensor state to Blackboard
    sensors_.writeBlackboard(blackboard_, world_, robot_);
}

void Simulation::update(float dt)
{
    robot_.update(dt);
    robot_.drainBattery();
    sensors_.writeBlackboard(blackboard_, world_, robot_);
}

const World& Simulation::getWorld() const
{
    return world_;
}

const Robot& Simulation::getRobot() const
{
    return robot_;
}

Robot& Simulation::getRobot()
{
    return robot_;
}