/**
 * @file simulation.cpp
 * @brief Implementation of World, Robot, Sensors and Simulation classes
 */

#include "simulation.h"

#include <cstdlib>
#include <ctime>
#include <cmath>

// ─────────────────────────────────────────────
//  Direction helpers
// ─────────────────────────────────────────────

Direction rotateClockwise90(Direction dir)
{
    // Rotating 90 degrees clockwise advances by 2 steps in the 8-direction enum
    return static_cast<Direction>((static_cast<int>(dir) + 2) % 8);
}

Direction rotateCounterClockwise90(Direction dir)
{
    // Rotating 90 degrees counter-clockwise retreats by 2 steps
    return static_cast<Direction>((static_cast<int>(dir) + 6) % 8);
}

Direction directionTo(Vector2 from, Vector2 to)
{
    float dx =  (to.x - from.x);
    float dy = -(to.y - from.y);  // Invert Y for Raylib screen coordinates (Y grows downward)

    float angle = std::atan2(dy, dx) * (180.0f / 3.14159265f);

    if (angle < 0.0f)
    {
        angle += 360.0f;
    }

    int sector = static_cast<int>((angle + 22.5f) / 45.0f) % 8;

    // Remap from atan2 sectors to Direction enum
    // atan2: E=0, NE=1, N=2, NW=3, W=4, SW=5, S=6, SE=7
    // Direction enum: N=0, NE=1, E=2, SE=3, S=4, SW=5, W=6, NW=7
    constexpr Direction remap[8] =
    {
        Direction::E,
        Direction::NE,
        Direction::N,
        Direction::NW,
        Direction::W,
        Direction::SW,
        Direction::S,
        Direction::SE
    };

    return remap[sector];
}

Direction snapToDirection(Vector2 force)
{
    // Reuse directionTo by treating force as an offset from origin
    return directionTo({ 0.0f, 0.0f }, { force.x, -force.y });
}

// ─────────────────────────────────────────────
//  World
// ─────────────────────────────────────────────

void World::generate()
{
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    // Fixed start and goal at opposite corners
    start = { 1.0f, 1.0f };
    goal  = { static_cast<float>(Config::GRID_WIDTH  - 2),
              static_cast<float>(Config::GRID_HEIGHT - 2) };

    // Clear both grids
    for (int y = 0; y < Config::GRID_HEIGHT; y++)
    {
        for (int x = 0; x < Config::GRID_WIDTH; x++)
        {
            grid[y][x] = false;
            inflated_grid[y][x] = false;
        }
    }

    // Place obstacles randomly, never on start or goal cells
    for (int y = 0; y < Config::GRID_HEIGHT; y++)
    {
        for (int x = 0; x < Config::GRID_WIDTH; x++)
        {
            bool is_start = (x == static_cast<int>(start.x) &&
                             y == static_cast<int>(start.y));
            bool is_goal = (x == static_cast<int>(goal.x)  &&
                            y == static_cast<int>(goal.y));

            if (is_start || is_goal)
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
}

void World::inflate()
{
    // Copy raw grid to inflated grid
    for (int y = 0; y < Config::GRID_HEIGHT; y++)
    {
        for (int x = 0; x < Config::GRID_WIDTH; x++)
        {
            inflated_grid[y][x] = grid[y][x];
        }
    }

    // Expand each obstacle cell by SAFETY_MARGIN
    for (int y = 0; y < Config::GRID_HEIGHT; y++)
    {
        for (int x = 0; x < Config::GRID_WIDTH; x++)
        {
            if (!grid[y][x])
            {
                continue;
            }

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

    // Always keep start and goal walkable regardless of inflation
    inflated_grid[static_cast<int>(start.y)][static_cast<int>(start.x)] = false;
    inflated_grid[static_cast<int>(goal.y)] [static_cast<int>(goal.x)] = false;
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

// ─────────────────────────────────────────────
//  Robot
// ─────────────────────────────────────────────

void Robot::init(Vector2 start)
{
    position = start;
    compass = Direction::N;
    battery = Config::BATTERY_MAX;
    speed = Config::MOVE_SPEED;
    wall_following = false;
    stuck_counter = 0;
}

void Robot::move(Direction dir, float dt)
{
    int d = static_cast<int>(dir);
    position.x += DIRECTION_DX[d] * speed * dt;
    position.y += DIRECTION_DY[d] * speed * dt;
    compass = dir;
}

void Robot::setCompass(Direction dir)
{
    compass = dir;
}

void Robot::drainBattery()
{
    battery -= Config::BATTERY_DRAIN;

    if (battery < 0.0f)
    {
        battery = 0.0f;
    }
}

Vector2 Robot::getCell() const
{
    return
    {
        std::floor(position.x),
        std::floor(position.y)
    };
}

// ─────────────────────────────────────────────
//  Sensors
// ─────────────────────────────────────────────

std::array<bool, Sensors::NEIGHBOR_COUNT> Sensors::scanNeighbors(
    const World& world, const Robot& robot) const
{
    std::array<bool, NEIGHBOR_COUNT> blocked = {};

    int rx = static_cast<int>(robot.getCell().x);
    int ry = static_cast<int>(robot.getCell().y);

    for (int i = 0; i < NEIGHBOR_COUNT; i++)
    {
        bool is_diagonal = (DIRECTION_DX[i] != 0 && DIRECTION_DY[i] != 0);

        if (is_diagonal && !Config::SENSOR_DIAGONAL)
        {
            blocked[i] = false;
            continue;
        }

        // Check all cells within SENSOR_RANGE in this direction
        for (int r = 1; r <= Config::SENSOR_RANGE; r++)
        {
            int nx = rx + DIRECTION_DX[i] * r;
            int ny = ry + DIRECTION_DY[i] * r;

            if (!world.inBounds(nx, ny) || world.grid[ny][nx])
            {
                blocked[i] = true;
                break;
            }
        }
    }

    return blocked;
}

Vector2 Sensors::computeResultantForce(const World& world, const Robot& robot) const
{
    // Attractive force: pulls robot toward goal
    Vector2 f_att =
    {
        Config::K_ATT * (world.goal.x - robot.position.x),
        Config::K_ATT * (world.goal.y - robot.position.y)
    };

    // Repulsive force: sum of pushes from all obstacles within REP_RANGE
    Vector2 f_rep = { 0.0f, 0.0f };

    int rx = static_cast<int>(robot.getCell().x);
    int ry = static_cast<int>(robot.getCell().y);

    for (int dy = -Config::REP_RANGE; dy <= Config::REP_RANGE; dy++)
    {
        for (int dx = -Config::REP_RANGE; dx <= Config::REP_RANGE; dx++)
        {
            int nx = rx + dx;
            int ny = ry + dy;

            if (!world.inBounds(nx, ny) || !world.grid[ny][nx])
            {
                continue;
            }

            float dist_x = robot.position.x - static_cast<float>(nx);
            float dist_y = robot.position.y - static_cast<float>(ny);
            float dist = std::sqrt(dist_x * dist_x + dist_y * dist_y);

            if (dist < 0.01f)
            {
                continue;
            }

            // Repulsive force magnitude: K_REP / d^2
            float magnitude = Config::K_REP / (dist * dist);

            f_rep.x += magnitude * (dist_x / dist);
            f_rep.y += magnitude * (dist_y / dist);
        }
    }

    return { f_att.x + f_rep.x, f_att.y + f_rep.y };
}

void Sensors::writeBlackboard(
    BT::Blackboard::Ptr blackboard,
    const World& world,
    const Robot& robot) const
{
    auto neighbors = scanNeighbors(world, robot);
    auto resultant_force = computeResultantForce(world, robot);
    Direction goal_dir = directionTo(robot.position, world.goal);
    bool is_stuck = robot.stuck_counter >= Config::STUCK_THRESHOLD;

    blackboard->set("neighbors_blocked", neighbors);
    blackboard->set("battery_level", robot.battery);
    blackboard->set("robot_position", robot.position);
    blackboard->set("compass_heading", robot.compass);
    blackboard->set("goal_direction", goal_dir);
    blackboard->set("resultant_force", resultant_force);
    blackboard->set("is_stuck", is_stuck);
}

// ─────────────────────────────────────────────
//  Simulation
// ─────────────────────────────────────────────

Simulation::Simulation(BT::Blackboard::Ptr blackboard)
    : blackboard_(blackboard)
{
    world_.generate();
    robot_.init(world_.start);

    // Write initial state to Blackboard before first BT tick
    sensors_.writeBlackboard(blackboard_, world_, robot_);
}

void Simulation::update(float dt)
{
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
