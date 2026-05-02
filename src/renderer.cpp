/**
 * @file renderer.cpp
 * @brief Implementation of the Renderer class
 */

#include "renderer.h"

#include <string>
#include <cmath>

// ─────────────────────────────────────────────
//  Colors
// ─────────────────────────────────────────────

static constexpr Color COLOR_GRID_BG = { 245, 245, 245, 255 };  ///< Free cell background
static constexpr Color COLOR_GRID_LINE = { 210, 210, 210, 255 };  ///< Grid line color
static constexpr Color COLOR_OBSTACLE = {  60,  60,  60, 255 };  ///< Obstacle cell
static constexpr Color COLOR_INFLATED = { 180, 180, 180, 255 };  ///< Inflated obstacle margin
static constexpr Color COLOR_ROBOT = {  30, 120, 255, 255 };  ///< Robot fill
static constexpr Color COLOR_ROBOT_DIR = { 255, 255, 255, 255 };  ///< Robot heading indicator
static constexpr Color COLOR_GOAL = {  50, 200,  80, 255 };  ///< Goal cell
static constexpr Color COLOR_FORCE = { 255,  80,  80, 255 };  ///< Force vector arrow
static constexpr Color COLOR_HUD_BG = {   0,   0,   0, 160 };  ///< HUD background
static constexpr Color COLOR_HUD_TEXT = { 240, 240, 240, 255 };  ///< HUD text
static constexpr Color COLOR_BATTERY_OK = {  50, 200,  80, 255 };  ///< Battery bar — normal
static constexpr Color COLOR_BATTERY_LO = { 220,  60,  60, 255 };  ///< Battery bar — low

// ─────────────────────────────────────────────
//  Direction names for HUD
// ─────────────────────────────────────────────

static const char* DIRECTION_NAME[] =
{
    "N", "NE", "E", "SE", "S", "SW", "W", "NW"
};

// ─────────────────────────────────────────────
//  Renderer
// ─────────────────────────────────────────────

Vector2 Renderer::gridToScreen(Vector2 grid_pos) const
{
    return
    {
        grid_pos.x * Config::CELL_SIZE + Config::CELL_SIZE * 0.5f,
        grid_pos.y * Config::CELL_SIZE + Config::CELL_SIZE * 0.5f
    };
}

void Renderer::drawWorld(const World& world)
{
    for (int y = 0; y < Config::GRID_HEIGHT; y++)
    {
        for (int x = 0; x < Config::GRID_WIDTH; x++)
        {
            int px = x * Config::CELL_SIZE;
            int py = y * Config::CELL_SIZE;

            if (world.grid[y][x])
            {
                // Real obstacle
                DrawRectangle(px, py, Config::CELL_SIZE, Config::CELL_SIZE, COLOR_OBSTACLE);
            }
            else if (world.inflated_grid[y][x])
            {
                // Inflated safety margin
                DrawRectangle(px, py, Config::CELL_SIZE, Config::CELL_SIZE, COLOR_INFLATED);
            }
            else
            {
                // Free cell
                DrawRectangle(px, py, Config::CELL_SIZE, Config::CELL_SIZE, COLOR_GRID_BG);
            }

            // Grid line
            DrawRectangleLines(px, py, Config::CELL_SIZE, Config::CELL_SIZE, COLOR_GRID_LINE);
        }
    }
}

void Renderer::drawGoal(const World& world)
{
    int px = static_cast<int>(world.goal.x) * Config::CELL_SIZE;
    int py = static_cast<int>(world.goal.y) * Config::CELL_SIZE;

    DrawRectangle(px, py, Config::CELL_SIZE, Config::CELL_SIZE, COLOR_GOAL);
    DrawText("G",
        px + Config::CELL_SIZE / 2 - 4,
        py + Config::CELL_SIZE / 2 - 6,
        12, WHITE);
}

void Renderer::drawRobot(const Robot& robot)
{
    Vector2 screen = gridToScreen(robot.position);
    float   radius = Config::CELL_SIZE * 0.38f;

    // Robot body
    DrawCircleV(screen, radius, COLOR_ROBOT);

    // Compass heading indicator — line from center toward facing direction
    int d  = static_cast<int>(robot.compass);
    float tip_x = screen.x + DIRECTION_DX[d] * radius * 0.85f;
    float tip_y = screen.y + DIRECTION_DY[d] * radius * 0.85f;

    DrawLineEx(screen, { tip_x, tip_y }, 2.5f, COLOR_ROBOT_DIR);
    DrawCircleV({ tip_x, tip_y }, 2.5f, COLOR_ROBOT_DIR);
}

void Renderer::drawForceVector(const Robot& robot, Vector2 force)
{
    Vector2 origin = gridToScreen(robot.position);

    float mag = std::sqrt(force.x * force.x + force.y * force.y);

    if (mag < 0.01f)
    {
        return;
    }

    // Normalize and scale for display
    float scale  = Config::CELL_SIZE * 1.5f;
    Vector2 tip  =
    {
        origin.x + (force.x / mag) * scale,
        origin.y + (force.y / mag) * scale
    };

    DrawLineEx(origin, tip, 2.0f, COLOR_FORCE);
    DrawCircleV(tip, 4.0f, COLOR_FORCE);
}

void Renderer::drawHUD(const Robot& robot)
{
    // HUD background
    DrawRectangle(8, 8, 200, 90, COLOR_HUD_BG);

    // Battery label
    DrawText("Battery", 16, 16, 12, COLOR_HUD_TEXT);

    // Battery bar background
    DrawRectangle(16, 32, 160, 12, { 80, 80, 80, 255 });

    // Battery bar fill
    float ratio    = robot.battery / Config::BATTERY_MAX;
    int   bar_w    = static_cast<int>(160 * ratio);
    Color bar_col  = robot.battery < Config::BATTERY_LOW ? COLOR_BATTERY_LO : COLOR_BATTERY_OK;
    DrawRectangle(16, 32, bar_w, 12, bar_col);

    // Battery percentage text
    std::string bat_str = std::to_string(static_cast<int>(robot.battery)) + "%";
    DrawText(bat_str.c_str(), 182, 30, 12, COLOR_HUD_TEXT);

    // Compass heading
    std::string compass_str = "Heading: ";
    compass_str += DIRECTION_NAME[static_cast<int>(robot.compass)];
    DrawText(compass_str.c_str(), 16, 52, 12, COLOR_HUD_TEXT);

    // Wall following status
    std::string mode_str = robot.wall_following ? "Mode: Wall Follow" : "Mode: Potential Fields";
    DrawText(mode_str.c_str(), 16, 70, 12, COLOR_HUD_TEXT);

    // Stuck counter
    std::string stuck_str = std::to_string(robot.stuck_counter);
    DrawText(stuck_str.c_str(), 16, 88, 12, COLOR_HUD_TEXT);
}

void Renderer::draw(const Simulation& simulation)
{
    const World& world = simulation.getWorld();
    const Robot& robot = simulation.getRobot();

    drawWorld(world);
    drawGoal(world);
    drawRobot(robot);

    // Draw force vector only when not wall following
    if (!robot.wall_following)
    {
        // Read resultant force directly from robot position delta
        // Force visualization uses the compass direction as proxy
        int d = static_cast<int>(robot.compass);
        Vector2 force =
        {
            static_cast<float>(DIRECTION_DX[d]),
            static_cast<float>(DIRECTION_DY[d])
        };
        drawForceVector(robot, force);
    }

    drawHUD(robot);
}