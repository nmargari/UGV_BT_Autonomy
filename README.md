# UGV_BT_Autonomy

A simulation of an autonomous Unmanned Ground Vehicle (UGV) navigating a 2D obstacle grid using a **Behavior Tree** for decision-making, **Potential Fields** for reactive navigation, and **Wall Following** as a fallback when the robot gets stuck.

---

## Technologies

| Technology | Version | Role |
|---|---|---|
| C++ | 17 | Implementation language |
| [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) | 4.9.0 | Behavior Tree engine |
| [Raylib](https://www.raylib.com/) | 4.5.0 | 2D simulation rendering |
| CMake | 3.20+ | Build system |
| Doxygen + Graphviz | — | API documentation generation |

---

## Architecture

The program is structured around **5 components** that communicate through a shared Blackboard:

```
Main Loop
├── Simulation (World · Robot · Sensors)
│   └── writes sensor data → Blackboard
├── BT Engine (Conditions · Actions)
│   ├── reads sensor data ← Blackboard
│   └── drives robot movement → Robot
└── Renderer
    └── reads state ← Simulation
```

### Main Loop

Runs at a fixed timestep (`Config::FPS_TARGET`) using an accumulator pattern for deterministic simulation. Each iteration:

1. `simulation.update()` — drains battery and writes fresh sensor data to the Blackboard
2. `bt_engine.tick()` — the BT reads the Blackboard and moves the robot
3. `renderer.draw()` — draws the current world state

### Simulation

Contains three sub-components:

- **World** — a 2D boolean grid of size `GRID_WIDTH × GRID_HEIGHT`. Randomly generated at startup with a configurable obstacle ratio. Maintains a second `inflated_grid` where obstacles are expanded by `SAFETY_MARGIN` cells in all directions, used by sensors to determine safe movement options.
- **Robot** — holds the current position (continuous), compass heading (`Direction` enum), battery level, speed, and wall-following state. Movement is driven exclusively by BT action nodes.
- **Sensors** — scans up to 8 neighboring cells within `SENSOR_RANGE`, computes the Potential Fields resultant force, and writes all results to the Blackboard every tick.

### Blackboard

The shared key-value store used for all communication between Simulation and BT Engine. No component accesses another directly — all data flows through the Blackboard.

| Key | Type | Written by | Description |
|---|---|---|---|
| `neighbors_blocked` | `array<bool, 8>` | Sensors | Blocked state per direction (N NE E SE S SW W NW) |
| `battery_level` | `float` | Sensors | Current battery level |
| `robot_position` | `Vector2` | Sensors | Current robot position in grid coordinates |
| `compass_heading` | `Direction` | Sensors | Current robot facing direction |
| `goal_direction` | `Direction` | Sensors | Closest compass direction toward goal |
| `resultant_force` | `Vector2` | Sensors | Potential Fields resultant force vector |
| `is_stuck` | `bool` | Sensors | True if robot has not made progress for `STUCK_THRESHOLD` steps |

### Navigation Strategy

Navigation is handled by two complementary algorithms managed by the Behavior Tree:

**Potential Fields (primary)** — each cell exerts a force on the robot:
- Attractive force from the goal pulls the robot toward it
- Repulsive forces from nearby obstacles push the robot away
- The robot moves in the direction of the resultant force, snapped to the nearest of the 8 compass directions

**Wall Following (fallback)** — activated when the robot is stuck in a local minimum:
- The robot follows the wall using the right-hand rule (configurable)
- Continues until the resultant force points away from the wall and progress resumes

The **compass** supports wall following by providing orientation awareness — the robot needs to know which side is "right" relative to its current heading.

### BT Engine

Builds and ticks the Behavior Tree using BehaviorTree.CPP. The tree structure is:

```
Root Selector
├── Sequence — Emergency
│   ├── IsBatteryLow?
│   └── ReturnToBase
└── Sequence — Navigate
    ├── Selector — Movement
    │   ├── Sequence — Potential Fields
    │   │   ├── IsDirectionClear?
    │   │   └── MoveTowardGoal
    │   └── WallFollow
    └── IsGoalReached?
```

### Renderer

Reads the Simulation state and draws the world using Raylib. Draws grid cells, obstacles, robot position, compass heading, goal, and the resultant force vector.

---

## Configuration

All parameters are defined as `constexpr` values in `include/config.h`. No recompilation of other files is needed when changing parameters.

| Parameter | Default | Description |
|---|---|---|
| `GRID_WIDTH` | 30 | Grid columns |
| `GRID_HEIGHT` | 30 | Grid rows |
| `OBSTACLE_RATIO` | 0.2 | Fraction of cells that are obstacles |
| `SAFETY_MARGIN` | 1 | Cells of clearance around obstacles |
| `MOVE_SPEED` | 5.0 | Robot speed in cells/second |
| `BATTERY_MAX` | 100.0 | Maximum battery level |
| `BATTERY_DRAIN` | 0.1 | Battery drain per simulation step |
| `BATTERY_LOW` | 20.0 | Threshold for return-to-base |
| `COMPASS_8DIR` | true | Enable 8-directional compass |
| `SENSOR_RANGE` | 1 | Sensor range in cells |
| `SENSOR_DIAGONAL` | true | Enable diagonal neighbor detection |
| `K_ATT` | 1.0 | Attractive force constant toward goal |
| `K_REP` | 2.0 | Repulsive force constant from obstacles |
| `REP_RANGE` | 3 | Obstacle repulsion range in cells |
| `STUCK_THRESHOLD` | 10 | Steps without progress before wall following |
| `GOAL_REACH_DIST` | 0.5 | Distance in cells to consider goal reached |
| `WALL_FOLLOW_RIGHT` | true | Follow wall on the right side |
| `CELL_SIZE` | 24 | Pixels per grid cell |
| `SCREEN_WIDTH` | 1280 | Window width in pixels |
| `SCREEN_HEIGHT` | 720 | Window height in pixels |
| `FPS_TARGET` | 60.0 | Target frames per second |

---

## Project Structure

```
UGV_BT_Autonomy/
├── CMakeLists.txt
├── Doxyfile
├── README.md
├── include/
│   ├── config.h          # All simulation parameters
│   ├── simulation.h      # World, Robot, Sensors, Simulation
│   ├── bt_engine.h       # BT setup and tick
│   ├── renderer.h        # Raylib 2D rendering
│   └── nodes/
│       ├── is_battery_low.h
│       ├── is_direction_clear.h
│       ├── is_goal_reached.h
│       ├── move_toward_goal.h
│       ├── wall_follow.h
│       └── return_to_base.h
└── src/
    ├── main.cpp
    ├── simulation.cpp
    ├── bt_engine.cpp
    ├── renderer.cpp
    └── nodes/
        ├── move_toward_goal.cpp
        ├── wall_follow.cpp
        └── return_to_base.cpp
```

---

## Building

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/UGV_BT_Autonomy
```

---

## Generating Documentation

```bash
doxygen Doxyfile
xdg-open docs/html/index.html
```

---

## Future Work

This project is designed as the first step toward a larger thesis on drone swarm orchestration. Planned extensions include:

- Integration with ROS 2 and Gazebo for realistic simulation
- Multi-agent coordination using a shared Blackboard over P2P communication
- Extension of the BT orchestration layer to a full swarm abstraction layer
