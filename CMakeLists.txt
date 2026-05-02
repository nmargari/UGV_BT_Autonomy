cmake_minimum_required(VERSION 3.20)
project(UGV_BT_Autonomy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp REQUIRED)
find_package(raylib REQUIRED)

add_executable(UGV_BT_Autonomy
    src/main.cpp
    src/simulation.cpp
    src/bt_engine.cpp
    src/renderer.cpp
    src/nodes/is_battery_low.cpp
    src/nodes/is_goal_reached.cpp
    src/nodes/is_direction_clear.cpp
    src/nodes/move_toward_goal.cpp
    src/nodes/wall_follow.cpp
    src/nodes/return_to_base.cpp
)

target_include_directories(UGV_BT_Autonomy PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(UGV_BT_Autonomy PRIVATE
    BT::behaviortree_cpp
    raylib
)