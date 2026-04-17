cmake_minimum_required(VERSION 3.20)
project(UGV_BT_Autonomy)

set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# BehaviorTree.CPP
find_package(behaviortree_cpp REQUIRED)

# Raylib
find_package(raylib REQUIRED)

# Executable
add_executable(UGV_BT_Autonomy
    src/main.cpp
    src/simulation.cpp
    src/bt_engine.cpp
    src/renderer.cpp
)

target_include_directories(UGV_BT_Autonomy PRIVATE
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${RLIMGUI_DIR}
)

target_link_libraries(UGV_BT_Autonomy PRIVATE
    BT::behaviortree_cpp
    raylib
)