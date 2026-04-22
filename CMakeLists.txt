cmake_minimum_required(VERSION 3.20)
project(UGV_BT_Autonomy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp REQUIRED)
find_package(raylib REQUIRED)

add_executable(UGV_BT_Autonomy
    src/main.cpp
    src/simulation.cpp
)

target_include_directories(UGV_BT_Autonomy PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(UGV_BT_Autonomy PRIVATE
    BT::behaviortree_cpp
    raylib
)
