cmake_minimum_required(VERSION 2.8.3)
project(active_3d_planning_voxgraph)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package()

cs_add_library(${PROJECT_NAME} SHARED
        # Map
        src/map/voxgraph.cpp
        # Initalization
        src/initialization/voxgraph_package.cpp
        # Planner Map Manager
        src/planner_map_manager/tools/merge_layer.cpp
        src/planner_map_manager/planner_map_manager.cpp
        )

cs_install()
cs_export()
