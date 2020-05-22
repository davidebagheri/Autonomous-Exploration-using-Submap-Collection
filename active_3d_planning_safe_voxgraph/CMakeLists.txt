cmake_minimum_required(VERSION 2.8.3)
project(active_3d_planning_safe_voxgraph)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package()

cs_add_library(${PROJECT_NAME}
        # Map
        src/map/safe_voxgraph.cpp
        # Initalization
        src/initialization/safe_voxgraph_package.cpp
        )

cs_install()
cs_export()
