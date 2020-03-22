cmake_minimum_required(VERSION 2.8.3)
project(active_3d_planning_naive_voxgraph)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package()

cs_add_library(${PROJECT_NAME}
        src/map/naive_voxgraph_map.cpp
        src/initialization/naive_voxgraph_map_package.cpp
        )

cs_install()
cs_export()
