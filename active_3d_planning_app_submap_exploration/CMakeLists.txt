cmake_minimum_required(VERSION 2.8.3)
project(active_3d_planning_app_submap_exploration)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        # Modules
        src/module/sensor_model/iterative_ray_caster_voxgraph.cpp
        src/module/trajectory_evaluator/naive_evaluator_voxgraph.cpp
        src/module/trajectory_evaluator/voxel_weight_evaluator.cpp
        # Global point selector
        src/global_point_selector/global_point_selector.cpp
        # Frontier selector
        src/global_point_selector/global_frontier_selector/submap_frontier_evaluator/frontier.cpp
        src/global_point_selector/global_frontier_selector/submap_frontier_evaluator/submap_frontiers.cpp
        src/global_point_selector/global_frontier_selector/submap_frontier_evaluator/submap_frontier_evaluator.cpp
        src/global_point_selector/global_frontier_selector/biggest_frontier_selector.cpp
        # Planners
        src/planner/voxgraph_local_planner.cpp
        src/planner/submap_exploration_planner.cpp
        )

############
# BINARIES #
############

cs_add_executable(voxgraph_local_planner_node
        src/voxgraph_local_planner_node.cpp
        )
target_link_libraries(voxgraph_local_planner_node ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_add_executable(submap_exploration_planner_node
        src/submap_exploration_planner_node.cpp
        )
target_link_libraries(submap_exploration_planner_node ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_add_executable(voxgraph_node_evaluator
        src/experiments/voxgraph_node_evaluator.cpp
        )
target_link_libraries(voxgraph_node_evaluator ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()
