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
        # Tools
        src/tools/transformations.cpp
        # Modules
        # Trajectory generator
        src/module/trajectory_generator/rrt_star_voxgraph.cpp
        # Trajectory evaluator
        src/module/sensor_model/iterative_ray_caster_voxgraph.cpp
        src/module/trajectory_evaluator/naive_evaluator_voxgraph.cpp
        src/module/trajectory_evaluator/voxel_weight_evaluator_voxgraph.cpp
        src/module/trajectory_evaluator/loop_closure_evaluator.cpp
        # Global point selector
        src/module/global_point_selector/global_point_selector.cpp
        # Frontier selector
        src/module/global_point_selector/global_frontier_selector/submap_frontier_evaluator/frontier.cpp
        src/module/global_point_selector/global_frontier_selector/submap_frontier_evaluator/submap_frontiers.cpp
        src/module/global_point_selector/global_frontier_selector/submap_frontier_evaluator/submap_frontier_evaluator.cpp
        src/module/global_point_selector/global_frontier_selector/global_frontier_selector.cpp
        src/module/global_point_selector/global_frontier_selector/biggest_frontier_selector.cpp
        src/module/global_point_selector/global_frontier_selector/euclidean_nearest_frontier_selector.cpp
        src/module/global_point_selector/global_frontier_selector/euclidean_normalized_frontier_selector.cpp
        src/module/global_point_selector/global_frontier_selector/nearest_frontier_selector.cpp
        src/module/global_point_selector/global_frontier_selector/normalized_frontier_selector.cpp
        # Planners
        src/planner/voxgraph_local_planner.cpp
        src/planner/submap_exploration_planner.cpp
        src/planner/naive_voxgraph_exploration_planner.cpp
        )

############
# BINARIES #
############
cs_add_executable(naive_voxgraph_exploration_planner_node
        src/naive_voxgraph_exploration_planner_node.cpp
        )
target_link_libraries(naive_voxgraph_exploration_planner_node ${catkin_LIBRARIES} ${PROJECT_NAME})

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
