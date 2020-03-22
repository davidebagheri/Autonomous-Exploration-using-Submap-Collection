#include "active_3d_planning_ros/module/module_factory_ros.h"
#include "active_3d_planning_app_submap_exploration/planner/naive_voxgraph_exploration_planner.h"

#include "active_3d_planning_naive_voxgraph/initialization/naive_voxgraph_map_package.h"
#include "active_3d_planning_mav/initialization/mav_package.h"

#include <glog/logging.h>
#include <chrono>
#include <thread>

int main(int argc, char **argv) {
    // leave some time for the rest to settle
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // init ros
    ros::init(argc, argv, "voxgraph_exploration_planner_node");

    // prevent the linker from optimizing these packages away...
    active_3d_planning::initialize::maps_package();
    active_3d_planning::initialize::mav_package();

    // Set logging to debug for testing
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    google::InitGoogleLogging(argv[0]);

    // node handles
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    // Setup
    active_3d_planning::ros::ModuleFactoryROS factory;
    active_3d_planning::Module::ParamMap param_map;
    active_3d_planning::ros::VoxgraphExplorationPlanner::setupFactoryAndParams(&factory, &param_map, nh_private);

    // Create and launch the planner
    active_3d_planning::ros::VoxgraphExplorationPlanner node(nh, nh_private, &factory, &param_map);
    node.planningLoop();
}
