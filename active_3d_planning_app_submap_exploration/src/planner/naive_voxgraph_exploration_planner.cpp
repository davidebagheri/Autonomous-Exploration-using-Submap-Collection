#include "active_3d_planning_app_submap_exploration/planner/naive_voxgraph_exploration_planner.h"

namespace active_3d_planning {
    namespace ros {
        VoxgraphExplorationPlanner::VoxgraphExplorationPlanner(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private,
                active_3d_planning::ModuleFactory *factory,
                active_3d_planning::ModuleBase::ParamMap *param_map)
                : RosPlanner(nh, nh_private, factory,param_map){

            // Get pointer to the set of maps
            naive_voxgraph_map_ = dynamic_cast<map::NaiveVoxgraphMap*>(map_.get());
            if (!naive_voxgraph_map_) {
                printError("'VoxgraphExplorationPlanner' requires a map of type 'NaiveVoxgraphMap'!");
            }
        }

        bool VoxgraphExplorationPlanner::requestNextTrajectory() {
            // Force the Voxgraph mapper to compute the Active Submap Esdf
            naive_voxgraph_map_->updateActiveSubmap();

            return RosPlanner::requestNextTrajectory();
        }
    }
}
