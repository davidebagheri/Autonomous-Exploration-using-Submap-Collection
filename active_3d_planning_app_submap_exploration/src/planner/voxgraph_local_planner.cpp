#include "active_3d_planning_app_submap_exploration/planner/voxgraph_local_planner.h"
#include "active_3d_planning_ros/module/module_factory_ros.h"
#include "active_3d_planning_voxgraph/map/voxgraph.h"

namespace active_3d_planning {
    namespace ros {
        VoxgraphLocalPlanner::VoxgraphLocalPlanner(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private,
                                                   active_3d_planning::ModuleFactory *factory,
                                                   active_3d_planning::ModuleBase::ParamMap *param_map)
                : RosPlanner(nh, nh_private, factory, param_map) {
            // Get pointer to the VoxgraphMap
            voxgraph_map_ptr_ = dynamic_cast<map::VoxgraphMap *>(map_.get());
            if (!voxgraph_map_ptr_) {
                printError("'VoxgraphLocalPlanner' requires a map of type 'VoxgraphMap'!");
            }
        }

        bool VoxgraphLocalPlanner::requestNextTrajectory() {
            if (voxgraph_map_ptr_->hasActiveMapFinished() &&
            voxgraph_map_ptr_->getSubmapCollection().getActiveSubmapID() > 1){
                voxgraph_map_ptr_->getPlannerMapManager().removeSubmapFromActiveSubmap(
                        voxgraph_map_ptr_->getSubmapCollection().getActiveSubmapID() - 2);
            }

            // Update the planning maps
            voxgraph_map_ptr_->updatePlanningMaps();
            return RosPlanner::requestNextTrajectory();
        }
    }
}
