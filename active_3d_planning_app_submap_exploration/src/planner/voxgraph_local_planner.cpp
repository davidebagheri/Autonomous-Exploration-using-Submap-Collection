#include "active_3d_planning_app_submap_exploration/planner/voxgraph_local_planner.h"
#include "active_3d_planning_ros/module/module_factory_ros.h"
#include "active_3d_planning_voxgraph/map/voxgraph.h"
#include "voxgraph_mapper_msgs/PlannerMapsSrv.h"

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
            if (voxgraph_map_ptr_->hasActiveMapFinished()) {// &&
                /*std::ofstream fout;
                std::ofstream fout1;
                fout.open("/home/davide/Desktop/local_planner_analysis1.txt", std::ios::app );
                fout1.open("/home/davide/Desktop/map_time.txt", std::ios::app );
                if (!fout.is_open()) std::cout << "not open";
                if (!fout1.is_open()) std::cout << "not open";

                fout << "\n--------------------" << voxgraph_map_ptr_->getSubmapCollection().getActiveSubmapID() + 1
                << "------------------------\n";
                fout1 << "\n--------------------" << voxgraph_map_ptr_->getSubmapCollection().getActiveSubmapID() + 1
                     << "------------------------\n";*/
                if (voxgraph_map_ptr_->getSubmapCollection().getActiveSubmapID() > 1) {
                    voxgraph_map_ptr_->getPlannerMapManager().removeSubmapFromActiveSubmap(
                            voxgraph_map_ptr_->getSubmapCollection().getActiveSubmapID() - 2);
                }
            }

            // Update the planning maps
            voxgraph_map_ptr_->updatePlanningMaps();
            return RosPlanner::requestNextTrajectory();
        }

        double VoxgraphLocalPlanner::getNumSamples(){
            std::vector<TrajectorySegment *> children;
            TrajectorySegment* child;

            // Initialize the queue
            children.push_back(current_segment_.get());

            // Reinitialize the counts
            int n_current_tree_samples = 0;
            while (!children.empty()){
                // Pop the first element
                child = children[0];
                children.erase(children.begin());

                // Update the counts
                n_current_tree_samples++;

                // Enqueue the children
                child->getChildren(&children);
            }
            return n_current_tree_samples;
        }

    }
}
