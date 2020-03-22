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
            if (naive_voxgraph_map_->hasActiveMapFinished()){
                /*std::ofstream fout;
                fout.open("/home/davide/Desktop/naive_planner_analysis1.txt", std::ios::app );
                if (!fout.is_open()) std::cout << "not open";

                fout << "\n--------------------" << naive_voxgraph_map_->getVoxgraphMapper().getSubmapCollection().getActiveSubmapID() + 1
                     << "------------------------\n";*/

            }
            return RosPlanner::requestNextTrajectory();
        }

        double VoxgraphExplorationPlanner::getNumSamples(){
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
