#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_frontier_selector/biggest_frontier_selector.h"

namespace active_3d_planning{
    ModuleFactoryRegistry::Registration<BiggestFrontierSelector> BiggestFrontierSelector::registration("BiggestFrontierSelector");

    BiggestFrontierSelector::BiggestFrontierSelector(PlannerI &planner) : GlobalFrontierSelector(planner){
        ros::NodeHandle nh_private("~");
        test_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>("centroids", 1, true);
    }

    bool BiggestFrontierSelector::selectFrontierToExplore(){
        int n_best_frontier_points = 0;
        int best_frontier_id;
        SubmapID best_submap_id;

        // Find the frontier with the maximum number of points
        for (auto& submap_frontier_pair : submap_frontier_evaluator_->getSubmapFrontiersMap()){
            for(int frontier_id = 0; frontier_id < submap_frontier_pair.second.getFrontiers().size(); frontier_id++){
                std::cout << submap_frontier_pair.second.getFrontier(frontier_id).getNumberOfPoints() << std::endl;
                int n_current_frontier_points = submap_frontier_pair.second.getFrontier(frontier_id).getNumberOfPoints();

                if (n_current_frontier_points > n_best_frontier_points){
                    n_best_frontier_points = n_current_frontier_points;
                    best_frontier_id = frontier_id;
                    best_submap_id = submap_frontier_pair.first;
                }
            }
        }
        // This means there are no more frontiers
        if (n_best_frontier_points == 0) {
            global_point_gain_ = 0.0;
            return false;
        }

        // Set target frontier and its gain
        target_frontier_id_ = best_frontier_id;
        target_submap_id_ = best_submap_id;
        global_point_gain_ = INFINITY;
        ROS_INFO("[Frontier selector] Best frontier: %d points in submap %d", n_best_frontier_points, target_submap_id_);
        return true;
    }
}