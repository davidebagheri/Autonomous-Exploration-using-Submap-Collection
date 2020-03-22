#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_frontier_selector/euclidean_nearest_frontier_selector.h"

namespace active_3d_planning{
    ModuleFactoryRegistry::Registration<EuclideanNearestFrontierSelector> EuclideanNearestFrontierSelector::registration("EuclideanNearestFrontierSelector");

    EuclideanNearestFrontierSelector::EuclideanNearestFrontierSelector(PlannerI &planner) : GlobalFrontierSelector(planner){}

    bool EuclideanNearestFrontierSelector::selectFrontierToExplore(){
        double distance = INFINITY;
        int best_frontier_id;
        SubmapID best_submap_id;

        // Find the nearest frontier
        for (auto& submap_frontier_pair : submap_frontier_evaluator_->getSubmapFrontiersMap()){
            for(int frontier_id = 0; frontier_id < submap_frontier_pair.second.getFrontiers().size(); frontier_id++){
                voxblox::Point centroid = submap_frontier_pair.second.getFrontier(frontier_id).getCentroid();
                Eigen::Vector3d centroid_d(centroid.x(), centroid.y(), centroid.z());
                // The distance is the norm of the RobotPosition-FronierCentroid vector
                double current_distance = (planner_.getCurrentPosition() - centroid_d).norm();
                if (current_distance < distance){
                    distance = current_distance;
                    best_submap_id = submap_frontier_pair.first;
                    best_frontier_id = frontier_id;
                }
            }
        }

        // This means there are no more frontiers
        if (distance == INFINITY) {
            global_point_gain_ = 0.0;
            return false;
        }

        // Set target frontier and its gain
        target_frontier_id_ = best_frontier_id;
        target_submap_id_ = best_submap_id;
        global_point_gain_ = INFINITY;
        ROS_INFO("[Nearest Frontier selector] Best frontier: %f far in submap %d", distance, target_submap_id_);
        return true;
    }
}