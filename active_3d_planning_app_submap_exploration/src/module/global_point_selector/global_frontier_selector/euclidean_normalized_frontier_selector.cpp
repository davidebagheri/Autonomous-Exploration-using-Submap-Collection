#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_frontier_selector/euclidean_normalized_frontier_selector.h"

namespace active_3d_planning {
    ModuleFactoryRegistry::Registration<EuclideanNormalizedFrontierSelector> EuclideanNormalizedFrontierSelector::registration(
            "EuclideanNormalizedFrontierSelector");

    EuclideanNormalizedFrontierSelector::EuclideanNormalizedFrontierSelector(PlannerI &planner) : GlobalFrontierSelector(
            planner) {}

    bool EuclideanNormalizedFrontierSelector::selectFrontierToExplore() {
        double value = 0.0;
        int best_frontier_id;
        SubmapID best_submap_id;
        int best_frontier_n_points;

        // Find the nearest frontier
        for (auto& submap_frontier_pair : submap_frontier_evaluator_->getSubmapFrontiersMap()){
            for(int frontier_id = 0; frontier_id < submap_frontier_pair.second.getFrontiers().size(); frontier_id++){

                // Compute euclidean distance to centroid
                voxblox::Point centroid = submap_frontier_pair.second.getFrontier(frontier_id).getCentroid();
                Eigen::Vector3d centroid_d(centroid.x(), centroid.y(), centroid.z());
                double euclidean_distance = (planner_.getCurrentPosition() - centroid_d).norm();

                // Get number of points
                double n_points = submap_frontier_pair.second.getFrontier(frontier_id).getNumberOfPoints();

                // Compute value
                double current_value = n_points / euclidean_distance;

                if (current_value > value){
                    value = current_value;
                    best_submap_id = submap_frontier_pair.first;
                    best_frontier_id = frontier_id;
                    best_frontier_n_points = n_points;
                }
            }
        }

        // This means there are no more frontiers
        if (value == 0.0) {
            global_point_gain_ = 0.0;
            return false;
        }

        // Set target frontier and its gain
        target_frontier_id_ = best_frontier_id;
        target_submap_id_ = best_submap_id;
        global_point_gain_ = best_frontier_n_points;
        ROS_INFO("[Euclidean Normalized Frontier selector] Best frontier: %d points in submap %d", best_frontier_n_points, target_submap_id_);
        return true;
    }
}