#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_frontier_selector/submap_frontier_evaluator/normalized_frontier_selector.h"

namespace active_3d_planning{
    ModuleFactoryRegistry::Registration<NormalizedFrontierSelector> NormalizedFrontierSelector::registration("NormalizedFrontierSelector");


    NormalizedFrontierSelector::NormalizedFrontierSelector(PlannerI &planner) : NearestFrontierSelector(planner){}

    bool NormalizedFrontierSelector::selectFrontierToExplore() {
        double value = 0.0;
        int n_best_frontier_points = 0;
        int best_frontier_id;
        SubmapID best_submap_id;

        // Find the nearest frontier
        for (auto &submap_frontiers_pair : submap_frontier_evaluator_->getSubmapFrontiersMap()) {
            SubmapFrontiers &submap_frontiers = submap_frontiers_pair.second;
            if (!submap_frontiers.empty()) {
                // Request the plan the path from the robot position to the origin of the submap containing the frontiers
                if (planToPoint(submap_frontiers.getOrigin())) {
                    // Request the path length
                    double current_distance = getPathLength();

                    for (int frontier_id = 0; frontier_id < submap_frontiers.getFrontiers().size(); frontier_id++) {

                        // Get number of points
                        double n_points = submap_frontiers.getFrontier(frontier_id).getNumberOfPoints();

                        // Compute value
                        double current_value = n_points / current_distance;

                        // Compare the values
                        if (current_value > value) {
                            value = current_value;
                            best_submap_id = submap_frontiers_pair.first;
                            best_frontier_id = frontier_id;
                            n_best_frontier_points = n_points;
                        }
                    }
                }
            }
        }

        // No frontiers found
        if (value == 0.0) {
            global_point_gain_ = 0.0;
            return false;
        } else {
            // Set target frontier and its gain
            target_frontier_id_ = best_frontier_id;
            target_submap_id_ = best_submap_id;
            global_point_gain_ = n_best_frontier_points;
            return true;
        }

    }

}