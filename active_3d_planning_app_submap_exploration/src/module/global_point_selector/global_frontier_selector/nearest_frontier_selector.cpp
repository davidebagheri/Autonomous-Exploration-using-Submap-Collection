#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_frontier_selector/nearest_frontier_selector.h"

namespace active_3d_planning{
    ModuleFactoryRegistry::Registration<NearestFrontierSelector> NearestFrontierSelector::registration("NearestFrontierSelector");

    NearestFrontierSelector::NearestFrontierSelector(PlannerI &planner) : GlobalFrontierSelector(planner){
        ros::NodeHandle nh("");
        ros::NodeHandle nh_private("~");

        plan_cln_ = nh_private.serviceClient<mav_planning_msgs::PlannerService>("plan");
        path_length_cln_ = nh_private.serviceClient<cblox_planning_msgs::PathLengthSrv>("path_length");
    }

    bool NearestFrontierSelector::selectFrontierToExplore(){
        double distance = INFINITY;
        int n_best_frontier_points = 0;
        int best_frontier_id;
        SubmapID best_submap_id;

        // Find the nearest frontier
        for (auto& submap_frontiers_pair : submap_frontier_evaluator_->getSubmapFrontiersMap()) {
            SubmapFrontiers &submap_frontiers = submap_frontiers_pair.second;
            if (!submap_frontiers.empty()) {
                // Request the plan the path from the robot position to the origin of the submap containing the frontiers
                if (planToPoint(submap_frontiers.getOrigin())) {
                    // Request the path length
                    double current_distance = getPathLength();

                    if (current_distance < distance){
                        best_submap_id = submap_frontiers_pair.first;
                        distance = current_distance;
                    }
                }
            }
        }

        // This means that no submap frontiers origin are reachable or there are no more frontiers
        if (distance == INFINITY) {
            global_point_gain_ = 0.0;
            return false;
        }

        // Choose the biggest frontier among the selected submap frontiers
        for(int frontier_id = 0;
        frontier_id < submap_frontier_evaluator_->getSubmapFrontiers(best_submap_id).getFrontiers().size();
        frontier_id++){
            int n_current_frontier_points =
                    submap_frontier_evaluator_->getSubmapFrontiers(best_submap_id).
                    getFrontier(frontier_id).getNumberOfPoints();
            if (n_current_frontier_points > n_best_frontier_points){
                n_best_frontier_points = n_current_frontier_points;
                best_frontier_id = frontier_id;
            }
        }

        // Set target frontier and its gain
        target_frontier_id_ = best_frontier_id;
        target_submap_id_ = best_submap_id;
        global_point_gain_ = n_best_frontier_points;
        ROS_INFO("[Nearest Frontier selector] Best frontier: %f m far in submap %d", distance, target_submap_id_);
        return true;
    }

    bool NearestFrontierSelector::planToPoint(const Point& point){
        mav_planning_msgs::PlannerService srv;

        // Set the current pose from the odometry
        srv.request.start_pose.pose.position.x = planner_.getCurrentPosition().x();
        srv.request.start_pose.pose.position.y = planner_.getCurrentPosition().y();
        srv.request.start_pose.pose.position.z = planner_.getCurrentPosition().z();

        // Set the goal pose
        srv.request.goal_pose.pose.position.x = point.x();
        srv.request.goal_pose.pose.position.y = point.y();
        srv.request.goal_pose.pose.position.z = point.z();

        // Global planning
        plan_cln_.call(srv);
        ROS_WARN("RISULTATO DI PLAN TO POINT %d", srv.response.success);
        return srv.response.success;
    }

    double NearestFrontierSelector::getPathLength(){
        cblox_planning_msgs::PathLengthSrv srv;

        if (path_length_cln_.call(srv)){
            return srv.response.path_length.data;
        } else {
            return INFINITY;
        }


    }

}