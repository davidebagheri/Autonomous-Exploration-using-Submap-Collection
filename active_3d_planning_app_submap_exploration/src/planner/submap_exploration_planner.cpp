#include "ros/ros.h"
#include "active_3d_planning_app_submap_exploration/planner/submap_exploration_planner.h"
#include <algorithm>
#include <random>

namespace active_3d_planning{
    namespace ros {
        SubmapExplorationPlanner::SubmapExplorationPlanner(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private,
                                                           ModuleFactory *factory,
                                                           Module::ParamMap *param_map)
                                                           : VoxgraphLocalPlanner(nh, nh_private, factory, param_map){
            SubmapExplorationPlanner::setupFromParamMap(param_map);
            advertiseServices();

            // Reset counts
            n_current_tree_samples_ = 0;
            current_tree_max_gain_ = 0.0;

            // Reset flags
            global_trajectory_planned_ = false;
            global_point_computed_ = false;
            global_plan_happened_ = false;
            global_trajectory_executed_ = false;
        }

        void SubmapExplorationPlanner::setupFromParamMap(Module::ParamMap *param_map){
            param_map_ = param_map;

            setParam<float>(param_map, "min_gain_threshold", &min_gain_threshold_, 500);
            setParam<int>(param_map, "n_samples_threshold", &n_samples_threshold_, 100);
            setParam<int>(param_map, "n_sample_tries_threshold", &n_samples_tries_threshold_, 3000);
            setParam<float>(param_map, "global_replan_pos_threshold", &global_replan_pos_threshold_, 0.3);
            setParam<double>(param_map, "nearest_submap_origin_max_distance", &nearest_submap_origin_max_distance_, 5.0);
            setParam<double>(param_map, "plan_time_limit", &plan_time_limit_, 4.0);

            // Setup members
            std::string args; // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "global_point_selector_args", &args,
                                  param_ns + "/global_point_selector");

            global_point_selector_ = getFactory().createModule<GlobalPointSelector>(
                    args, *this, verbose_modules_);
        }

        void SubmapExplorationPlanner::advertiseServices(){
            global_planning_cln_ = nh_private_.serviceClient<mav_planning_msgs::PlannerService>("plan");
            publish_global_trajectory_cln_ = nh_private_.serviceClient<std_srvs::Empty>("publish_path");
            publish_global_trajectory_srv_ = nh_private_.advertiseService("global_trajectory_executed",
                    &SubmapExplorationPlanner::globalTrajectoryCallback,
                    this);
        }

        void SubmapExplorationPlanner::plan(){
            run_srv_ = VoxgraphLocalPlanner::nh_private_.advertiseService("toggle_running", &RosPlanner::runSrvCallback,
                                                                          dynamic_cast<RosPlanner*>(this));
            running_ = true;
            std::clock_t timer;
            while (::ros::ok()) {
                if (planning_) {
                    switch (stateMachine()) {
                        case LocalPlanning:
                            localPlanningIteration();
                            break;
                        case GlobalPlanning:
                            globalPlanningIteration();
                            break;
                    }
                }

                if (p_log_performance_) {
                    timer = std::clock();
                }
                ::ros::spinOnce();
                if (p_log_performance_) {
                    perf_log_data_[5] += (double) (std::clock() - timer) / CLOCKS_PER_SEC;
                }
            }
        }

        PlanningStatus SubmapExplorationPlanner::stateMachine(){
            if (isLocalGainLow()){
                if (global_plan_happened_) {
                    return LocalPlanning;
                }

                // Compute the gain of the best global point
                if (!global_point_computed_){
                    global_point_selector_->computeGlobalPoint();
                    global_point_computed_ = true;
                }
                // Compare with the current maximum gain in the tree
                if (global_point_selector_->getGlobalPointGain() > current_tree_max_gain_) {
                    return GlobalPlanning;
                }
            }
            return LocalPlanning;
        }

        bool SubmapExplorationPlanner::isLocalGainLow(){
            if (((n_current_tree_samples_ > n_samples_threshold_)
                 or (new_segment_tries_ > n_samples_tries_threshold_))
                and (current_tree_max_gain_ < min_gain_threshold_)) {
                return true;
            }
            return false;
        }

        void SubmapExplorationPlanner::localPlanningIteration(){
            if (voxgraph_map_ptr_->hasActiveMapFinished()) {
                // Update the global point selector
                global_point_selector_->update();

                // Remove the oldest submap from the active submap
                if (voxgraph_map_ptr_->getSubmapCollection().getActiveSubmapID() > 1){
                    SubmapID oldest_active_submap_id = voxgraph_map_ptr_->getSubmapCollection().getActiveSubmapID() - 2;
                    voxgraph_map_ptr_->getPlannerMapManager().removeSubmapFromActiveSubmap(
                            oldest_active_submap_id);
                }

                // Reset control variable
                global_point_computed_ = false;
                global_plan_happened_ = false;
                global_trajectory_executed_ = false;
            }
            loopIteration();
        }

        bool SubmapExplorationPlanner::requestNextTrajectory(){
            // Update the planning maps
            voxgraph_map_ptr_->updatePlanningMaps();


            // Request trajectory
            RosPlanner::requestNextTrajectory();

            // Update the counts
            getNumSamplesAndMaxGain();
        }

        void SubmapExplorationPlanner::globalPlanningIteration(){
            static int marker_id = 0;
            if (!global_trajectory_planned_) {
                // Keep on with the local planning if there are no submaps yet
                if (voxgraph_map_ptr_->getSubmapCollection().empty() or
                voxgraph_map_ptr_->getSubmapCollection().getActiveSubmapID() < 1){
                    localPlanningIteration();
                    return;
                }

                // Get the global point
                global_point_selector_->getGlobalPoint(&global_point_goal_);

                // Do not plan to the nearest origin if it is already close to the robot
                Eigen::Vector3d voxel_position(global_point_goal_.pose.position.x,
                                               global_point_goal_.pose.position.y,
                                               global_point_goal_.pose.position.z);

                // The robot is already in the right position or the global point is close to the robot
                if (arrivedToPoint(global_point_goal_) or (voxgraph_map_ptr_->isObserved(voxel_position))) {
                    ROS_INFO("[Global Planning] The robot is already in right position. Continue with the local planning");
                    global_plan_happened_ = true;
                    resetGlobalPlanning();
                    return;
                } else {
                    // Try to reach the given global point
                    if (voxgraph_map_ptr_->isInsideAllSubmaps(voxel_position) && globalPlanToPoint(global_point_goal_)) {
                        ROS_INFO("[Global Planning] Going to the requested global point");
                        global_point_planned_ = global_point_goal_;
                        publishGlobalWaypoints();
                        global_point_selector_->update(false);
                        // Try to reach a point near the given global point
                    } else if (planToNearPoint(global_point_goal_)){
                            ROS_INFO("[Global Planning] Going to a neighbour of the requested global point");
                            publishGlobalWaypoints();
                            global_point_selector_->update(false);
                    } else {
                        // Otherwise go to the nearest submap origin
                        if (planToNearestSubmapOrigin(global_point_goal_)){
                            publishGlobalWaypoints();
                            global_point_selector_->update(false);
                            ROS_INFO("[Global Planning] Going to the nearest submap origin");
                        } else {
                            // White flag: continue with the local planning
                            ROS_INFO("[Global Planning] Not found any trajectory near the requested global point. Continuing the local planning");
                            global_plan_happened_ = true;
                            resetGlobalPlanning();
                            return;
                        }
                    }
                }
            }

            if (arrivedToPoint(global_point_planned_)){
                ROS_INFO("[Global Planning] Robot has arrived to the selected point");
                global_plan_happened_ = true;
                resetLocalPlanning();
                resetGlobalPlanning();
            }
        }

        bool SubmapExplorationPlanner::planToNearPoint(const geometry_msgs::PoseStamped& goal_point){
            std::vector<Eigen::Vector3d> candidates;
            Eigen::Vector3d goal_point_d(goal_point.pose.position.x, goal_point.pose.position.y, goal_point.pose.position.z);

            // Get free points near the goal point
            voxgraph_map_ptr_->getFreeNeighbouringPoints(goal_point_d, &candidates);

            // Try to plan to them in a random order
            auto rng = std::default_random_engine {};
            std::shuffle(candidates.begin(), candidates.end(), rng);

            ::ros::Time begin = ::ros::Time::now();
            for (auto candidate : candidates){
                // Set a time limit
                ::ros::Time begin_iteration = ::ros::Time::now();
                if (((begin_iteration - begin).toSec()) > plan_time_limit_) return false;

                geometry_msgs::PoseStamped candidate_pose;
                candidate_pose.pose.position.x = candidate.x();
                candidate_pose.pose.position.y = candidate.y();
                candidate_pose.pose.position.z = candidate.z();
                if (globalPlanToPoint(candidate_pose)) {
                    global_point_planned_ = candidate_pose;
                    return true;
                }
            }
            return false;
        }

        bool SubmapExplorationPlanner::planToNearestSubmapOrigin(const geometry_msgs::PoseStamped& goal_point){
            geometry_msgs::PoseStamped nearest_submap_origin = getNearestSubmapOrigin(goal_point);

            // Do not plan to it if the point is too far from the real goal
            Eigen::Vector3d nearest_origin_d(nearest_submap_origin.pose.position.x,
                                             nearest_submap_origin.pose.position.y,
                                             nearest_submap_origin.pose.position.z);
            if (distance(nearest_origin_d, goal_point) > nearest_submap_origin_max_distance_) return false;


            if (globalPlanToPoint(nearest_submap_origin)){
                global_point_planned_ = nearest_submap_origin;
                return true;
            }
            return false;
        }

        void SubmapExplorationPlanner::resetLocalPlanning() {
            // Reset the state machine control variables
            global_trajectory_planned_ = false;
            global_point_selector_->resetGlobalPointGain();
            n_current_tree_samples_ = 0;
            current_tree_max_gain_ = 0.0;

            // Reinitialize the local planner and the active area
            RosPlanner::initializePlanning();
            voxgraph_map_ptr_->getPlannerMapManager().emptyActiveSubmap();

            // Reinitialize the local planner backtracker
            std::string args;
            std::string param_ns = (*param_map_)["param_namespace"];
            setParam<std::string>(param_map_, "back_tracker_args", &args,
                                  param_ns + "/back_tracker");

            back_tracker_ = getFactory().createModule<BackTracker>(
                    args, *this, false);
        }

        void SubmapExplorationPlanner::resetGlobalPlanning() {
            global_trajectory_planned_ = false;
            global_point_selector_->resetGlobalPointGain();
        }

        bool SubmapExplorationPlanner::arrivedToPoint(const geometry_msgs::PoseStamped& point){
            return (distance(current_position_, point) < global_replan_pos_threshold_);
        }

        float SubmapExplorationPlanner::distance(const Eigen::Vector3d point_A,
                const geometry_msgs::PoseStamped& point_B){
            return std::sqrt(std::pow(point_B.pose.position.x - point_A.x(),2) +
                             std::pow(point_B.pose.position.y - point_A.y(),2) +
                             std::pow(point_B.pose.position.z - point_A.z(),2));
        }

        bool SubmapExplorationPlanner::globalPlanToPoint(const geometry_msgs::PoseStamped& goal_point){
            mav_planning_msgs::PlannerService srv;

            // Set the current pose from the odometry
            srv.request.start_pose.pose.position.x = current_position_.x();
            srv.request.start_pose.pose.position.y = current_position_.y();
            srv.request.start_pose.pose.position.z = current_position_.z();

            // Set the goal pose
            srv.request.goal_pose.pose = goal_point.pose;

            // Global planning
            global_planning_cln_.call(srv);

            if (srv.response.success) {
                global_trajectory_planned_ = true;
            }
            return srv.response.success;
        }

        void SubmapExplorationPlanner::publishGlobalWaypoints(){
            std_srvs::Empty srv;
            if (publish_global_trajectory_cln_.call(srv)){
                ROS_INFO("[Global Planning] Waypoints published for trajectory computation");
                global_trajectory_executed_ = true;
            }
        }

        void SubmapExplorationPlanner::getNumSamplesAndMaxGain(){
            std::vector<TrajectorySegment *> children;
            TrajectorySegment* child;

            // Initialize the queue
            children.push_back(current_segment_.get());

            // Reinitialize the counts
            n_current_tree_samples_ = 0;
            current_tree_max_gain_ = current_segment_->gain;

            while (!children.empty()){
                // Pop the first element
                child = children[0];
                children.erase(children.begin());

                // Update the counts
                n_current_tree_samples_++;
                if (child->gain > current_tree_max_gain_) current_tree_max_gain_ = child->gain;

                // Enqueue the children
                child->getChildren(&children);
            }
        }

        geometry_msgs::PoseStamped SubmapExplorationPlanner::getNearestSubmapOrigin(const geometry_msgs::PoseStamped& point){
            double distance_to_point = INFINITY;
            geometry_msgs::PoseStamped result;
            for (auto& submap_id : voxgraph_map_ptr_->getSubmapCollection().getIDs()){
                Point submap_origin(voxgraph_map_ptr_->getSubmapCollection().getSubmap(submap_id).getPose().getPosition());
                Eigen::Vector3d submap_origin_d(submap_origin.x(), submap_origin.y(), submap_origin.z());
                if (distance(submap_origin_d, point) < distance_to_point){
                    result.pose.position.x = submap_origin_d.x();
                    result.pose.position.y = submap_origin_d.y();
                    result.pose.position.z = submap_origin_d.z();
                }
            }
            return result;
        }

        bool SubmapExplorationPlanner::globalTrajectoryCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
            res.success = global_trajectory_executed_;
            return true;
        }
    }
}

