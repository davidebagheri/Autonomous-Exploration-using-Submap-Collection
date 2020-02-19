#include "ros/ros.h"
#include "active_3d_planning_app_submap_exploration/planner/submap_exploration_planner.h"

namespace active_3d_planning{
    namespace ros {
        SubmapExplorationPlanner::SubmapExplorationPlanner(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private,
                                                           ModuleFactory *factory,
                                                           Module::ParamMap *param_map)
                                                           : VoxgraphLocalPlanner(nh, nh_private, factory, param_map){
            SubmapExplorationPlanner::setupFromParamMap(param_map);
            advertiseServices();
            advertisePublishers();
            n_current_tree_samples_ = 0;
            current_tree_max_gain_ = 0;
            global_trajectory_planned_ = false;
        }

        void SubmapExplorationPlanner::setupFromParamMap(Module::ParamMap *param_map){
            setParam<float>(param_map, "min_gain_threshold", &min_gain_threshold_, 100);
            setParam<int>(param_map, "n_samples_threshold", &n_samples_threshold_, 100);
            setParam<int>(param_map, "n_sample_tries_threshold", &n_samples_tries_threshold_, 1000);
            setParam<float>(param_map, "global_replan_pos_threshold", &global_replan_pos_threshold_, 0.5);

            // Setup members
            std::string args; // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "global_point_selector_args", &args,
                                  param_ns + "/global_point_selector");

            global_point_selector_ = getFactory().createModule<GlobalPointSelector>(
                    args, *this, verbose_modules_);
        }

        void SubmapExplorationPlanner::advertisePublishers(){
            goal_point_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
                    "global_point",0);
        }

        void SubmapExplorationPlanner::advertiseServices(){
            global_planning_cln_ = nh_private_.serviceClient<mav_planning_msgs::PlannerService>("plan");
            publish_global_trajectory_cln_ = nh_private_.serviceClient<std_srvs::Empty>("publish_path");

            switch_plan_ = nh_private_.advertiseService("switch_plan", &SubmapExplorationPlanner::switchPlan, this);
        }

        bool SubmapExplorationPlanner::switchPlan(std_srvs::Empty::Request &req, std_srvs::Empty::Response& res){
           static Eigen::Vector3d start;
            static Eigen::Vector3d end;
            static Eigen::Vector3d now;

            if (plan_flag_==true) {
                resetLocalPlanning();
            }
            plan_flag_ = !plan_flag_;
            return true;
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
            if ((plan_flag_) or
              (((n_current_tree_samples_ > n_samples_threshold_)
                    or (new_segment_tries_ > n_samples_tries_threshold_))
                    and (current_tree_max_gain_ < min_gain_threshold_))){
                return GlobalPlanning;
            }
            return LocalPlanning;
        }

        void SubmapExplorationPlanner::localPlanningIteration(){
            getNumSamplesAndMaxGain();


            static SubmapID active_submap_id = 0;

            if (!voxgraph_map_ptr_->getSubmapCollection().empty()) {
                if (voxgraph_map_ptr_->getSubmapCollection().getActiveSubmapID() != active_submap_id){
                    active_submap_id = voxgraph_map_ptr_->getSubmapCollection().getActiveSubmapID();
                    global_point_selector_->update(active_submap_id - 1);
                }
            }
            loopIteration();
        }


        void SubmapExplorationPlanner::globalPlanningIteration(){
            if (!global_trajectory_planned_) {
                // Request best point and compute trajectory
                if ((voxgraph_map_ptr_->getSubmapCollection().empty()) or
                        !(global_point_selector_->getGlobalPoint(&global_point_goal_))) {
                    loopIteration();
                    getNumSamplesAndMaxGain();
                    return;
                }

                // The robot is already in the right position
                if (arrivedToPoint(global_point_goal_)) {
                    resetLocalPlanning();
                    return;
                }

                // Planning to the requested point
                if (globalPlanToPoint(global_point_goal_)) {

                    visualization_msgs::Marker marker;

                    marker.header.frame_id = "world";
                    marker.header.stamp = ::ros::Time();
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.scale.x = 0.20;
                    marker.scale.y = 0.20;
                    marker.scale.z = 0.20;
                    marker.lifetime.sec = 10;
                    marker.color.r = 1;
                    marker.color.g = 0;
                    marker.color.b = 0;
                    marker.color.a = 1;

                    marker.id = 0;
                    marker.pose.position.x = global_point_goal_.pose.position.x;
                    marker.pose.position.y = global_point_goal_.pose.position.y;
                    marker.pose.position.z = global_point_goal_.pose.position.z;

                    goal_point_pub_.publish(marker);

                    ROS_WARN("[Global Planning] The requested point is reachable. Publishing the waypoints");
                    publishGlobalWaypoints();
                    global_trajectory_planned_ = true;
                } else {
                    resetLocalPlanning();
                    ROS_WARN("[Global Planning] The requested point is not reachable");
                    return;
                }
            }

            if (arrivedToPoint(global_point_goal_)){
                resetLocalPlanning();
            }
        }

        void SubmapExplorationPlanner::resetLocalPlanning() {
            // Reset the state machine control variables
            global_trajectory_planned_ = false;
            n_current_tree_samples_ = 0;
            current_tree_max_gain_ = 0;
            RosPlanner::initializePlanning();
            voxgraph_map_ptr_->getPlannerMapManager().emtpyActiveSubmap();
            std::cout << "n samples: " << n_current_tree_samples_ << std::endl;
        }

        bool SubmapExplorationPlanner::arrivedToPoint(const geometry_msgs::PoseStamped& point){
            return distanceRobotToPoint(point) < global_replan_pos_threshold_;
        }


        float SubmapExplorationPlanner::distanceRobotToPoint(const geometry_msgs::PoseStamped& point){
            return std::sqrt(std::pow(point.pose.position.x - current_position_.x(),2) +
                              std::pow(point.pose.position.y - current_position_.y(),2) +
                              std::pow(point.pose.position.z - current_position_.z(),2));
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

            return srv.response.success;
        }

        void SubmapExplorationPlanner::publishGlobalWaypoints(){
            std_srvs::Empty srv;
            if (publish_global_trajectory_cln_.call(srv)){
                ROS_INFO("[Global Planning] Waypoints published for trajectory computation");
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

        void SubmapExplorationPlanner::updateGlobalPointSelector(const SubmapID& finished_submap_ID){
            global_point_selector_->update(finished_submap_ID);
        }

        const Point SubmapExplorationPlanner::getRobotPosition(){
            return Point((float)current_position_.x(), (float)current_position_.y(), (float)current_position_.z());
        }

    }
}

