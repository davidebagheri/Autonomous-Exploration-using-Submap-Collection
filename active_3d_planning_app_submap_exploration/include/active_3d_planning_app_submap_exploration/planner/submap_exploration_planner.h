#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_SUBMAP_EXPLORATION_PLANNER_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_SUBMAP_EXPLORATION_PLANNER_H

#include "active_3d_planning_app_submap_exploration/planner/global_local_planner_I.h"
#include "active_3d_planning_app_submap_exploration/planner/voxgraph_local_planner.h"
#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_frontier_selector/biggest_frontier_selector.h"
#include "mav_planning_msgs/PlannerService.h"

namespace active_3d_planning{
    namespace ros {
        class SubmapExplorationPlanner : public GlobalLocalPlannerI, public VoxgraphLocalPlanner{
        public:
            SubmapExplorationPlanner(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private, ModuleFactory *factory,
                                      Module::ParamMap *param_map);
            virtual ~SubmapExplorationPlanner() = default;

            virtual void plan() override;

            bool requestNextTrajectory() override;;

        protected:
            bool isLocalGainLow();

            virtual void setupFromParamMap(Module::ParamMap *param_map) override;

            virtual PlanningStatus stateMachine() override;

            virtual void localPlanningIteration() override;

            virtual void globalPlanningIteration() override;

            void getNumSamplesAndMaxGain();

            void advertiseServices();

            void subscribeToTopics();

            void currentTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::Ptr& msg);

            bool globalPlanToPoint(const geometry_msgs::PoseStamped& goal_point);

            bool planToNearPoint(const geometry_msgs::PoseStamped& goal_point);

            bool planToNearestSubmapOrigin(const geometry_msgs::PoseStamped& goal_point);

            float distance(const Eigen::Vector3d point_A, const geometry_msgs::PoseStamped& point_B);

            geometry_msgs::PoseStamped getNearestSubmapOrigin(const geometry_msgs::PoseStamped& point);

            void publishGlobalWaypoints();

            bool arrivedToGlobalTrajEnd();

            void resetLocalPlanning();

            void resetGlobalPlanning();

            void stopRobot();

            // Ros Services
            ::ros::ServiceClient global_planning_cln_;
            ::ros::ServiceClient publish_global_trajectory_cln_;
            ::ros::ServiceServer publish_global_trajectory_srv_;
            ::ros::Subscriber trajectory_sub_;

            // Variables
            geometry_msgs::PoseStamped global_point_goal_O_;    // The first objective point that the planner tries to reach
            geometry_msgs::PoseStamped global_point_planned_M_; // The point that the planner actually reaches in Mission frame
            EigenTrajectoryPointVector planned_global_trajectory_;
            int global_replan_current_times_;

            int n_current_tree_samples_;    // Current number of samples in the local planner tree
            float current_tree_max_gain_;   // Max Gain in the local planner tree

            // State machine control flag
            bool global_trajectory_planned_;
            bool global_point_computed_;
            bool global_plan_happened_;     // Whether a global plan happened in a voxgraph interval

            // Params
            float min_gain_threshold_;  // Threshold under which local area is observed enough
            int n_samples_threshold_;   // Threshold over which neighbouring area is sampled enough
            int n_samples_tries_threshold_; // Threshold over which enough samples in neighbouring area are tried
            float global_replan_pos_threshold_;     // Below this distance the robot is arrived to the global point
            double nearest_submap_origin_max_distance_; // Maximum distance from nearest submap origin to frontier centroid
            int global_replan_max_times_;   // Max number of replans to reach a global point
            double plan_time_limit_;    // Time limit for global planning computation

            // Global frontier selector
            std::unique_ptr<GlobalPointSelector> global_point_selector_;

            // Param map
            Module::ParamMap *param_map_;
        };
    }
}
#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_SUBMAP_EXPLORATION_PLANNER_H
