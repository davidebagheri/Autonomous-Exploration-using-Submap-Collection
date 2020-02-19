#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_SUBMAP_EXPLORATION_PLANNER_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_SUBMAP_EXPLORATION_PLANNER_H

#include "active_3d_planning_app_submap_exploration/planner/global_local_planner_I.h"
#include "active_3d_planning_app_submap_exploration/planner/voxgraph_local_planner.h"
#include "active_3d_planning_app_submap_exploration/global_point_selector/global_frontier_selector/biggest_frontier_selector.h"
#include "mav_planning_msgs/PlannerService.h"

namespace active_3d_planning{
    namespace ros {
        class SubmapExplorationPlanner : public GlobalLocalPlannerI, public VoxgraphLocalPlanner{
        public:
            SubmapExplorationPlanner(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private, ModuleFactory *factory,
                                      Module::ParamMap *param_map);
            virtual ~SubmapExplorationPlanner() = default;

            virtual void plan() override;

            const Point getRobotPosition();

        protected:
            virtual void setupFromParamMap(Module::ParamMap *param_map) override;

            virtual PlanningStatus stateMachine() override;

            virtual void localPlanningIteration() override;

            virtual void globalPlanningIteration() override;

            void getNumSamplesAndMaxGain();

            void advertiseServices();

            void advertisePublishers();

            bool globalPlanToPoint(const geometry_msgs::PoseStamped& goal_point);

            float distanceRobotToPoint(const geometry_msgs::PoseStamped& point);

            void publishGlobalWaypoints();

            bool arrivedToPoint(const geometry_msgs::PoseStamped& point);

            void resetLocalPlanning();

            void updateGlobalPointSelector(const SubmapID& finished_submap_ID);

            // Ros Service Clients
            ::ros::ServiceClient global_planning_cln_;
            ::ros::ServiceClient publish_global_trajectory_cln_;

            // Variables
            geometry_msgs::PoseStamped global_point_goal_;

            // Counts
            int n_current_tree_samples_;
            float current_tree_max_gain_;

            // State machine control flag
            bool global_trajectory_planned_;

            // Params
            float min_gain_threshold_;
            int n_samples_threshold_;
            int n_samples_tries_threshold_;
            float global_replan_pos_threshold_;

            // Global frontier selector
            //std::unique_ptr<BiggestFrontierSelector> global_frontier_selector_;
            std::unique_ptr<GlobalPointSelector> global_point_selector_;

            // Ros publisher
            ::ros::Publisher frontiers_pub_;
            ::ros::Publisher goal_point_pub_;

            // Test
            ::ros::ServiceServer switch_plan_;
            bool plan_flag_ = false;
            bool switchPlan(std_srvs::Empty::Request &req, std_srvs::Empty::Response& res);
        };
    }
}
#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_SUBMAP_EXPLORATION_PLANNER_H
