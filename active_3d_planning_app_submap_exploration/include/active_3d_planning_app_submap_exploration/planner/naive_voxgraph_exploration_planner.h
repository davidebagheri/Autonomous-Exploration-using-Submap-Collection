#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NAIVE_VOXGRAPH_EXPLORATION_PLANNER_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NAIVE_VOXGRAPH_EXPLORATION_PLANNER_H

#include "active_3d_planning_naive_voxgraph/map/naive_voxgraph_map.h"
#include "active_3d_planning_ros/planner/ros_planner.h"


namespace active_3d_planning {
    namespace ros {
        class VoxgraphExplorationPlanner : public RosPlanner{
        public:
            VoxgraphExplorationPlanner(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private, ModuleFactory *factory,
                    Module::ParamMap *param_map);

            virtual ~VoxgraphExplorationPlanner() = default;

            virtual bool requestNextTrajectory() override;

        protected:
            map::NaiveVoxgraphMap* naive_voxgraph_map_;
        };
    }
}

#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NAIVE_VOXGRAPH_EXPLORATION_PLANNER_H
