#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_MAPUPDATEPLANNER_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_MAPUPDATEPLANNER_H

#include <active_3d_planning_voxgraph/map/voxgraph.h>
#include "active_3d_planning_ros/planner/ros_planner.h"
#include "active_3d_planning_ros/tools/ros_conversion.h"

namespace active_3d_planning {
    namespace ros {
        class VoxgraphLocalPlanner : public RosPlanner {
        public:
            VoxgraphLocalPlanner(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private, ModuleFactory *factory,
                                 Module::ParamMap *param_map);

            virtual ~VoxgraphLocalPlanner() = default;

            virtual bool requestNextTrajectory() override;

        protected:
            map::VoxgraphMap* voxgraph_map_ptr_;
        };
    }
}


#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_MAPUPDATEPLANNER_H
