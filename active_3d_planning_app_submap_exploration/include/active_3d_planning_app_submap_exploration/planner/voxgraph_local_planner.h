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




            /// Test
            //void loopIteration() override;
            double getNumSamples();
            /*double time = 0;
            int n = 0;
            int counter = 0;
            double n_samples_average = 0;
            bool flag = true;*/
        };
    }
}


#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_MAPUPDATEPLANNER_H
