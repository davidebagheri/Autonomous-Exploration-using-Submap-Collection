#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_POINT_SELECTOR_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_POINT_SELECTOR_H

#include <active_3d_planning_voxgraph/planner_map_manager/planner_map_manager.h>
#include "active_3d_planning_core/planner/planner_I.h"
#include "voxgraph/common.h"
#include "geometry_msgs/PoseStamped.h"
#include <active_3d_planning_core/module/module.h>
#include "active_3d_planning_voxgraph/map/voxgraph.h"

namespace active_3d_planning {
    class GlobalPointSelector : public Module {
    public:
        GlobalPointSelector(PlannerI &planner);

        void setupFromParamMap(Module::ParamMap *param_map);

        virtual void update(SubmapID finished_submap_ID) = 0;

        virtual bool getGlobalPoint(geometry_msgs::PoseStamped* goal) = 0;

        bool isFree(const voxblox::Point& point);

    protected:
        map::VoxgraphMap* map_;

        float robot_radius;
    };

}

#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_POINT_SELECTOR_H
