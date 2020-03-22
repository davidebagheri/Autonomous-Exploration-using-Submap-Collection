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

        virtual void update(bool active_submap_finished = true) = 0;

        virtual bool computeGlobalPoint() = 0;

        virtual bool getGlobalPoint(geometry_msgs::PoseStamped* goal);

        void setupFromParamMap(ParamMap *param_map) override;

        virtual const float& getGlobalPointGain() {
            return global_point_gain_;
        };

        void resetGlobalPointGain(){
            global_point_gain_ = 0.0;
        }

    protected:
        // Map
        map::VoxgraphMap* map_;

        // variables
        float global_point_gain_;
        voxblox::Point global_point_;
    };
}

#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_POINT_SELECTOR_H
