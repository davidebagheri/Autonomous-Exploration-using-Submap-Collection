#include "active_3d_planning_app_submap_exploration/global_point_selector/global_point_selector.h"

namespace active_3d_planning{
    GlobalPointSelector::GlobalPointSelector(PlannerI &planner) : Module(planner) {
        map_ = dynamic_cast<map::VoxgraphMap*>(&planner_.getMap());
        if (!map_) {
            planner_.printError("'GlobalPointSelector' requires a map of type 'VoxgraphMap'!");
        }
    }

    void GlobalPointSelector::setupFromParamMap(Module::ParamMap *param_map) {
        map_ = dynamic_cast<map::VoxgraphMap *>(&planner_.getMap());
        if (!map_) {
            planner_.printError("'BiggestFrontierSelector' requires a map of type 'VoxgraphMap'!");
        }

        setParam<float>(param_map, "robot_radius",&robot_radius, 1.0);
    }


    bool GlobalPointSelector::isFree(const voxblox::Point &point){
        Eigen::Vector3d point_d((double)point.x(), (double)point.y(), (double)point.z());

        double distance;
        bool is_free = false;

        if (!map_->getSubmapCollection().empty()) {
            for (SubmapID id = 0; id <= map_->getSubmapCollection().getActiveSubmapID(); id++){
                if (map_->getSubmapCollection().getSubmapPtr(id)->getEsdfMapPtr()->getDistanceAtPosition(point_d, &distance)){
                    if (distance > robot_radius) is_free = true;
                    else return false;
                }
            }
        }
        return is_free;
    }


}