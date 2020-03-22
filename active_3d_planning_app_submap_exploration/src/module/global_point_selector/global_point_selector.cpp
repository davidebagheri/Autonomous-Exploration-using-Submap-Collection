#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_point_selector.h"

namespace active_3d_planning{
    GlobalPointSelector::GlobalPointSelector(PlannerI &planner) : Module(planner) {
        map_ = dynamic_cast<map::VoxgraphMap*>(&planner_.getMap());
        if (!map_) {
            planner_.printError("'GlobalPointSelector' requires a map of type 'VoxgraphMap'!");
        }
    }

    void GlobalPointSelector::setupFromParamMap(ParamMap *param_map){
        /*std::string ns = (*param_map)["param_namespace"];
        std::string submap_frontier_evaluator_args;
        setParam<std::string>(param_map, "global_gain_computer",
                              &submap_frontier_evaluator_args, ns + "/global_gain_computer");

        gain_computer_ = planner_.getFactory().createModule<GlobalGainComputer>(
                submap_frontier_evaluator_args, planner_, verbose_modules_);*/
    }

    bool GlobalPointSelector::getGlobalPoint(geometry_msgs::PoseStamped* goal) {
        goal->pose.position.x = global_point_.x();
        goal->pose.position.y = global_point_.y();
        goal->pose.position.z = global_point_.z();
    }
}