#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_FRONTIER_SELECTOR_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_FRONTIER_SELECTOR_H

#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_frontier_selector/submap_frontier_evaluator/submap_frontier_evaluator.h"
#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_point_selector.h"

namespace active_3d_planning {

class GlobalFrontierSelector : public GlobalPointSelector {
    public:
        GlobalFrontierSelector(PlannerI &planner);

        virtual void setupFromParamMap(Module::ParamMap *param_map);

        void update(bool active_submap_finished = true) override;

        virtual bool selectFrontierToExplore() = 0;

        virtual bool computeGlobalPoint() override;

        SubmapFrontierEvaluator& getFrontierEvaluator(){
            return *submap_frontier_evaluator_;
        }

    protected:
        // Variables
        SubmapID target_submap_id_;
        int target_frontier_id_;
        voxblox::Transformation active_submap_pose_;

        // Frontier evaluator
        std::unique_ptr<SubmapFrontierEvaluator> submap_frontier_evaluator_;
    };

}
#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_FRONTIER_SELECTOR_H
