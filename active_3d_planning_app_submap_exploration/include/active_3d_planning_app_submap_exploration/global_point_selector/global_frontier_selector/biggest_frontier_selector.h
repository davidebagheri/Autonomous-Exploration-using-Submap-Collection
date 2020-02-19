#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_POINT_SELECTOR_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_FRONTIER_SELECTOR_H

#include "active_3d_planning_app_submap_exploration/global_point_selector/global_frontier_selector/submap_frontier_evaluator/submap_frontier_evaluator.h"
#include "active_3d_planning_core/module/module_factory_registry.h"
#include "active_3d_planning_app_submap_exploration/global_point_selector/global_point_selector.h"

namespace active_3d_planning {
    class BiggestFrontierSelector : public GlobalPointSelector{
    public:
        BiggestFrontierSelector(PlannerI &planner);

        bool getGlobalPoint(geometry_msgs::PoseStamped* goal) override;

        bool findBiggerFrontier(SubmapID* submap_id, int* frontier_id);

        void selectPointForFrontierExploration(const SubmapID& submap_id, const int&frontier_id, geometry_msgs::PoseStamped* goal);

        void getRandomPointNearPosition(const Point& point, Point* random_point, float distance);

        Eigen::Vector3f getDirection(const float &theta, const float &phi);

        void update(SubmapID finished_submap_ID) override;

        SubmapFrontierEvaluator& getFrontierEvaluator(){
            return *submap_frontier_evaluator_;
        }

        void setupFromParamMap(Module::ParamMap *param_map) override;

    private:
        static ModuleFactoryRegistry::Registration<BiggestFrontierSelector> registration;

        std::unique_ptr<SubmapFrontierEvaluator> submap_frontier_evaluator_;
    };
}

#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_POINT_SELECTOR_H
