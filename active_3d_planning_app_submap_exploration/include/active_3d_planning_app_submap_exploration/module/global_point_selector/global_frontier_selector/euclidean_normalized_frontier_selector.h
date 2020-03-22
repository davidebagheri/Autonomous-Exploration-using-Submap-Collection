#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_WEIGHTED_FRONTIER_SELECTOR_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_WEIGHTED_FRONTIER_SELECTOR_H
#include "active_3d_planning_core/module/module_factory_registry.h"
#include "global_frontier_selector.h"

namespace active_3d_planning {
    class EuclideanNormalizedFrontierSelector : public GlobalFrontierSelector{
    public:
        EuclideanNormalizedFrontierSelector(PlannerI &planner);

        bool selectFrontierToExplore() override;
    private:
        static ModuleFactoryRegistry::Registration<EuclideanNormalizedFrontierSelector> registration;
    };
}

#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_WEIGHTED_FRONTIER_SELECTOR_H
