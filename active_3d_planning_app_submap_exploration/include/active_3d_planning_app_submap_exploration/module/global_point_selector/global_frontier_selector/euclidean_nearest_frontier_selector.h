#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NEARESTFRONTIERSELECTOR_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NEARESTFRONTIERSELECTOR_H

#include "active_3d_planning_core/module/module_factory_registry.h"
#include "global_frontier_selector.h"

namespace active_3d_planning {
    class EuclideanNearestFrontierSelector : public GlobalFrontierSelector{
    public:
        EuclideanNearestFrontierSelector(PlannerI &planner);

        bool selectFrontierToExplore() override;
    private:
        static ModuleFactoryRegistry::Registration<EuclideanNearestFrontierSelector> registration;
    };
}

#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NEARESTFRONTIERSELECTOR_H
