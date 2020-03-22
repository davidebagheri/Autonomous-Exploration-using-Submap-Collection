#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NORMALIZED_FRONTIER_SELECTOR_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NORMALIZED_FRONTIER_SELECTOR_H

#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_frontier_selector/nearest_frontier_selector.h"

namespace active_3d_planning {
    class NormalizedFrontierSelector : public NearestFrontierSelector {
    public:
        NormalizedFrontierSelector(PlannerI &planner);

        virtual bool selectFrontierToExplore() override;

    protected:
        static ModuleFactoryRegistry::Registration<NormalizedFrontierSelector> registration;
    };
}

#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NORMALIZED_FRONTIER_SELECTOR_H
