#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NEAREST_FRONTIER_SELECTOR_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NEAREST_FRONTIER_SELECTOR_H

#include "active_3d_planning_core/module/module_factory_registry.h"
#include "global_frontier_selector.h"
#include "cblox_planning_msgs/PathLengthSrv.h"
#include "mav_planning_msgs/PlannerService.h"

namespace active_3d_planning {
    class NearestFrontierSelector : public GlobalFrontierSelector{
    public:
        NearestFrontierSelector(PlannerI &planner);

        virtual bool selectFrontierToExplore() override;

        bool planToPoint(const Point& point);

        double getPathLength();

    protected:
        static ModuleFactoryRegistry::Registration<NearestFrontierSelector> registration;

        ros::ServiceClient path_length_cln_;
        ros::ServiceClient plan_cln_;
    };
}

#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NEAREST_FRONTIER_SELECTOR_H
