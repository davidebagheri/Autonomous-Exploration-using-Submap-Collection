#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_ITERATIVE_RAY_CASTER_VOXGRAPH_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_ITERATIVE_RAY_CASTER_VOXGRAPH_H

#include "active_3d_planning_core/module/sensor_model/iterative_ray_caster.h"
#include "active_3d_planning_voxgraph/map/voxgraph.h"


namespace active_3d_planning {
    namespace sensor_model {

        // Iterative rayaster readapted in order to make it work on the Current Neighbours submap,
        // the union of all the submaps overlapping with the voxgraph active submap
        class IterativeRayCasterVoxgraph : public IterativeRayCaster {
        public:
            explicit IterativeRayCasterVoxgraph(PlannerI &planner);

            virtual bool getVisibleVoxels(std::vector<Eigen::Vector3d> *result,
                                          const Eigen::Vector3d &position,
                                          const Eigen::Quaterniond &orientation) override;

        protected:
            static ModuleFactoryRegistry::Registration<IterativeRayCasterVoxgraph> registration;

            map::VoxgraphMap* map_;
        };
    }
}


#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_ITERATIVE_RAY_CASTER_VOXGRAPH_H
