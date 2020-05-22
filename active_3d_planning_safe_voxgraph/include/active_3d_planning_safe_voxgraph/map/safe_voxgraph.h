#ifndef ACTIVE_3D_PLANNING_SAFE_VOGRAPH_MAP_H
#define ACTIVE_3D_PLANNING_SAFE_VOGRAPH_MAP_H

#include "active_3d_planning_voxgraph/map/voxgraph.h"

namespace active_3d_planning {
    namespace map {

        // Voxblox as a map representation
        class SafeVoxgraphMap : public VoxgraphMap {
        public:
            SafeVoxgraphMap(PlannerI &planner) : VoxgraphMap(planner){};

            // check collision for a single pose
            virtual bool isTraversable(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) override;

        protected:
            static ModuleFactoryRegistry::Registration<SafeVoxgraphMap> registration;
        };

    } // namespace map
} // namespace active_3d_planning

#endif // ACTIVE_3D_PLANNING_SAFE_VOGRAPH_MAP_H
