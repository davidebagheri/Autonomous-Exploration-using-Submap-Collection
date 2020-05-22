#include "active_3d_planning_safe_voxgraph/map/safe_voxgraph.h"

namespace active_3d_planning {
    namespace map {
        ModuleFactoryRegistry::Registration<SafeVoxgraphMap> SafeVoxgraphMap::registration("SafeVoxgraphMap");
		
		// A point is traversable if observed in the active submap and traversable in all the submaps
        bool SafeVoxgraphMap::isTraversable(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
            double distance = 0.0;
            if (isObserved(position) && isTraversableInAllSubmaps(position)){
              return true;
            }
            return false;
        }
    } // namespace map
} // namespace active_3d_planning
