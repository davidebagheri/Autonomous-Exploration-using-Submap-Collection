#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_RRT_STAR_VOXGRAPH_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_RRT_STAR_VOXGRAPH_H

#include "active_3d_planning_core/module/trajectory_generator/rrt_star.h"
#include "active_3d_planning_voxgraph/map/voxgraph.h"

namespace active_3d_planning {
    namespace trajectory_generator {
        class RRTStarVoxgraph : public RRTStar {
        public:
            RRTStarVoxgraph(PlannerI &planner);

            virtual bool selectSegment(TrajectorySegment **result,
                                       TrajectorySegment *root) override;
        protected:
            static ModuleFactoryRegistry::Registration<RRTStarVoxgraph> registration;

            map::VoxgraphMap* map_;
        };
    }
}

#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_RRT_STAR_VOXGRAPH_H
