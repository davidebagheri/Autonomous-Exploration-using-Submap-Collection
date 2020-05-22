#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NAIVE_EVALUATOR_VOXGRAPH_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NAIVE_EVALUATOR_VOXGRAPH_H

#include "active_3d_planning_core/module/trajectory_evaluator/naive_evaluator.h"
#include "active_3d_planning_voxgraph/map/voxgraph.h"

namespace active_3d_planning {
    namespace trajectory_evaluator {

        class NaiveEvaluatorVoxgraph : public NaiveEvaluator {
        public:
            NaiveEvaluatorVoxgraph(PlannerI &planner);

            virtual void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<NaiveEvaluatorVoxgraph> registration;

            map::VoxgraphMap* map_;

            virtual bool computeGainFromVisibleVoxels(TrajectorySegment *traj_in) override;

        };
    }
}


#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_NAIVE_EVALUATOR_VOXGRAPH_H
