#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_LOOP_CLOSURE_EVALUATOR_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_LOOP_CLOSURE_EVALUATOR_H

#include "active_3d_planning_app_submap_exploration/module/trajectory_evaluator/naive_evaluator_voxgraph.h"

namespace active_3d_planning {
    namespace trajectory_evaluator {
        class LoopClosureEvaluator : public NaiveEvaluatorVoxgraph {
        public:
            LoopClosureEvaluator(PlannerI& planner);

            virtual void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<LoopClosureEvaluator> registration;

            virtual bool computeGainFromVisibleVoxels(TrajectorySegment *traj_in) override;

            virtual double loopClosureGain(double n_submap);

            // Param
            double time_scale_;
            double gain_scale_;
            double gain_threshold_;
        };
    }
}

#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_LOOP_CLOSURE_EVALUATOR_H
