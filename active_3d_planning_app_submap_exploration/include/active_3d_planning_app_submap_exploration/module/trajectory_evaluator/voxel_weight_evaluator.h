#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_VOXEL_WEIGHT_EVALUATOR_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_VOXEL_WEIGHT_EVALUATOR_H

#include "active_3d_planning_core/module/trajectory_evaluator/voxel_weight_evaluator.h"
#include "active_3d_planning_voxgraph/map/voxgraph.h"

namespace active_3d_planning {
    namespace trajectory_evaluator {

        class VoxelWeightEvaluatorVoxgraph : public VoxelWeightEvaluator {
        public:
            VoxelWeightEvaluatorVoxgraph(PlannerI& planner);

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<VoxelWeightEvaluatorVoxgraph> registration;

            map::VoxgraphMap* map_;

            // methods
            virtual double getVoxelValue(const Eigen::Vector3d &voxel,
                                         const Eigen::Vector3d &origin);
        };
    }
}


#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_VOXEL_WEIGHT_EVALUATOR_H
