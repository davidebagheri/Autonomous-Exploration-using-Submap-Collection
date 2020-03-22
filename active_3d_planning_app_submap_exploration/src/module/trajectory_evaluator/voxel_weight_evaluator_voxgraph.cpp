#include "active_3d_planning_app_submap_exploration/module/trajectory_evaluator/voxel_weight_evaluator_voxgraph.h"

namespace active_3d_planning {
    namespace trajectory_evaluator {

        ModuleFactoryRegistry::Registration<VoxelWeightEvaluatorVoxgraph>
                VoxelWeightEvaluatorVoxgraph::registration("VoxelWeightEvaluatorVoxgraph");

        VoxelWeightEvaluatorVoxgraph::VoxelWeightEvaluatorVoxgraph(PlannerI& planner) : VoxelWeightEvaluator(planner){}

        void VoxelWeightEvaluatorVoxgraph::setupFromParamMap(Module::ParamMap *param_map){
            VoxelWeightEvaluator::setupFromParamMap(param_map);

            map_ = dynamic_cast<map::VoxgraphMap*>(&planner_.getMap());
            if (!map_) {
                planner_.printError("'VoxelWeightEvaluatorVoxgraph' requires a map of type 'VoxgraphMap'!");
            }
        }

        double VoxelWeightEvaluatorVoxgraph::getVoxelValue(const Eigen::Vector3d &voxel, const Eigen::Vector3d &origin){
            unsigned char voxel_state = map_->getCurrentNeighboursVoxelState(voxel);
            if (voxel_state == map::TSDFMap::OCCUPIED) {
                // Surface voxel
                double z = (voxel - origin).norm();
                double spanned_angle = 2.0 * atan2(c_voxel_size_, z * 2.0);
                double new_weight = std::pow(spanned_angle, 2.0) /
                                    (p_ray_angle_x_ * p_ray_angle_y_) /
                                    std::pow(z, 2.0);
                double gain =
                        new_weight / (new_weight + map_->getCurrentNeighboursVoxelWeight(voxel));
                if (gain > p_min_impact_factor_) {
                    return gain;
                }
            } else if (voxel_state == map::TSDFMap::UNKNOWN) {
                // Unobserved voxels
                if (p_frontier_voxel_weight_ > 0.0) {
                    if (isFrontierVoxel(voxel)) {
                        return p_frontier_voxel_weight_;
                    }
                }
                return p_new_voxel_weight_;
            }
            return 0;
        }
    }
}
