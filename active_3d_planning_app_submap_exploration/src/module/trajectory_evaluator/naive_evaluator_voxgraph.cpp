#include "active_3d_planning_app_submap_exploration/module/trajectory_evaluator/naive_evaluator_voxgraph.h"


namespace active_3d_planning {
    namespace trajectory_evaluator {
// Factory Registration
        ModuleFactoryRegistry::Registration<NaiveEvaluatorVoxgraph>
                NaiveEvaluatorVoxgraph::registration("NaiveEvaluatorVoxgraph");

        NaiveEvaluatorVoxgraph::NaiveEvaluatorVoxgraph(PlannerI &planner) : NaiveEvaluator(planner){
        }

        void NaiveEvaluatorVoxgraph::setupFromParamMap(Module::ParamMap *param_map){
            NaiveEvaluator::setupFromParamMap(param_map);

            map_ = dynamic_cast<map::VoxgraphMap*>(&planner_.getMap());
            if (!map_) {
                planner_.printError("'NaiveEvaluatorVoxgraph' requires a map of type 'VoxgraphMap'!");
            }
        }

        bool NaiveEvaluatorVoxgraph::computeGainFromVisibleVoxels(TrajectorySegment *traj_in){
            if (!traj_in->info) {
                traj_in->gain = 0.0;
                return false;
            }
            // remove all already observed voxels, count number of new voxels
            SimulatedSensorInfo *info =
                    reinterpret_cast<SimulatedSensorInfo *>(traj_in->info.get());
            info->visible_voxels.erase(
                    std::remove_if(info->visible_voxels.begin(), info->visible_voxels.end(),
                                   [this](const Eigen::Vector3d &voxel) {
                                       return map_->isObservedInCurrentNeighbours(voxel);
                                   }),
                    info->visible_voxels.end());
            traj_in->gain = (double) info->visible_voxels.size();
            return true;
        }


    }
}
