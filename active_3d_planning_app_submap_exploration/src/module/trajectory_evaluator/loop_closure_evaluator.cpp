#include "active_3d_planning_app_submap_exploration/module/trajectory_evaluator/loop_closure_evaluator.h"

namespace active_3d_planning {
    namespace trajectory_evaluator {
// Factory Registration
        ModuleFactoryRegistry::Registration<LoopClosureEvaluator>
                LoopClosureEvaluator::registration("LoopClosureEvaluator");

        LoopClosureEvaluator::LoopClosureEvaluator(PlannerI &planner) : NaiveEvaluatorVoxgraph(planner){}

        void LoopClosureEvaluator::setupFromParamMap(Module::ParamMap *param_map){
            NaiveEvaluator::setupFromParamMap(param_map);

            map_ = dynamic_cast<map::VoxgraphMap*>(&planner_.getMap());
            if (!map_) {
                planner_.printError("'LoopClosureEvaluator' requires a map of type 'VoxgraphMap'!");
            }

            setParam<double>(param_map, "time_scale", &time_scale_, 0.1);
            setParam<double>(param_map, "gain_scale", &gain_scale_, 60);
            setParam<double>(param_map, "gain_threshold", &gain_threshold_, 15);
        }

        bool LoopClosureEvaluator::computeGainFromVisibleVoxels(TrajectorySegment *traj_in){
            // Count the number of visible new voxels
            if (!NaiveEvaluatorVoxgraph::computeGainFromVisibleVoxels(traj_in)) return false;

            // Add a function of the number of submaps containing the visible voxels in order to aim at loop closures
            SimulatedSensorInfo *info =
                    reinterpret_cast<SimulatedSensorInfo *>(traj_in->info.get());

            // Do not consider the trajectories with a too low gain
            if (traj_in->gain > gain_threshold_) {
                std::vector<SubmapID> visible_submaps;
                int n_visible_voxels = info->visible_voxels.size();
                int n_voxels_to_check = std::min(30, n_visible_voxels);

                // Count the submaps containg some random visible voxels of the trajectory
                for (int i = 0; i < n_voxels_to_check; i++){
                    int voxel_random_idx = rand() % n_visible_voxels;

                    for (auto& submap_containing_point :
                    map_->getSubmapsIncludingPoint(info->visible_voxels[voxel_random_idx])){
                        // Check if not already added
                        for (const auto& submap_encoutered_id : visible_submaps){
                            if (submap_containing_point == submap_encoutered_id) break;

                            if (submap_encoutered_id == visible_submaps.back())
                                visible_submaps.push_back(submap_containing_point);
                        }
                    }

                }
                // Add loop closure function gain
                traj_in->gain += loopClosureGain(visible_submaps.size());
            }
            return true;
        }

        double LoopClosureEvaluator::loopClosureGain(double n_submap){
            double delta_t = ros::Time::now().toSec() / 60; // time from the beginning in min
            return gain_scale_ * n_submap * (1 - std::exp(-time_scale_ * delta_t));
        }
    }
}
