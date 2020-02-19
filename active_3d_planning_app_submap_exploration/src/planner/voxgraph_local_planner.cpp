#include "active_3d_planning_app_submap_exploration/planner/voxgraph_local_planner.h"
#include "active_3d_planning_ros/module/module_factory_ros.h"
#include "active_3d_planning_voxgraph/map/voxgraph.h"
#include "voxgraph_mapper_msgs/PlannerMapsSrv.h"

namespace active_3d_planning {
    namespace ros {
        VoxgraphLocalPlanner::VoxgraphLocalPlanner(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private,
                                                   active_3d_planning::ModuleFactory *factory,
                                                   active_3d_planning::ModuleBase::ParamMap *param_map)
                                           : RosPlanner(nh, nh_private, factory,param_map){
            // Get pointer to the VoxgraphMap
            voxgraph_map_ptr_ = dynamic_cast<map::VoxgraphMap*>(map_.get());
            if (!voxgraph_map_ptr_) {
                printError("'VoxgraphLocalPlanner' requires a map of type 'VoxgraphMap'!");
            }
        }

        bool VoxgraphLocalPlanner::requestNextTrajectory(){
            if (voxgraph_map_ptr_->hasActiveMapFinished()){
                voxgraph_map_ptr_->getPlannerMapManager().emtpyActiveSubmap();
            }
            // Update the planning maps
            voxgraph_map_ptr_->updatePlanningMaps();

            if (current_segment_->children.empty()) {
                // No trajectories available: call the backtracker
                if (current_segment_->trajectory.size() > 0) {
                    back_tracker_->trackBack(current_segment_.get());
                }
                return false;
            }
            // Performance tracking
            double perf_runtime;
            double perf_vis = 0.0;
            double perf_next;
            double perf_uptg;
            double perf_upte;
            std::clock_t timer;
            if (p_log_performance_) {
                timer = std::clock();
            }

            // Visualize candidates
            std::vector<TrajectorySegment *> trajectories_to_vis;
            current_segment_->getTree(&trajectories_to_vis);
            trajectories_to_vis.erase(
                    trajectories_to_vis.begin()); // remove current segment (root)
            if (p_visualize_) {
                publishTrajectoryVisualization(trajectories_to_vis);
            }
            int num_trajectories = trajectories_to_vis.size();
            if (p_verbose_ || p_log_performance_) {
                perf_runtime = std::chrono::duration<double>(std::clock() - info_timing_).count() / 1e6;
                info_timing_ = std::clock();
                if (p_verbose_) {
                    std::stringstream ss;
                    ss << "Replanning!\n(" << std::setprecision(3) << perf_runtime << "s elapsed, "
                       << num_trajectories - info_count_ + 1 << " new, " << num_trajectories << " total, "
                       << info_killed_next_ + 1 << " killed by root change, " << info_killed_update_
                       << " killed while updating)";
                    printInfo(ss.str());
                }
            }
            if (p_log_performance_) {
                perf_vis += (double) (std::clock() - timer) / CLOCKS_PER_SEC;
                timer = std::clock();
            }
            // Select best next trajectory and update root
            int next_segment = trajectory_evaluator_->selectNextBest(current_segment_.get());
            current_segment_ = std::move(current_segment_->children[next_segment]);
            current_segment_->parent = nullptr;
            current_segment_->gain = 0.0;
            current_segment_->cost = 0.0;
            current_segment_->value = 0.0;
            if (p_log_performance_) {
                perf_next = (double) (std::clock() - timer) / CLOCKS_PER_SEC;
            }
            trajectories_to_vis.clear();
            current_segment_->getTree(&trajectories_to_vis);
            info_killed_next_ = num_trajectories - trajectories_to_vis.size();
            // Move
            EigenTrajectoryPointVector trajectory;
            trajectory_generator_->extractTrajectoryToPublish(&trajectory, *current_segment_);
            if (trajectory.size() > 0) {
                current_segment_->trajectory = trajectory;
                requestMovement(trajectory);
                target_position_ = trajectory.back().position_W;
                target_yaw_ = trajectory.back().getYaw();
                back_tracker_->segmentIsExecuted(*current_segment_);
            }

            // Visualize
            if (p_log_performance_) {
                timer = std::clock();
            }
            if (p_visualize_) {
                publishEvalVisualization(*current_segment_);
                publishCompletedTrajectoryVisualization(*current_segment_);
            }
            if (p_log_performance_) {
                perf_vis += (double) (std::clock() - timer) / CLOCKS_PER_SEC;
                timer = std::clock();
            }

            // recursive tree pass from root to leaves
            updateGeneratorStep(current_segment_.get());
            if (p_log_performance_) {
                perf_uptg = (double) (std::clock() - timer) / CLOCKS_PER_SEC;
                timer = std::clock();
            }
            updateEvaluatorStep(current_segment_.get());
            if (p_log_performance_) {
                perf_upte = (double) (std::clock() - timer) / CLOCKS_PER_SEC;
            }
            trajectories_to_vis.clear();
            current_segment_->getTree(&trajectories_to_vis);
            info_killed_update_ = num_trajectories - trajectories_to_vis.size() - info_killed_next_;

            // Performance log and printing
            if (p_verbose_ || p_log_performance_) {
                trajectories_to_vis.clear();
                current_segment_->getTree(&trajectories_to_vis);
                info_count_ = trajectories_to_vis.size();

                if (p_log_performance_) {
                    perf_log_file_ << "\n" << perf_runtime << "," << num_trajectories << ","
                                   << info_count_ << "," << perf_log_data_[0] << ","
                                   << perf_log_data_[1] << "," << perf_log_data_[2] << ","
                                   << perf_log_data_[3] << "," << perf_log_data_[4] << ","
                                   << perf_next << "," << perf_uptg << "," << perf_upte << ","
                                   << perf_vis << "," << (double) (std::clock() - perf_cpu_timer_) /
                                                         CLOCKS_PER_SEC;
                    std::fill(perf_log_data_.begin(), perf_log_data_.begin() + 5, 0.0); // reset count
                    perf_cpu_timer_ = std::clock();
                }
            }

            // Update tracking values
            new_segment_tries_ = 0;
            new_segments_ = 0;
            min_new_value_reached_ = p_min_new_value_ == 0.0;
            target_reached_ = false;

            // Performance log
            if (p_log_performance_) {
                perf_log_file_ << "," << (::ros::Time::now() - ros_timing_).toSec() << "," << perf_log_data_[5];
                perf_cpu_timer_ = std::clock();
                ros_timing_ = ::ros::Time::now();
                perf_log_data_[5] = 0;        // reset count
            }
            return true;
        }
    }
}
