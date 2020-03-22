#include "active_3d_planning_app_submap_exploration/module/trajectory_generator/rrt_star_voxgraph.h"

namespace active_3d_planning {
    namespace trajectory_generator {
        ModuleFactoryRegistry::Registration<RRTStarVoxgraph> RRTStarVoxgraph::registration("RRTStarVoxgraph");

        RRTStarVoxgraph::RRTStarVoxgraph(PlannerI &planner) : RRTStar(planner) {
            map_ = dynamic_cast<map::VoxgraphMap*>(&planner_.getMap());
            if (!map_) {
                planner_.printError("'NaiveEvaluatorVoxgraph' requires a map of type 'VoxgraphMap'!");
            }
        }

        bool RRTStarVoxgraph::selectSegment(TrajectorySegment **result,
                                            TrajectorySegment *root) {
            // If the root has changed, reset the kdtree and populate with the current
            // trajectory tree
            if (previous_root_ != root) {
                resetTree(root);
                previous_root_ = root;
                if (p_rewire_update_) {
                    rewireIntermediate(root);
                }
                if (p_sampling_mode_ == "semilocal") {
                    // Check whether the minimum number of local points is achieved and store
                    // how many to go
                    double query_pt[3] = {root->trajectory.back().position_W.x(),
                                          root->trajectory.back().position_W.y(),
                                          root->trajectory.back().position_W.z()};
                    std::size_t ret_index[p_semilocal_count_];
                    double out_dist[p_semilocal_count_];
                    nanoflann::KNNResultSet<double> resultSet(p_semilocal_count_);
                    resultSet.init(ret_index, out_dist);
                    kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
                    semilocal_count_ = p_semilocal_count_;
                    for (int i = 0; i < resultSet.size(); ++i) {
                        if (out_dist[p_semilocal_count_ - 1] <=
                            p_semilocal_radius_max_ * p_semilocal_radius_max_) {
                            semilocal_count_ -= 1;
                        }
                    }
                }
            }
            // If the root has changed, reset the kdtree and populate with the current
            // trajectory tree
            if (previous_root_ != root) {
                resetTree(root);
                previous_root_ = root;
                if (p_sampling_mode_ == "semilocal") {
                    // Check whether the minimum number of local points is achieved and store
                    // how many to go
                    double query_pt[3] = {root->trajectory.back().position_W.x(),
                                          root->trajectory.back().position_W.y(),
                                          root->trajectory.back().position_W.z()};
                    std::size_t ret_index[p_semilocal_count_];
                    double out_dist[p_semilocal_count_];
                    nanoflann::KNNResultSet<double> resultSet(p_semilocal_count_);
                    resultSet.init(ret_index, out_dist);
                    kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
                    semilocal_count_ = p_semilocal_count_;
                    for (int i = 0; i < resultSet.size(); ++i) {
                        if (out_dist[p_semilocal_count_ - 1] <=
                            p_semilocal_radius_max_ * p_semilocal_radius_max_) {
                            semilocal_count_ -= 1;
                        }
                    }
                }
            }

            // sample candidate points
            bool goal_found = false;
            Eigen::Vector3d goal_pos;
            int counter = 0;
            while (!goal_found && counter <= p_maximum_tries_) {
                if (p_maximum_tries_ > 0) {
                    counter++;
                }
                goal_pos = root->trajectory.back().position_W;
                sampleGoal(&goal_pos);
                if (p_crop_segments_ || checkTraversable(goal_pos) || map_->isInsideActiveSubmap(goal_pos)) {
                    goal_found = true;
                }
            }

            if (!goal_found) {
                *result = nullptr;
                return false;
            }

            // find closest point in kdtree
            double query_pt[3] = {goal_pos.x(), goal_pos.y(), goal_pos.z()};
            std::size_t ret_index;
            double out_dist_sqr;
            nanoflann::KNNResultSet<double> resultSet(1);
            resultSet.init(&ret_index, &out_dist_sqr);
            if (!kdtree_->findNeighbors(resultSet, query_pt,
                                        nanoflann::SearchParams(10))) {
                *result = nullptr;
                return false;
            }

            // Valid target found
            goal_pos_ = goal_pos;
            *result = tree_data_.data[ret_index];
            return true;
        }
    }
}