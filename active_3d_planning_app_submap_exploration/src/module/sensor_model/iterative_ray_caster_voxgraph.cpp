#include "active_3d_planning_app_submap_exploration/module/sensor_model/iterative_ray_caster_voxgraph.h"

namespace active_3d_planning {
    namespace sensor_model {

        ModuleFactoryRegistry::Registration <IterativeRayCasterVoxgraph>
                IterativeRayCasterVoxgraph::registration("IterativeRayCasterVoxgraph");

        IterativeRayCasterVoxgraph::IterativeRayCasterVoxgraph(PlannerI &planner) : IterativeRayCaster(planner){
            map_ = dynamic_cast<map::VoxgraphMap*>(&planner.getMap());
            if (!map_) {
                planner_.printError("'IterativeRayCasterVoxgraph' requires a map of type 'VoxgraphMap'!");
            }
        }

        bool IterativeRayCasterVoxgraph::getVisibleVoxels(
                std::vector<Eigen::Vector3d> *result, const Eigen::Vector3d &position,
                const Eigen::Quaterniond &orientation) {
            // Setup ray table (contains at which segment to start, -1 if occluded
            ray_table_ = Eigen::ArrayXXi::Zero(c_res_x_, c_res_y_);

            // Ray-casting
            Eigen::Vector3d camera_direction;
            Eigen::Vector3d direction;
            Eigen::Vector3d current_position;
            Eigen::Vector3d voxel_center;
            double distance;
            bool cast_ray;
            double map_distance;
            for (int i = 0; i < c_res_x_; ++i) {
                for (int j = 0; j < c_res_y_; ++j) {
                    int current_segment = ray_table_(i, j); // get ray starting segment
                    if (current_segment < 0) {
                        continue; // already occluded ray
                    }
                    CameraModel::getDirectionVector(&camera_direction,
                                                    (double) i / ((double) c_res_x_ - 1.0),
                                                    (double) j / ((double) c_res_y_ - 1.0));
                    direction = orientation * camera_direction;
                    distance = c_split_distances_[current_segment];
                    cast_ray = true;
                    while (cast_ray) {
                        // iterate through all splits (segments)
                        while (distance < c_split_distances_[current_segment + 1]) {
                            current_position = position + distance * direction;
                            distance += p_ray_step_;

                            // Check voxel occupied
                            if (map_->getCurrentNeighboursVoxelState(current_position) == map::OccupancyMap::OCCUPIED) {
                                // Occlusion, mark neighboring rays as occluded
                                markNeighboringRays(i, j, current_segment, -1);
                                cast_ray = false;
                                break;
                            }

                            // Add point (duplicates are handled in
                            // CameraModel::getVisibleVoxelsFromTrajectory)
                            map_->getCurrentNeighboursVoxelCenter(&voxel_center, current_position);
                            result->push_back(voxel_center);
                        }
                        if (cast_ray) {
                            current_segment++;
                            if (current_segment >= c_n_sections_) {
                                cast_ray = false; // done
                            } else {
                                // update ray starts of neighboring rays
                                markNeighboringRays(i, j, current_segment - 1, current_segment);
                            }
                        }
                    }
                }
            }
            return true;
        }

    }
}
