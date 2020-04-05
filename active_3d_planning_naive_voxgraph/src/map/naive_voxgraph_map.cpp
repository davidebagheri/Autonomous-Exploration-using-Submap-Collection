#include "active_3d_planning_naive_voxgraph/map/naive_voxgraph_map.h"
#include "active_3d_planning_core/data/system_constraints.h"
#include "voxblox_ros/ros_params.h"

namespace active_3d_planning {
    namespace map {

        ModuleFactoryRegistry::Registration<NaiveVoxgraphMap> NaiveVoxgraphMap::registration("NaiveVoxgraphMap");

        NaiveVoxgraphMap::NaiveVoxgraphMap(PlannerI &planner) : TSDFMap(planner) {}

        void NaiveVoxgraphMap::setupFromParamMap(Module::ParamMap *param_map) {
            // create an esdf server
            ros::NodeHandle nh("");
            ros::NodeHandle nh_private("~");

            // Set istances of Voxgraph, the Planner Map Manager and the Frontiers Evaluator
            voxgraph_mapper_.reset(new voxgraph::VoxgraphMapper(nh, nh_private));
            // cache constants
            c_voxel_size_ = voxgraph_mapper_->getSubmapConfig().esdf_voxel_size;
            c_block_size_ = voxgraph_mapper_->getSubmapConfig().esdf_voxel_size *
                            voxgraph_mapper_->getSubmapConfig().esdf_voxels_per_side;
            c_maximum_weight_ = voxblox::getTsdfIntegratorConfigFromRosParam(
                    nh_private).max_weight;    // direct access is not exposed
        }


        bool NaiveVoxgraphMap::isTraversable(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
            return false;
            double distance = 0.0;

            voxblox::Point voxel_position(position.x(), position.y(), position.z());

            for (const auto& submap_id : voxgraph_mapper_->getSubmapCollection().getIDs()){
                voxblox::Point voxel_position_S = voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getInversePose() * voxel_position;
                Eigen::Vector3d position_S = Eigen::Vector3d(voxel_position_S.x(), voxel_position_S.y(), voxel_position_S.z());
                if ((voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getEsdfMap().
                        getDistanceAtPosition(position_S, &distance))
                    and
                    (distance < planner_.getSystemConstraints().collision_radius)) {
                    return false;
                }
            }
            return (distance > planner_.getSystemConstraints().collision_radius);
        }

        bool NaiveVoxgraphMap::isObserved(const Eigen::Vector3d &point) {
            return false;
            if (voxgraph_mapper_->getSubmapCollection().empty()) {
                return false;
            };

            voxblox::Point voxel_position(point.x(), point.y(), point.z());
            for (const auto& submap_id : voxgraph_mapper_->getSubmapCollection().getIDs()) {
                voxblox::Point voxel_point_S = voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getInversePose() * voxel_position;
                Eigen::Vector3d point_S = Eigen::Vector3d(voxel_point_S.x(), voxel_point_S.y(), voxel_point_S.z());

                if (voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getEsdfMap().isObserved(point_S))
                    return true;
            }
            return false;
        }

        // get occupancy
        unsigned char NaiveVoxgraphMap::getVoxelState(const Eigen::Vector3d &point) {
            voxblox::Point voxel_position(point.x(), point.y(), point.z());

            double distance = 0.0;
            bool free = false;
            for (const auto& submap_id : voxgraph_mapper_->getSubmapCollection().getIDs()) {
                voxblox::Point voxel_point_S = voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getInversePose() * voxel_position;
                Eigen::Vector3d point_S = Eigen::Vector3d(voxel_point_S.x(), voxel_point_S.y(), voxel_point_S.z());
                if ((voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getEsdfMap().
                        getDistanceAtPosition(point_S, &distance))) {

                    // if that point is occupied in a submap, return occupied
                    if (distance < planner_.getSystemConstraints().collision_radius) {
                        return NaiveVoxgraphMap::OCCUPIED;
                    } else {
                        free = true;
                    }
                }
            }

            if (free) {
                return NaiveVoxgraphMap::FREE; }
            else {
                return NaiveVoxgraphMap::UNKNOWN; }
        }

        // get voxel size
        double NaiveVoxgraphMap::getVoxelSize() {
            return c_voxel_size_;
        }

        // get the center of a voxel from input point
        bool NaiveVoxgraphMap::getVoxelCenter(Eigen::Vector3d *center, const Eigen::Vector3d &point) {
                *center = point;
        }

        // get the stored TSDF distance
        double NaiveVoxgraphMap::getVoxelDistance(const Eigen::Vector3d &point) {
            voxblox::Point voxel_position(point.x(), point.y(), point.z());
            float distance = INFINITY;

            for (const auto& submap_id : voxgraph_mapper_->getSubmapCollection().getIDs()) {
                voxblox::Point voxel_point_S = voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getInversePose() * voxel_position;
                Eigen::Vector3d point_S = Eigen::Vector3d(voxel_point_S.x(), voxel_point_S.y(), voxel_point_S.z());

                voxblox::Block<voxblox::TsdfVoxel>::Ptr block = voxgraph_mapper_->getSubmapCollection().
                        getSubmapPtr(submap_id)->getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(
                        voxel_point_S);
                if (block) {
                    voxblox::TsdfVoxel *tsdf_voxel = block->getVoxelPtrByCoordinates(voxel_point_S);
                    if (tsdf_voxel) {
                        distance = std::min(distance, tsdf_voxel->distance);
                    }
                }
            }
            if (distance != INFINITY) { return distance; }
            else { return 0.0; }
        }

        // get the stored weight as the sum of the weights in all the submap
        double NaiveVoxgraphMap::getVoxelWeight(const Eigen::Vector3d &point) {
            voxblox::Point voxel_position(point.x(), point.y(), point.z());
            double weight = 0.0;

            for (const auto& submap_id : voxgraph_mapper_->getSubmapCollection().getIDs()) {
                voxblox::Point voxel_point_S = voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getInversePose() * voxel_position;

                voxblox::Block<voxblox::TsdfVoxel>::Ptr block = voxgraph_mapper_->getSubmapCollection().
                        getSubmapPtr(submap_id)->getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(
                        voxel_point_S);
                if (block) {
                    voxblox::TsdfVoxel *tsdf_voxel = block->getVoxelPtrByCoordinates(voxel_point_S);
                    if (tsdf_voxel) {
                        weight += tsdf_voxel->weight;
                    }
                }
            }
            return std::min(weight, c_maximum_weight_);
        }

        // get the maximum allowed weight (return 0 if using uncapped weights)
        double NaiveVoxgraphMap::getMaximumWeight() {
            return c_maximum_weight_;
        }

        void NaiveVoxgraphMap::updateActiveSubmap(){
            // Update Esdf
            voxgraph_mapper_->getSubmapCollection().getActiveSubmapPtr()->updateEsdf();
        }

        bool NaiveVoxgraphMap::hasActiveMapFinished(){
            static int active_submap_id = 0;

            if (!voxgraph_mapper_->getSubmapCollection().empty()) {
                if (voxgraph_mapper_->getSubmapCollection().getActiveSubmapID() != active_submap_id){
                    active_submap_id = voxgraph_mapper_->getSubmapCollection().getActiveSubmapID();
                    return true;
                }
            }
            ROS_WARN("IS hasActiveMapFinished fine");
            return false;
        }


    }
}
