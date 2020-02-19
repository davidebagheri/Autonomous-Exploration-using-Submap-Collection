#include "active_3d_planning_voxgraph/map/voxgraph.h"
#include "active_3d_planning_core/data/system_constraints.h"

#include <voxblox_ros/ros_params.h>

namespace active_3d_planning {
    namespace map {
        ModuleFactoryRegistry::Registration<VoxgraphMap> VoxgraphMap::registration("VoxgraphMap");

        VoxgraphMap::VoxgraphMap(PlannerI &planner) : TSDFMap(planner) {}

        void VoxgraphMap::setupFromParamMap(Module::ParamMap *param_map) {
            // create an esdf server
            ros::NodeHandle nh("");
            ros::NodeHandle nh_private("~");

            // Set istances of Voxgraph, the Planner Map Manager and the Frontiers Evaluator
            voxgraph_mapper_.reset(new voxgraph::VoxgraphMapper(nh, nh_private));
            planner_map_manager_.reset(new PlannerMapManager(
                    &voxgraph_mapper_->getSubmapCollection(),
                    voxgraph_mapper_->getSubmapConfig()));

            // Ros publishers
            pointcloud_tsdf_active_submap_pub_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(
                    "pointcloud_tsdf_active_submap", 1, true);
            pointcloud_esdf_active_submap_pub_  = nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(
                    "pointcloud_esdf_active_submap", 1, true);
            pointcloud_tsdf_current_neighbours_pub_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(
                    "pointcloud_tsdf_current_neighbours", 1, true);
            pointcloud_esdf_current_neighbours_pub_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(
                    "pointcloud_esdf_current_neighbours", 1, true);

            // cache constants
            c_voxel_size_ = voxgraph_mapper_->getSubmapConfig().esdf_voxel_size;
            c_block_size_ = voxgraph_mapper_->getSubmapConfig().esdf_voxel_size *
                            voxgraph_mapper_->getSubmapConfig().esdf_voxels_per_side;
            c_maximum_weight_ = voxblox::getTsdfIntegratorConfigFromRosParam(
                    nh_private).max_weight;    // direct access is not exposed
        }

        bool VoxgraphMap::isTraversable(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
            double distance = 0.0;
            if (planner_map_manager_->getActiveSubmapPtr()->getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
                // This means the voxel is observed
                return (distance > planner_.getSystemConstraints().collision_radius);
            }
            return false;
        }

        bool VoxgraphMap::isObserved(const Eigen::Vector3d &point) {
            return planner_map_manager_->getActiveSubmapPtr()->getEsdfMapPtr()->isObserved(point);
        }

        // get occupancy
        unsigned char VoxgraphMap::getVoxelState(const Eigen::Vector3d &point) {
            double distance = 0.0;
            if (planner_map_manager_->getActiveSubmapPtr()->getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) {
                // This means the voxel is observed
                if (distance < c_voxel_size_) {
                    return VoxgraphMap::OCCUPIED;
                } else {
                    return VoxgraphMap::FREE;
                }
            } else {
                return VoxgraphMap::UNKNOWN;
            }
        }

        // get voxel size
        double VoxgraphMap::getVoxelSize() {
            return c_voxel_size_;
        }

        // get the center of a voxel from input point
        bool VoxgraphMap::getVoxelCenter(Eigen::Vector3d *center, const Eigen::Vector3d &point) {
            voxblox::BlockIndex block_id = planner_map_manager_->getActiveSubmapPtr()->getEsdfMapPtr()->
                    getEsdfLayerPtr()->computeBlockIndexFromCoordinates(point.cast<voxblox::FloatingPoint>());
            *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_).cast<double>();
            voxblox::VoxelIndex voxel_id = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
                    (point - *center).cast<voxblox::FloatingPoint>(), 1.0 / c_voxel_size_);
            *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_).cast<double>();
            return true;
        }

        // get the stored TSDF distance
        double VoxgraphMap::getVoxelDistance(const Eigen::Vector3d &point) {
            voxblox::Point voxblox_point(point.x(), point.y(), point.z());
            voxblox::Block<voxblox::TsdfVoxel>::Ptr block = planner_map_manager_->getActiveSubmapPtr()->
                    getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(voxblox_point);
            if (block) {
                voxblox::TsdfVoxel *tsdf_voxel = block->getVoxelPtrByCoordinates(voxblox_point);
                if (tsdf_voxel) {
                    return tsdf_voxel->distance;
                }
            }
            return 0.0;
        }

        // get the stored weight
        double VoxgraphMap::getVoxelWeight(const Eigen::Vector3d &point) {
            voxblox::Point voxblox_point(point.x(), point.y(), point.z());
            voxblox::Block<voxblox::TsdfVoxel>::Ptr block = planner_map_manager_->getActiveSubmapPtr()->
                    getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(voxblox_point);
            if (block) {
                voxblox::TsdfVoxel *tsdf_voxel = block->getVoxelPtrByCoordinates(voxblox_point);
                if (tsdf_voxel) {
                    return tsdf_voxel->weight;
                }
            }
            return 0.0;
        }

        // get the maximum allowed weight (return 0 if using uncapped weights)
        double VoxgraphMap::getMaximumWeight() {
            return c_maximum_weight_;
        }

        bool VoxgraphMap::isObservedInCurrentNeighbours(const Eigen::Vector3d &point) {
            return planner_map_manager_->getCurrentNeighboursPtr()->getEsdfMapPtr()->isObserved(point);
        }

        // get occupancy
        unsigned char VoxgraphMap::getCurrentNeighboursVoxelState(const Eigen::Vector3d &point) {
            double distance = 0.0;
            if (planner_map_manager_->getCurrentNeighboursPtr()->getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) {
                    // This means the voxel is observed
                if (distance < c_voxel_size_) {
                    return VoxgraphMap::OCCUPIED;
                } else {
                    return VoxgraphMap::FREE;
                }
            } else {
                return VoxgraphMap::UNKNOWN;
            }
        }

        // get the center of a voxel from input point
        bool VoxgraphMap::getCurrentNeighboursVoxelCenter(Eigen::Vector3d *center, const Eigen::Vector3d &point) {
            voxblox::BlockIndex block_id = planner_map_manager_->getCurrentNeighboursPtr()->getEsdfMapPtr()->
                    getEsdfLayerPtr()->computeBlockIndexFromCoordinates(point.cast<voxblox::FloatingPoint>());
            *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_).cast<double>();
            voxblox::VoxelIndex voxel_id = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
                    (point - *center).cast<voxblox::FloatingPoint>(), 1.0 / c_voxel_size_);
            *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_).cast<double>();
            return true;
        }

        // get the stored TSDF distance
        double VoxgraphMap::getCurrentNeighboursVoxelDistance(const Eigen::Vector3d &point) {
            voxblox::Point voxblox_point(point.x(), point.y(), point.z());
            voxblox::Block<voxblox::TsdfVoxel>::Ptr block = planner_map_manager_->getCurrentNeighboursPtr()->
                    getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(voxblox_point);
            if (block) {
                voxblox::TsdfVoxel *tsdf_voxel = block->getVoxelPtrByCoordinates(voxblox_point);
                if (tsdf_voxel) {
                    return tsdf_voxel->distance;
                }
            }
            return 0.0;
        }

        // get the stored weight
        double VoxgraphMap::getCurrentNeighboursVoxelWeight(const Eigen::Vector3d &point) {
            voxblox::Point voxblox_point(point.x(), point.y(), point.z());
            voxblox::Block<voxblox::TsdfVoxel>::Ptr block = planner_map_manager_->getCurrentNeighboursPtr()->
                    getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(voxblox_point);
            if (block) {
                voxblox::TsdfVoxel *tsdf_voxel = block->getVoxelPtrByCoordinates(voxblox_point);
                if (tsdf_voxel) {
                    return tsdf_voxel->weight;
                }
            }
            return 0.0;
        }


        void VoxgraphMap::updatePlanningMaps(){
            if (voxgraph_mapper_->getSubmapCollection().empty()) return;

            // Update planner maps
            planner_map_manager_->updateActiveSubmap();
            planner_map_manager_->updateCurrentNeighbours(
                    voxgraph_mapper_->getSubmapCollection().getActiveSubmapID());

            // Planner maps visualization
            publishActiveSubmap();
            publishCurrentNeighbours();
        }

        bool VoxgraphMap::hasActiveMapFinished(){
            static SubmapID active_submap_id = 0;

            if (!voxgraph_mapper_->getSubmapCollection().empty()) {
                if (voxgraph_mapper_->getSubmapCollection().getActiveSubmapID() != active_submap_id){
                    active_submap_id = voxgraph_mapper_->getSubmapCollection().getActiveSubmapID();
                    return true;
                }
            }
            return false;
        }

        void VoxgraphMap::publishActiveSubmap(){
            if (pointcloud_tsdf_active_submap_pub_.getNumSubscribers() > 0){
                pcl::PointCloud<pcl::PointXYZI> tsdf_pointcloud;
                tsdf_pointcloud.header.frame_id = "world";
                voxblox::createDistancePointcloudFromTsdfLayer(
                        planner_map_manager_->getActiveSubmap().getTsdfMap().getTsdfLayer(),
                        &tsdf_pointcloud);
                pointcloud_tsdf_active_submap_pub_.publish(tsdf_pointcloud);
            }

            if (pointcloud_esdf_active_submap_pub_.getNumSubscribers() > 0){
                pcl::PointCloud<pcl::PointXYZI> esdf_pointcloud;
                esdf_pointcloud.header.frame_id = "world";
                voxblox::createDistancePointcloudFromEsdfLayer(
                        planner_map_manager_->getActiveSubmap().getEsdfMap().getEsdfLayer(),
                        &esdf_pointcloud);
                pointcloud_esdf_active_submap_pub_.publish(esdf_pointcloud);
            }
        }

        void VoxgraphMap::publishCurrentNeighbours(){
            if (pointcloud_tsdf_current_neighbours_pub_.getNumSubscribers() > 0){
                pcl::PointCloud<pcl::PointXYZI> tsdf_pointcloud;
                tsdf_pointcloud.header.frame_id = "world";
                voxblox::createDistancePointcloudFromTsdfLayer(
                        planner_map_manager_->getCurrentNeighbours().getTsdfMap().getTsdfLayer(),
                        &tsdf_pointcloud);
                pointcloud_tsdf_current_neighbours_pub_.publish(tsdf_pointcloud);
            }

            if (pointcloud_esdf_current_neighbours_pub_.getNumSubscribers() > 0){
                pcl::PointCloud<pcl::PointXYZI> esdf_pointcloud;
                esdf_pointcloud.header.frame_id = "world";
                voxblox::createDistancePointcloudFromEsdfLayer(
                        planner_map_manager_->getCurrentNeighbours().getEsdfMap().getEsdfLayer(),
                        &esdf_pointcloud);
                pointcloud_esdf_current_neighbours_pub_.publish(esdf_pointcloud);
            }
        }
    } // namespace map
} // namespace active_3d_planning
