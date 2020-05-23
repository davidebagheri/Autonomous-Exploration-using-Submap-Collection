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

            // Get params
            setParam<bool>(param_map, "tsdf_needed", &tsdf_needed_, true);
            setParam<double>(param_map, "search_distance", &search_distance_, 3.0);
            setParam<double>(param_map, "search_step", &search_step_, 0.5);
            setParam<double>(param_map, "neighbourhood_distance_", &neighbourhood_distance_, 2.0);

            // Set istances of Voxgraph, the Planner Map Manager and the Frontiers Evaluator
            voxgraph_mapper_.reset(new voxgraph::VoxgraphMapper(nh, nh_private));
            planner_map_manager_.reset(new PlannerMapManager(
                    &voxgraph_mapper_->getSubmapCollection(),
                    &voxgraph_mapper_->getMapTracker(),
                    &voxgraph_mapper_->getRegistrationConstraint(),
                    tsdf_needed_,
                    voxgraph_mapper_->getSubmapConfig()));


            std::string temp_args;
            std::string ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "bounding_volume_args", &temp_args,"/map_bounding_volume");

            // Ros publishers
            pointcloud_tsdf_active_submap_pub_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(
                    "pointcloud_tsdf_active_submap", 1, true);
            pointcloud_esdf_active_submap_pub_  = nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(
                    "pointcloud_esdf_active_submap", 1, true);
            pointcloud_tsdf_current_neighbours_pub_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(
                    "pointcloud_tsdf_current_neighbours", 1, true);
            pointcloud_esdf_current_neighbours_pub_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(
                    "pointcloud_esdf_current_neighbours", 1, true);

            global_candidates_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>(
                    "global_candidates", 1, true);

            // cache constants
            c_voxel_size_ = voxgraph_mapper_->getSubmapConfig().esdf_voxel_size;
            c_block_size_ = voxgraph_mapper_->getSubmapConfig().esdf_voxel_size *
                            voxgraph_mapper_->getSubmapConfig().esdf_voxels_per_side;
            c_maximum_weight_ = voxblox::getTsdfIntegratorConfigFromRosParam(
                    nh_private).max_weight;    // direct access is not exposed

            c_neighbor_robot_[0] = Eigen::Vector3d(planner_.getSystemConstraints().collision_radius, 0, 0);
            c_neighbor_robot_[1] = Eigen::Vector3d(-planner_.getSystemConstraints().collision_radius, 0, 0);
            c_neighbor_robot_[2] = Eigen::Vector3d(0, planner_.getSystemConstraints().collision_radius, 0);
            c_neighbor_robot_[3] = Eigen::Vector3d(0, -planner_.getSystemConstraints().collision_radius, 0);
            c_neighbor_robot_[4] = Eigen::Vector3d(0, 0, planner_.getSystemConstraints().collision_radius);
            c_neighbor_robot_[5] = Eigen::Vector3d(0, 0, -planner_.getSystemConstraints().collision_radius);
        }

        bool VoxgraphMap::isTraversable(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
            double distance = 0.0;

            if (planner_map_manager_->getActiveSubmapPtr()->getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
                // This means the voxel is observed
                return (distance > planner_.getSystemConstraints().collision_radius);
            }
            return false;
        }

        bool VoxgraphMap::isTraversable(const double& collision_radius, const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation){
            double distance = 0.0;

            if (planner_map_manager_->getActiveSubmapPtr()->getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
                // This means the voxel is observed
                return (distance > collision_radius);
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

        // Collision check considering all the submaps
        bool VoxgraphMap::isTraversableInAllSubmaps(const Eigen::Vector3d &point){
            double distance = 0.0;
            voxblox::Point point_M(point.x(), point.y(), point.z());

            for (auto submap_id : voxgraph_mapper_->getSubmapCollection().getIDs()){
                voxblox::Point point_S = voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getInversePose() * point_M;
                Eigen::Vector3d point_S_d(point_S.x(), point_S.y(), point_S.z());

                if (voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id)
                .getEsdfMap().getDistanceAtPosition(point_S_d, &distance)){
                    if (distance < planner_.getSystemConstraints().collision_radius) return false;
                }
            }
            return (distance > planner_.getSystemConstraints().collision_radius);
        }

        bool VoxgraphMap::isObservedInAllSubmaps(const Eigen::Vector3d &point){
            voxblox::Point point_M(point.x(), point.y(), point.z());

            for (auto submap_id : voxgraph_mapper_->getSubmapCollection().getIDs()) {
                voxblox::Point point_S = voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getInversePose() * point_M;
                Eigen::Vector3d point_S_d(point_S.x(), point_S.y(), point_S.z());
                if (voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getEsdfMap().isObserved(point_S_d)){
                    return true;
                }
            }
            return false;
        }

        bool VoxgraphMap::isInsideAllSubmaps(const Eigen::Vector3d &point){
            for (int i = 0; i < 6; i++){
                if (!isObservedInAllSubmaps(point + c_neighbor_robot_[i])) return false;
            }
            return true;;
        }

        bool VoxgraphMap::isInsideActiveSubmap(const Eigen::Vector3d &point){
            for (int i = 0; i < 6; i++){
                //if (!isObserved(point + c_neighbor_robot_[i])) return false;
                double distance = 0;
                if (planner_map_manager_->getActiveSubmapPtr()->getEsdfMapPtr()
                ->getDistanceAtPosition(point + c_neighbor_robot_[i], &distance)) {
                    // This means the voxel is observed
                    if (distance > 0) continue;
                }
                return false;
            }
            return true;;
        }

        bool VoxgraphMap::isTraversableClosePath(const EigenTrajectoryPointVector& trajectory){
            for (const EigenTrajectoryPoint &point : trajectory) {
                if ((point.position_W - planner_.getCurrentPosition()).norm() > neighbourhood_distance_) continue;

                if (!isTraversable(point.position_W, point.orientation_W_B)) return false;
            }
            return true;
        }

        void VoxgraphMap::getFreeNeighbouringPoints(const Eigen::Vector3d &point, std::vector<Eigen::Vector3d>* free_points){
            visualization_msgs::Marker marker;
            visualization_msgs::MarkerArray marker_array;
            marker.header.frame_id = "world";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1;
            marker.id = 0;

            free_points->clear();
            Eigen::Vector3d candidate_point(point.x() - search_distance_/2,
                    point.y() - search_distance_/2,
                    point.z() - search_distance_/2);

            double x_max = point.x() + search_distance_/2;
            double y_max = point.y() + search_distance_/2;
            double z_max = point.z() + search_distance_/2;

            while (candidate_point.x() <= x_max){
                while (candidate_point.y() <= y_max){
                    while (candidate_point.z() <= z_max){
                        voxblox::Point candidate_point_M = get_T_M_O() * voxblox::Point(candidate_point.x(), candidate_point.y(), candidate_point.z());
                        Eigen::Vector3d candidate_point_d_M(candidate_point_M.x(), candidate_point_M.y(), candidate_point_M.z());
                        if (isTraversableInAllSubmaps(candidate_point_d_M)){
                            free_points->emplace_back(candidate_point_d_M);
                            marker.color.r = 1;
                            marker.color.g = 0;
                            marker.color.b = 0;
                            marker.pose.position.x = candidate_point.x();
                            marker.pose.position.y = candidate_point.y();
                            marker.pose.position.z = candidate_point.z();
                            marker.id++;
                            marker_array.markers.emplace_back(marker);
                        } else {
                            marker.color.r = 0;
                            marker.color.g = 0;
                            marker.color.b = 1;
                            marker.id++;
                            marker.pose.position.x = candidate_point.x();
                            marker.pose.position.y = candidate_point.y();
                            marker.pose.position.z = candidate_point.z();
                            marker_array.markers.emplace_back(marker);
                        }
                        candidate_point.z() += search_step_;
                    }
                    candidate_point.z() = point.z() - search_distance_/2;
                    candidate_point.y() += search_step_;
                }
                candidate_point.y() = point.y() - search_distance_/2;
                candidate_point.x() += search_step_;
            }

            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 0;
            marker.pose.position.x = point.x();
            marker.pose.position.y = point.y();
            marker.pose.position.z = point.z();
            marker_array.markers.emplace_back(marker);
            marker.id++;
            global_candidates_pub_.publish(marker_array);
        }

        // Update the Active submap and the Current neighbours map used for planning
        void VoxgraphMap::updatePlanningMaps(){
            if (voxgraph_mapper_->getSubmapCollection().empty()) return;

            // Update planner maps
            planner_map_manager_->updateActiveSubmap();

            planner_map_manager_->updateCurrentNeighbours(
                    voxgraph_mapper_->getSubmapCollection().getActiveSubmapID());

            // Planner maps visualization
            publishPlannerActiveSubmap();
            publishCurrentNeighbours();
        }

        bool VoxgraphMap::hasActiveMapFinished(bool update){
            static SubmapID active_submap_id = 0;
            if (!voxgraph_mapper_->getSubmapCollection().empty()) {
                if (voxgraph_mapper_->getSubmapCollection().getActiveSubmapID() != active_submap_id){
                    if (update) {
                        active_submap_id = voxgraph_mapper_->getSubmapCollection().getActiveSubmapID();
                    }
                    return true;
                }
            }
            return false;
        }

        // Visualization
        void VoxgraphMap::publishPlannerActiveSubmap(){
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

        std::vector<SubmapID> VoxgraphMap::getSubmapsIncludingPoint(const Eigen::Vector3d& point){
            std::vector<SubmapID> result;
            voxblox::Point voxel_point(point.x(), point.y(), point.z());

            for (auto submap_id : voxgraph_mapper_->getSubmapCollection().getIDs()){
                voxblox::Point voxel_point_S =
                        voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getPose().inverse() * get_T_M_O() *
                        voxel_point;
                Eigen::Vector3d voxel_point_S_d(voxel_point_S.x(), voxel_point_S.y(), voxel_point_S.z());

                if (voxgraph_mapper_->getSubmapCollection().getSubmap(submap_id).getEsdfMap().isObserved(voxel_point_S_d)){
                    result.push_back(submap_id);
                }
            }
            return result;
        }


    } // namespace map
} // namespace active_3d_planning
