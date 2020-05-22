#ifndef ACTIVE_3D_PLANNING_VOXBLOX_MAP_VOXBLOX_H
#define ACTIVE_3D_PLANNING_VOXBLOX_MAP_VOXBLOX_H

#include "active_3d_planning_core/map/map.h"
#include "active_3d_planning_core/map/tsdf_map.h"
#include <active_3d_planning_core/module/module_factory_registry.h>
#include "active_3d_planning_voxgraph/planner_map_manager/planner_map_manager.h"
#include "voxgraph/frontend/voxgraph_mapper.h"
#include "geometry_msgs/PoseStamped.h"
#include "active_3d_planning_core/data/bounding_volume.h"
#include <memory>

namespace active_3d_planning {
    namespace map {

        // Voxblox as a map representation
        class VoxgraphMap : public TSDFMap {
        public:
            VoxgraphMap(PlannerI &planner);

            // implement virtual methods
            void setupFromParamMap(Module::ParamMap *param_map) override;

            // get the maximum allowed weight (return 0 if using uncapped weights)
            virtual double getMaximumWeight() override;

            // get voxel size
            virtual double getVoxelSize() override;

            /* This methods are used to check the feasibility of the trajectory, o they work on the active submap*/
            // check collision for a single pose
            virtual bool isTraversable(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) override;

            virtual bool isTraversable(const double& collision_radius, const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);

            // check whether point is part of the map
            virtual bool isObserved(const Eigen::Vector3d &point) override;

            // get occupancy
            virtual unsigned char getVoxelState(const Eigen::Vector3d &point) override;

            // get the center of a voxel from input point
            virtual bool getVoxelCenter(Eigen::Vector3d *center, const Eigen::Vector3d &point) override;

            // get the stored distance
            virtual double getVoxelDistance(const Eigen::Vector3d &point) override;

            // get the stored weight
            virtual double getVoxelWeight(const Eigen::Vector3d &point) override;

            /* This methods are used for computing the gain of the trajectory, so they work on the local area */
            // check whether point is part of the map
            bool isObservedInCurrentNeighbours(const Eigen::Vector3d &point);

            // get occupancy
            unsigned char getCurrentNeighboursVoxelState(const Eigen::Vector3d &point);

            // get the center of a voxel from input point
            bool getCurrentNeighboursVoxelCenter(Eigen::Vector3d *center, const Eigen::Vector3d &point);

            // get the stored TSDF distance
            double getCurrentNeighboursVoxelDistance(const Eigen::Vector3d &point);

            // get the stored weight
            double getCurrentNeighboursVoxelWeight(const Eigen::Vector3d &point);

            // Check if a trajectory is traversable in the active submap
            bool isTraversableClosePath(const EigenTrajectoryPointVector& trajectory);

            // check collision in all the submaps
            bool isTraversableInAllSubmaps(const Eigen::Vector3d &point);

            // check observability in all the submaps
            bool isObservedInAllSubmaps(const Eigen::Vector3d &point);

            // check whether the points in the 6-connectivity are observed in the submaps
            bool isInsideAllSubmaps(const Eigen::Vector3d &point);

            // check whether the points in the 6-connectivity are observed in the active submap
            bool isInsideActiveSubmap(const Eigen::Vector3d &point);

            // get Free points in a square centered in the given point
            void getFreeNeighbouringPoints(const Eigen::Vector3d &point, std::vector<Eigen::Vector3d>* free_points);

            std::vector<SubmapID> getSubmapsIncludingPoint(const Eigen::Vector3d& point);

            void updatePlanningMaps();

            bool hasActiveMapFinished(bool update = true);

            void publishPlannerActiveSubmap();

            void publishCurrentNeighbours();

            void publishActiveSubmap(){
                voxgraph_mapper_->publishActiveSubmap();
            }

            voxgraph::VoxgraphSubmapCollection &getSubmapCollection() {
                return voxgraph_mapper_->getSubmapCollection();
            }

            const voxgraph::VoxgraphSubmap::Config& getSubmapConfig(){
                return voxgraph_mapper_->getSubmapConfig();
            }

            PlannerMapManager& getPlannerMapManager(){
                return *planner_map_manager_;
            }

            const std::list<voxgraph::RegistrationConstraint>& getRegistrationConstraint(){
                return voxgraph_mapper_->getRegistrationConstraint();
            }

            voxblox::Transformation get_T_M_O(){
                return voxgraph_mapper_->getMapTracker().get_T_M_O();
            }

        protected:
            static ModuleFactoryRegistry::Registration<VoxgraphMap> registration;

            std::unique_ptr<voxgraph::VoxgraphMapper> voxgraph_mapper_;
            std::unique_ptr<PlannerMapManager> planner_map_manager_;
            std::unique_ptr<BoundingVolume> bounding_box_;

            // Publishers
            ::ros::Publisher pointcloud_tsdf_active_submap_pub_;
            ::ros::Publisher pointcloud_esdf_active_submap_pub_;
            ::ros::Publisher pointcloud_tsdf_current_neighbours_pub_;
            ::ros::Publisher pointcloud_esdf_current_neighbours_pub_;

            ::ros::Publisher global_candidates_pub_;

            // cache constants
            double c_voxel_size_;
            double c_block_size_;
            double c_maximum_weight_;
            double search_distance_;
            double search_step_;
            double neighbourhood_distance_;
            Eigen::Vector3d c_neighbor_robot_[6];
            bool tsdf_needed_;
        };

    } // namespace map
} // namespace active_3d_planning

#endif // ACTIVE_3D_PLANNING_VOXGRAPH_MAP_VOXGRAPH_H
