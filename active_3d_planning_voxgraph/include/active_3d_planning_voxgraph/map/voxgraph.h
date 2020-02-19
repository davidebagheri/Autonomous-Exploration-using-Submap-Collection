#ifndef ACTIVE_3D_PLANNING_VOXBLOX_MAP_VOXBLOX_H
#define ACTIVE_3D_PLANNING_VOXBLOX_MAP_VOXBLOX_H

#include "active_3d_planning_core/map/map.h"
#include "active_3d_planning_core/map/tsdf_map.h"
#include <active_3d_planning_core/module/module_factory_registry.h>
#include "active_3d_planning_voxgraph/planner_map_manager/planner_map_manager.h"
#include "voxgraph/frontend/voxgraph_mapper.h"
#include "geometry_msgs/PoseStamped.h"
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

            void updatePlanningMaps();

            bool hasActiveMapFinished();

            void publishActiveSubmap();

            void publishCurrentNeighbours();

            void publishFrontiers();

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

            const voxblox::Point getRobotPosition(){
                return voxgraph_mapper_->getRobotPosition();
            }

        protected:
            static ModuleFactoryRegistry::Registration<VoxgraphMap> registration;

            std::unique_ptr<voxgraph::VoxgraphMapper> voxgraph_mapper_;
            std::unique_ptr<PlannerMapManager> planner_map_manager_;

            // Publishers
            ::ros::Publisher pointcloud_tsdf_active_submap_pub_;
            ::ros::Publisher pointcloud_esdf_active_submap_pub_;
            ::ros::Publisher pointcloud_tsdf_current_neighbours_pub_;
            ::ros::Publisher pointcloud_esdf_current_neighbours_pub_;

            // cache constants
            double c_voxel_size_;
            double c_block_size_;
            double c_maximum_weight_;
        };

    } // namespace map
} // namespace active_3d_planning

#endif // ACTIVE_3D_PLANNING_VOXGRAPH_MAP_VOXGRAPH_H
