#ifndef ACTIVE_3D_PLANNING_MAPS_NAIVE_VOXGRAPH_MAP_H
#define ACTIVE_3D_PLANNING_MAPS_NAIVE_VOXGRAPH_MAP_H

#include "active_3d_planning_core/map/tsdf_map.h"
#include "voxgraph/frontend/voxgraph_mapper.h"
#include <active_3d_planning_core/module/module_factory_registry.h>

#include <memory>

namespace active_3d_planning {
    namespace map {
        class NaiveVoxgraphMap : public TSDFMap { //, public active_3d_planning::ModuleBase {
        public:
            NaiveVoxgraphMap(PlannerI &planner);

            // implement virtual methods
            void setupFromParamMap(Module::ParamMap *param_map) override;

            // check collision for a single pose
            virtual bool isTraversable(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) override;

            // check whether point is part of the map
            virtual bool isObserved(const Eigen::Vector3d &point) override;

            // get occupancy
            virtual unsigned char getVoxelState(const Eigen::Vector3d &point) override;

            // get voxel size
            virtual double getVoxelSize() override;

            // get the center of a voxel from input point
            virtual bool getVoxelCenter(Eigen::Vector3d *center, const Eigen::Vector3d &point) override;

            // get the stored distance
            virtual double getVoxelDistance(const Eigen::Vector3d &point) override;

            // get the stored weight
            virtual double getVoxelWeight(const Eigen::Vector3d &point) override;

            // get the maximum allowed weight (return 0 if using uncapped weights)
            virtual double getMaximumWeight() override;

            void updateActiveSubmap();

            bool hasActiveMapFinished();

            void updateOverlappingSubmaps();

            std::vector<cblox::SubmapID> getSubmapsOnWhichSearching();

            voxgraph::VoxgraphMapper& getVoxgraphMapper(){ return *voxgraph_mapper_;}

        protected:
            static ModuleFactoryRegistry::Registration<NaiveVoxgraphMap> registration;

            std::unique_ptr<voxgraph::VoxgraphMapper> voxgraph_mapper_;

            // cache constants
            double c_voxel_size_;
            double c_block_size_;
            double c_maximum_weight_;

            // Mission to Odom transform
            voxblox::Transformation T_M_O_;
        };
    }
}

#endif //ACTIVE_3D_PLANNING_MAPS_NAIVE_VOXGRAPH_MAP_H
