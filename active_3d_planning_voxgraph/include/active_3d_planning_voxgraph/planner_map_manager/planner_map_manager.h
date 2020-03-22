#ifndef ACTIVE_3D_PLANNING_VOXGRAPH_PLANNER_MAP_MANAGER_H
#define ACTIVE_3D_PLANNING_VOXGRAPH_PLANNER_MAP_MANAGER_H

#include "voxgraph/frontend/voxgraph_mapper.h"
#include "voxblox/integrator/merge_integration.h"
#include "active_3d_planning_voxgraph/planner_map_manager/tools/merge_layer.h"
#include <vector>
#include <thread>

namespace active_3d_planning {
    using SubmapID = voxgraph::SubmapID;
    using Transformation = voxblox::Transformation;

    class PlannerMapManager {
    public:

        PlannerMapManager(voxgraph::VoxgraphSubmapCollection* submap_collection_ptr,
                std::list<voxgraph::RegistrationConstraint>* voxgraph_registration_constraint_ptr,
                bool tsdf_needed,
                voxgraph::VoxgraphSubmap::Config config)
                : active_submap_ptr_(std::make_shared<cblox::TsdfEsdfSubmap>(config)),
                  current_neighbours_ptr_(std::make_shared<cblox::TsdfEsdfSubmap>(config)),
                  tsdf_needed_(tsdf_needed){
            config_ = config;
            submap_collection_ptr_ = submap_collection_ptr;
            voxgraph_registration_constraint_ptr_ = voxgraph_registration_constraint_ptr;
        }

        void updateActiveSubmap();

        void updateCurrentNeighbours(const SubmapID& active_submap_id);

        void getUpdatedBlocksLayer(Layer<TsdfVoxel>* tsdf_layer_in,
                                   Layer<TsdfVoxel>* tsdf_layer_out,
                                   voxblox::Update::Status bit);

        // This function gets the IDs of the submaps overlapping with the active map
        std::vector<SubmapID> getOverlappingSubmapIDs();

        // This function distinguish among ID to add, to remove or already present comparing an incoming
        // list with local_area_submap_IDs_ and updating the latter
        void selectCurrentNeighboursSubmapIDs(const std::vector<SubmapID>& full_list,
                std::vector<SubmapID>* IDs_to_add,
                std::vector<SubmapID>* IDs_to_remove);

        void getEsdfFromTsdfLayer(Layer<TsdfVoxel>* tsdf_layer,
                                  Layer<EsdfVoxel>* esdf_layer,
                                  const voxblox::EsdfIntegrator::Config& esdf_integrator_config);

        void removeSubmapFromActiveSubmap(const SubmapID& id);

        void emptyActiveSubmap();

        void addSubmapEsdfInCurrentNeighbours(const voxblox::Layer<EsdfVoxel>& esdf_layer,
                                              const SubmapID id);

        void erodeSubmapFromCurrentNeighbours(const SubmapID& id);

        bool isInNeighbouringSubmapList(const SubmapID& id);

        const cblox::TsdfEsdfSubmap& getActiveSubmap(){
            return *active_submap_ptr_;
        }

        cblox::TsdfEsdfSubmap::Ptr getActiveSubmapPtr(){
            return active_submap_ptr_;
        }

        const cblox::TsdfEsdfSubmap& getCurrentNeighbours(){
            return *current_neighbours_ptr_;
        }

        cblox::TsdfEsdfSubmap::Ptr getCurrentNeighboursPtr(){
            return current_neighbours_ptr_;
        }


    private:
        voxgraph::VoxgraphSubmap::Config config_;

        // Planner maps
        cblox::TsdfEsdfSubmap::Ptr active_submap_ptr_;
        cblox::TsdfEsdfSubmap::Ptr current_neighbours_ptr_;

        // Voxgraph submap collection and registration constraints
        voxgraph::VoxgraphSubmapCollection* submap_collection_ptr_;
        std::list<voxgraph::RegistrationConstraint>* voxgraph_registration_constraint_ptr_;

        // Current neighbours blocks map
        std::map<SubmapID,BlockIndexList> current_neighbours_block_index_map_;

        // Params
        bool tsdf_needed_;  // If true, even the active submap and current neighbours Tsdfs are computed
    };
}

#endif //ACTIVE_3D_PLANNING_VOXGRAPH_PLANNER_MAP_MANAGER_H
