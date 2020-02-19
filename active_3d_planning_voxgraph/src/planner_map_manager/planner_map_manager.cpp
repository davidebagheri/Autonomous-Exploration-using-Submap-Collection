#include "active_3d_planning_voxgraph/planner_map_manager/planner_map_manager.h"

namespace active_3d_planning {
    void PlannerMapManager::getUpdatedBlocksLayer(Layer<TsdfVoxel>* tsdf_layer_in,
                                                  Layer<TsdfVoxel>* tsdf_layer_out,
                                                  voxblox::Update::Status bit = voxblox::Update::kEsdf){

        const std::bitset<voxblox::Update::kCount> reset; // Reset bitset

        // Get only the updated blocks
        BlockIndexList updated_blocks_kesdf;

        tsdf_layer_in->getAllUpdatedBlocks(
                bit, &updated_blocks_kesdf);

        // Store the updated blocks in the input tsdf layer
        std::pair<BlockIndex, Block<TsdfVoxel>::Ptr> block_pair;

        for (auto block_idx : updated_blocks_kesdf){
            // Add new blocks
            block_pair.first = block_idx;
            block_pair.second = tsdf_layer_in->getBlockPtrByIndex(block_idx);

            tsdf_layer_out->insertBlock(block_pair);

            // Reset the update flag
            tsdf_layer_in->getBlockPtrByIndex(block_idx)->set_updated(reset);
        }
    }

    void PlannerMapManager::getEsdfFromTsdfLayer(Layer<TsdfVoxel>* tsdf_layer,
                                                 Layer<EsdfVoxel>* esdf_layer,
                                                 const voxblox::EsdfIntegrator::Config& esdf_integrator_config = voxblox::EsdfIntegrator::Config()){

        voxblox::EsdfIntegrator esdf_integrator(esdf_integrator_config,
                                                tsdf_layer,
                                                esdf_layer);
        // Generate the ESDF
        esdf_integrator.updateFromTsdfLayerBatch();
    }

    template <typename VoxelType>
    void PlannerMapManager::transformLayer(const Layer<VoxelType>& layer_in, Transformation T_out_in, Layer<VoxelType>* layer_out){
        Layer<VoxelType> transformed_tsdf_layer(layer_in.voxel_size(), layer_in.voxels_per_side());

        voxblox::naiveTransformLayer(layer_in, T_out_in, &transformed_tsdf_layer);

        mergeLayerAInLayerB(transformed_tsdf_layer, layer_out);
    }

    void PlannerMapManager::updateActiveSubmap() {
        Layer<TsdfVoxel> tsdf_updated_blocks_layer(config_.tsdf_voxel_size,
                                                   config_.tsdf_voxels_per_side);
        Layer<TsdfVoxel> tsdf_transformed_updated_blocks_layer(config_.tsdf_voxel_size,
                                                               config_.tsdf_voxels_per_side);
        Layer<EsdfVoxel> esdf_transformed_updated_blocks_layer(config_.esdf_voxel_size,
                                                               config_.esdf_voxels_per_side);

        // Put the updated block into a layer
        getUpdatedBlocksLayer(submap_collection_ptr_->getActiveSubmapPtr()->getTsdfMapPtr()->getTsdfLayerPtr(),
                              &tsdf_updated_blocks_layer);

        // Transform the TSDF updated blocks layer in mission frame
        voxblox::transformLayer(tsdf_updated_blocks_layer,
                       submap_collection_ptr_->getActiveSubmap().getPose(),
                       &tsdf_transformed_updated_blocks_layer);

        // Generate ESDF of the updated blocks layer
        getEsdfFromTsdfLayer(&tsdf_transformed_updated_blocks_layer, &esdf_transformed_updated_blocks_layer);

        //Merge Tsdf
        copyTsdfLayerAInLayerB(tsdf_transformed_updated_blocks_layer,
                               active_submap_ptr_->getTsdfMapPtr()->getTsdfLayerPtr());

        //Merge Esdf
        copyEsdfLayerAInLayerB(esdf_transformed_updated_blocks_layer,
                               active_submap_ptr_->getEsdfMapPtr()->getEsdfLayerPtr());
    }

    void PlannerMapManager::emtpyActiveSubmap(){
        active_submap_ptr_->getEsdfMapPtr()->getEsdfLayerPtr()->removeAllBlocks();
        active_submap_ptr_->getTsdfMapPtr()->getTsdfLayerPtr()->removeAllBlocks();
    }


    std::vector<SubmapID> PlannerMapManager::getOverlappingSubmapIDs() {
        // Since the bounding boxes are computed only when a submap is finished and no more,
        // I create a copy on which I can compute the bounding box at every time step
        // (Maybe I can change the voxgraph code)
        voxgraph::VoxgraphSubmap active_submap(submap_collection_ptr_->getActiveSubmap().getPose(),
                                     submap_collection_ptr_->getActiveSubmapID(),
                                     submap_collection_ptr_->getActiveSubmap().getTsdfMap().getTsdfLayer());

        // Get all submap IDs
        std::vector<SubmapID> submap_ids(submap_collection_ptr_->getIDs());

        // Retrieve the overlapping submap ids
        std::vector<SubmapID> overlapping_submap_ids;
        overlapping_submap_ids.reserve(submap_collection_ptr_->getIDs().size() - 1);

        for (const auto &id : submap_ids) {
            if (id != active_submap.getID()) {
                if (submap_collection_ptr_->getSubmap(id).overlapsWith(active_submap)) {
                    overlapping_submap_ids.emplace_back(id);
                }
            }
        }

        return overlapping_submap_ids;
    }

    void PlannerMapManager::selectCurrentNeighboursSubmapIDs(const std::vector<SubmapID>& full_list,
                                                             std::vector<SubmapID>* IDs_to_add,
                                                             std::vector<SubmapID>* IDs_to_remove){

        // IDs to add
        for (auto id : full_list){
            if (!isInNeighbouringSubmapList(id))
                IDs_to_add->emplace_back(id);
        }
        // IDs to remove
        for (auto id_block_pair : current_neighbours_block_index_map_){
            auto it = std::find(full_list.begin(), full_list.end(), id_block_pair.first);
            if (it == full_list.end()) {
                IDs_to_remove->emplace_back(id_block_pair.first);
            }
        }
    }

    void PlannerMapManager::updateCurrentNeighbours(const SubmapID& active_submap_id){
        std::vector<SubmapID> IDs_to_add;
        std::vector<SubmapID> IDs_to_remove;

        // Get the submap that overlap with the active submap
        std::vector<SubmapID> overlapping_submap_IDs = getOverlappingSubmapIDs();

        // Choose the submaps to add and to remove
        selectCurrentNeighboursSubmapIDs(overlapping_submap_IDs, &IDs_to_add, &IDs_to_remove);

        // Erode the submaps
        std::vector<SubmapID>::iterator it;

        for (auto id_to_remove : IDs_to_remove){
            erodeSubmapFromCurrentNeighbours(id_to_remove);
        }

        // Add the submaps
        for (auto id_to_add : IDs_to_add){
            Layer<TsdfVoxel> tsdf_layer_to_add(config_.tsdf_voxel_size, config_.tsdf_voxels_per_side);
            Layer<EsdfVoxel> esdf_layer_to_add(config_.esdf_voxel_size, config_.esdf_voxels_per_side);

            // Transform submap TSDF
            std::thread transform_tsdf(&PlannerMapManager::transformLayer<TsdfVoxel>,
                                       this,
                                       submap_collection_ptr_->getSubmap(id_to_add).getTsdfMap().getTsdfLayer(),
                                       submap_collection_ptr_->getSubmap(id_to_add).getPose(),
                                       &tsdf_layer_to_add);

            // Transform submap ESDF
            std::thread transform_esdf(&PlannerMapManager::transformLayer<EsdfVoxel>,
                                       this,
                                       submap_collection_ptr_->getSubmap(id_to_add).getEsdfMap().getEsdfLayer(),
                                       submap_collection_ptr_->getSubmap(id_to_add).getPose(),
                                       &esdf_layer_to_add);

            transform_tsdf.join();
            transform_esdf.join();

            // Merge
            addSubmapInCurrentNeighbours(tsdf_layer_to_add, esdf_layer_to_add, id_to_add);
        }

        // Add active submap to current neighbours
        addSubmapInCurrentNeighbours(active_submap_ptr_->getTsdfMap().getTsdfLayer(),
                active_submap_ptr_->getEsdfMap().getEsdfLayer(),
                active_submap_id);
    }

    // Merge submap current neighbours
    void PlannerMapManager::addSubmapInCurrentNeighbours(const voxblox::Layer<TsdfVoxel>& tsdf_layer,
                                                         const voxblox::Layer<EsdfVoxel>& esdf_layer,
                                                         const SubmapID id){

        // Merge the incoming submap Tsdf layer into the local area map layer and retrieve the added blocks indexes
        BlockIndexList block_indexes_added;

        mergeLayerAndGetUpdatedBlocksIDS(tsdf_layer,
                                         current_neighbours_ptr_->getTsdfMapPtr()->getTsdfLayerPtr(),
                                         &block_indexes_added);

        // Merge the incoming submap Esdf layer
        mergeLayerAInLayerB<EsdfVoxel>(esdf_layer,
                                       current_neighbours_ptr_->getEsdfMapPtr()->getEsdfLayerPtr());

        // Add the new block indexes in the map
        std::map<SubmapID,BlockIndexList>::iterator it;
        it = current_neighbours_block_index_map_.find(id);
        if (it == current_neighbours_block_index_map_.end()) {
            current_neighbours_block_index_map_.insert(std::pair<SubmapID, BlockIndexList>(id, block_indexes_added));
        } else {
            for (const auto block_id : block_indexes_added){
                current_neighbours_block_index_map_[id].push_back(block_id);
            }
        }
    }

    void PlannerMapManager::erodeSubmapFromCurrentNeighbours(const SubmapID& id){
        // Remove all the blocks corresponding to a submap id
        for (auto block_id : current_neighbours_block_index_map_[id]){
            current_neighbours_ptr_->getTsdfMapPtr()->getTsdfLayerPtr()->removeBlock(block_id);
            current_neighbours_ptr_->getEsdfMapPtr()->getEsdfLayerPtr()->removeBlock(block_id);
        }

        // Remove the id from the map
        current_neighbours_block_index_map_.erase(id);
    }

    bool PlannerMapManager::isInNeighbouringSubmapList(const SubmapID& id){
        for(const auto& id_block_pair : current_neighbours_block_index_map_) {
            if (id_block_pair.first == id) {
                return true;
            }
        }
        return false;
    }
}
