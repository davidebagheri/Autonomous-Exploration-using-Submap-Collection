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
                                                 const voxblox::EsdfIntegrator::Config& esdf_integrator_config =
                                                         voxblox::EsdfIntegrator::Config()){
        voxblox::EsdfIntegrator esdf_integrator(esdf_integrator_config, tsdf_layer, esdf_layer);
        // Generate the ESDF
        esdf_integrator.updateFromTsdfLayerBatch();
    }

    void PlannerMapManager::updateActiveSubmap() {
        BlockIndexList new_blocks;
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
        /// Test
        //ros::Time beg = ros::Time::now();
        voxblox::transformLayer(tsdf_updated_blocks_layer, submap_collection_ptr_->getActiveSubmap().getPose(),
                                &tsdf_transformed_updated_blocks_layer);
        //ros::Time end = ros::Time::now();

        // Update the active submap Tsdf
        copyTsdfLayerAInLayerBAndSetBlockID(tsdf_transformed_updated_blocks_layer,
                active_submap_ptr_->getTsdfMapPtr()->getTsdfLayerPtr(),
                submap_collection_ptr_->getActiveSubmapID());

        // Generate the updated Esdf
        active_submap_ptr_->updateEsdf();

        //return (end-beg).toNSec() / 1000000;
    }

    void PlannerMapManager::removeSubmapFromActiveSubmap(const SubmapID& id){
        BlockIndexList block_idx_list;
        Layer<TsdfVoxel>* tsdf_layer = active_submap_ptr_->getTsdfMapPtr()->getTsdfLayerPtr();
        Layer<EsdfVoxel>* esdf_layer = active_submap_ptr_->getEsdfMapPtr()->getEsdfLayerPtr();

        tsdf_layer->getAllAllocatedBlocks(&block_idx_list);

        for (const BlockIndex& block_idx : block_idx_list) {
            if (tsdf_layer->getBlockPtrByIndex(block_idx)->getSubmapID() == id){
                tsdf_layer->removeBlock(block_idx);
                esdf_layer->removeBlock(block_idx);
            }
        }
    }

    void PlannerMapManager::emptyActiveSubmap(){
        active_submap_ptr_->getTsdfMapPtr()->getTsdfLayerPtr()->removeAllBlocks();
        active_submap_ptr_->getEsdfMapPtr()->getEsdfLayerPtr()->removeAllBlocks();
    }

    std::vector<SubmapID> PlannerMapManager::getOverlappingSubmapIDs() {
        // Since the bounding boxes are computed only when a submap is finished and no more,
        // I create a copy on which I can compute the bounding box at every time step

        voxgraph::VoxgraphSubmap active_submap(submap_collection_ptr_->getActiveSubmap().getPose(),
                                               submap_collection_ptr_->getActiveSubmapID(),
                                               submap_collection_ptr_->getActiveSubmap().getTsdfMap().getTsdfLayer());
        // add the submaps overlapping with the active submap
        std::vector<SubmapID> overlapping_submap_ids;

        for (const auto &id : submap_collection_ptr_->getIDs()) {
            if (id != active_submap.getID()) {
                if (submap_collection_ptr_->getSubmap(id).overlapsWith(active_submap)) {
                    overlapping_submap_ids.emplace_back(id);
                }
            }
        }

        // Add the submaps overlapping with the previous active submap
        if (submap_collection_ptr_->getActiveSubmapID() > 0) {
            for (auto registration_contraint : *voxgraph_registration_constraint_ptr_) {
                if ((registration_contraint.getConfig().first_submap_id == (submap_collection_ptr_->getActiveSubmapID() - 1)) &&
                    (find(overlapping_submap_ids.begin(), overlapping_submap_ids.end(),
                          registration_contraint.getConfig().second_submap_id) ==
                            overlapping_submap_ids.end())) {
                    overlapping_submap_ids.emplace_back(registration_contraint.getConfig().second_submap_id);
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
        ros::Time beg = ros::Time::now();
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
            Layer<EsdfVoxel> esdf_layer_to_add(config_.esdf_voxel_size, config_.esdf_voxels_per_side);
            voxblox::naiveTransformLayer(submap_collection_ptr_->getSubmap(id_to_add).getEsdfMap().getEsdfLayer(),
                                         submap_collection_ptr_->getSubmap(id_to_add).getPose(),
                                         &esdf_layer_to_add);
            // Merge Esdf
            addSubmapEsdfInCurrentNeighbours(esdf_layer_to_add, id_to_add);

            // Merge Tsdf
            if (tsdf_needed_) {
                Layer<TsdfVoxel> tsdf_layer_to_add(config_.tsdf_voxel_size, config_.tsdf_voxels_per_side);
                voxblox::naiveTransformLayer(submap_collection_ptr_->getSubmap(id_to_add).getTsdfMap().getTsdfLayer(),
                                             submap_collection_ptr_->getSubmap(id_to_add).getPose(),
                                             &tsdf_layer_to_add);
                mergeTsdfLayerAInLayerB(tsdf_layer_to_add,
                                        current_neighbours_ptr_->getTsdfMapPtr()->getTsdfLayerPtr());
            }
        }

        // Add active submap to current neighbours
        addSubmapEsdfInCurrentNeighbours(active_submap_ptr_->getEsdfMap().getEsdfLayer(), active_submap_id);
        if (tsdf_needed_) {
            mergeTsdfLayerAInLayerB(active_submap_ptr_->getTsdfMap().getTsdfLayer(),
                                    current_neighbours_ptr_->getTsdfMapPtr()->getTsdfLayerPtr());
        }
    }

    // Merge submap current neighbours
    void PlannerMapManager::addSubmapEsdfInCurrentNeighbours(const voxblox::Layer<EsdfVoxel>& esdf_layer,
                                                             const SubmapID id){
        // Merge the incoming submap Tsdf layer into the local area map layer and retrieve the added blocks indexes
        BlockIndexList block_indexes_added;
        pessimisticEsdfMerge(esdf_layer,
                             current_neighbours_ptr_->getEsdfMapPtr()->getEsdfLayerPtr(),
                             &block_indexes_added);
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
            current_neighbours_ptr_->getEsdfMapPtr()->getEsdfLayerPtr()->removeBlock(block_id);
            if (tsdf_needed_) current_neighbours_ptr_->getTsdfMapPtr()->getTsdfLayerPtr()->removeBlock(block_id);
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
