#include "active_3d_planning_voxgraph/planner_map_manager/tools/merge_layer.h"

namespace active_3d_planning{

    void pessimisticEsdfVoxelMerge(const EsdfVoxel& voxel_A, EsdfVoxel* voxel_B) {
        if (voxel_A.observed && voxel_B->observed) {
            voxel_B->distance = std::min(voxel_A.distance, voxel_B->distance);
        } else if (voxel_A.observed && !voxel_B->observed) {
            voxel_B->distance = voxel_A.distance;
        }
        voxel_B->observed = voxel_B->observed || voxel_A.observed;
    }

    template <>
    void copyVoxelAInVoxelB(const TsdfVoxel& voxel_A, TsdfVoxel* voxel_B){
        voxel_B->distance = voxel_A.distance;
        voxel_B->color = voxel_A.color;
        voxel_B->weight = voxel_A.weight;
    }

    template <>
    void copyVoxelAInVoxelB(const EsdfVoxel& voxel_A, EsdfVoxel* voxel_B){
        if (voxel_A.observed) {
            voxel_B->distance = voxel_A.distance;
        }
        voxel_B->observed = voxel_B->observed || voxel_A.observed;
    }

    template <typename VoxelType>
    void copyBlockAInBlockB(const Block<VoxelType>& block_A, Block<VoxelType>* block_B_ptr) {
        CHECK_EQ(block_A.voxel_size(), block_B_ptr->voxel_size());
        CHECK_EQ(block_A.voxels_per_side(), block_B_ptr->voxels_per_side());

        for (voxblox::IndexElement voxel_idx = 0;
             voxel_idx < static_cast<voxblox::IndexElement>(block_A.num_voxels()); ++voxel_idx) {
            copyVoxelAInVoxelB(block_A.getVoxelByLinearIndex(voxel_idx),
                               &(block_B_ptr->getVoxelByLinearIndex(voxel_idx)));
        }
    }

    void copyEsdfLayerAInLayerB(const Layer<EsdfVoxel>& layer_A, Layer<EsdfVoxel>* layer_B){
        CHECK_NOTNULL(layer_B);

        const Layer<EsdfVoxel>* layer_A_ptr = &layer_A;

        BlockIndexList block_idx_list_A;
        layer_A.getAllAllocatedBlocks(&block_idx_list_A);

        for (const BlockIndex& block_idx : block_idx_list_A) {
            typename Block<EsdfVoxel>::ConstPtr block_A_ptr =
                    layer_A_ptr->getBlockPtrByIndex(block_idx);
            typename Block<EsdfVoxel>::Ptr block_B_ptr =
                    layer_B->getBlockPtrByIndex(block_idx);

            if (!block_B_ptr) {
                block_B_ptr = layer_B->allocateBlockPtrByIndex(block_idx);
            }

            if ((block_A_ptr != nullptr) && (block_B_ptr != nullptr)) {
                copyBlockAInBlockB(*block_A_ptr, block_B_ptr.get());
            }
        }
    }

    void copyTsdfLayerAInLayerBAndSetBlockID(const Layer<TsdfVoxel>& layer_A, Layer<TsdfVoxel>* layer_B, uint8_t id){
        CHECK_NOTNULL(layer_B);

        const Layer<TsdfVoxel>* layer_A_ptr = &layer_A;

        BlockIndexList block_idx_list_A;
        layer_A.getAllAllocatedBlocks(&block_idx_list_A);

        for (const BlockIndex& block_idx : block_idx_list_A) {
            typename Block<TsdfVoxel>::ConstPtr block_A_ptr =
                    layer_A_ptr->getBlockPtrByIndex(block_idx);
            typename Block<TsdfVoxel>::Ptr block_B_ptr =
                    layer_B->getBlockPtrByIndex(block_idx);

            // Set the passed as argument id to all the updated blocks
            if (!block_B_ptr) {
                block_B_ptr = layer_B->allocateBlockPtrByIndex(block_idx);
                block_B_ptr->setSubmapID(id);
                // Set the kEsdf bit to true
                block_B_ptr->set_updated(std::bitset<voxblox::Update::Status::kCount>("100"));
            }

            if ((block_A_ptr != nullptr) && (block_B_ptr != nullptr)) {
                copyBlockAInBlockB(*block_A_ptr, block_B_ptr.get());
                block_B_ptr->setSubmapID(id);
                // Set the kEsdf bit to true
                block_B_ptr->set_updated(std::bitset<voxblox::Update::Status::kCount>("100"));
            }
        }
    }

    void copyTsdfLayerAInLayerB(const Layer<TsdfVoxel>& layer_A, Layer<TsdfVoxel>* layer_B){
        CHECK_NOTNULL(layer_B);

        const Layer<TsdfVoxel>* layer_A_ptr = &layer_A;


        BlockIndexList block_idx_list_A;
        layer_A.getAllAllocatedBlocks(&block_idx_list_A);

        for (const BlockIndex& block_idx : block_idx_list_A) {
            typename Block<TsdfVoxel>::ConstPtr block_A_ptr =
                    layer_A_ptr->getBlockPtrByIndex(block_idx);
            typename Block<TsdfVoxel>::Ptr block_B_ptr =
                    layer_B->getBlockPtrByIndex(block_idx);

            if (!block_B_ptr) {
                block_B_ptr = layer_B->allocateBlockPtrByIndex(block_idx);
            }

            if ((block_A_ptr != nullptr) && (block_B_ptr != nullptr)) {
                copyBlockAInBlockB(*block_A_ptr, block_B_ptr.get());
            }
        }
    }

    template <typename VoxelType>
    void mergeBlockAInBlockB(const Block<VoxelType>& block_A, Block<VoxelType>* block_B_ptr) {
        CHECK_EQ(block_A.voxel_size(), block_B_ptr->voxel_size());
        CHECK_EQ(block_A.voxels_per_side(), block_B_ptr->voxels_per_side());

        for (voxblox::IndexElement voxel_idx = 0;
             voxel_idx < static_cast<voxblox::IndexElement>(block_A.num_voxels()); ++voxel_idx) {
            voxblox::mergeVoxelAIntoVoxelB<VoxelType>(
                    block_A.getVoxelByLinearIndex(voxel_idx),
                    &(block_B_ptr->getVoxelByLinearIndex(voxel_idx)));
        }
    }

    void pessimisticEsdfBlockMerge(const Block<EsdfVoxel>& block_A, Block<EsdfVoxel>* block_B_ptr) {
        CHECK_EQ(block_A.voxel_size(), block_B_ptr->voxel_size());
        CHECK_EQ(block_A.voxels_per_side(), block_B_ptr->voxels_per_side());

        for (voxblox::IndexElement voxel_idx = 0;
             voxel_idx < static_cast<voxblox::IndexElement>(block_A.num_voxels()); ++voxel_idx) {
            pessimisticEsdfVoxelMerge(
                    block_A.getVoxelByLinearIndex(voxel_idx),
                    &(block_B_ptr->getVoxelByLinearIndex(voxel_idx)));
        }
    }

    template <typename VoxelType>
    void mergeLayerAInLayerB(const Layer<VoxelType>& layer_A,
                             Layer<VoxelType>* layer_B) {
        CHECK_NOTNULL(layer_B);

        const Layer<VoxelType>* layer_A_ptr = &layer_A;

        BlockIndexList block_idx_list_A;
        layer_A.getAllAllocatedBlocks(&block_idx_list_A);

        for (const BlockIndex& block_idx : block_idx_list_A) {
            typename Block<VoxelType>::ConstPtr block_A_ptr =
                    layer_A_ptr->getBlockPtrByIndex(block_idx);
            typename Block<VoxelType>::Ptr block_B_ptr =
                    layer_B->getBlockPtrByIndex(block_idx);

            if (!block_B_ptr) {
                block_B_ptr = layer_B->allocateBlockPtrByIndex(block_idx);
            }

            if ((block_A_ptr != nullptr) && (block_B_ptr != nullptr)) {
                mergeBlockAInBlockB(*block_A_ptr, block_B_ptr.get());
            }
        }
    }

    void pessimisticEsdfMerge(const Layer<EsdfVoxel>& layer_A,
                              Layer<EsdfVoxel>* layer_B,
                              BlockIndexList* new_blocks_list) {
        CHECK_NOTNULL(layer_B);

        // if voxel layout is different resample layer A to match B
        const Layer<EsdfVoxel>* layer_A_ptr = &layer_A;

        BlockIndexList block_idx_list_A;
        layer_A.getAllAllocatedBlocks(&block_idx_list_A);
        BlockIndexList new_blocks;

        for (const BlockIndex& block_idx : block_idx_list_A) {
            typename Block<EsdfVoxel>::ConstPtr block_A_ptr =
                    layer_A_ptr->getBlockPtrByIndex(block_idx);
            typename Block<EsdfVoxel>::Ptr block_B_ptr =
                    layer_B->getBlockPtrByIndex(block_idx);

            if (!block_B_ptr) {
                block_B_ptr = layer_B->allocateBlockPtrByIndex(block_idx);
                new_blocks.emplace_back(block_idx);
            }

            if ((block_A_ptr != nullptr) && (block_B_ptr != nullptr)) {
                pessimisticEsdfBlockMerge(*block_A_ptr, block_B_ptr.get());
            }
        }
        *new_blocks_list = new_blocks;
    }

    void mergeEsdfLayerAInLayerB(const Layer<EsdfVoxel>& layer_A, Layer<EsdfVoxel>* layer_B){
        mergeLayerAInLayerB<EsdfVoxel>(layer_A, layer_B);
    }

    void mergeTsdfLayerAInLayerB(const Layer<TsdfVoxel>& layer_A, Layer<TsdfVoxel>* layer_B){
        mergeLayerAInLayerB<TsdfVoxel>(layer_A, layer_B);
    }

}