#ifndef ACTIVE_3D_PLANNING_VOXGRAPH_MERGE_LAYER_H
#define ACTIVE_3D_PLANNING_VOXGRAPH_MERGE_LAYER_H

#include "voxblox/core/layer.h"

namespace active_3d_planning{
    using TsdfVoxel = voxblox::TsdfVoxel;
    using EsdfVoxel = voxblox::EsdfVoxel;
    template <typename VoxelType>
    using Layer = voxblox::Layer<VoxelType>;
    template <typename VoxelType>
    using Block = voxblox::Block<VoxelType>;
    using BlockIndex = voxblox::BlockIndex;
    using BlockIndexList = voxblox::BlockIndexList;

    template <typename VoxelType>
    void copyVoxelAInVoxelB(const VoxelType& voxel_A, VoxelType* voxel_B);

    template <>
    void copyVoxelAInVoxelB(const TsdfVoxel& voxel_A, TsdfVoxel* voxel_B);

    template <>
    void copyVoxelAInVoxelB(const EsdfVoxel& voxel_A, EsdfVoxel* voxel_B);

    void copyEsdfLayerAInLayerB(const Layer<EsdfVoxel>& layer_A, Layer<EsdfVoxel>* layer_B);

    void copyTsdfLayerAInLayerB(const Layer<TsdfVoxel>& layer_A, Layer<TsdfVoxel>* layer_B);

    template <typename VoxelType>
    void mergeBlockAInBlockB(const Block<VoxelType>& block_A, Block<VoxelType>* block_B_ptr);

    template <typename VoxelType>
    void copyBlockAInBlockB(const Block<VoxelType>& block_A, Block<VoxelType>* block_B_ptr);

    template <typename VoxelType>
    void mergeLayerAInLayerB(const Layer<VoxelType>& layer_A, Layer<VoxelType>* layer_B);

    void mergeEsdfLayerAInLayerB(const Layer<EsdfVoxel>& layer_A, Layer<EsdfVoxel>* layer_B);

    void mergeTsdfLayerAInLayerB(const Layer<TsdfVoxel>& layer_A, Layer<TsdfVoxel>* layer_B);

    void mergeLayerAndGetUpdatedBlocksIDS(const Layer<TsdfVoxel>& layer_A, Layer<TsdfVoxel>* layer_B, BlockIndexList* new_blocks_list);
}


#endif //ACTIVE_3D_PLANNING_VOXGRAPH_MERGE_LAYER_H
