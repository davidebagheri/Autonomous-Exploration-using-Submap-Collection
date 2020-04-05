#ifndef ACTIVE_3D_PLANNING_VOXGRAPH_SUBMAP_FRONTIER_EVALUATOR_H
#define ACTIVE_3D_PLANNING_VOXGRAPH_SUBMAP_FRONTIER_EVALUATOR_H

#include "submap_frontiers.h"
#include "active_3d_planning_voxgraph/planner_map_manager/tools/merge_layer.h"
#include "voxgraph/frontend/voxgraph_mapper.h"
#include "active_3d_planning_core/module/module.h"
#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_voxgraph/map/voxgraph.h"
#include "active_3d_planning_core/data/bounding_volume.h"

namespace active_3d_planning {
    using SubmapID = voxgraph::SubmapID;
    using VoxelStatus = voxblox::VoxelStatus;

    class SubmapFrontierEvaluator : public Module{
    public:
        SubmapFrontierEvaluator(PlannerI &planner);

        void setupFromParamMap(Module::ParamMap *param_map) override;

        void wavefrontFrontierDetector(const Point& pose,
                                       Layer<EsdfVoxel>& input_layer,
                                       std::vector<Frontier>* frontiers_vector);

        void updateSubmapFrontiers();

        void updateFrontiersPoses();

        void updateFrontiersPoints();

        void transformSubmapFrontiers(const SubmapID& submap_id, const Transformation& T_new_old);

        void computeSubmapFrontiers(const SubmapID& submap_id);

        void setSubmapFrontiersPose(const SubmapID& submap_id, const Transformation& T_M_S);

        void publishAllSubmapFrontiers();

        SubmapFrontiers& getSubmapFrontiers(SubmapID submap_id);

        void selectFirstFreeLayerPoint(const Layer<EsdfVoxel>& layer, Point* result);

        const Transformation getSubmapFrontiersPose(const SubmapID& submap_id);

        void getSubmapBlocksFromLayer(const SubmapID& id,
                                      const cblox::TsdfEsdfSubmap& submap,
                                      Layer<EsdfVoxel>* submap_layer);

        std::map<SubmapID,SubmapFrontiers>& getSubmapFrontiersMap(){
            return submap_frontiers_map_;
        }

    protected:
        static ModuleFactoryRegistry::Registration<SubmapFrontierEvaluator> registration;

        void getNeighbouringSubmaps(const SubmapID& submap_id,
                                    const std::list<voxgraph::RegistrationConstraint>& registration_contraints,
                                    std::vector<SubmapID>* neighbouring_submap_ids);

        bool isFrontier(const Point &voxel, const Layer<EsdfVoxel>& layer);

        bool isFrontier(const Point &voxel, const std::vector<SubmapID>& neighbouring_submap_ids);

        void setFrontierOpenList(const Point& point, Layer<EsdfVoxel>& layer);

        void setFrontierCloseList(const Point& point, Layer<EsdfVoxel>& layer);

        void setMapOpenList(const Point& point, Layer<EsdfVoxel>& layer);

        void setMapCloseList(const Point& point, Layer<EsdfVoxel>& layer);

        bool isMapOpenList(const Point& point, const Layer<EsdfVoxel>& layer);

        bool isMapCloseList(const Point& point, const Layer<EsdfVoxel>& layer);

        bool isFrontierOpenList(const Point& point, const Layer<EsdfVoxel>& layer);

        bool isFrontierCloseList(const Point& point, const Layer<EsdfVoxel>& layer);

        bool p_accurate_frontiers_;     // True: explicitely compute all frontier voxels (may degrade performance),
        // false: estimate frontier voxels by checking only some neighbors (detection depends on previous views)

        // bounding box
        std::unique_ptr<BoundingVolume> bounding_volume_;

        // Submap frontier container
        std::map<SubmapID,SubmapFrontiers> submap_frontiers_map_;

        // Transformation from Odom to Mission frame
        Transformation T_M_O_ = voxblox::Transformation();

        // Map
        map::VoxgraphMap* map_;

        // constants
        float c_voxel_size_;
        Point c_neighbor_voxels_[26];
        int n_frontier_points_threshold_;   // If the number of points in a frontier is below this number, it is erased

        // Ros publisher for visualization
        ::ros::Publisher frontier_pub_;
        ::ros::Publisher quarantine_pub_;
    };
}

#endif //ACTIVE_3D_PLANNING_VOXGRAPH_SUBMAP_FRONTIER_EVALUATOR_H
