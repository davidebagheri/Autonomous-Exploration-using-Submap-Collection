#include "cblox/core/submap_collection.h"
#include "cblox_planning/cblox_extension/submap_server.h"
#include "std_srvs/Empty.h"
#include "pcl/point_cloud.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "active_3d_planning_voxgraph/planner_map_manager/tools/merge_layer.h"
#include "voxblox_ros/esdf_server.h"

namespace cblox {
    template<>
    inline const SubmapCollection<TsdfEsdfSubmap>::Ptr&
    SubmapServer<TsdfEsdfSubmap>::getSubmapCollectionPtr() const {
        return submap_collection_ptr_;
    }
}


namespace active_3d_planning{
class VoxgraphNodeEvaluator{


public:
    VoxgraphNodeEvaluator(const ros::NodeHandle &nh,
                           const ros::NodeHandle &nh_private);
    bool evaluate(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res);

    std::string evaluateSingle(std::string map_name);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::ServiceServer eval_srv_;
    ros::Publisher global_map_pub_;
    ros::Publisher bounding_box_pub_;
    ros::Publisher good_pointcloud_pub_;

    cblox::SubmapServer<cblox::TsdfEsdfSubmap> submap_server_;

    voxblox::BoundingBox bounding_box_;

    // params
    std::string p_target_dir_;
    std::string log_file_path_;
    std::ofstream log_file_;

    bool p_evaluate_;
    bool p_evaluate_ground_truth_;
};

VoxgraphNodeEvaluator::VoxgraphNodeEvaluator(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
        nh_(nh),
        nh_private_(nh_private),
        submap_server_(nh, nh_private){
    global_map_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI>>("voxels", 1, true);
    bounding_box_pub_ = nh_private_.advertise<visualization_msgs::Marker>("buonding_box", 1, true);

    eval_srv_ = nh_private_.advertiseService("evaluate", &VoxgraphNodeEvaluator::evaluate, this);
    nh_private_.param<float>("/evaluation_bounding_volume/x_min",bounding_box_.min_vertex.x(), 0.0);
    nh_private_.param<float>("/evaluation_bounding_volume/x_max",bounding_box_.max_vertex.x(), 0.0);
    nh_private_.param<float>("/evaluation_bounding_volume/y_min", bounding_box_.min_vertex.y(), 0.0);
    nh_private_.param<float>("/evaluation_bounding_volume/y_max", bounding_box_.max_vertex.y(), 0.0);
    nh_private_.param<float>("/evaluation_bounding_volume/z_min", bounding_box_.min_vertex.z(), 0.0);
    nh_private_.param<float>("/evaluation_bounding_volume/z_max", bounding_box_.max_vertex.z(), 0.0);
}

bool VoxgraphNodeEvaluator::evaluate(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res) {
    // Get target directory
    nh_private_.getParam("target_directory", p_target_dir_);

    log_file_path_ = p_target_dir_ + "/data_log.txt";
    std::ifstream read_log(log_file_path_.c_str());
    std::ifstream data_file((p_target_dir_ + "/voxblox_data.csv").c_str());
    if (!(read_log.is_open() && data_file.is_open())) {
        std::cout << p_target_dir_ + "/voxblox_data.csv" << std::endl;
        std::cout << log_file_path_.c_str() << std::endl;
        ROS_ERROR("Unable to load the data and/or log files.");
        read_log.close();
        data_file.close();
        return false;
    }

    // Get action to perform
    nh_private_.param("evaluate", p_evaluate_, true);
    nh_private_.param("evaluate_ground_truth", p_evaluate_ground_truth_, true);

    // Check not previously evaluated
    std::string line;
    if (p_evaluate_) {
        while (std::getline(read_log, line)) {
            if (line == "[FLAG] Evaluated") {
                ROS_INFO("This file was already evaluated.");
                p_evaluate_ = false;
            }
        }
    }
    read_log.close();
    if (!p_evaluate_) return true;

    // Setup log file
    log_file_.open(log_file_path_.c_str(), std::ios::app);

    // Parse and evaluate all lines in the data file
    std::string map_name;
    int n_maps = 0;
    std::ofstream fout;
    if (p_evaluate_) {
        fout.open((p_target_dir_ + "/voxblox_data_temp.csv").c_str(),
                  std::ios::out);
    }
    ROS_INFO("Start processing maps.");
    while (std::getline(data_file, line)) {
        map_name = line.substr(0, line.find(","));
        if (map_name == "MapName") {
            map_name = "Header";
        } else if (map_name == "Unit") {
            map_name = "Unit";
        } else {
            if (!std::ifstream(
                    (p_target_dir_ + "/voxgraph_collections/" + map_name + ".vxgrp")
                            .c_str())) {
                ROS_INFO("Skipping map '%s' (non-existant)", map_name.c_str());
                if (p_evaluate_) {
                    fout << line + ",0,0,0,0,0,0,0\n"; /// TODO: check if less zeros
                }
                continue;
            }
            n_maps++;
        }
        ROS_INFO("Processing: %s", map_name.c_str());
        std::string result = evaluateSingle(map_name);
        if (p_evaluate_) {
            fout << line + "," + result + "\n";
        }
    }
    data_file.close();
    if (p_evaluate_) {
        fout.close();
    }
    ROS_INFO("Finished processing maps.");

    // Finish
    if (p_evaluate_) {
        if (std::rename((p_target_dir_ + "/voxblox_data_temp.csv").c_str(),
                        (p_target_dir_ + "/voxblox_data.csv").c_str()) != 0) {
            ROS_ERROR("Could not rename temp csv file!");
        }
        log_file_ << "[FLAG] Evaluated\n";
        ROS_INFO("Voxgraph evaluation finished succesfully.");
    }
}

std::string VoxgraphNodeEvaluator::evaluateSingle(std::string map_name) {
    static int marker_id = 0;
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    pcl::PointCloud<pcl::PointXYZI> good_pointcloud;

    if (map_name == "Header") {
        return "UnknownVoxels,UnknownVoxelsGroundTruth";
    } else if (map_name == "Unit") {
        return "percent,percent";
    }

    int observed_voxels = 0;

    float map_volume = (bounding_box_.max_vertex.x() - bounding_box_.min_vertex.x()) *
                       (bounding_box_.max_vertex.y() - bounding_box_.min_vertex.y()) *
                       (bounding_box_.max_vertex.z() - bounding_box_.min_vertex.z());
    float voxel_volume = std::pow(submap_server_.getSubmapCollectionPtr()->getConfig().esdf_voxel_size, 3);

    // Load map
    submap_server_.getSubmapCollectionPtr()->clear();
    submap_server_.loadMap(p_target_dir_ + "/voxgraph_collections/" + map_name + ".vxgrp");

    voxblox::Layer<voxblox::EsdfVoxel> global_map(submap_server_.getSubmapCollectionPtr()->getConfig().tsdf_voxel_size,
                                                  submap_server_.getSubmapCollectionPtr()->getConfig().tsdf_voxels_per_side);

    // Compute global map
    for (auto id : submap_server_.getSubmapCollectionPtr()->getIDs()) {
        voxblox::Layer<voxblox::EsdfVoxel> temp(submap_server_.getSubmapCollectionPtr()->getConfig().esdf_voxel_size,
                                                submap_server_.getSubmapCollectionPtr()->getConfig().esdf_voxels_per_side);
        voxblox::naiveTransformLayer(submap_server_.getSubmapCollectionPtr()->getSubmap(id).getEsdfMap().getEsdfLayer(),
                                submap_server_.getSubmapCollectionPtr()->getSubmap(id).getPose(),
                                &temp);
        mergeEsdfLayerAInLayerB(temp, &global_map);

        // Visualize
        if (global_map_pub_.getNumSubscribers() > 0) {
            pcl::PointCloud<pcl::PointXYZI> tsdf_pointcloud;
            tsdf_pointcloud.header.frame_id = "world";
            voxblox::createDistancePointcloudFromEsdfLayer(
                    global_map,
                    &tsdf_pointcloud);
            global_map_pub_.publish(tsdf_pointcloud);
        }
        if (bounding_box_pub_.getNumSubscribers() > 0) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time();
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = (bounding_box_.max_vertex.x() + bounding_box_.min_vertex.x()) / 2;
            marker.pose.position.y = (bounding_box_.max_vertex.y() + bounding_box_.min_vertex.y()) / 2;
            marker.pose.position.z = (bounding_box_.max_vertex.z() + bounding_box_.min_vertex.z()) / 2;
            marker.scale.x = (bounding_box_.max_vertex.x() - bounding_box_.min_vertex.x());
            marker.scale.y = (bounding_box_.max_vertex.y() - bounding_box_.min_vertex.y());
            marker.scale.z = (bounding_box_.max_vertex.z() - bounding_box_.min_vertex.z());
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            bounding_box_pub_.publish( marker );
        }
    }

    voxblox::BlockIndexList block_idx_list;
    global_map.getAllAllocatedBlocks(&block_idx_list);

    for (const voxblox::BlockIndex &block_idx : block_idx_list) {
        typename voxblox::Block<voxblox::EsdfVoxel>::ConstPtr block = global_map.getBlockPtrByIndex(block_idx);

        // Iterate over all the voxels in the layer
        for (voxblox::IndexElement voxel_idx = 0;
             voxel_idx < static_cast<voxblox::IndexElement>(block->num_voxels()); ++voxel_idx) {
            // Bounding box check
            voxblox::Point voxel_coords = block->computeCoordinatesFromLinearIndex(voxel_idx);
            if (!bounding_box_.contains(voxel_coords)) continue;

            if (block->getVoxelByLinearIndex(voxel_idx).observed == true) {
                observed_voxels++;
            }
        }
    }

    float unknown_voxels_pct = 1.0 - ((float) observed_voxels) / (map_volume / voxel_volume);

    // Build result
    std::ostringstream result("");
    result << unknown_voxels_pct;

    // Load Map ground truth
    if (p_evaluate_ground_truth_) {
        std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer;
        voxblox::Interpolator<voxblox::TsdfVoxel>::Ptr interpolator;
        voxblox::io::LoadLayer<voxblox::TsdfVoxel>(
                p_target_dir_ + "/voxblox_collections/" + map_name + ".vxblx", &tsdf_layer);
        interpolator.reset(
                new voxblox::Interpolator<voxblox::TsdfVoxel>(tsdf_layer.get()));

        int ground_truth_observed = 0;

        voxblox::BlockIndexList block_idx_list;
        tsdf_layer->getAllAllocatedBlocks(&block_idx_list);

        for (const voxblox::BlockIndex &block_idx : block_idx_list) {
            typename voxblox::Block<voxblox::TsdfVoxel>::ConstPtr block = tsdf_layer->getBlockPtrByIndex(block_idx);

            // Iterate over all the voxels in the layer
            for (voxblox::IndexElement voxel_idx = 0;
                 voxel_idx < static_cast<voxblox::IndexElement>(block->num_voxels()); ++voxel_idx) {
                // Bounding box check
                voxblox::Point voxel_coords = block->computeCoordinatesFromLinearIndex(voxel_idx);
                if (!bounding_box_.contains(voxel_coords)) continue;

                if (block->getVoxelByLinearIndex(voxel_idx).weight > 0) {
                    ground_truth_observed++;
                }
            }
        }
        std::cout << "oh grande, ci sono n voxel " << ground_truth_observed << std::endl;
        float unknown_voxels_ground_truth_pct = 1.0 - ((float) ground_truth_observed) / (map_volume / voxel_volume);

        result << "," << unknown_voxels_ground_truth_pct;
    }

    return result.str();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "evaluation_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    active_3d_planning::VoxgraphNodeEvaluator eval(nh, nh_private);
    ros::spin();
    return 0;
}
