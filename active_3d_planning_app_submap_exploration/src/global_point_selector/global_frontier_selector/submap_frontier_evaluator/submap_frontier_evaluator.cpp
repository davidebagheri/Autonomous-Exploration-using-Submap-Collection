#include "active_3d_planning_app_submap_exploration/global_point_selector/global_frontier_selector/submap_frontier_evaluator/submap_frontier_evaluator.h"

namespace active_3d_planning {
    // Registration
    ModuleFactoryRegistry::Registration<SubmapFrontierEvaluator> SubmapFrontierEvaluator::registration("SubmapFrontierEvaluator");

    SubmapFrontierEvaluator::SubmapFrontierEvaluator(PlannerI &planner) : Module(planner){}

    void SubmapFrontierEvaluator::setupFromParamMap(Module::ParamMap *param_map){
        setParam<float>(param_map, "voxel_size", &c_voxel_size_, 0.20);
        setParam<bool>(param_map, "accurate_frontiers", &p_accurate_frontiers_, false);
        setParam<int>(param_map, "n_frontier_points_threshold", &n_frontier_points_threshold_, 30);

        map_ = dynamic_cast<map::VoxgraphMap*>(&planner_.getMap());
        if (!map_) {
            planner_.printError("'SubmapFrontierEvaluator' requires a map of type 'VoxgraphMap'!");
        }

        std::string temp_args;
        std::string ns = (*param_map)["param_namespace"];
        setParam<std::string>(param_map, "bounding_volume_args", &temp_args,"/target_bounding_volume");
        bounding_volume_ = planner_.getFactory().createModule<BoundingVolume>(temp_args, planner_, verbose_modules_);

        // Publisher for visualization
        ::ros::NodeHandle nh_private("~");
        frontier_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>(
                "submap_frontiers", 0 );


        // Set random seed
        srand (time(NULL));

        // initialize neighbor offsets
        c_neighbor_voxels_[0] = Point(c_voxel_size_, 0, 0);
        c_neighbor_voxels_[1] = Point(-c_voxel_size_, 0, 0);
        c_neighbor_voxels_[2] = Point(0, c_voxel_size_, 0);
        c_neighbor_voxels_[3] = Point(0, -c_voxel_size_, 0);
        c_neighbor_voxels_[4] = Point(0, 0, c_voxel_size_);
        c_neighbor_voxels_[5] = Point(0, 0, -c_voxel_size_);
        c_neighbor_voxels_[0] = Point(c_voxel_size_, 0, 0);
        c_neighbor_voxels_[1] = Point(c_voxel_size_, c_voxel_size_, 0);
        c_neighbor_voxels_[2] = Point(c_voxel_size_, -c_voxel_size_, 0);
        c_neighbor_voxels_[3] = Point(c_voxel_size_, 0, c_voxel_size_);
        c_neighbor_voxels_[4] = Point(c_voxel_size_, c_voxel_size_, c_voxel_size_);
        c_neighbor_voxels_[5] = Point(c_voxel_size_, -c_voxel_size_, c_voxel_size_);
        c_neighbor_voxels_[6] = Point(c_voxel_size_, 0, -c_voxel_size_);
        c_neighbor_voxels_[7] = Point(c_voxel_size_, c_voxel_size_, -c_voxel_size_);
        c_neighbor_voxels_[8] = Point(c_voxel_size_, -c_voxel_size_, -c_voxel_size_);
        c_neighbor_voxels_[9] = Point(0, c_voxel_size_, 0);
        c_neighbor_voxels_[10] = Point(0, -c_voxel_size_, 0);
        c_neighbor_voxels_[11] = Point(0, 0, c_voxel_size_);
        c_neighbor_voxels_[12] = Point(0, c_voxel_size_, c_voxel_size_);
        c_neighbor_voxels_[13] = Point(0, -c_voxel_size_, c_voxel_size_);
        c_neighbor_voxels_[14] = Point(0, 0, -c_voxel_size_);
        c_neighbor_voxels_[15] = Point(0, c_voxel_size_, -c_voxel_size_);
        c_neighbor_voxels_[16] = Point(0, -c_voxel_size_, -c_voxel_size_);
        c_neighbor_voxels_[17] = Point(-c_voxel_size_, 0, 0);
        c_neighbor_voxels_[18] = Point(-c_voxel_size_, c_voxel_size_, 0);
        c_neighbor_voxels_[19] = Point(-c_voxel_size_, -c_voxel_size_, 0);
        c_neighbor_voxels_[20] = Point(-c_voxel_size_, 0, c_voxel_size_);
        c_neighbor_voxels_[21] = Point(-c_voxel_size_, c_voxel_size_, c_voxel_size_);
        c_neighbor_voxels_[22] = Point(-c_voxel_size_, -c_voxel_size_, c_voxel_size_);
        c_neighbor_voxels_[23] = Point(-c_voxel_size_, 0, -c_voxel_size_);
        c_neighbor_voxels_[24] = Point(-c_voxel_size_, c_voxel_size_, -c_voxel_size_);
        c_neighbor_voxels_[25] = Point(-c_voxel_size_, -c_voxel_size_, -c_voxel_size_);

    }

    void SubmapFrontierEvaluator::getNeighbouringSubmaps(const SubmapID &submap_id,
                                                         const std::list<voxgraph::RegistrationConstraint> &registration_contraints,
                                                         std::vector<SubmapID> *neighbouring_submap_ids) {

        // Build up a vector made of the submap id, followed by the neighbouring submaps
        neighbouring_submap_ids->emplace_back(submap_id);

        // Check among the registration constraints if there are neighbouring submaps
        for (auto id : registration_contraints) {
            if ((id.getConfig().first_submap_id == submap_id) &&
                (find(neighbouring_submap_ids->begin(), neighbouring_submap_ids->end(),
                      id.getConfig().second_submap_id) ==
                 neighbouring_submap_ids->end())) {
                neighbouring_submap_ids->emplace_back(id.getConfig().second_submap_id);
            }
        }
        return;
    }

    void SubmapFrontierEvaluator::selectFirstFreeLayerPoint(const Layer<EsdfVoxel>& layer, Point* result){
        BlockIndexList block_list;
        layer.getAllAllocatedBlocks(&block_list);

        for (const auto& block_idx : block_list){
            typename Block<EsdfVoxel>::ConstPtr block_ptr =
                    layer.getBlockPtrByIndex(block_idx);

            for (voxblox::IndexElement voxel_idx = 0;
            voxel_idx < static_cast<voxblox::IndexElement>(block_ptr->num_voxels()); ++voxel_idx) {
                EsdfVoxel voxel = block_ptr->getVoxelByLinearIndex(voxel_idx);
                if (voxel.distance > 0){
                    *result = block_ptr->computeCoordinatesFromLinearIndex(voxel_idx);
                    return;
                }
            }
        }
    }


    void SubmapFrontierEvaluator::wavefrontFrontierDetector(const Point& pose,
                                                            Layer<EsdfVoxel>& input_layer,
                                                            const Layer<EsdfVoxel>& frontier_check_layer,
                                                            std::vector<Frontier>* frontiers_vector){
        frontiers_vector->clear();

        if (input_layer.getVoxelPtrByCoordinates(pose) == nullptr) {
            ROS_WARN("The submap doesn't contain the given point");
            return;
        }

        // Get voxel center
        voxblox::BlockIndex block_id = input_layer.computeBlockIndexFromCoordinates(pose);
        Point center_point = voxblox::getOriginPointFromGridIndex(block_id, input_layer.block_size());
        voxblox::VoxelIndex voxel_id = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
                (pose - center_point), 1.0 / c_voxel_size_);
        center_point += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_);

        // Empty the frontier vector
        frontiers_vector->clear();

        // Queues and containers
        Point p;
        Point q;
        Point w;
        Point v;
        std::vector<Point> queue_m;
        std::vector<Point> queue_f;
        std::vector<Point> new_frontier;

        // Initialize queue_m with the robot pose
        queue_m.emplace_back(center_point);

        // Mark pose as Map-Open-List
        setMapOpenList(pose, input_layer);

        while (!(queue_m.empty())){
            // Pop the first element from queue_m
            p = queue_m[0];
            queue_m.erase(queue_m.begin());

            if (isMapCloseList(p, input_layer)) continue;

            if (isFrontier(p, frontier_check_layer)){
                queue_f.clear();
                new_frontier.clear();

                queue_f.emplace_back(p);
                // Mark pose as Frontier-Open-List
                setFrontierOpenList(p, input_layer);

                while (!(queue_f.empty())){
                    // Pop the first element from queue_f
                    q = queue_f[0];
                    queue_f.erase(queue_f.begin());
                    if (isMapCloseList(q, input_layer) || isFrontierCloseList(q, input_layer)) continue;


                    if (isFrontier(q, frontier_check_layer)){
                        new_frontier.emplace_back(q);

                        for (int i = 0; i < 26; i++){
                            w = q + c_neighbor_voxels_[i];
                            if (input_layer.getVoxelPtrByCoordinates(w) == nullptr) continue;

                            if (!isFrontierOpenList(w, input_layer) || !isFrontierCloseList(w, input_layer) || !isMapCloseList(w, input_layer)){
                                if (bounding_volume_->contains(Eigen::Vector3d((double)w.x(), (double)w.y(), (double) w.z()))) {
                                    queue_f.emplace_back(w);
                                    setFrontierOpenList(w, input_layer);
                                }
                            }
                        }
                    }
                    // Mark pose as Frontier-Close-List
                    setFrontierCloseList(q, input_layer);
                }
                // Add the frontier if it has a considerable number of points
                if (new_frontier.size() > n_frontier_points_threshold_) {
                    frontiers_vector->emplace_back(Frontier(new_frontier));
                }

                for (const auto& point : new_frontier){
                    setMapCloseList(point, input_layer);
                }
            }

            for (int i = 0; i < 26; i++) {
                v = p + c_neighbor_voxels_[i];
                if (input_layer.getVoxelPtrByCoordinates(v) == nullptr) {
                    continue;
                }

                if (!isMapOpenList(v, input_layer) && !isMapCloseList(v, input_layer)) {
                    // Check if v has neighbouring open space
                    for (int j = 0; j < 6; j++) {
                        if (input_layer.getVoxelPtrByCoordinates(v + c_neighbor_voxels_[j]) == nullptr) continue;
                        if (input_layer.getVoxelPtrByCoordinates(v + c_neighbor_voxels_[j])->distance > 0) {
                            if (bounding_volume_->contains(Eigen::Vector3d((double)v.x(), (double)v.y(), (double) v.z()))) {
                                queue_m.emplace_back(v);
                                setMapOpenList(v, input_layer);
                            }
                            break;
                        }
                    }
                }
            }
            setMapCloseList(p, input_layer);
        }
    }

    void SubmapFrontierEvaluator::computeSubmapFrontiers(const Point pose,
                                                         const SubmapID &submap_id,
                                                         const Transformation &T_M_S,
                                                         Layer<EsdfVoxel> &active_submap,
                                                         const Layer<EsdfVoxel> &local_area) {
        Point starting_point = pose;
        if ((active_submap.getVoxelPtrByCoordinates(pose) == nullptr) || (active_submap.getVoxelPtrByCoordinates(pose)->distance < 0)) {
            selectFirstFreeLayerPoint(active_submap, &starting_point);
        }
        std::vector<Frontier> frontiers_vector;

        // Get frontiers through Wavefront Frontier Detector algorithm
        wavefrontFrontierDetector(starting_point, active_submap, local_area, &frontiers_vector);

        // Add the found frontiers to the submap frontier map
        submap_frontiers_map_.insert(std::pair<SubmapID, SubmapFrontiers>(submap_id,
                                                                          SubmapFrontiers(frontiers_vector, T_M_S)));
    }

    bool SubmapFrontierEvaluator::isFrontier(const Point& voxel,
                                             const Layer<EsdfVoxel>& layer){
        const EsdfVoxel* voxel_ptr = layer.getVoxelPtrByCoordinates(voxel);

        if (voxel_ptr == nullptr){
            return false;
        }

        if (voxel_ptr->observed) return false;

        // Check all neighboring voxels
        if (!p_accurate_frontiers_) {
            for (int i = 0; i < 6; ++i) {
                const EsdfVoxel* neighbouring_voxel_ptr = layer.getVoxelPtrByCoordinates(voxel + c_neighbor_voxels_[i]);
                if ((neighbouring_voxel_ptr != nullptr) && (neighbouring_voxel_ptr->distance > 0)) {
                    return true;
                }
            }
        }
        else {
            for (int i = 0; i < 26; ++i) {
                const EsdfVoxel* neighbouring_voxel_ptr = layer.getVoxelPtrByCoordinates(voxel + c_neighbor_voxels_[i]);
                if ((neighbouring_voxel_ptr != nullptr) && (neighbouring_voxel_ptr->distance > 0))
                    return true;
            }
        }
        return false;
    }

    bool SubmapFrontierEvaluator::isFrontier(const Point &voxel,
                                             const std::vector<SubmapID>& neighbouring_submap_ids){
        Transformation T_S_M;

        for (const auto& neigh_submap_id : neighbouring_submap_ids) {
            // Get the voxel from mission frame to the submap frame
            T_S_M = submap_frontiers_map_[neigh_submap_id].getInversePose();
            const EsdfVoxel* voxel_ptr = map_->getSubmapCollection().getSubmap(neigh_submap_id).getEsdfMap().
                    getEsdfLayer().getVoxelPtrByCoordinates(T_S_M * voxel);

            // Check if the frontier is still unobserved in all the possible submaps
            if ((voxel_ptr != nullptr) && (voxel_ptr->observed)) {
                return false;
            }
            else {
                if (neigh_submap_id == neighbouring_submap_ids.back()) {
                    return true;
                }
            }

        }
    }

    void SubmapFrontierEvaluator::setSubmapFrontiersPose(const SubmapID& submap_id, const Transformation& T_M_S){
        std::map<SubmapID, SubmapFrontiers>::iterator  it = submap_frontiers_map_.find(submap_id);

        if(it != submap_frontiers_map_.end()) submap_frontiers_map_[submap_id].setPose(T_M_S);
    }

    const Transformation SubmapFrontierEvaluator::getSubmapFrontiersPose(const SubmapID& submap_id){
        std::map<SubmapID, SubmapFrontiers>::iterator  it = submap_frontiers_map_.find(submap_id);
        if(it != submap_frontiers_map_.end() ) {
            return submap_frontiers_map_[submap_id].getPose();
        }
        else{
            ROS_WARN("The pose can't be retrieved since it has not been registered yet");
        }
    }

    void SubmapFrontierEvaluator::transformSubmapFrontiers(const SubmapID& submap_id, const Transformation& T_new_old){
        std::map<SubmapID, SubmapFrontiers>::iterator  it = submap_frontiers_map_.find(submap_id);
        if(it != submap_frontiers_map_.end() ) {
            submap_frontiers_map_[submap_id].transformSubmapFrontiers(T_new_old);
        }
        else{
            ROS_WARN("The submap can not be transformed since it has not been registered yet");
        }
    }
    void SubmapFrontierEvaluator::updateSubmapFrontiers(
            const std::list<voxgraph::RegistrationConstraint>& registration_contraints){
        // Update the poses
        updateFrontiersPoses();

        // Update the points
        updateFrontiersPoints(registration_contraints);
    }

    void SubmapFrontierEvaluator::updateFrontiersPoses(){
        SubmapID submap_id;
        Transformation T_M_S2; // current submap pose
        Transformation T_S2_S1; // transformation from the old submap pose to the new one

        for (auto& submap_frontiers_pair : submap_frontiers_map_){
            submap_id = submap_frontiers_pair.first;
            T_M_S2 = map_->getSubmapCollection().getSubmap(submap_id).getPose();
            T_S2_S1 = T_M_S2.inverse() * getSubmapFrontiersPose(submap_id);

            transformSubmapFrontiers(submap_id, T_S2_S1);
            setSubmapFrontiersPose(submap_id, T_M_S2);
        }
    }

    void SubmapFrontierEvaluator::updateFrontiersPoints(
            const std::list<voxgraph::RegistrationConstraint>& registration_contraints){
        SubmapID submap_id;
        std::vector<SubmapID> neighbouring_submap_ids;

        for (auto& submap_frontier_pair : submap_frontiers_map_){
            // Get neighbouring submap ids
            neighbouring_submap_ids.clear();
            submap_id = submap_frontier_pair.first;
            getNeighbouringSubmaps(submap_id, registration_contraints, &neighbouring_submap_ids);

            // For every saved frontier point, check if it is still a frontier point considering the neighbouring submaps
            SubmapFrontiers& submap_frontiers = submap_frontier_pair.second;
            for (int frontier_idx = (submap_frontiers.getFrontiers().size()-1); frontier_idx >= 0; frontier_idx--){
                Frontier& frontier = submap_frontiers.getFrontier(frontier_idx);

                // Set the centroid to the sum of the points coordinates
                frontier.setCentroid(frontier.getCentroid() * frontier.getNumberOfPoints());

                for (int frontier_point_idx = (frontier.getPoints().size() - 1); frontier_point_idx >= 0; frontier_point_idx--){
                    if (!isFrontier(frontier.getPoint(frontier_point_idx), neighbouring_submap_ids)){
                        frontier.setCentroid(frontier.getCentroid() - frontier.getPoint(frontier_point_idx));
                        frontier.removePoint(frontier_point_idx);
                    }

                    // Remove the entire frontier if the number of its points gets less than the minimum frontier threshold
                    if (frontier.getNumberOfPoints() < n_frontier_points_threshold_){
                        submap_frontiers.removeFrontier(frontier_idx);
                        break;
                    }
                }
                // Get the new centroid
                frontier.setCentroid(frontier.getCentroid() / frontier.getNumberOfPoints());
            }
        }
    }

    void SubmapFrontierEvaluator::computeFrontiersOnGlobalMap(const Point& pose){

        std::map<SubmapID, Layer<EsdfVoxel>> transformed_esdf_submaps_map;
        std::vector<Frontier> frontiers_vector;
        Layer<EsdfVoxel> global_map_esdf(map_->getSubmapCollection().getConfig().esdf_voxel_size,
                                         map_->getSubmapCollection().getConfig().esdf_voxels_per_side);

        // Compute the global map Esdf
        for (int submap_id = 0; submap_id < map_->getSubmapCollection().getActiveSubmapID(); submap_id++){

            Layer<EsdfVoxel> transformed_submap_esdf(map_->getSubmapCollection().getConfig().esdf_voxel_size,
                                                     map_->getSubmapCollection().getConfig().esdf_voxels_per_side);

            // Transform the submap esdf in mission frame
            naiveTransformLayer(map_->getSubmapCollection().getSubmap(submap_id).getEsdfMap().getEsdfLayer(),
                                map_->getSubmapCollection().getSubmap(submap_id).getPose(),
                                &transformed_submap_esdf);
            // Insert the submap into the transformed submaps map
            transformed_esdf_submaps_map.insert(std::pair<SubmapID, Layer<EsdfVoxel>>(submap_id, transformed_submap_esdf));

            // Merge the submap into the global map esdf layer
            mergeLayerAInLayerB(transformed_submap_esdf, &global_map_esdf);
        }

        // Compute the frontiers for each submap
        for (auto& submap_esdf_pair : transformed_esdf_submaps_map){
            // search the frontiers starting the algorithm from the origin of the submap frame

            wavefrontFrontierDetector(map_->getSubmapCollection().getSubmap(submap_esdf_pair.first).getPose().getPosition(),
                                      submap_esdf_pair.second,
                                      global_map_esdf,
                                      &frontiers_vector);

            // Add the frontiers to the submao frontiers map
            for (const auto& frontier : frontiers_vector){
                submap_frontiers_map_[submap_esdf_pair.first].addFrontier(frontier);
            }
        }
    }

    SubmapFrontiers& SubmapFrontierEvaluator::getSubmapFrontiers(SubmapID submap_id) {
        std::map<SubmapID ,SubmapFrontiers>::iterator it;

        it = submap_frontiers_map_.find(submap_id);
        if (it != submap_frontiers_map_.end())
            return submap_frontiers_map_[submap_id];
    }

    void SubmapFrontierEvaluator::publishSubmapFrontiers(const SubmapID& id){
        static int marker_id = 0;
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;

        // Get the submap frontier and its color
        SubmapFrontiers& submap_frontiers = getSubmapFrontiers(id);
        const voxblox::Color& color = submap_frontiers.getColor();

        marker.header.frame_id = "world";
        marker.header.stamp = ::ros::Time();
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = c_voxel_size_;
        marker.scale.y = c_voxel_size_;
        marker.scale.z = c_voxel_size_;
        marker.lifetime.sec = 9;
        marker.color.r = (float)color.r / 255;
        marker.color.g = (float)color.g / 255;
        marker.color.b = (float)color.b / 255;
        marker.color.a = (float)color.a / 255;

        for (auto &frontier : submap_frontiers.getFrontiers()){
            for (const auto &point : frontier.getPoints()) {
                marker.id = marker_id;
                marker.pose.position.x = point.x();
                marker.pose.position.y = point.y();
                marker.pose.position.z = point.z();
                marker_array.markers.push_back(marker);
                marker_id++;
                if (marker_id == INT_MAX) marker_id = 0;
            }
        }
        frontier_pub_.publish(marker_array);
    }

    void SubmapFrontierEvaluator::publishAllSubmapFrontiers(){
        if (frontier_pub_.getNumSubscribers() > 0) {
            for (const auto &submap_frontier_pair : submap_frontiers_map_) {
                publishSubmapFrontiers(submap_frontier_pair.first);
            }
        }
    }

    void SubmapFrontierEvaluator::setFrontierOpenList(const Point& point, Layer<EsdfVoxel>& layer){
        EsdfVoxel* esdf_voxel_ptr = layer.getVoxelPtrByCoordinates(point);
        if (esdf_voxel_ptr == nullptr){
            ROS_WARN("Voxel not allocated");
            return;
        }
        esdf_voxel_ptr->frontier_status[VoxelStatus::FrontierOpenList] = true;
    }

    void SubmapFrontierEvaluator::setFrontierCloseList(const Point& point, Layer<EsdfVoxel>& layer){
        EsdfVoxel* esdf_voxel_ptr = layer.getVoxelPtrByCoordinates(point);
        if (esdf_voxel_ptr == nullptr){
            ROS_WARN("Voxel not allocated");
            return;
        }
        esdf_voxel_ptr->frontier_status[VoxelStatus::FrontierCloseList] = true;
    }

    void SubmapFrontierEvaluator::setMapOpenList(const Point& point, Layer<EsdfVoxel>& layer){
        EsdfVoxel* esdf_voxel_ptr = layer.getVoxelPtrByCoordinates(point);
        if (esdf_voxel_ptr == nullptr){
            ROS_WARN("Voxel not allocated");
            return;
        }
        esdf_voxel_ptr->frontier_status[VoxelStatus::MapOpenList] = true;
    }

    void SubmapFrontierEvaluator::setMapCloseList(const Point& point, Layer<EsdfVoxel>& layer){
        EsdfVoxel* esdf_voxel_ptr = layer.getVoxelPtrByCoordinates(point);
        if (esdf_voxel_ptr == nullptr){
            ROS_WARN("Voxel not allocated");
            return;
        }
        esdf_voxel_ptr->frontier_status[VoxelStatus::MapCloseList] = true;
    }

    bool SubmapFrontierEvaluator::isMapOpenList(const Point& point, const Layer<EsdfVoxel>& layer){
        const EsdfVoxel* esdf_voxel_ptr = layer.getVoxelPtrByCoordinates(point);
        if (esdf_voxel_ptr == nullptr){
            ROS_WARN("Voxel not allocated");
            return false;
        }
        return esdf_voxel_ptr->frontier_status[VoxelStatus::MapOpenList];
    }

    bool SubmapFrontierEvaluator::isMapCloseList(const Point& point, const Layer<EsdfVoxel>& layer){
        const EsdfVoxel* esdf_voxel_ptr = layer.getVoxelPtrByCoordinates(point);
        if (esdf_voxel_ptr == nullptr){
            ROS_WARN("Voxel not allocated");
            return false;
        }
        return esdf_voxel_ptr->frontier_status[VoxelStatus::MapCloseList];
    }

    bool SubmapFrontierEvaluator::isFrontierOpenList(const Point& point, const Layer<EsdfVoxel>& layer){
        const EsdfVoxel* esdf_voxel_ptr = layer.getVoxelPtrByCoordinates(point);
        if (esdf_voxel_ptr == nullptr){
            ROS_WARN("Voxel not allocated");
            return false;
        }
        return esdf_voxel_ptr->frontier_status[VoxelStatus::FrontierOpenList];
    }

    bool SubmapFrontierEvaluator::isFrontierCloseList(const Point& point, const Layer<EsdfVoxel>& layer){
        const EsdfVoxel* esdf_voxel_ptr = layer.getVoxelPtrByCoordinates(point);
        if (esdf_voxel_ptr == nullptr){
            ROS_WARN("Voxel not allocated");
            return false;
        }
        return esdf_voxel_ptr->frontier_status[VoxelStatus::FrontierCloseList];
    }
}
