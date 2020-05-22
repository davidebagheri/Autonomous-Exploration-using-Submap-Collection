#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_frontier_selector/submap_frontier_evaluator/submap_frontier_evaluator.h"

namespace active_3d_planning {
    // Registration
    ModuleFactoryRegistry::Registration<SubmapFrontierEvaluator> SubmapFrontierEvaluator::registration("SubmapFrontierEvaluator");

    SubmapFrontierEvaluator::SubmapFrontierEvaluator(PlannerI &planner) : Module(planner){}

    void SubmapFrontierEvaluator::setupFromParamMap(Module::ParamMap *param_map){
        setParam<float>(param_map, "voxel_size", &c_voxel_size_, 0.20);
        setParam<bool>(param_map, "accurate_frontiers", &p_accurate_frontiers_, false);
        setParam<int>(param_map, "n_frontier_points_threshold", &n_frontier_points_threshold_, 50);

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
        frontier_pub_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("submap_frontiers", 0 );
        quarantine_pub_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("quarantine_frontiers", 0 );

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
        for (auto registration_contraint : registration_contraints) {
            if ((registration_contraint.getConfig().first_submap_id == submap_id) &&
                (find(neighbouring_submap_ids->begin(), neighbouring_submap_ids->end(),
                      registration_contraint.getConfig().second_submap_id) ==
                 neighbouring_submap_ids->end())) {
                neighbouring_submap_ids->emplace_back(registration_contraint.getConfig().second_submap_id);
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
                                                            std::vector<Frontier>* frontiers_vector){
        frontiers_vector->clear();

        if (input_layer.getVoxelPtrByCoordinates(pose) == nullptr) {
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

            //if (isFrontier(p, frontier_check_layer)){
            if (isFrontier(p, input_layer)){
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


                    //if (isFrontier(q, frontier_check_layer)){
                    if (isFrontier(q, input_layer)){
                        new_frontier.emplace_back(q);

                        for (int i = 0; i < 26; i++){
                            w = q + c_neighbor_voxels_[i];
                            if (input_layer.getVoxelPtrByCoordinates(w) == nullptr) continue;

                            if (!isFrontierOpenList(w, input_layer) || !isFrontierCloseList(w, input_layer) || !isMapCloseList(w, input_layer)){
                                if (bounding_volume_->contains(Eigen::Vector3d(w.x(), w.y(),  w.z())) && w.z() < 1.1) {
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
                            if (bounding_volume_->contains(Eigen::Vector3d((double)v.x(), (double)v.y(), (double) v.z()))&& v.z() < 1.1) {
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

    void SubmapFrontierEvaluator::computeSubmapFrontiers(const SubmapID &submap_id){
        std::vector<Frontier> frontiers_vector;
        const cblox::TsdfEsdfSubmap* active_submap = map_->getPlannerMapManager().getActiveSubmapPtr().get();

        // Get the submap esdf with the id passed as argument
        Layer<EsdfVoxel> submap_layer(active_submap->getEsdfMap().voxel_size(),
                active_submap->getEsdfMap().voxels_per_side());
        getSubmapBlocksFromLayer(submap_id, *active_submap, &submap_layer);

        // Select an other point if the pose is not in the esdf layer
        Eigen::Vector3d starting_point_d = planner_.getCurrentPosition();
        Point starting_point = Point(starting_point_d.x(), starting_point_d.y(), starting_point_d.z());
        if ((submap_layer.getVoxelPtrByCoordinates(starting_point) == nullptr) ||
            (submap_layer.getVoxelPtrByCoordinates(starting_point)->distance < 0)) {
            selectFirstFreeLayerPoint(submap_layer, &starting_point);
        }

        // Get frontiers through Wavefront Frontier Detector algorithm
        wavefrontFrontierDetector(starting_point, submap_layer, &frontiers_vector);

        // Add the found frontiers to the submap frontier map
        std::map<SubmapID, SubmapFrontiers>::iterator  it = submap_frontiers_map_.find(submap_id);

        if (it == submap_frontiers_map_.end()) {
            submap_frontiers_map_.insert(std::pair<SubmapID, SubmapFrontiers>(submap_id,
                                                                              SubmapFrontiers(frontiers_vector,
                                                                                      map_->getSubmapCollection().
                                                                                      getSubmapPtr(submap_id)->getPose())));
        } else {
            for (const auto& frontier : frontiers_vector){
                submap_frontiers_map_[submap_id].addFrontier(frontier);
            }
        }
    }

    void SubmapFrontierEvaluator::getSubmapBlocksFromLayer(const SubmapID& id,
                                                           const cblox::TsdfEsdfSubmap& submap,
                                                           Layer<EsdfVoxel>* submap_layer){
        // Copy the Esdf layer
        *submap_layer = Layer<EsdfVoxel>(submap.getEsdfMap().getEsdfLayer());

        BlockIndexList submap_layer_blocks;
        submap_layer->getAllAllocatedBlocks(&submap_layer_blocks);
        // Remove all the blocks without the id passed as argument
        for (const auto& block_idx : submap_layer_blocks){
            if (submap.getTsdfMap().getTsdfLayer().getBlockPtrByIndex(block_idx)->getSubmapID() != id){
                submap_layer->removeBlock(block_idx);
            }
        }
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
                    getEsdfLayer().getVoxelPtrByCoordinates(T_S_M * T_M_O_ * voxel);

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

    void SubmapFrontierEvaluator::updateSubmapFrontiers(){
        // Update the poses
        updateFrontiersPoses();

        // Update the points
        updateFrontiersPoints();
    }

    void SubmapFrontierEvaluator::updateFrontiersPoses(){
        SubmapID submap_id;
        Transformation T_M_S2; // current submap pose
        Transformation T_M_S1; // transformation from the old submap pose to the new one

        Transformation T_M_O_new = map_->get_T_M_O();
        Transformation T_M_O_old = T_M_O_;

        for (auto& submap_frontiers_pair : submap_frontiers_map_){
            submap_id = submap_frontiers_pair.first;
            T_M_S2 = map_->getSubmapCollection().getSubmap(submap_id).getPose();
            T_M_S1 = getSubmapFrontiersPose(submap_id);

            transformSubmapFrontiers(submap_id, T_M_O_new.inverse() * T_M_S2 * T_M_S1.inverse() * T_M_O_old);
            setSubmapFrontiersPose(submap_id, T_M_S2);
        }
        T_M_O_ = T_M_O_new;
    }

    void SubmapFrontierEvaluator::updateFrontiersPoints(){
        SubmapID submap_id;
        std::vector<SubmapID> neighbouring_submap_ids;

        for (auto& submap_frontier_pair : submap_frontiers_map_){
            // Get neighbouring submap ids
            neighbouring_submap_ids.clear();
            submap_id = submap_frontier_pair.first;
            getNeighbouringSubmaps(submap_id, map_->getRegistrationConstraint(), &neighbouring_submap_ids);


            SubmapFrontiers& submap_frontiers = submap_frontier_pair.second;
            for (int frontier_idx = (submap_frontiers.getFrontiers().size()-1); frontier_idx >= 0; frontier_idx--){
                Frontier& frontier = submap_frontiers.getFrontier(frontier_idx);

                // For every saved frontier point, check if it is still a frontier point considering the neighbouring submaps
                for (int frontier_point_idx = (frontier.getPoints().size() - 1); frontier_point_idx >= 0; frontier_point_idx--) {
                    if (!isFrontier(frontier.getPoint(frontier_point_idx), neighbouring_submap_ids)) {
                        frontier.addQuarantinePoint(frontier.getPoint(frontier_point_idx));
                        frontier.removePoint(frontier_point_idx);
                    }
                }

                // Reintegrate the quarantine points
                for (int quarantine_point_idx = (frontier.getQuarantinePoints().size() - 1);
                     quarantine_point_idx >= 0;
                     quarantine_point_idx--) {

                    if (isFrontier(frontier.getQuarantinePoint(quarantine_point_idx), neighbouring_submap_ids)) {
                        frontier.addPoint(frontier.getQuarantinePoint(quarantine_point_idx));
                        frontier.removeQuarantinePoint(quarantine_point_idx);
                    }
                }

                // Compute the new centroid
                frontier.computeCentroid();
            }
        }
    }


    SubmapFrontiers& SubmapFrontierEvaluator::getSubmapFrontiers(SubmapID submap_id) {
        std::map<SubmapID ,SubmapFrontiers>::iterator it;

        it = submap_frontiers_map_.find(submap_id);
        if (it != submap_frontiers_map_.end())
            return submap_frontiers_map_[submap_id];
    }

    void SubmapFrontierEvaluator::publishAllSubmapFrontiers(){
        if (frontier_pub_.getNumSubscribers() > 0) {
            pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
            pointcloud.header.frame_id = "world";

            for (const auto &submap_frontier_pair : submap_frontiers_map_) {
                // Get the submap frontier and its color
                SubmapFrontiers& submap_frontiers = getSubmapFrontiers(submap_frontier_pair.first);
                const voxblox::Color& color = submap_frontiers.getColor();

                pcl::PointXYZRGB point;
                point.r = color.r;
                point.g = color.g;
                point.b = color.b;
                point.a = 255;

                for (auto &frontier : submap_frontiers.getFrontiers()){
                    for (const auto &frontier_point : frontier.getPoints()) {
                        point.x = frontier_point.x();
                        point.y = frontier_point.y();
                        point.z = frontier_point.z();
                        pointcloud.push_back(point);
                    }
                }
            }
            frontier_pub_.publish(pointcloud);
        }

        if (quarantine_pub_.getNumSubscribers() > 0) {
            pcl::PointCloud<pcl::PointXYZRGB> quarantine_pointcloud;
            quarantine_pointcloud.header.frame_id = "world";

            for (const auto &submap_frontier_pair : submap_frontiers_map_) {
                // Get the submap frontier and its color
                SubmapFrontiers &submap_frontiers = getSubmapFrontiers(submap_frontier_pair.first);
                const voxblox::Color &color = submap_frontiers.getColor();

                pcl::PointXYZRGB quarantine_point;
                quarantine_point.a = 255;

                for (auto &frontier : submap_frontiers.getFrontiers()) {
                    for (const auto &frontier_point : frontier.getQuarantinePoints()) {
                        quarantine_point.x = frontier_point.x();
                        quarantine_point.y = frontier_point.y();
                        quarantine_point.z = frontier_point.z();
                        quarantine_point.r = 255;
                        quarantine_point.g = 255;
                        quarantine_point.b = 255;
                        quarantine_pointcloud.push_back(quarantine_point);
                    }
                }
            }
            quarantine_pub_.publish(quarantine_pointcloud);
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
