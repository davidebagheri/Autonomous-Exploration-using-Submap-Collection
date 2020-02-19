#include "active_3d_planning_app_submap_exploration/global_point_selector/global_frontier_selector/biggest_frontier_selector.h"

namespace active_3d_planning{
    ModuleFactoryRegistry::Registration<BiggestFrontierSelector> BiggestFrontierSelector::registration("BiggestFrontierSelector");

    BiggestFrontierSelector::BiggestFrontierSelector(PlannerI &planner) : GlobalPointSelector(planner){}

    void BiggestFrontierSelector::setupFromParamMap(Module::ParamMap *param_map){
        GlobalPointSelector::setupFromParamMap(param_map);

        std::string ns = (*param_map)["param_namespace"];
        std::string submap_frontier_evaluator_args;
        setParam<std::string>(param_map, "submap_frontier_evaluator_args",
                              &submap_frontier_evaluator_args, ns + "/submap_frontier_evaluator");

        submap_frontier_evaluator_ = planner_.getFactory().createModule<SubmapFrontierEvaluator>(
                submap_frontier_evaluator_args, planner_, verbose_modules_);
        submap_frontier_evaluator_->setupFromParamMap(param_map);
    }

    bool BiggestFrontierSelector::getGlobalPoint(geometry_msgs::PoseStamped* goal){
        SubmapID submap_id;
        int frontier_id;

        // There are no more reachable frontiers
        if (!findBiggerFrontier(&submap_id, &frontier_id)) return false;

        selectPointForFrontierExploration(submap_id, frontier_id, goal);

        return true;
    }

    bool BiggestFrontierSelector::findBiggerFrontier(SubmapID* submap_id, int* frontier_id){
        int n_best_frontier_points = 0;

        // Find the frontier with the maximum number of points
        for (auto& submap_frontier_pair : submap_frontier_evaluator_->getSubmapFrontiersMap()){
            for(int frontier_idx = 0; frontier_idx < submap_frontier_pair.second.getFrontiers().size(); frontier_idx++){
                int n_current_frontier_points = submap_frontier_pair.second.getFrontier(frontier_idx).getNumberOfPoints();

                if (n_current_frontier_points > n_best_frontier_points){
                    n_best_frontier_points = n_current_frontier_points;
                    *frontier_id = frontier_idx;
                    *submap_id = submap_frontier_pair.first;
                }
            }
        }
        // This means there are no more frontiers
        if (n_best_frontier_points == 0) return false;
        ROS_INFO("best frontiers: %d points in submap %d", n_best_frontier_points, *submap_id);
        return true;
    }

    void BiggestFrontierSelector::selectPointForFrontierExploration(const SubmapID& submap_id,
                                                                    const int& frontier_id,
                                                                    geometry_msgs::PoseStamped* goal){
        // Get the centroid of the frontier
        Point centroid = submap_frontier_evaluator_->getSubmapFrontiers(submap_id).getFrontier(frontier_id).getCentroid();
        Point sample_near_centroid;

        /*if (isFree(centroid)){
            goal->pose.position.x = centroid.x();
            goal->pose.position.y = centroid.y();
            goal->pose.position.z = centroid.z();
            return;
        } else{
            // The search distance is the half of the side of the frontier if it is a square, limited to 2 meters
            float search_distance = std::min<float>(std::sqrt(
                    submap_frontier_evaluator_->getSubmapFrontiers(submap_id).getFrontier(frontier_id).getNumberOfPoints()
                    * submap_frontier_evaluator_->getVoxelSize() / 2), 2);
            for(int i = 0; i < 100; i++){
                getRandomPointNearPosition(centroid, &sample_near_centroid, search_distance);
                if (isFree(sample_near_centroid)){
                    goal->pose.position.x = sample_near_centroid.x();
                    goal->pose.position.y = sample_near_centroid.y();
                    goal->pose.position.z = sample_near_centroid.z();
                    return;
                }
            }
        }*/

        // Get the frame origin of the submap in which there is the selected frontier
        goal->pose.position.x = (double)submap_frontier_evaluator_->getSubmapFrontiersPose(submap_id).getPosition().x();
        goal->pose.position.y = (double)submap_frontier_evaluator_->getSubmapFrontiersPose(submap_id).getPosition().y();
        goal->pose.position.z = (double)submap_frontier_evaluator_->getSubmapFrontiersPose(submap_id).getPosition().z();
    }

    void BiggestFrontierSelector::getRandomPointNearPosition(const Point& point, Point* random_point, float distance){
        *random_point = point + getDirection(((float)(rand() / RAND_MAX)) * M_PI * 2,
                                             (float)(rand() / RAND_MAX)* M_PI * 2) * distance;
    }

    Eigen::Vector3f BiggestFrontierSelector::getDirection(const float &theta, const float &phi) {
        // return the (x,y,z) versor from polar and azimuthal angle
        return Eigen::Vector3f(std::cos(phi) * std::cos(theta), std::cos(phi) * std::sin(theta), std::sin(phi));
    }

    void BiggestFrontierSelector::update(SubmapID finished_submap_ID){
        voxgraph::VoxgraphSubmap::Ptr finished_submap = map_->getSubmapCollection().getSubmapPtr(finished_submap_ID);
        getFrontierEvaluator().computeSubmapFrontiers(
                map_->getRobotPosition(),
                finished_submap->getID(),
                finished_submap->getPose(),
                *map_->getPlannerMapManager().getActiveSubmapPtr()->getEsdfMapPtr()->getEsdfLayerPtr(),
                map_->getPlannerMapManager().getCurrentNeighbours().getEsdfMap().getEsdfLayer());

        // Update all the frontiers
        getFrontierEvaluator().updateSubmapFrontiers(map_->getRegistrationConstraint());

        // Visualize
        getFrontierEvaluator().publishAllSubmapFrontiers();
    }
}