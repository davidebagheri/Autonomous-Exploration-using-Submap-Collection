#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_frontier_selector/global_frontier_selector.h"

namespace active_3d_planning {
    GlobalFrontierSelector::GlobalFrontierSelector(PlannerI &planner) :
            GlobalPointSelector(planner){}

    void GlobalFrontierSelector::setupFromParamMap(Module::ParamMap *param_map){
        std::string ns = (*param_map)["param_namespace"];
        std::string submap_frontier_evaluator_args;
        setParam<std::string>(param_map, "submap_frontier_evaluator_args",
                              &submap_frontier_evaluator_args, ns + "/submap_frontier_evaluator");

        submap_frontier_evaluator_ = planner_.getFactory().createModule<SubmapFrontierEvaluator>(
                submap_frontier_evaluator_args, planner_, verbose_modules_);
        submap_frontier_evaluator_->setupFromParamMap(param_map);
    }

    void GlobalFrontierSelector::update(bool active_submap_finished){
        SubmapID submap_id;

        // Get the ID of the submap on which computing the frontier
        if (active_submap_finished){
            submap_id = map_->getSubmapCollection().getActiveSubmapID() - 1;
        } else {
            submap_id = map_->getSubmapCollection().getActiveSubmapID();
        }

        // Compute the frontiers on the just finished active submap
        submap_frontier_evaluator_->computeSubmapFrontiers(submap_id);

        // Update all the frontiers
        if (active_submap_finished) {
            submap_frontier_evaluator_->updateSubmapFrontiers();
        }

        // Set the active submap pose to the new
        active_submap_pose_ = map_->getSubmapCollection().getActiveSubmapPose();

        // Visualize
        submap_frontier_evaluator_->publishAllSubmapFrontiers();
    }

    bool GlobalFrontierSelector::computeGlobalPoint(){
        //if (submap_frontier_evaluator_->empty()){
          //  submap_frontier_evaluator_->computeFrontiersOnGlobalMap();
        //}

        if (selectFrontierToExplore()) {
            // Store the centroid of the selected frontier
            global_point_ = submap_frontier_evaluator_->getSubmapFrontiers(target_submap_id_).
                    getFrontier(target_frontier_id_).getCentroid();
            return true;
        } else {
            return false;
        }

    }
}
