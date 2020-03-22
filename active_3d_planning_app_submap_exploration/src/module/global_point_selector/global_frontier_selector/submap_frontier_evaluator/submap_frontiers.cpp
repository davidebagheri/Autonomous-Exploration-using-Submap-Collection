#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_frontier_selector/submap_frontier_evaluator/submap_frontiers.h"

namespace active_3d_planning{

    SubmapFrontiers::SubmapFrontiers(){
        // Possible colors
        std::vector<voxblox::Color> color_vector = {voxblox::Color::Red(), voxblox::Color::Green(), voxblox::Color::Blue(), voxblox::Color::Orange(),
                                                    voxblox::Color::Gray(), voxblox::Color::Pink(), voxblox::Color::Purple(), voxblox::Color::Teal()};


        int color_index = rand() % color_vector.size();
        color_ = color_vector[color_index];
    }

    SubmapFrontiers::SubmapFrontiers(const std::vector<Frontier>& frontiers, const Transformation& T_M_S){
        T_M_S_ = T_M_S;
        submap_frontiers_ = frontiers;


        // Select a random color for the frontiers of this submap
        srand (time(NULL));

        // Possible colors
        std::vector<voxblox::Color> color_vector = {voxblox::Color::Red(), voxblox::Color::Green(), voxblox::Color::Blue(), voxblox::Color::Orange(),
                                                    voxblox::Color::Gray(), voxblox::Color::Pink(), voxblox::Color::Purple(), voxblox::Color::Teal()};


        int color_index = rand() % color_vector.size();
        color_ = color_vector[color_index];
    }

    SubmapFrontiers::SubmapFrontiers(const Transformation& T_M_S){
        T_M_S_ = T_M_S;

        // Select a random color for the frontiers of this submap
        srand (time(NULL));

        // Possible colors
        std::vector<voxblox::Color> color_vector = {voxblox::Color::Red(), voxblox::Color::Green(), voxblox::Color::Blue(), voxblox::Color::Orange(),
                                           voxblox::Color::Gray(), voxblox::Color::Pink(), voxblox::Color::Purple(), voxblox::Color::Teal()};

        int color_index = rand() % color_vector.size();
        color_ = color_vector[color_index];
    }

    void SubmapFrontiers::addFrontier(const Frontier& frontier){
        submap_frontiers_.emplace_back(frontier);
    }

    void SubmapFrontiers::removeFrontier(const int& frontier_idx){
        submap_frontiers_.erase(submap_frontiers_.begin() + frontier_idx);
    }

    void SubmapFrontiers::setPose(const Transformation& T_M_S){
        T_M_S_ = T_M_S;
        T_S_M_ = T_M_S.inverse();
    }

    const Transformation& SubmapFrontiers::getPose(){
        return T_M_S_;
    }

    const Transformation& SubmapFrontiers::getInversePose(){
        return T_S_M_;
    }

    std::vector<Frontier>& SubmapFrontiers::getFrontiers(){
        return submap_frontiers_;
    }

    Frontier& SubmapFrontiers::getFrontier(const int& frontier_indx){
        return submap_frontiers_[frontier_indx];
    }

    void SubmapFrontiers::transformSubmapFrontiers(const Transformation& T_new_old){
        for (auto& frontier : submap_frontiers_){
            frontier.transformFrontier(T_new_old);
        }
    }

    const voxblox::Color& SubmapFrontiers::getColor(){
        return color_;
    }

    const Point& SubmapFrontiers::getOrigin(){
        return T_M_S_.getPosition();
    }

    bool SubmapFrontiers::empty(){
        return submap_frontiers_.empty();
    }

    bool SubmapFrontiers::exists(int frontier_idx){
        return frontier_idx < (submap_frontiers_.size() - 1);
    }
}