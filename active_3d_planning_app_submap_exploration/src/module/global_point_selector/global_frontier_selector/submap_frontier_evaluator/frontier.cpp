#include "active_3d_planning_app_submap_exploration/module/global_point_selector/global_frontier_selector/submap_frontier_evaluator/frontier.h"
#include <iostream>

namespace active_3d_planning{
    Frontier::Frontier(){
        n_points_ = 0;
    }

    Frontier::Frontier(const std::vector<Point>& points)
            : frontier_points_(points){
        n_points_ = frontier_points_.size();
        computeCentroid();
    }

    const std::vector<Point>& Frontier::getPoints()const{
        return frontier_points_;
    }

    const std::vector<Point>& Frontier::getQuarantinePoints()const{
        return quarantine_points_;
    }

    const Point& Frontier::getPoint(const int& point_idx) const{
        return frontier_points_[point_idx];
    }

    const Point& Frontier::getQuarantinePoint(const int& point_idx) const{
        return quarantine_points_[point_idx];
    }

    void Frontier::removePoint(const int& point_idx){
        quarantine_points_.push_back(frontier_points_[point_idx]);
        frontier_points_.erase(frontier_points_.begin() + point_idx);
        n_points_--;
    }

    void Frontier::removeQuarantinePoint(const int &point_idx){
        quarantine_points_.erase(quarantine_points_.begin() + point_idx);
    }

    void Frontier::addQuarantinePoint(const Point &new_point) {
        quarantine_points_.push_back(new_point);
    }

    void Frontier::addPoint(const Point &new_point){
        frontier_points_.push_back(new_point);
        n_points_++;
    }

    void Frontier::transformFrontier(const Transformation& T_S2_S1){
        for (auto& voxel_center : frontier_points_){
            voxel_center = T_S2_S1 * voxel_center;
        }
        for (auto& quarantine_voxel_center : quarantine_points_){
            quarantine_voxel_center = T_S2_S1 * quarantine_voxel_center;
        }

        centroid_ = T_S2_S1 * centroid_;
    }

    const int& Frontier::getNumberOfPoints(){
        return n_points_;
    }


    const Point& Frontier::getCentroid() const{
        return centroid_;
    }

    void Frontier::setCentroid(const Point& new_centroid){
        centroid_ = new_centroid;
    }


    void Frontier::computeCentroid(){
        float x = 0;
        float y = 0;
        float z = 0;
        float n_points = frontier_points_.size();

        for (const auto& frontier_point : frontier_points_){
            x += frontier_point.x();
            y += frontier_point.y();
            z += frontier_point.z();
        }
        centroid_ = Point(x,y,z) / n_points;
    }

}
