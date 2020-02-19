#include "active_3d_planning_app_submap_exploration/global_point_selector/global_frontier_selector/submap_frontier_evaluator/frontier.h"

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

    const Point& Frontier::getPoint(const int& point_idx) const{
        return frontier_points_[point_idx];
    }

    void Frontier::removePoint(const int& point_idx){
        frontier_points_.erase(frontier_points_.begin() + point_idx);
        n_points_--;
    }

    void Frontier::transformFrontier(const Transformation& T_M_S){
        for (auto& voxel_center : frontier_points_){
            voxel_center = T_M_S * voxel_center;
        }
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
