#ifndef ACTIVE_3D_PLANNING_VOXGRAPH_FRONTIER_H
#define ACTIVE_3D_PLANNING_VOXGRAPH_FRONTIER_H

#include "voxblox/core/layer.h"

namespace active_3d_planning {
    using Point = voxblox::Point;
    using Transformation = voxblox::Transformation;
    class Frontier {
    public:
        Frontier();

        Frontier(const std::vector <Point> &points);

        const std::vector <Point> &getPoints() const;

        const std::vector<Point>& getQuarantinePoints()const;

        const Point &getPoint(const int &point_idx) const;

        const Point &getQuarantinePoint(const int &point_idx) const;

        const int &getNumberOfPoints();

        const Point &getCentroid() const;

        void computeCentroid();

        void setCentroid(const Point &new_centroid);

        void removePoint(const int &point_idx);

        void removeQuarantinePoint(const int &point_idx);

        void addPoint(const Point &new_point);

        void addQuarantinePoint(const Point &new_point);

        void transformFrontier(const Transformation &T_M_S);

    private:
        int n_points_;

        Point centroid_;

        std::vector <Point> frontier_points_;

        std::vector <Point> quarantine_points_;
    };
}

#endif //ACTIVE_3D_PLANNING_VOXGRAPH_FRONTIER_H
