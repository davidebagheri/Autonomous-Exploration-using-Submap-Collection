#ifndef ACTIVE_3D_PLANNING_VOXGRAPH_SUBMAP_FRONTIERS_H
#define ACTIVE_3D_PLANNING_VOXGRAPH_SUBMAP_FRONTIERS_H

#include "frontier.h"

namespace active_3d_planning{

    class SubmapFrontiers {
    public:
        SubmapFrontiers();

        SubmapFrontiers(const std::vector<Frontier> &frontiers, const Transformation &T_M_S);

        SubmapFrontiers(const Transformation &T_M_S);

        void addFrontier(const Frontier &frontier);

        void removeFrontier(const int &frontier_idx);

        void setPose(const Transformation &T_M_S);

        const Transformation &getPose();

        const Transformation &getInversePose();

        std::vector<Frontier> &getFrontiers();

        Frontier &getFrontier(const int &frontier_indx);

        void transformSubmapFrontiers(const Transformation &T_new_old);

        const voxblox::Color &getColor();

        const Point& getOrigin();

        bool empty();

        bool exists(int frontier_idx);

    private:
        Transformation T_M_S_;

        Transformation T_S_M_;

        std::vector<Frontier> submap_frontiers_;

        voxblox::Color color_;
    };
}

#endif //ACTIVE_3D_PLANNING_VOXGRAPH_SUBMAP_FRONTIERS_H
