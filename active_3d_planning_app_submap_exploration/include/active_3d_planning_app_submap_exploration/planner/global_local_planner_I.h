#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_LOCAL_PLANNER_I_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_LOCAL_PLANNER_I_H

namespace active_3d_planning{

    enum PlanningStatus{LocalPlanning, GlobalPlanning};

    class GlobalLocalPlannerI{
    public:
        virtual ~GlobalLocalPlannerI() = default;

        virtual void plan() = 0;

    protected:
        virtual PlanningStatus stateMachine() = 0;

        virtual void localPlanningIteration() = 0;

        virtual void globalPlanningIteration() = 0;
    };
}
#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_GLOBAL_LOCAL_PLANNER_I_H
