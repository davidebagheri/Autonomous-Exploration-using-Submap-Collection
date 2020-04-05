#include "active_3d_planning_app_submap_exploration/tools/transformations.h"

namespace active_3d_planning {
    void fromVoxbloxPointToPoseStamped(const voxblox::Point& voxblox_point, geometry_msgs::PoseStamped* pose){
        pose->pose.position.x = voxblox_point.x();
        pose->pose.position.y = voxblox_point.y();
        pose->pose.position.z = voxblox_point.z();
    }

    void fromPoseStampedToVoxbloxPoint(const geometry_msgs::PoseStamped& pose, voxblox::Point* voxblox_point){
        voxblox_point->x() = pose.pose.position.x;
        voxblox_point->y() = pose.pose.position.y;
        voxblox_point->z() = pose.pose.position.z;
    }

    void transformPoseStamped(geometry_msgs::PoseStamped* pose_out, voxblox::Transformation T_out_in,
            const geometry_msgs::PoseStamped& pose_in){
        voxblox::Point point_in;
        voxblox::Point point_out;

        // Convert to point
        fromPoseStampedToVoxbloxPoint(pose_in, &point_in);

        // Transform
        point_out = T_out_in * point_in;

        // Set values
        fromVoxbloxPointToPoseStamped(point_out, pose_out);
    }

    void fromVoxbloxPointToEigen(const voxblox::Point& voxblox_point, Eigen::Vector3d* vector){
        vector->x() = voxblox_point.x();
        vector->y() = voxblox_point.y();
        vector->z() = voxblox_point.z();
    }

    void fromEigentoVoxbloxPoint(const Eigen::Vector3d& vector, voxblox::Point* voxblox_point){
        voxblox_point->x() = vector.x();
        voxblox_point->y() = vector.y();
        voxblox_point->z() = vector.z();
    }

    void transformEigen(Eigen::Vector3d* vector_out, voxblox::Transformation T_out_in, const Eigen::Vector3d& vector_in){
        voxblox::Point point_in;
        voxblox::Point point_out;

        // Convert to point
        fromEigentoVoxbloxPoint(vector_in, &point_in);

        // Transform
        point_out = T_out_in * point_in;

        // Set values
        fromVoxbloxPointToEigen(point_out, vector_out);
    }

    void fromEigenToPoseStamped(const Eigen::Vector3d& vector, geometry_msgs::PoseStamped* pose){
        pose->pose.position.x = vector.x();
        pose->pose.position.y = vector.y();
        pose->pose.position.z = vector.z();
    }

    void fromPoseStampedToEigen(const geometry_msgs::PoseStamped pose, Eigen::Vector3d* vector){
        vector->x() = pose.pose.position.x;
        vector->y() = pose.pose.position.y;
        vector->z() = pose.pose.position.z;
    }

    inline void msgToVectorEigen(const geometry_msgs::Vector3& msg,
                                 Eigen::Vector3d* eigen) {
        assert(eigen != NULL);
        eigen->x() = msg.x;
        eigen->y() = msg.y;
        eigen->z() = msg.z;
    }

    inline void quaternionEigenToMsg(const geometry_msgs::Quaternion& msg,
                                     Eigen::Quaterniond* eigen) {
        assert(eigen != NULL);
        eigen->x() = msg.x ;
        eigen->y() = msg.y;
        eigen->z() = msg.z;
        eigen->w() = msg.w;
    }


    void EigenFromMsgMultiDofJointTrajectoryPoint(const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg,
                                                  EigenTrajectoryPoint* trajectory_point){
        msgToVectorEigen(msg.transforms[0].translation, &trajectory_point->position_W);

        quaternionEigenToMsg(msg.transforms[0].rotation, &trajectory_point->orientation_W_B);
    }


}