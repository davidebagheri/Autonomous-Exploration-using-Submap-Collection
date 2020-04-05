#ifndef ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_TRANSFORMATIONS_H
#define ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_TRANSFORMATIONS_H

#include "voxblox/core/common.h"
#include "geometry_msgs/PoseStamped.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "active_3d_planning_core/data/trajectory.h"

namespace active_3d_planning {
    void fromVoxbloxPointToPoseStamped(const voxblox::Point& voxblox_point, geometry_msgs::PoseStamped* pose);

    void fromPoseStampedToVoxbloxPoint(const geometry_msgs::PoseStamped& pose, voxblox::Point* voxblox_point);

    void transformPoseStamped(geometry_msgs::PoseStamped* pose_out, voxblox::Transformation T_out_in, const geometry_msgs::PoseStamped& pose_in);

    void fromVoxbloxPointToEigen(const voxblox::Point& voxblox_point, Eigen::Vector3d* vector);

    void fromEigentoVoxbloxPoint(const Eigen::Vector3d& vector, voxblox::Point* voxblox_point);

    void transformEigen(Eigen::Vector3d* vector_out, voxblox::Transformation T_out_in, const Eigen::Vector3d& vector_in);

    void fromEigenToPoseStamped(const Eigen::Vector3d& vector, geometry_msgs::PoseStamped* pose);

    void fromPoseStampedToEigen(const geometry_msgs::PoseStamped pose, Eigen::Vector3d* vector);

    inline void msgToVectorEigen(const geometry_msgs::Vector3& msg, Eigen::Vector3d* eigen);

    inline void quaternionEigenToMsg(const geometry_msgs::Quaternion& msg, Eigen::Quaterniond* eigen);

    void EigenFromMsgMultiDofJointTrajectoryPoint(const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg,
                                                  EigenTrajectoryPoint* trajectory_point);
};


#endif //ACTIVE_3D_PLANNING_APP_SUBMAP_EXPLORATION_TRANSFORMATIONS_H
