# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "active_3d_planning_core;active_3d_planning_ros;active_3d_planning_mav;active_3d_planning_voxgraph;active_3d_planning_safe_voxgraph;active_3d_planning_naive_voxgraph;roscpp;rospy;voxblox_ros;voxblox;trajectory_msgs;mav_msgs;std_msgs;sensor_msgs;geometry_msgs;nav_msgs;cblox_planning".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lactive_3d_planning_app_submap_exploration".split(';') if "-lactive_3d_planning_app_submap_exploration" != "" else []
PROJECT_NAME = "active_3d_planning_app_submap_exploration"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
