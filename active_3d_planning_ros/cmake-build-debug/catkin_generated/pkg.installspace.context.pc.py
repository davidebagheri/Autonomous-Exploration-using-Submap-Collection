# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "active_3d_planning_core;roscpp;rospy;trajectory_msgs;std_msgs;sensor_msgs;geometry_msgs;nav_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lactive_3d_planning_ros".split(';') if "-lactive_3d_planning_ros" != "" else []
PROJECT_NAME = "active_3d_planning_ros"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
