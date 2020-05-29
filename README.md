# Autonomous-Exploration-using-Submap-Collection
This package contains two motion planners for drones used for autonomous exploration of unknown environments, using submap-based map representation ([Voxgraph](https://github.com/ethz-asl/voxgraph))
- Submap-based RRT* Receding-Horizon Next-Best-View: a unique RH-NBV exploration planner using the map lookup system described [here](https://www.cs.cmu.edu/~kaess/pub/Ho18iros.pdf)
- our planner: a combination of a RH-NBV planner used as local planner and a frontier based planner used as global.

The packages is based on [mav_active_3d_planning](https://github.com/ethz-asl/mav_active_3d_planning).

To ensure compatibility, the following branches should be used:
* `Voxgraph`: `feature/exploration_planner`
* `Voxblox` : `feature/exploration_planner`
* `Cblox` : `feature/exploration_planner`
* `mav_voxblox_planning` : `feature/exploration_planner`

## Table of Contents
- [Packages](#packages)
- [Installation](#installation)
- [Global Path Planning](#global-path-planning)
  - [Demo](#demo)
  - [Executables](#executables)
- [Local Path Planning](#local-path-planning)
  - [Executables](#executables)
- [Planning Infrastructure](#planning-infrastructure)
  - [Executables](#executables)
  - [Frames](#frames)
## Packages
This repo is divide in three packages:
- **active_3d_planning_app_submap_exploration**:
contains modules composing the main structure of the two planners, among which modules for a general global-local planner, for frontier detection and selection, and for RH-NBV planners adapted to the submap-based representation.  

- **active_3d_planning_voxgraph**:
using Voxgraph as map representation and includes modules specific for our planner.
  
- **active_3d_planning_naive_voxgraph**:
using Voxgraph as map representation and includes modules specific for Submap-based RRT* Receding-Horizon Next-Best-View.

## Installation
Firstly, ROS need to be installed (see instructions [here](http://wiki.ros.org/Installation/Ubuntu)), then setup a catkin workspace as explained [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
After that clone the repository in source folder and compile:
```
cd ~/catkin_ws/src/
git clone git@github.com:ethz-asl/asldoc-2019-ma-gasserl.git
catkin build active_3d_planning_app_submap_exploration 
```
`active_3d_planning_voxgraph` and `active_3d_planning_naive_voxgraph` are dependencies of `active_3d_planning_app_submap_exploration`, so the aforementioned build command should be enough to compile all the packages.

## Demo
# Exploration experiments without drift

# Exploration experiments with drift

# Evaluation


