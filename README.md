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
In order to see the planners in action, start an unreal_cv_ros game, tab out of game it (Alt+Tab for Binary, Ctrl+Shift+F1 for Editor) and then run the following command:
```
roslaunch active_3d_planning_app_submap_exploration naive_voxgraph_exploration_planner_run.launch data_directory:=/path/to/my_data_dir
```
to use the Submap-based RRT* Receding-Horizon Next-Best-View, or 
```
roslaunch active_3d_planning_app_submap_exploration submap_exploration_planner_run.launch data_directory:=/path/to/my_data_dir
```
to use our planner.

These launch files simulate a Firefly drone in the game, which is free to move around in the game for 30 minutes guided by the planner.

The simulations that these executables run are supposed to be without sensor drift, so make sure the noise values in `catkin_ws/src/voxgraph_planner/active_3d_planning_app_submap_exploration/cfg/voxgraph/odometry_simulator.yaml` are set to zero.

During the simulation raw data get stored in the folder `my_data_dir`, passed as parameter. Then, it is  possible to process them with:
```
roslaunch active_3d_planning_app_submap_exploration eval_data.launch target_directory:=/path/to/my_data_dir
```
This measure the amount of observed volume over time and create a plot for each simulation. 
If multiple experiments are done, after the evaluation have been launched, it is possible to calculate average and standard deviation over all the executed experiments with:
```
roslaunch active_3d_planning_app_submap_exploration plot_experiment_series.launch target_directory:=/path/to/my_data_dir
```
This creates a plot in my_data_dir showing the average observed volume as a continuous line and the standard deviation as adjacent shaded area.


# Exploration experiments with drift

# Evaluation


