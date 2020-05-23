#!/bin/bash

# *** Args (need to be set) ***
n_experiments=6
target_dir="/home/davide/data/experiment_evaluation"		# Can reuse same dir to add experiments
clear_maps=true		# Irreversibly remove maps after evaluation to save disk space
evaluate_ground_truth=true #set true to evaluate on voxblox node working with ground truth odometry
freq=30
dur=30

# *** Run experiments ***
echo "Starting experiment${experiment} series of ${n_experiments} runs at '${target_dir}'!"

# Create dir
if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi

# Run the experiments
for (( i=0; i<=n_experiments; i++ ))
do  
  # run experiment
  roslaunch active_3d_planning_app_submap_exploration submap_exploration_planner_with_GT_evaluation.launch data_directory:=$target_dir record_data:=true data_frequency:=$freq time_limit:=$dur experiment_config:="Maze.yaml"

  # evaluate
   roslaunch active_3d_planning_app_submap_exploration eval_data.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_maps clear_voxgraph_maps:=$clear_maps evaluate:=true experiment_config:="Maze.yaml" evaluate_ground_truth:=$evaluate_ground_truth
done

pkill  experiment4.sh
pkill  experiment4-Lin

roslaunch active_3d_planning_app_submap_exploration plot_experiment_series.launch target_directory:=$target_dir	
