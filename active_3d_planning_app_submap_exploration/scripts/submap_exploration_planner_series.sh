#!/bin/bash

# *** Args (need to be set) ***
n_experiments=10
target_dir="/media/davide/DATA/data/submap_exploration_planner/NoDrift/Final4"		# Can reuse same dir to add experiments
clear_maps=true		# Irreversibly remove maps after evaluation to save disk space
freq=30
dur=32

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
  roslaunch active_3d_planning_app_submap_exploration submap_exploration_planner_with_drift.launch data_directory:=$target_dir record_data:=true data_frequency:=$freq time_limit:=$dur experiment_config:="Maze.yaml"

  # evaluate
   roslaunch active_3d_planning_app_submap_exploration eval_data.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_maps evaluate:=true experiment_config:="Maze.yaml"
done

pkill  experiment4.sh
pkill  experiment4-Lin
