#!/bin/bash

# *** Args (need to be set) ***
n_experiments=5
target_dir="/media/davide/DATA/data/naive_voxgraph_planner/WithDrift/Drift0_001"		# Can reuse same dir to add experiments
clear_voxgraph_maps=true		# Irreversibly remove maps after evaluation to save disk space
freq=30
dur=30

# *** Run experiments ***
echo "Starting experiment${experiment} series of ${n_experiments} runs at '${target_dir}'!"

# Create dir
if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi

# Run the experiments
for (( i=1; i<=n_experiments; i++ ))
do  
  # run experiment

  roslaunch active_3d_planning_app_submap_exploration naive_voxgraph_exploration_planner_with_drift.launch data_directory:=$target_dir record_data:=true data_frequency:=$freq time_limit:=$dur

  # evaluate
   roslaunch active_3d_planning_app_submap_exploration eval_data.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxgraph_maps evaluate:=true
	done

pkill experiment4.sh
pkill experiment4-Lin




