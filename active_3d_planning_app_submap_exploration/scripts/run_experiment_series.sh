#!/bin/bash

# *** Args (need to be set) ***
n_experiments=1
target_dir="/home/davide/data/experiment_evaluation"		# Can reuse same dir to add experiments
clear_voxgraph_maps=true		# Irreversibly remove maps after evaluation to save disk space
experiment=2		# For auto configs of exp1/2 (city/windmill)
freq=20
dur=1

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
  roslaunch active_3d_planning_app_submap_exploration global_local_planner_real.launch data_directory:=$target_dir record_data:=true data_frequency:=$freq time_limit:=$dur

  # evaluate
   roslaunch active_3d_planning_app_submap_exploration eval_data.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxgraph_maps evaluate:=true
done


