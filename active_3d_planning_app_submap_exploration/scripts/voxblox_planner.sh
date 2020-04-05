#!/bin/bash

# *** Args (need to be set) ***
n_experiments=5
target_dir="/media/davide/DATA/data/submap_exploration_planner/WithDrift/VoxbloxPlanner"		# Can reuse same dir to add experiments
clear_voxblox_maps=true		# Irreversibly remove maps after evaluation to save disk space
experiment=2		# For auto configs of exp1/2 (city/windmill)


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
  roslaunch active_3d_planning_app_submap_exploration RH_NBV_planner_voxblox.launch data_directory:=$target_dir record_data:=true experiment_config:=$cfg data_frequency:=$freq time_limit:=$dur

  # evaluate
   roslaunch active_3d_planning_app_reconstruction evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps gt_file_path:=$pcl evaluate:=true
done

roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir series:=true 
