#!/usr/bin/env python

import csv
import datetime
import numpy as np
import os
import re
# ros
import rospy
import shutil
# Python
import sys
# Plotting
from matplotlib import pyplot as plt
from std_srvs.srv import Empty


class EvalPlotting:
    def __init__(self):
        # Parse parameters
        target_dir = rospy.get_param('~target_directory')
        self.ns_voxgraph = rospy.get_param('~ns_eval_voxgraph_node', '/voxgraph_node_evaluator')
        self.evaluate = rospy.get_param('~evaluate', True)
        self.create_plots = rospy.get_param('~create_plots', True)
        self.show_plots = rospy.get_param('~show_plots', False)  # Auxiliary param, prob removed later
        self.clear_voxgraph_maps = rospy.get_param('~clear_voxgraph_maps', False)  # rm all maps after eval (disk space!)
        self.clear_voxblox_maps = rospy.get_param('~clear_voxblox_maps', False)  # rm all maps after eval (disk space!)
        self.unobservable_points_pct = rospy.get_param('~unobservable_points_pct', 0.0)  # Exlude unobservable points
        # from the plots (in percent of total)

        # Setup
        self.eval_log_file = None

        rospy.wait_for_service(self.ns_voxgraph + "/evaluate")
        self.eval_voxgraph_srv = rospy.ServiceProxy(self.ns_voxgraph + "/evaluate", Empty)

        # Evaluate
        dir_expression = re.compile('\d{8}_\d{6}')  # Only check the default names
        subdirs = [o for o in os.listdir(target_dir) if os.path.isdir(os.path.join(target_dir, o)) and 
                   dir_expression.match(o)]
        if len(subdirs) == 0:
            rospy.loginfo("No recent directories in target dir '%s' to evaluate.", target_dir)
            sys.exit(-1)

        for subdir in subdirs:
            self.run_single_evaluation(os.path.join(target_dir, subdir))

        rospy.loginfo("\n" + "*" * 53 + "\n* Evaluation completed successfully, shutting down. *\n" + "*" * 53)

    def run_single_evaluation(self, target_dir):
        rospy.loginfo("Starting evaluation on target '%s'.", target_dir)
        # Check target dir is valid (approximately)
        if not os.path.isfile(os.path.join(target_dir, "data_log.txt")):
            rospy.logerr("Invalid target directory.")
            return

        if self.evaluate:
            # Set params and call the voxblox evaluator
            rospy.set_param(self.ns_voxgraph + "/target_directory", target_dir)
            try:
                self.eval_voxgraph_srv()
            except:
                rospy.logerr("eval_voxblox service call failed. Shutting down.")
                sys.exit(-1)

        # Reopen logfile
        self.eval_log_file = open(os.path.join(target_dir, "data_log.txt"), 'a+')

        if self.create_plots:
            # Create dirs
            if not os.path.isdir(os.path.join(target_dir, "graphs")):
                os.mkdir(os.path.join(target_dir, "graphs"))

            if os.path.isfile(os.path.join(target_dir, "voxblox_data.csv")):
                # Read voxblox data file
                data_voxblox = self.read_voxblox_data(os.path.join(target_dir, "voxblox_data.csv"))
                if len(data_voxblox['RosTime']) > 1:
                    self.plot_sim_overview(data_voxblox, target_dir)
                else:
                    rospy.loginfo("Too few entries in 'voxblox_data.csv', skipping dependent graphs.")
            else:
                rospy.loginfo("No 'voxblox_data.csv' found, skipping dependent graphs.")

            # Finish
            if self.clear_voxgraph_maps:
                # Remove all voxblox maps to free up disk space
                shutil.rmtree(os.path.join(target_dir, 'voxgraph_collections'), ignore_errors=True)
            if self.clear_voxblox_maps:
                # Remove all voxblox maps to free up disk space
                shutil.rmtree(os.path.join(target_dir, 'voxblox_collections'), ignore_errors=True)
            self.eval_log_file.close()

    
    def read_voxblox_data(self, file_name):
        # Read voxblox data file
        data_voxblox = {}
        headers = None
        with open(file_name) as infile:
            reader = csv.reader(infile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            for row in reader:
                if row[0] == 'MapName':
                    headers = row
                    for header in headers:
                        data_voxblox[header] = []
                    continue
                if row[0] != 'Unit':
                    for i in range(len(row)):
                        data_voxblox[headers[i]].append(row[i])
        return data_voxblox

    def plot_sim_overview(self, data, target_dir):
        rospy.loginfo("Creating Graphs: SimulationOverview")
        unit = "s"

        x = np.array(data['RosTime'], dtype=float)
        if x[-1] >= 300:
            unit = "min"
            x = np.divide(x, 60)
        plt.figure()
        unknown = np.array(data['UnknownVoxels'], dtype=float)
        if np.max(unknown) > 0:
            # compensate unobservable voxels
            unknown = (unknown - self.unobservable_points_pct) / (
                    1.0 - self.unobservable_points_pct)  # compensate invisible
            unknown = np.maximum(unknown, np.zeros_like(unknown))
            plt.ylabel('Unknown Voxels [%]')
            plt.xlabel('Time [min]')
            plt.ylim(0, 100)
        plt.plot(x, unknown, 'g-', label='Unknown Voxels')
        plt.xlim(left=0, right=x[-1])

        save_name = os.path.join(target_dir, "graphs", "ObservedVoxels.png")
        plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')
        self.writelog("Created graph 'SimulationOverview'.")

        if self.show_plots:
            rospy.loginfo("Displaying '%s'. Close to continue...", save_name)
            plt.show()
            
    def writelog(self, text):
        # In case of simulation data being stored, maintain a log file
        if not self.evaluate:
            return
        self.eval_log_file.write(datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") + text + "\n")
            

if __name__ == '__main__':
    rospy.init_node('eval_plotting_node', anonymous=True)
    ep = EvalPlotting()
