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
    """
    This is the main evaluation node. It expects the data folders and files to have the format hardcoded in the
    eval_data_node and calls the eval_voxblox_node to execute c++ code. Pretty ugly and non-general code but just
    needs to work in this specific case atm...
    """

    def __init__(self):
        # Parse parameters
        target_dir = rospy.get_param('~target_directory')
        self.ns_voxgraph = rospy.get_param('~ns_eval_voxgraph_node', '/voxgraph_node_evaluator')
        self.evaluate = rospy.get_param('~evaluate', True)
        self.create_plots = rospy.get_param('~create_plots', True)
        self.show_plots = rospy.get_param('~show_plots', False)  # Auxiliary param, prob removed later
        self.clear_voxblox_maps = rospy.get_param('~clear_voxblox_maps', False)  # rm all maps after eval (disk space!)
        self.unobservable_points_pct = rospy.get_param('~unobservable_points_pct', 0.0)  # Exlude unobservable points
        # from the plots (in percent of total)

        # Setup
        self.eval_log_file = None
        print("waiting for: " +self.ns_voxgraph + "/evaluate")
        rospy.wait_for_service(self.ns_voxgraph + "/evaluate")
        self.eval_voxgraph_srv = rospy.ServiceProxy(self.ns_voxgraph + "/evaluate", Empty)
        print("service:::::::"+self.ns_voxgraph + "/evaluate")

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

            #if os.path.isfile(os.path.join(target_dir, "performance_log.csv")):
                # Read performance data file
             #   data_perf = {}
              #  headers = None
               # with open(os.path.join(target_dir, "performance_log.csv")) as infile:
                #    reader = csv.reader(infile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                 #   for row in reader:
                  #      if row[0] == 'RunTime':
                   #         headers = row
                    #        for header in headers:
                     #           data_perf[header] = []
                      #      continue
                       # for i in range(len(row)):
                        #    data_perf[headers[i]].append(row[i])

                # Create graph
                #if len(data_perf['RosTime']) > 1:
                 #   self.plot_perf_overview(data_perf, target_dir)
                #else:
                 #   rospy.loginfo("Too few entries in 'performance_log.csv', skipping dependent graphs.")
            #else:
             #   rospy.loginfo("No 'performance_log.csv' found, skipping dependent graphs.")

            # Finish
            if self.clear_voxblox_maps:
                # Remove all voxblox maps to free up disk space
                shutil.rmtree(os.path.join(target_dir, 'voxgraph_collections'), ignore_errors=True)
            self.eval_log_file.close()

    @staticmethod
    def read_voxblox_data(file_name):
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

        unknown = np.array(data['UnknownVoxels'], dtype=float)
        if np.max(unknown) > 0:
            # compensate unobservable voxels
            unknown = (unknown - self.unobservable_points_pct) / (
                    1.0 - self.unobservable_points_pct)  # compensate invisible
            unknown = np.maximum(unknown, np.zeros_like(unknown))
            plt.ylabel('Unknown Voxels [%]')
            plt.ylim(0, 1)
        plt.plot(x, unknown, 'g-')
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

    """def plot_perf_overview(self, data, target_dir):
        rospy.loginfo("Creating Graphs: PerformanceOverview")

        x = np.cumsum(np.array(data['RosTime'], dtype=float))
        unit = "s"
        if x[-1] >= 300:
            unit = "min"
            x = np.true_divide(x, 60)
        y_select = np.array(data['Select'], dtype=float)
        y_expand = np.array(data['Expand'], dtype=float)
        y_gain = np.array(data['Gain'], dtype=float)
        y_cost = np.array(data['Cost'], dtype=float)
        y_value = np.array(data['Value'], dtype=float)
        y_next = np.array(data['NextBest'], dtype=float)
        y_upTG = np.array(data['UpdateTG'], dtype=float)
        y_upTE = np.array(data['UpdateTE'], dtype=float)
        y_vis = np.array(data['Visualization'], dtype=float)
        y_ros = np.array(data['RosCallbacks'], dtype=float)
        y_tot = np.array(data['Total'], dtype=float)

        y0 = np.divide(y_select, y_tot)
        y1 = np.divide(y_expand, y_tot) + y0
        y2 = np.divide(y_gain, y_tot) + y1
        y3 = np.divide(y_cost, y_tot) + y2
        y4 = np.divide(y_value, y_tot) + y3
        y5 = np.divide(y_next, y_tot) + y4
        y6 = np.divide(y_upTG, y_tot) + y5
        y7 = np.divide(y_upTE, y_tot) + y6
        y8 = np.divide(y_vis, y_tot) + y7
        y9 = 1.0 - np.divide(y_ros, y_tot)

        sum_tot = np.sum(y_tot)
        s0 = np.sum(y_select) / sum_tot * 100
        s1 = np.sum(y_expand) / sum_tot * 100
        s2 = np.sum(y_gain) / sum_tot * 100
        s3 = np.sum(y_cost) / sum_tot * 100
        s4 = np.sum(y_value) / sum_tot * 100
        s5 = np.sum(y_next) / sum_tot * 100
        s6 = np.sum(y_upTG) / sum_tot * 100
        s7 = np.sum(y_upTE) / sum_tot * 100
        s8 = np.sum(y_vis) / sum_tot * 100
        s9 = np.sum(y_ros) / sum_tot * 100
        s10 = 100 - s0 - s1 - s2 - s3 - s4 - s5 - s6 - s7 - s8 - s9

        x = np.repeat(x, 2)
        x = np.concatenate((np.array([0]), x[:-1]))
        y0 = np.repeat(y0, 2)
        y1 = np.repeat(y1, 2)
        y2 = np.repeat(y2, 2)
        y3 = np.repeat(y3, 2)
        y4 = np.repeat(y4, 2)
        y5 = np.repeat(y5, 2)
        y6 = np.repeat(y6, 2)
        y7 = np.repeat(y7, 2)
        y8 = np.repeat(y8, 2)
        y9 = np.repeat(y9, 2)

        fig = plt.figure()
        axes = [plt.subplot2grid((5, 1), (0, 0), rowspan=3), plt.subplot(5, 1, 4), plt.subplot(5, 1, 5)]

        axes[0].fill_between(x, 0, y0, facecolor="#a1b400", alpha=.5)
        axes[0].fill_between(x, y0, y1, facecolor="#009000", alpha=.5)
        axes[0].fill_between(x, y1, y2, facecolor="#dc1000", alpha=.5)
        axes[0].fill_between(x, y2, y3, facecolor="#ff4f00", alpha=.5)
        axes[0].fill_between(x, y3, y4, facecolor="#ffb800", alpha=.5)
        axes[0].fill_between(x, y4, y5, facecolor="#ffff00", alpha=.5)
        axes[0].fill_between(x, y5, y6, facecolor="#00eebc", alpha=.5)
        axes[0].fill_between(x, y6, y7, facecolor="#d800dd", alpha=.5)
        axes[0].fill_between(x, y7, y8, facecolor="#a3baff", alpha=.5)
        axes[0].fill_between(x, y8, y9, facecolor="#cccccc", alpha=.5)
        axes[0].fill_between(x, y9, 1, facecolor="#606060", alpha=.5)
        axes[0].set_xlim(left=0, right=x[-1])
        axes[0].set_ylim(bottom=0, top=1)
        axes[0].set_title("Percentage of CPU Time Spent per Function")
        axes[0].set_ylabel('Percent [%]')

        x = np.cumsum(np.array(data['RosTime'], dtype=float))
        if unit == "min":
            x = np.true_divide(x, 60)
        n_trajectories = np.array(data['NTrajectories'], dtype=int)
        n_after_update = np.array(data['NTrajAfterUpdate'], dtype=int)
        n_after_update = np.concatenate((np.array([0]), n_after_update[:-1]))
        n_new = n_trajectories - n_after_update

        axes[1].plot(x, n_trajectories, 'b-')
        axes[1].plot(x, n_new, 'g-')
        axes[1].fill_between(x, 0, n_new, facecolor="#009000", alpha=.3)
        axes[1].fill_between(x, n_new, n_trajectories, facecolor="#0000ff", alpha=.3)
        axes[1].set_xlim(left=0, right=x[-1])
        axes[1].set_ylim(bottom=0)
        axes[1].set_title("Trajectory Tree Size")
        axes[1].set_ylabel('TrajectorySegments [-]')
        axes[1].legend(["Total", "New"], loc='upper left', fancybox=True)

        x = np.array([])
        ros_time = np.array(data['RosTime'], dtype=float)
        cpu_times = [np.array(data['Total'], dtype=float),
                     y_select + y_expand + y_gain + y_cost + y_value + y_next + y_upTE + y_upTG]  # Total, Planning
        cpu_use = [np.array([])] * len(cpu_times)
        i = 0
        averaging_threshold = 2.0  # seconds, for smoothing
        t_curr = ros_time[0]
        x_curr = 0
        cpu_curr = [time[0] for time in cpu_times]
        while i + 1 < len(ros_time):
            i = i + 1
            if t_curr >= averaging_threshold:
                for j in range(len(cpu_times)):
                    cpu_use[j] = np.append(cpu_use[j], cpu_curr[j] / t_curr)
                x_curr = x_curr + t_curr
                x = np.append(x, x_curr)
                t_curr = ros_time[i]
                for j in range(len(cpu_times)):
                    cpu_curr[j] = cpu_times[j][i]
            else:
                t_curr = t_curr + ros_time[i]
                for j in range(len(cpu_times)):
                    cpu_curr[j] = cpu_curr[j] + cpu_times[j][i]

        if unit == "min":
            x = np.true_divide(x, 60)

        axes[2].plot(x, cpu_use[0], 'k-')
        axes[2].plot(x, cpu_use[1], linestyle='-', color='#5492E7')
        axes[2].plot(np.array([0, x[-1]]), np.array([1, 1]), linestyle='-', color='0.7', alpha=0.8)
        axes[2].set_xlim(left=0, right=x[-1])
        axes[2].set_ylim(bottom=0)
        axes[2].set_ylabel('CPU Usage [cores]')
        axes[2].set_title("Planner Consumed CPU Time per Simulated Time")
        axes[2].set_xlabel('Simulated Time [%s]' % unit)
        axes[2].legend(["Process", "Planning"], loc='upper left', fancybox=True)

        fig.set_size_inches(15, 15, forward=True)
        plt.tight_layout()

        box = axes[0].get_position()
        axes[0].set_position([box.x0, box.y0 + box.height * 0.16, box.width, box.height * 0.84])
        legend = ["({0:02.1f}%) Select".format(s0), "({0:02.1f}%) Expand".format(s1), "({0:02.1f}%) Gain".format(s2),
                  "({0:02.1f}%) Cost".format(s3), "({0:02.1f}%) Value".format(s4), "({0:02.1f}%) NextBest".format(s5),
                  "({0:02.1f}%) updateGen".format(s6), "({0:02.1f}%) UpdateEval".format(s7),
                  "({0:02.1f}%) Vis".format(s8), "({0:02.1f}%) Other".format(s10), "({0:02.1f}%) ROS".format(s9)]
        axes[0].legend(legend, loc='upper center', bbox_to_anchor=(0.5, -0.04), ncol=6, fancybox=True)

        save_name = os.path.join(target_dir, "graphs", "PerformanceOverview.png")
        plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')
        self.writelog("Created graph 'PerformanceOverview'.")

        if self.show_plots:
            rospy.loginfo("Displaying '%s'. Close to continue...", save_name)
            plt.show()"""
            

if __name__ == '__main__':
    rospy.init_node('eval_plotting_node', anonymous=True)
    ep = EvalPlotting()
