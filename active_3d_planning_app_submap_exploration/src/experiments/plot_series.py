#!/usr/bin/env python

import csv
import datetime
import numpy as np
import os
import re
# ros
import rospy

# Plotting
from matplotlib import pyplot as plt


class EvalPlotting:
    def __init__(self):
        # Parse parameters
        target_dir = rospy.get_param('~target_directory')


        # Evaluate        
        planners_subdirs = [os.path.join(target_dir, o) for o in os.listdir(target_dir)]
        plt.rcParams.update({'font.size': 15})
    
        plt.figure()
        for subdir in planners_subdirs:
            planner_name = self.get_planner_name(os.path.split(subdir)[1])
            self.compute_series(subdir, planner_name)
        
        save_name = os.path.join(target_dir, "SeriesOverview.png")
        plt.legend()
        plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')
        rospy.loginfo("\n" + "*" * 53 + "\n* Evaluation completed successfully, shutting down. *\n" + "*" * 53)
        
    def get_planner_name(self, planner_dir):
        if (planner_dir == "naive_voxgraph_planner"): 
            return "Submap-based RRT* RH-NBV [8]"
        if (planner_dir == "safe_submap_exploration_planner"): 
            return "Safe Submap Exploration Planner"
        if (planner_dir == "submap_exploration_planner"): 
            return "Ours"
        if (planner_dir == "voxgraph_local_planner"): 
            return "Voxgraph Local Planner"
        else:
            return planner_dir
        
    def compute_series(self, target_dir, planner_name):
        rospy.loginfo("Evaluating experiment series at '%s'", target_dir)

        # Setup a directory for data, plots, ...
        folder_name = "series_evaluation"
        if not os.path.isdir(os.path.join(target_dir, folder_name)):
            os.mkdir(os.path.join(target_dir, folder_name))
        self.eval_log_file = open(os.path.join(target_dir, folder_name, "eval_log.txt"), 'a')
        
        # Read all the data
        dir_expression = re.compile('\d{8}_\d{6}')
        subdirs = [o for o in os.listdir(target_dir) if os.path.isdir(os.path.join(target_dir, o)) and
				   dir_expression.match(o)]
        rospy.loginfo("Evaluating '%s' (%i subdirs) for the series performance computation." % (target_dir, len(subdirs)))
        self.writelog("Evaluating '%s' (%i subdirs) for the series performance computation." % (target_dir, len(subdirs)))
        voxblox_data = []
        max_data_length = 0
        names = []
        for o in subdirs:
          
            if os.path.isfile((os.path.join(target_dir, o, "graphs", "ObservedVoxels.png"))):
                # Valid evaluated directory

                data = self.read_voxblox_data(os.path.join(target_dir, o, "voxblox_data.csv"))
                max_data_length = max(max_data_length, len(data["RosTime"]))
                voxblox_data.append(data)
                names.append(o)
            else:
                rospy.logwarn("Experiment at '%s' not properly evaluated!", o)
                self.writelog("Experiment at '%s' not properly evaluated!" % o)

        if max_data_length < 2:
            rospy.loginfo("No valid experiments found, stopping series evaluation.")
            self.writelog("No valid experiments found, stopping series evaluation.")
            self.eval_log_file.close()
            return

        # Create common data timeline by averaging measurement times (these should be similar)
        data_file = open(os.path.join(target_dir, folder_name, "series_data.csv"), 'wb')
        data_writer = csv.writer(data_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL,
				                 lineterminator='\n')
        means = {}
        std_devs = {}
        keys = voxblox_data[0].keys()
        keys.remove('RosTime')
        keys = ['RosTime'] + keys  # RosTime is expected as the first argument
        prev_pcls = [0.0] * len(voxblox_data)
        for key in keys:
            means[key] = np.array([])
            std_devs[key] = np.array([])
        for i in range(max_data_length):
            line = []
            if i == 0:
                header_line = []
                for key in keys:
                    header_line.extend((key, ''))
                    for name in names:
                        line.append(name)
                        header_line.append('')
                    line.extend(("Mean", "StdDev"))
                data_writer.writerow(header_line)
                data_writer.writerow(line)
                line = []
            for key in keys:
                values = []
                for dataset in voxblox_data:
                    if i < len(dataset[key]):
                        if key == 'NPointclouds':
                            # These need to accumulate
                            ind = voxblox_data.index(dataset)
                            prev_pcls[ind] = prev_pcls[ind] + float(dataset[key][i])
                            line.append(prev_pcls[ind])
                            values.append(prev_pcls[ind])
                        else:
                            line.append(dataset[key][i])
                            values.append(dataset[key][i])
                    else:
                        line.append("")
                values = np.array(values, dtype=float)
                mean = np.mean(values)
                std = np.std(values)
                means[key] = np.append(means[key], mean)
                std_devs[key] = np.append(std_devs[key], std)
                line.extend((mean, std))
            data_writer.writerow(line)
        data_file.close()
        
		# Create plot
        rospy.loginfo("Creating graph 'SeriesOverview'")
        x = means['RosTime']
        unit = "s"
        if x[-1] >= 300:
            unit = "min"
            x = np.divide(x, 60)
            cpu_use = np.zeros(np.shape(means['CPUTime']))
        cpu_std = np.zeros(np.shape(means['CPUTime']))
        for i in range(len(means['CPUTime']) - 1):
            div = (means['RosTime'][i + 1] - means['RosTime'][i])
            cpu_use[i] = (means['CPUTime'][i + 1]) / div
            cpu_std[i] = (std_devs['CPUTime'][i + 1]) / div
        cpu_use[-1] = cpu_use[-2]
        cpu_use = np.repeat(cpu_use, 2)
        cpu_std[-1] = cpu_std[-2]
        cpu_std = np.repeat(cpu_std, 2)

		# Plot ends of data series for unequal lengths
        early_stops = []
        x_early = []
        for i in range(len(voxblox_data)):
            dataset = voxblox_data[i]
            length = len(dataset['RosTime']) - 1
            if length < max_data_length - 1:
                early_stops.append(length)
                x_early.append(float(dataset['RosTime'][length]))
                self.writelog("Early stop detected for '%s' at %.2fs." % (names[i], float(dataset['RosTime'][length])))

		
		# Compensate unobservable voxels
        if np.max(means['UnknownVoxels']) > 0:
            unknown = means['UnknownVoxels'] 
            std_devs['UnknownVoxels'] = std_devs['UnknownVoxels'] * 100
            unknown = np.maximum(unknown, np.zeros_like(unknown)) * 100
            if (planner_name == "Normalized Frontier"):
                plt.plot(x, unknown, '-g',label = planner_name)
                plt.fill_between(x, unknown - std_devs['UnknownVoxels'],
				                    unknown + std_devs['UnknownVoxels'],
                                    color = 'g',
				                    alpha=.2)
            elif (planner_name == "Biggest Frontier"):
                plt.plot(x, unknown, color='orange',label = planner_name)
                #plt.fill_between(x, unknown - std_devs['UnknownVoxels'],
				 #                   unknown + std_devs['UnknownVoxels'],
                  #                  color = 'orange', 
				   #                 alpha=.2)
            elif (planner_name == "Nearest Frontier"):
                plt.plot(x, unknown, color='blue',label = planner_name)
                #plt.fill_between(x, unknown - std_devs['UnknownVoxels'],
				 #                   unknown + std_devs['UnknownVoxels'],
                  #                  color = 'blue', 
				   #                 alpha=.2)
            elif (planner_name == "Euclidean Normalized Frontier"):
                plt.plot(x, unknown, '-r',label = planner_name)
                plt.fill_between(x, unknown - std_devs['UnknownVoxels'],
				                    unknown + std_devs['UnknownVoxels'],
                                    color = 'r', 
				                    alpha=.2)
            elif (planner_name == "Euclidean Nearest Frontier"):
                plt.plot(x, unknown, '-r',label = planner_name)
                plt.fill_between(x, unknown - std_devs['UnknownVoxels'],
				                    unknown + std_devs['UnknownVoxels'],
                                    color = 'r', 
				                    alpha=.2)
            
            else: 
                plt.plot(x, unknown, label = planner_name)
                #plt.fill_between(x, unknown - std_devs['UnknownVoxels'],
    			#	                    unknown + std_devs['UnknownVoxels'],
    			#	                    alpha=.2)
            plt.plot([x[i] for i in early_stops], [means['UnknownVoxelsGroundTruth'][i] for i in early_stops], 'kx',
				            markersize=9, markeredgewidth=2)
            plt.ylabel('Unknown Voxels [%]')
            plt.xlabel('Time [min]')
            plt.ylim(0, 100)
        
        plt.xlim(left=0, right=x[-1])
        
        folder_name = "series_evaluation"
        self.writelog("Created graph 'SeriesOverview'.")
        self.eval_log_file.close()
        
    def writelog(self, text):
        if self.eval_log_file is not None:
            self.eval_log_file.write(datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") + text + "\n")
	

    @staticmethod
    def read_voxblox_data(file_name):
        # Read voxblox data file
        data_voxblox = {}
        headers = None
        with open(file_name) as infile:
            reader = csv.reader(infile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            for row in reader:
                if row[0] == 'MapName' or row[0] == "RosTimeM":
                    headers = row
                    for header in headers:
                        data_voxblox[header] = []
                    continue
                if row[0] != 'Unit':
                    for i in range(len(row)):
                        data_voxblox[headers[i]].append(row[i])
        return data_voxblox

if __name__ == '__main__':
    rospy.init_node('eval_plotting_node', anonymous=True)
    ep = EvalPlotting()
