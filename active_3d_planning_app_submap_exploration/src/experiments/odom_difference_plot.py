#!/usr/bin/env python

import csv
import matplotlib.pyplot as plt


def read_odom_data(file_name):
    # Read voxblox data file
    data = {}
    headers = None
    with open(file_name) as infile:
        reader = csv.reader(infile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for row in reader:
            if row[0] == 'x_base_link':
                headers = row
                for header in headers:
                    data[header] = []
                continue
            else:
                for i in range(len(row)):
                    data[headers[i]].append(row[i])
    return data

# Read data
file_name= "/home/davide/Desktop/prova.csv"
data = read_odom_data(file_name)

# Plot
plt.plot(data["x_base_link"], data["y_base_link"], label="Ground truth odometry")
plt.xticks(range(10))
plt.yticks(range(10))

plt.xlim([0,10])
plt.ylim([0,10])

#plt.plot(data["x_drifted_frame"], data["y_drifted_frame"], label="Drifted odometry")

plt.legend()
