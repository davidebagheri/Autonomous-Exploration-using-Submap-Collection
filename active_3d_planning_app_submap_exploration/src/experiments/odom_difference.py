#!/usr/bin/env python

import rospy
import tf
import csv

def read_coordinates(listener, mission_frame, base_link, drifted_frame):
    data = {}
    success = 1
    
    try:
        (pos_base_link, quat_base_link) = listener.lookupTransform(mission_frame, base_link, rospy.Time(0))
        (pos_drifted_frame, quat_drifted_frame) = listener.lookupTransform(mission_frame, drifted_frame, rospy.Time(0))
        
        data["base_link"] = pos_base_link
        data["drifted_frame"] = pos_drifted_frame
        
        success = 1
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("non posso prendere la trasformazione " )
        success = 0
    
    return (success, data)
    

def write_data(eval_writer, data):
    eval_writer.writerow([round(data["base_link"][0], 3), round(data["base_link"][1], 3), round(data["base_link"][2],3), round(data["drifted_frame"][0],3), round(data["drifted_frame"][1],3), round(data["drifted_frame"][2],3)])

if __name__ == '__main__':
    rospy.init_node('odometry_difference_node', anonymous=True)
    
    # Get params
    mission_frame = rospy.get_param('~mission_frame', 'mission')
    base_link = rospy.get_param('~base_link_frame', 'firefly/base_link')
    drifted_frame = rospy.get_param('~drifted_frame', 'imu')
    data_file_name = rospy.get_param('~data_file_name', '/home/davide/Desktop/prova.csv')
    evaluation_time_rate = rospy.get_param('~evaluation_time_rate', '5')
    
    
    # Open file
    data_file = open(data_file_name, 'wb')		 
    eval_writer = csv.writer(data_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL, 
                              lineterminator='\n')
    eval_writer.writerow(['x_base_link', 'y_base_link', 'z_base_link', 'x_drifted_frame', 'y_drifted_frame', 'z_drifted_frame'])
    
    # tf listener
    listener = tf.TransformListener()
    
    
    # Loop
    rate = rospy.Rate(float(1/float(evaluation_time_rate)))
    
    while not rospy.is_shutdown():
        # Read
        (success, data) = read_coordinates(listener, mission_frame, base_link, drifted_frame)
        
        # Write
        if success == 1:
            write_data(eval_writer, data)
        
        rate.sleep()
        
    
    
    


