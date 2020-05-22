#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf

def read_coordinates(listener, mission_frame, base_link, drifted_frame):
    data = {}
    success = 1
    
    try:
        (pos_base_link, quat_base_link) = listener.lookupTransform(mission_frame, base_link, rospy.Time(0))
        (pos_drifted_frame, quat_drifted_frame) = listener.lookupTransform(mission_frame, drifted_frame, rospy.Time(0))
        
        data["base_link_pos"] = pos_base_link 
        data["base_link_quat"] = quat_base_link 
        data["drifted_frame_pos"] = pos_drifted_frame
        data["drifted_frame_quat"] = quat_drifted_frame
        
        success = 1
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("non posso prendere la trasformazione di " + str( base_link) + " o " + str(drifted_frame))
        success = 0
    
    return (success, data)
    

if __name__ == '__main__':
    rospy.init_node('odometry_difference_node', anonymous=True)
    
    # Get params
    mission_frame = rospy.get_param('~mission_frame', 'mission')
    base_link = rospy.get_param('~base_link_frame', 'firefly/base_link')
    drifted_frame = rospy.get_param('~drifted_frame', 'imu')    
        
    # tf listener
    listener = tf.TransformListener()
    
    #publisher
    base_link_pub = rospy.Publisher('base_link', PoseStamped, queue_size=10)   
    drifted_frame_pub = rospy.Publisher('drifted_frame', PoseStamped, queue_size=10)   
    
    
    # Loop
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        # Read
        (success, data) = read_coordinates(listener, mission_frame, base_link, drifted_frame)
        
        # Write
        if success == 1:
              pose1 = PoseStamped()
              pose1.header.stamp = rospy.Time.now()
              pose1.header.frame_id = "mission"
              pose1.pose.position.x = data["base_link_pos"][0]
              pose1.pose.position.y = data["base_link_pos"][1]
              pose1.pose.position.z = data["base_link_pos"][2]
              pose1.pose.orientation.x = data["base_link_quat"][0]
              pose1.pose.orientation.y = data["base_link_quat"][1]
              pose1.pose.orientation.z = data["base_link_quat"][2]
              pose1.pose.orientation.w = data["base_link_quat"][3]
              base_link_pub.publish(pose1)
              
              pose2 = PoseStamped()
              pose2.header.stamp = rospy.Time.now()
              pose2.header.frame_id = "mission"
              pose2.pose.position.x = data["drifted_frame_pos"][0]
              pose2.pose.position.y = data["drifted_frame_pos"][1]
              pose2.pose.position.z = data["drifted_frame_pos"][2]
              pose2.pose.orientation.x = data["drifted_frame_quat"][0]
              pose2.pose.orientation.y = data["drifted_frame_quat"][1]
              pose2.pose.orientation.z = data["drifted_frame_quat"][2]
              pose2.pose.orientation.w = data["drifted_frame_quat"][3]
              drifted_frame_pub.publish(pose2)
    
        rate.sleep()
    


