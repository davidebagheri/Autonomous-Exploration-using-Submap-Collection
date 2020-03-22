#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "readTFframe");
    ros::NodeHandle nh;
    tf::TransformListener listener;
    double distance_run = 0;
    tf::Vector3 previous_position;
    tf::StampedTransform transform;

    // Get first position
    listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(30.0) );
    listener.lookupTransform("/world", "/camera_link", ros::Time(0), transform);
    previous_position = transform.getOrigin();
    ros::Rate rate(20);

    /// Test
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.scale.x = 0.7;
    marker.scale.y = 0.7;
    marker.scale.z = 0.7;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;


    while(ros::ok()){
        // Get current position
        listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/world","/firefly/base_link",  ros::Time(0), transform);
        tf::Vector3 current_position = transform.getOrigin();

        // Compute distance
        std::cout << "length: " << (current_position - previous_position).length() << std::endl;
        if ((current_position - previous_position).length() > 0.05) {
            distance_run += (current_position - previous_position).length();
            previous_position = current_position;
        }

        marker.pose.position.x = current_position.x();
        marker.pose.position.y = current_position.y();
        marker.pose.position.z = current_position.z();
        vis_pub.publish( marker );

        std::cout << "Distance run: "<< distance_run << std::endl;
        rate.sleep();
    }

    return 0;
}