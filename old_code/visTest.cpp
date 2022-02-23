#include <ros/ros.h>
//Visualisation includes
#include <visualization_msgs/Marker.h>
ros::Publisher visualMarkerPub;



int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "basic_shapes");
	ros::NodeHandle nh;	
     // a ROS publisher for the marker array
    visualMarkerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	ros::spin();
    
    //Start the marker
    visualization_msgs::Marker marker;

    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    
    marker.ns = "basic_shapes";
    marker.id = 0;
    // Set the marker type.  
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 5.0;
    marker.scale.y = 5.0;
    marker.scale.z = 5.0;

    // Set the colour,
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    visualMarkerPub.publish(marker);

   
}