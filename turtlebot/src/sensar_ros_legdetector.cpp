/*****************************
sensar_ros_point.cpp
SENSAR
Andre Cleaver
Tufts University
*****************************/

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "people_msgs/PositionMeasurement.h"
#include "people_msgs/PositionMeasurementArray.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/Marker.h"
#include <sstream>
#include <string>

using namespace std;

const std::string TOPIC_IN  = "/leg_tracker_measurements";
const std::string TOPIC_OUT = "/SENSAR/leg_detector_marker";
const std::string FRAME_IN  = "map";
const std::string FRAME_OUT = "base_link";

people_msgs::PositionMeasurement temp;

geometry_msgs::TransformStamped transformToBase_link;
people_msgs::PositionMeasurementArray latestMsg;
geometry_msgs::PointStamped previousMsg;

ros::Publisher vis_pub;

const int FREQUENCY = 30;


void publishLatest()
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = FRAME_OUT;
	marker.header.stamp = ros::Time();
	marker.ns = "LEGS";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 1;
	marker.pose.position.y = 1;
	marker.pose.position.z = 1;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.06;
	marker.scale.y = 0.06;
	marker.scale.z = 0.06;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	
    vis_pub.publish(marker);  
}

void pointCallback(const people_msgs::PositionMeasurementArray::ConstPtr& inMsg)
{
    latestMsg = *inMsg;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "sensar_ros_point");
    ros::NodeHandle n;
 
    vis_pub = n.advertise<visualization_msgs::Marker>(TOPIC_OUT, 0);
    
    ros::Subscriber globalSub  = n.subscribe(TOPIC_IN, 5, pointCallback);
    
    tf2_ros::Buffer tBuffer;
    tf2_ros::TransformListener tf2_listener (tBuffer);

    ros::Rate rate(FREQUENCY);
    
    while(ros::ok())
    {
		try
		{
			transformToBase_link = tBuffer.lookupTransform(FRAME_OUT, FRAME_IN, ros::Time(0));
		}
		
        catch(tf2::TransformException e)
        {
            ROS_INFO("%s \n", e.what());
        }

        ros::spinOnce();    
        publishLatest();
        rate.sleep();
    }
    
    return 0;
}
