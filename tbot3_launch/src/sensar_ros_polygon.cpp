/*****************************
sensar_ros_polygon.cpp
SENSAR
Andre Cleaver
Tufts University
*****************************/

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <sstream>
#include <string>

using namespace std;

const std::string TOPIC_IN  = "/move_base/local_costmap/footprint";
const std::string TOPIC_OUT = "/SENSAR/polygon";
const std::string FRAME_IN  = "odom";
const std::string FRAME_OUT = "base_link";

const int FREQUENCY = 1.8;

geometry_msgs::TransformStamped transformToBase_link;
geometry_msgs::PolygonStamped latestMsg;
geometry_msgs::PolygonStamped previousMsg;
ros::Publisher relativePub;


geometry_msgs::Point toPoint(geometry_msgs::Point32 pt)
{
	geometry_msgs::Point point;
	point.x = pt.x;
	point.y = pt.y;
	point.z = pt.z;
	return point;
}

geometry_msgs::Point32 toPoint32(geometry_msgs::Point pt)
{
	geometry_msgs::Point32 point32;
	point32.x = pt.x;
	point32.y = pt.y;
	point32.z = pt.z;
	return point32;
}

geometry_msgs::PolygonStamped transformLocalization(geometry_msgs::PolygonStamped input)
{
    geometry_msgs::PolygonStamped transformed = input; 
    transformed.header.frame_id = FRAME_OUT;

	for(int i = 0; i < transformed.polygon.points.size(); i++)
    {
		geometry_msgs::PoseStamped point;
		point.pose.position = toPoint(transformed.polygon.points[i]);
		tf2::doTransform(point, point, transformToBase_link);
		transformed.polygon.points[i] = toPoint32(point.pose.position);
    }

    return transformed;
}

void publishLatest()
{
    relativePub.publish(transformLocalization(latestMsg));
}

void footprintCallback(const geometry_msgs::PolygonStamped::ConstPtr& inMsg)
{
    latestMsg = *inMsg;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "sensar_ros_footprint");
    ros::NodeHandle n;
 
    relativePub = n.advertise<geometry_msgs::PolygonStamped>(TOPIC_OUT, 5);
    ros::Subscriber globalSub  = n.subscribe(TOPIC_IN, 5, footprintCallback);
    
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
