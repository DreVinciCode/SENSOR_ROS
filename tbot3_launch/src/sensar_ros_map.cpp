/*****************************
sensar_ros_map.cpp
SENSAR
Andre Cleaver
Tufts University
*****************************/

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <sstream>
#include <string>

using namespace std;

const std::string TOPIC_IN  = "/map";
const std::string TOPIC_OUT = "/SENSAR/Map";
const std::string FRAME_IN  = "map";
const std::string FRAME_OUT = "base_link";

const int FREQUENCY = 10;

geometry_msgs::TransformStamped transformToBase_link;
nav_msgs::OccupancyGrid latestMsg;
nav_msgs::OccupancyGrid previousMsg;

ros::Publisher relativePub;

nav_msgs::OccupancyGrid transformLocalization(nav_msgs::OccupancyGrid input)
{
    nav_msgs::OccupancyGrid transformed = input; 
    transformed.header.frame_id = FRAME_OUT;

    geometry_msgs::PoseStamped stampedPose;
    stampedPose.header = input.header;
    stampedPose.pose = transformed.info.origin;
    tf2::doTransform(stampedPose, stampedPose, transformToBase_link);
    transformed.info.origin = stampedPose.pose;

    return transformed;
}

void publishLatest()
{
    relativePub.publish(transformLocalization(latestMsg));
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& inMsg)
{
    latestMsg = *inMsg;

    if (latestMsg.data != previousMsg.data)
    {
        previousMsg = latestMsg;
        publishLatest();
    }
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "sensar_ros_map");
    ros::NodeHandle n;
 
    relativePub = n.advertise<nav_msgs::OccupancyGrid>(TOPIC_OUT, 5);
    ros::Subscriber globalSub  = n.subscribe(TOPIC_IN, 5, mapCallback);
    
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

