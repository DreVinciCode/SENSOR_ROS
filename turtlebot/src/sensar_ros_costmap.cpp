/*****************************
sensar_ros_costmap.cpp
SENSAR
Andre Cleaver
Tufts University
*****************************/

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <sstream>
#include <string>

using namespace std;

const std::string TOPIC_IN  = "/move_base/local_costmap/costmap";
const std::string TOPIC_UPDATE = "/move_base/local_costmap/costmap_updates";
const std::string TOPIC_OUT = "/SENSAR/costmap";
const std::string FRAME_IN  = "odom";
const std::string FRAME_OUT = "base_link";

const int FREQUENCY = 1.8;

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

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& inMsg)
{
    latestMsg = *inMsg;
}

int getIndex(int x, int y)
{
	int sx = latestMsg.info.width;
	return y * sx + x;
}

void costmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& inMsg)
{
	int index = 0;
	for(int y=inMsg->y; y< inMsg->y+inMsg->height; y++)
	{
		for(int x=inMsg->x; x< inMsg->x+inMsg->width; x++)
		{
			latestMsg.data[getIndex(x,y)] = inMsg->data[index++];
		}
	}

	publishLatest();
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "sensar_ros_costmap");
    ros::NodeHandle n;
 
    relativePub = n.advertise<nav_msgs::OccupancyGrid>(TOPIC_OUT, 5);
    ros::Subscriber globalSub  = n.subscribe(TOPIC_IN, 5, costmapCallback);
    ros::Subscriber globalSubUpdate = n.subscribe(TOPIC_UPDATE, 5, costmapUpdateCallback);
    
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
