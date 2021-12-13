/*****************************
sensar_ros_map.cpp
SENSAR
Andre Cleaver
Tufts University
*****************************/

#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include <sstream>
#include <string>

using namespace std;

const std::string TOPIC_IN  = "/map";
const std::string TOPIC_OUT = "/SENSAR/Map";

const int FREQUENCY = 30;

nav_msgs::OccupancyGrid latestMsg;
nav_msgs::OccupancyGrid previousMsg;

ros::Publisher relativePub;

nav_msgs::OccupancyGrid transformLocalization(nav_msgs::OccupancyGrid input)
{
    nav_msgs::OccupancyGrid transformed = input;    

    return transformed;
}

void publishLatest()
{
    relativePub.publish(transformLocalization(latestMsg));
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& inMsg)
{
    latestMsg = *inMsg;
    if (latestMsg != previousMsg)
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
    
    ros::Rate rate(FREQUENCY);

    ros::spin();    
    
    return 0;
}

