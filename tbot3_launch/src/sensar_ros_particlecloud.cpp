/*****************************
sensar_ros_particlecloud.cpp
SENSAR
Andre Cleaver
Tufts University
*****************************/

#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"
#include <sstream>
#include <string>

const std::string TOPIC_IN  = "/particlecloud";
const std::string TOPIC_OUT = "/SENSAR/particlecloud";
const std::string FRAME_IN  = "map";
const std::string FRAME_OUT = "base_link";

using namespace std;

const int FREQUENCY = 30;

geometry_msgs::PoseArray latestMsg;
geometry_msgs::PoseArray previousMsg;
ros::Publisher particleCloudPub;

geometry_msgs::PoseArray poseTransformation(geometry_msgs::PoseArray input)
{
    geometry_msgs::PoseArray transformed = input; 
    return transformed;
}

void publishLatest()
{
    particleCloudPub.publish(poseTransformation(latestMsg));
}

void cloudCallback(const geometry_msgs::PoseArray::ConstPtr& inMsg)
{
    latestMsg = *inMsg;
    publishLatest();

}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "sensar_ros_particlecloud");
    ros::NodeHandle n;

    particleCloudPub = n.advertise<geometry_msgs::PoseArray>(TOPIC_OUT,5);
    ros::Subscriber globalSub  = n.subscribe(TOPIC_IN, 5, cloudCallback);

    ros::Rate rate(FREQUENCY);

    while(ros::ok())
    {
        ros::spinOnce();    
        publishLatest();
        rate.sleep();
    }

    return 0;
}
