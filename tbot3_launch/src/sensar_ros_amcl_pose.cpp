/*****************************
sensar_ros_amcl_pose.cpp
SENSAR
Andre Cleaver
Tufts University
*****************************/

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

const std::string TOPIC_IN  = "/amcl_pose";
const std::string TOPIC_OUT = "/SENSAR/amcl_pose";
const int FREQUENCY = 30;

geometry_msgs::PoseWithCovarianceStamped latestMsg;
geometry_msgs::PoseWithCovarianceStamped previousMsg;
ros::Publisher amcl_posePub;

geometry_msgs::PoseWithCovarianceStamped poseTransformation(geometry_msgs::PoseWithCovarianceStamped input)
{
    geometry_msgs::PoseWithCovarianceStamped message = input; 
    return message;
}

void publishLatest()
{
    amcl_posePub.publish(poseTransformation(latestMsg));
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& inMsg)
{
    latestMsg = *inMsg;

    if (latestMsg.pose != previousMsg.pose)
    {
        previousMsg = latestMsg;
        publishLatest();
    }
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "sensar_ros_amcl_pose");
    ros::NodeHandle n;

    amcl_posePub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(TOPIC_OUT,5);
    ros::Subscriber globalSub  = n.subscribe(TOPIC_IN, 5, poseCallback);

    ros::Rate rate(FREQUENCY);

    while(ros::ok())
    {
        ros::spinOnce();    
        publishLatest();
        rate.sleep();
    }

    return 0;
}