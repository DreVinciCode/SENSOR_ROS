/*****************************
sensar_ros_path.cpp
SENSAR
Andre Cleaver
Tufts University
*****************************/

#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <sstream>
#include <string>

const std::string TOPIC_IN  = "/move_base/NavfnROS/plan";
const std::string TOPIC_OUT = "/SENSAR/path";
const std::string FRAME_IN  = "map";
const std::string FRAME_OUT = "base_link";

const int REDUCTION_FACTOR  = 2;
const int FREQUENCY         = 5;

geometry_msgs::TransformStamped transformToBase_link;
nav_msgs::Path latestMsg;
ros::Publisher relativePub;

nav_msgs::Path reducePoints(nav_msgs::Path input)
{
    nav_msgs::Path output = input;
    int reduced_size = input.poses.size() / REDUCTION_FACTOR;
    output.poses.resize(reduced_size);

    for(int i = 0; i < reduced_size; i++)
    {
        output.poses[i] = input.poses[i*REDUCTION_FACTOR];
    }

    return output;
}

nav_msgs::Path transformPath(nav_msgs::Path input)
{
    nav_msgs::Path transformed = input;
    transformed.header.frame_id = FRAME_OUT;
    
    for(int i = 0; i < transformed.poses.size(); i++)
    {
        tf2::doTransform(transformed.poses[i], transformed.poses[i], transformToBase_link);
    }

    return transformed;
}

void pathCallback(const nav_msgs::Path::ConstPtr& inMsg)
{
    latestMsg = reducePoints(*inMsg);
}

void publishLatest()
{
    if(latestMsg.poses.size() > 1)
    {
        relativePub.publish(transformPath(latestMsg));
    }
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "sensar_ros_path");
    ros::NodeHandle n;
      
    relativePub = n.advertise<nav_msgs::Path>(TOPIC_OUT, 5);
    ros::Subscriber globalSub  = n.subscribe(TOPIC_IN, 5, pathCallback);
    
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