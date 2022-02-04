/*****************************
sensar_ros_pointcloud.cpp
SENSAR
Andre Cleaver
Tufts University
*****************************/

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <sstream>
#include <string>

using namespace std;

const std::string TOPIC_IN  = "/camera/depth/points";
const std::string TOPIC_OUT = "/SENSAR/pointcloud";
const std::string FRAME_IN  = "camera_rgb_optical_frame";
const std::string FRAME_OUT = "base_link";

const int FREQUENCY = 1.8;

sensor_msgs::PointCloud2 latestMsg;
geometry_msgs::TransformStamped transformToBase_link;

ros::Publisher relativePub;

sensor_msgs::PointCloud2 transformLocalization(sensor_msgs::PointCloud2 input)
{
    sensor_msgs::PointCloud2 transformed = input; 

    return transformed;
}

void publishLatest()
{
    relativePub.publish(transformLocalization(latestMsg));
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    latestMsg = *cloud_msg;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "sensar_ros_pointcloud");
    ros::NodeHandle n;
 
    relativePub = n.advertise<sensor_msgs::PointCloud2>(TOPIC_OUT, 5);
    ros::Subscriber globalSub  = n.subscribe(TOPIC_IN, 5, pointCloudCallback);
    
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
