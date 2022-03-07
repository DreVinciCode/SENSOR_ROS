/*****************************
sensar_ros_point.cpp
SENSAR
Andre Cleaver
Tufts University
*****************************/

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <sstream>
#include <string>

using namespace std;

const std::string TOPIC_IN  = "/SENSAR/random_point";
const std::string TOPIC_OUT = "/SENSAR/point";
const std::string TOPIC_CLICKED_POINT = "/clicked_point";
const std::string FRAME_IN  = "map";
const std::string FRAME_OUT = "base_link";

geometry_msgs::TransformStamped transformToBase_link;
geometry_msgs::PointStamped latestMsg;
geometry_msgs::PointStamped previousMsg;
ros::Publisher relativePub;
ros::Publisher clickedPointPub;

const int FREQUENCY = 1.8;
bool receivedMessage = false;

geometry_msgs::PointStamped transformLocalization(geometry_msgs::PointStamped input)
{
    geometry_msgs::PointStamped transformed = input; 
    transformed.header.frame_id = FRAME_OUT;

	geometry_msgs::PoseStamped point;
	point.pose.position = transformed.point;
	tf2::doTransform(point, point, transformToBase_link);
	transformed.point = point.pose.position;

    return transformed;
}

void publishLatest()
{
    relativePub.publish(transformLocalization(latestMsg));
    clickedPointPub.publish(transformLocalization(latestMsg));
    receivedMessage = false;
}

void pointCallback(const geometry_msgs::PointStamped::ConstPtr& inMsg)
{
    latestMsg = *inMsg;
    receivedMessage = true;

}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "sensar_ros_point");
    ros::NodeHandle n;
 
    relativePub = n.advertise<geometry_msgs::PointStamped>(TOPIC_OUT, 5);
    clickedPointPub = n.advertise<geometry_msgs::PointStamped>(TOPIC_CLICKED_POINT, 5);
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

        if(receivedMessage)
        {
            publishLatest();
        }   

        rate.sleep();
    }
    
    return 0;
}
