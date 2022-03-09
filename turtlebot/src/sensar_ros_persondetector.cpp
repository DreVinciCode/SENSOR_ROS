/*****************************
sensar_ros_persondetector.cpp
SENSAR
Andre Cleaver
Tufts University
*****************************/

#include "people_msgs/PositionMeasurement.h"
#include "people_msgs/PositionMeasurementArray.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <sstream>
#include <string>

using namespace std;

const std::string TOPIC_IN  = "/people_tracker_measurements";
const std::string TOPIC_OUT = "/SENSAR/people_detector_marker";
const std::string FRAME_IN  = "map";
const std::string FRAME_OUT = "base_link";

std::string rosparam = "/SENSAR/person_threshold";

people_msgs::PositionMeasurementArray latestMsg;
ros::Publisher vis_pub;

const int FREQUENCY = 50;

void publishLatest()
{
	visualization_msgs::MarkerArray markerArray;

    for(int i = 0; i < latestMsg.people.size(); i++)
    {
		people_msgs::PositionMeasurement leg = latestMsg.people[i];
		
		visualization_msgs::Marker marker;
		marker.header.frame_id = FRAME_OUT;
		marker.header.stamp = ros::Time();
		marker.ns = "PEOPLE";
		marker.id = i;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
			
		marker.pose.position.x = leg.pos.x;
		marker.pose.position.y = leg.pos.y;
		marker.pose.position.z = leg.pos.z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		marker.color.a = 1.0; 
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		marker.text = std::to_string(leg.reliability);
		
		markerArray.markers.push_back(marker);
	}
	
    vis_pub.publish(markerArray);  
}

people_msgs::PositionMeasurementArray filteredArray(people_msgs::PositionMeasurementArray input)
{
    people_msgs::PositionMeasurementArray filteredList;

    for(int i = 0; i < input.people.size(); i++)
    {
		float rel = input.people[i].reliability;
		
		double threshold;
		ros::param::get(rosparam, threshold);
		
		if(rel > threshold)
		{
			filteredList.people.push_back(input.people[i]);
		}	
	}	
	
	return filteredList;
}

void legdetectorCallback(const people_msgs::PositionMeasurementArray::ConstPtr& inMsg)
{
    latestMsg = filteredArray(*inMsg); 		
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "sensar_ros_persondetector");
    ros::NodeHandle n;
    
    n.setParam(rosparam, -1.0);
    
    vis_pub = n.advertise<visualization_msgs::MarkerArray>(TOPIC_OUT, 5);
    ros::Subscriber globalSub  = n.subscribe(TOPIC_IN, 5, legdetectorCallback);
    
    ros::Rate rate(FREQUENCY);
    
    while(ros::ok())
    {
        ros::spinOnce();    
        publishLatest();
        rate.sleep();
    }
    
    return 0;
}
