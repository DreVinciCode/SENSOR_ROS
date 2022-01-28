/*****************************
sensar_ros_point.cpp
SENSAR
Andre Cleaver
Tufts University
*****************************/

using namespace std;

const std::string TOPIC_IN  = "/clicked_point";
const std::string TOPIC_OUT = "/SENSAR/point";
const std::string FRAME_IN  = "map";
const std::string FRAME_OUT = "base_link";

geometry_msgs::TransformStamped transformToBase_link;
