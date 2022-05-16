# SENSAR_ROS

Welcome to SENSAR! (Seeing Everything iN Situ with Augmented Reality)
In this project, we created a visualization tool that displays various robotoic information in context of the environment with the use of augmented reality technology.
For anyone who may encounter a new robot for the first time it is natural that they may be hesitent to be near a robot. People will wonder what is the robot doing? Can I walk near or pass the robot? Is the robot broken? Should I be doing something? With SENSAR, we hope to build a person's willingness to interact with the robot by making the robot transparent with its intentions and observations. By gazing at the robot, a person will be able to obtain the information they might seek as it is visualized over the real world. 
SENSAR was developed with Unity and utilizes Vuforia Engine to track the robot's pose. This repo works with the Unity project [SENSAR](https://github.com/DreVinciCode/SENSAR) to visualize incomming robotic data. 
The scripts provided in this repo runs specific scripts to publish filited and transformed data to the AR device.

Robotic data that SENSAR can currently display include:
- Navigation Planner
- Person Detector
- Lidar laserscan
- OccupancyGrid
- Safefy Clearance
- Localization Points
- Loaded Map of Environment
- Diagnostic Information


Adding future arduino components; install with the following:
$ sudo apt-get install ros-kinetic-rosserial-arduino ros-kinetic-rosserial ros-kinetic-rosserial-embeddedlinux ros-kinetic-rosserial-windows ros-kinetic-rosserial-server ros-kinetic-rosserial-python

Install rosbridge_suite through commandline:
sudo apt-get install ros-noetic-rosbridge-suite

May need to install people_msgs:
sudo apt-get install ros-kinetic-people-msgs


