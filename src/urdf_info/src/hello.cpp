//This is a ROS versiojn of the standard "hello, world" program
//This header defines the standard ROS classes

#include <ros/ros.h>

int main(int argc, char **argv)
{
	//initialize the ROS system
	ros::init(argc, argv, "hello_ros");
	
	//Establish the program as a ROS node
	ros::NodeHandle nh;
	
	//Send some output as a log message
	ROS_INFO_STREAM("Hellow, ROS!");
}
