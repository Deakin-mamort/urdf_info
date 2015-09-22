#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;
using namespace boost;

int MAX_JOINTS = 50;

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
	
   	
  ros::NodeHandle node;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    const char *args[] = { "head", "right_upper_shoulder", "right_lower_shoulder", "right_upper_elbow", "right_lower_elbow", "right_upper_forearm", "right_lower_forearm", "right_wrist",
						   "left_upper_shoulder", "left_lower_shoulder", "left_upper_elbow", "left_lower_elbow", "left_upper_forearm", "left_lower_forearm", "left_wrist"};
						   
	int s = sizeof(args)/sizeof(*args);
	std::vector<std::string> links(args, args+s);
    
    //error first round    
    try{
      listener.lookupTransform("/base", "/head",ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
   
    }
    
    //Obtain coordinates of joints
	for(int i=0; i < links.size(); i++)
    {
		
		listener.lookupTransform("/base", links[i],ros::Time(0), transform);
		
		cout << transform.getOrigin().x() << endl;
		cout << transform.getOrigin().y() << endl;       
		cout << transform.getOrigin().z() << endl;
	}
	
	//fill remaining inputs with zeo
	for(int i = s; i < MAX_JOINTS; i++)
    {
		
		//listener.lookupTransform("/base", links[i],ros::Time(0), transform);
		
		//cout << setw(25) << left << links[i] << '\t';
		cout << "0" << endl;
		cout << "0" << endl;
		cout << "0" << endl;
	}
	
  return 0;
};
