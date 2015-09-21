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


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
	
   	
  ros::NodeHandle node;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    const char *args[] = { "head", "right_upper_shoulder", "right_lower_shoulder", "right_upper_elbow", "right_lower_elbow", "right_upper_forearm", "right_lower_forearm", "right_wrist",
						   "left_upper_shoulder", "left_lower_shoulder", "left_upper_elbow", "left_lower_elbow", "left_upper_forearm", "left_lower_forearm", "left_wrist"};
	std::vector<std::string> links(args, args+15);
    
    //error first round    
    try{
      listener.lookupTransform("/base", "/head",ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
   
    }
   
    for(int i=0; i < links.size(); i++)
    {
		//error first round    
		try{
			listener.lookupTransform("/base", links[i],ros::Time(0), transform);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		
		cout << setw(25) << left << links[i] << '\t';
		cout << setw(10) << left << transform.getOrigin().x() << '\t';      
		cout << setw(10) << left << transform.getOrigin().y() << '\t';      
		cout << setw(10) << left << transform.getOrigin().z() << endl;
	}
	
  return 0;
};
