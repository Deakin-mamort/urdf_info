#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

using namespace std;

int main(int argc, char **argv)
{
      int i = 0;
	  bool test=0;
	  ros::init(argc, argv, "move_group_interface_tutorial");
	  ros::NodeHandle node_handle;
	  ros::AsyncSpinner spinner(1);
	  spinner.start();

	  //Allow time for RVIZ to load
	  //sleep(20.0);
      vector<moveit::core::JointModel *> joints;

      //Obtain model from rosparam
      robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
      //Load model
      robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
      //Load model state
      robot_state::RobotState state = moveit::core::RobotState(kinematic_model);
      //Obtain joint group list (serial chain list)
      vector<string> groups = kinematic_model->getJointModelGroupNames();
      //Obtain active joint list
      joints = kinematic_model->getActiveJointModels();
      state.setToRandomPositions();
      state.update();

      //list groups

      //for (vector<string>::iterator it = groups.begin(); it != groups.end(); it++) {cout << *it << endl;}
      //cout << (joints[it]->getName()) << endl;

      for (vector<moveit::core::JointModel *>::iterator it = joints.begin(); it != joints.end(); it++)
      {
          cout << joints[i]->getName() << endl;
          i++;
      }
      //cout << endl;
      //for (vector<moveit::core::JointModel *>::iterator it = joints.begin(); it != joints.end(); it++) {cout << *it << endl;}

      //Assign group
	  moveit::planning_interface::MoveGroup group("right_arm");

	  //Create planning interface
	  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	  //Publish visualization plans to RVIZ
	  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	  moveit_msgs::DisplayTrajectory display_trajectory;

      /*
	  //Publish information
      ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
      ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
      ROS_INFO(group.getName().c_str());
      ROS_INFO(group.getEndEffector().c_str());

	  //Set plan
      geometry_msgs::Pose target_pose1;
      target_pose1.orientation.w = 0.1779;
      target_pose1.position.x = 0.4618;
      target_pose1.position.y = 0.5103;
      target_pose1.position.z = -0.5057;
      group.setPoseTarget(target_pose1, "left_gripper");
	  group.setRandomTarget();

      //Visualize Pre-Plan
	  moveit::planning_interface::MoveGroup::Plan my_plan;
	  bool success = group.plan(my_plan);
	  	  if(success==1){test=1;}
	  success = group.move();
	  	  if(success==1){test=1;}
	  	  else {test=0;}

      vector<double> group_variable_values;
	    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

	  if(test==1)
          cout<< endl << "Good" << endl << endl;
	  else
          cout<< endl << "Bad" << endl << endl;
*/
}


