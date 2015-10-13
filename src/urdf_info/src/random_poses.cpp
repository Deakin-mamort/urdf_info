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

void printActiveJoints (vector<moveit::core::JointModel *> jointList);
void printJointGroups (vector<string> jointGroups);

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
      //state.setToRandomPositions();
      //state.update();

      //Assign group
      moveit::planning_interface::MoveGroup group("both_arms");

      //Create planning interface
      //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      //Publish visualization plans to RVIZ
      //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
      //moveit_msgs::DisplayTrajectory display_trajectory;

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
      */

      moveit::planning_interface::MoveGroup::Plan my_plan;
      bool success=false;
      int good=0;
      int bad=0;

      //Run Random Poses
      for (int i=0; i<10; i++){
        group.setRandomTarget();
        success = group.plan(my_plan);

        if(success==1){
            //test=1;
            success = group.move();
            if (success == 1) {
                cout<< endl << "Good Move" << endl << endl;
                ros::Duration(30).sleep();
            }
            else{cout<< endl << "Bad Move" << endl << endl;}
        }
        else {cout<< endl << "Bad Plan" << endl << endl;}

        //vector<double> group_variable_values;
        //group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
      }
}
//list active joints
void printActiveJoints(vector<moveit::core::JointModel *> jointList){
    int i = 0;
    for (vector<moveit::core::JointModel *>::iterator it = jointList.begin(); it != jointList.end(); it++)
    {
        cout << jointList[i]->getName() << endl;
        i++;
    }
    cout << endl;
}

//list joint groups
void printJointGroups (vector<string> jointGroups){
    for (vector<string>::iterator it = jointGroups.begin(); it != jointGroups.end(); it++){cout << *it << endl;}
}
