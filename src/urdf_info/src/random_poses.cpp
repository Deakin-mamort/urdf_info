#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <fstream>

using namespace std;
using namespace boost;

int MAX_JOINTS = 50;

void printActiveJoints (vector<moveit::core::JointModel *> jointList);
void printJointGroups (vector<string> jointGroups);
void coords();

int main(int argc, char **argv)
{     
      // Check the number of parameters
      if (argc < 2) {
          //std::cerr << "Usage: " << argv[0] << " NAME" << std::endl;
          cout << "requires no of poses" << endl;
          return 1;
      }

      ros::init(argc, argv, "random_poses");
      ros::AsyncSpinner spinner(1);
      spinner.start();
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

      //Assign group
      moveit::planning_interface::MoveGroup group("manipulator");
      moveit::planning_interface::MoveGroup::Plan my_plan;
      bool success=false;

      int poses = atoi(argv[1]);
      //Run Random Poses
      for (int i=0; i<poses; i++){
        group.setRandomTarget();
        success = group.plan(my_plan);

        if(success==1){
            success = group.move();
            if (success == 1) {
                //cout << "Good Move" << endl;
                coords();
                ros::Duration(10).sleep();
            }
            else{i--;}//cout<< "Bad Move" << endl;}
        }
        else {i--;}//cout<< "Bad Plan" <<endl;}
      }
}

void coords(){

    //Baxter
    //const char *args[] = { "head", "right_upper_shoulder", "right_lower_shoulder", "right_upper_elbow", "right_lower_elbow", "right_upper_forearm", "right_lower_forearm", "right_wrist",
    //                       "left_upper_shoulder", "left_lower_shoulder", "left_upper_elbow", "left_lower_elbow", "left_upper_forearm", "left_lower_forearm", "left_wrist"};

    const char *args[] = { "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "tool0" };
    int s = sizeof(args)/sizeof(*args);
    std::vector<std::string> links(args, args+s);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::Matrix3x3 m;
    double r,p,y;
    fstream file("joints.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    ros::Duration(1.0).sleep();
    //error first round?
    try{
      listener.lookupTransform("/base_link", "/link_1",ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();

    }

    //Obtain coordinates of joints
    for(int i=0; i < links.size(); i++){

        listener.lookupTransform("/base_link", links[i],ros::Time(0), transform);

        file << transform.getOrigin().x() << ", ";
        file << transform.getOrigin().y() << ", ";
        file << transform.getOrigin().z() << ", ";
        m = transform.getBasis();
        m.getRPY(r,p,y);
        file << r << ", ";
        file << p << ", ";
        file << y << ", ";
        ros::Duration(1.0).sleep();

    }

    //fill remaining inputs with zeo
    for(int i = s; i < MAX_JOINTS-1; i++){
        file << "0" << ", ";
        file << "0" << ", ";
        file << "0" << ", ";
        file << "0" << ", ";
        file << "0" << ", ";
        file << "0" << ", ";
    }
    file << "0" << ", ";
    file << "0" << ", ";
    file << "0" << ", ";
    file << "0" << ", ";
    file << "0" << ", ";
    file << "0" << endl;
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
