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

int MAX_JOINTS = 55;

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
      moveit::planning_interface::MoveGroup group("body");
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
                ros::Duration(1).sleep();
            }
            else{i--;}//cout<< "Bad Move" << endl;}
        }
        else {i--;}//cout<< "Bad Plan" <<endl;}
      }
}

void coords(){
/*
    //Baxter
    const char *args[] = { "head",
                           "right_upper_shoulder",
                           "right_lower_shoulder",
                           "right_upper_elbow",
                           "right_lower_elbow",
                           "right_upper_forearm",
                           "right_lower_forearm",
                           "right_wrist",
                           "left_upper_shoulder",
                           "left_lower_shoulder",
                           "left_upper_elbow",
                           "left_lower_elbow",
                           "left_upper_forearm",
                           "left_lower_forearm",
                           "left_wrist"};

    //ABB & FANUC Manipulators
    const char *args[] = { "link_1",
                           "link_2",
                           "link_3",
                           "link_4",
                           "link_5",
                           "link_6",
                           "tool0"};

    //NAO
    const char *args[] = {"Head",
                          "Neck",
                          "LAnklePitch",
                          "l_ankle",
                          "LForeArm",
                          "LElbow",
                          "LFinger11_link",
                          "LFinger12_link",
                          "LFinger13_link",
                          "LFinger21_link",
                          "LFinger22_link",
                          "LFinger23_link",
                          "l_gripper",
                          "LThigh",
                          "LHip",
                          "LPelvis",
                          "LTibia",
                          "LShoulder",
                          "LBicep",
                          "LThumb1_link",
                          "LThumb2_link",
                          "l_wrist",
                          "RAnklePitch",
                          "r_ankle",
                          "RForeArm",
                          "RElbow",
                          "RFinger11_link",
                          "RFinger12_link",
                          "RFinger13_link",
                          "RFinger21_link",
                          "RFinger22_link",
                          "RFinger23_link",
                          "r_gripper",
                          "RThigh",
                          "RHip",
                          "RPelvis",
                          "RTibia",
                          "RShoulder",
                          "RBicep",
                          "RThumb1_link",
                          "RThumb2_link",
                          "r_wrist"};



    //ROMEO
    const char *args[] = {"HeadPitch_link",                
			  "HeadRoll_link",                
			  "LAnklePitch_link",     
			  "l_ankle",
			  "LForeArm",
			  "LElbow",
			  "LFinger12_link",        
			  "LFinger13_link",            
			  "LFinger21_link",                
			  "LFinger22_link",               
			  "LFinger23_link",           
			  "LFinger31_link",                
			  "LFinger32_link",               
			  "LFinger33_link",                
			  "l_gripper",
			  "LThigh",
			  "LHip", 
			  "LHipYaw_link",              
			  "LTibia",
			  "LShoulder",
			  "LShoulderYaw_link",             
			  "LThumb1_link",                  
			  "LThumb2_link",                 
			  "LThumb3_link",                 
			  "l_wrist",
			  "LWristRoll_link",               
			  "LWristYaw_link",                
			  "NeckPitch_link",                
			  "NeckYaw_link",                  
			  "RAnklePitch_link",     	
			  "r_ankle",
			  "RForeArm",
			  "RElbow",
			  "RFinger12_link",        
			  "RFinger13_link",            
			  "RFinger21_link",                
			  "RFinger22_link",               
			  "RFinger23_link",           	
			  "RFinger31_link",                
			  "RFinger32_link",               
			  "RFinger33_link",               
			  "r_gripper",
			  "RThigh",
			  "RHip", 
			  "RHipYaw_link",             
			  "RTibia",
			  "RShoulder",
			  "RShoulderYaw_link",             
			  "RThumb1_link",                  
			  "RThumb2_link",                 
			  "RThumb3_link",                 
			  "r_wrist",
			  "RWristRoll_link",               
			  "RWristYaw_link",                
			  "body"};
*/

// REEMC-FULLBODY

    const char *args[] = {"arm_left_1_link",         
			  "arm_left_2_link",         
			  "arm_left_3_link",         
			  "arm_left_4_link",         
			  "arm_left_5_link",         
			  "arm_left_6_link",         
			  "arm_left_7_link",         
			  "arm_right_1_link",        
			  "arm_right_2_link",        
			  "arm_right_3_link",        
			  "arm_right_4_link",        
			  "arm_right_5_link",        
			  "arm_right_6_link",        
			  "arm_right_7_link",        
			  "hand_left_index_1_link",  
			  "hand_left_index_2_link",  
			  "hand_left_index_3_link",  
			  "hand_left_index_link",    
			  "hand_left_middle_1_link", 
			  "hand_left_middle_2_link", 
			  "hand_left_middle_3_link", 
			  "hand_left_middle_link",   
			  "hand_left_thumb_link",    
			  "hand_right_index_1_link",  
			  "hand_right_index_2_link", 
			  "hand_right_index_3_link", 
			  "hand_right_index_link",   
			  "hand_right_middle_1_link",
			  "hand_right_middle_2_link",
			  "hand_right_middle_3_link",
			  "hand_right_middle_link",  
			  "hand_right_thumb_link",   
			  "head_1_link",             
			  "head_2_link",             
			  "leg_left_1_link",         
			  "leg_left_2_link",         
			  "leg_left_3_link",         
			  "leg_left_4_link",         
			  "leg_left_5_link",         
			  "leg_left_6_link",         
			  "leg_right_1_link",        
			  "leg_right_2_link",        
			  "leg_right_3_link",        
			  "leg_right_4_link",        
			  "leg_right_5_link",        
			  "leg_right_6_link",        
			  "torso_1_link",            
			  "torso_2_link"};

    int s = sizeof(args)/sizeof(*args);
    std::vector<std::string> links(args, args+s);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::Matrix3x3 m;
    double r,p,y;
    fstream file("joints.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    //error first round?
/*
    try{
      listener.lookupTransform("/base_link", "/head",ros::Time(0), transform);
      ros::Duration(1.0).sleep();
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();

    }
*/
    //Obtain coordinates of joints
    for(int i=0; i < links.size(); i++){
        ros::Duration(0.2).sleep();
        listener.lookupTransform("/base_link", links[i],ros::Time(0), transform);

        file << transform.getOrigin().x() << ", ";
        file << transform.getOrigin().y() << ", ";
        file << transform.getOrigin().z() << ", ";
        m = transform.getBasis();
        m.getRPY(r,p,y);
        file << r << ", ";
        file << p << ", ";
        file << y << ", ";
        //llros::Duration(1.0).sleep();

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
