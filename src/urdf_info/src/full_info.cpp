#include "urdf_model/model.h"
#include "urdf_parser/urdf_parser.h"
#include "kdl_parser/kdl_parser.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>


#define redtxt	"\x1b[1;32m"
#define bluetxt "\x1b[1;34m"
#define magentatxt "\x1b[1;35m"
#define cyantxt "\x1b[1;36m"
#define whitetxt "\x1b[1;37m"
#define greentxt "\x1b[1;32m"
#define lightgreentxt "\x1b[0;32m"
#define ascii_arrow "\u2514 "

using namespace urdf;
using namespace std;
using namespace boost;

string jtypeToString(int type)
{
	string name;
	
	switch(type)
	{
		case 1:
			name = "Revolute";
			break;
		case 2:
			name = "Continous";
			break;
		case 3:
			name = "Prismatic";
			break;
		case 4:
			name = "Floating";
			break;
		case 5:
			name = "Planar";
			break;
		case 6:
			name = "Fixed";
			break;
		default:
			name = "UNKNOWN";
	}
	
	return name;
}

void printJointList(shared_ptr<ModelInterface> robot)
{
	cout << bluetxt << "<-------------------- Joint List -------------------->" << whitetxt << endl;
	int count=0;
	for (map<string,shared_ptr<Joint> >::iterator joint = robot->joints_.begin();joint != robot->joints_.end(); joint++)
	{
			string joint_type = jtypeToString(joint->second->type);
			count++;
			if(joint_type != "Fixed" && joint_type != "UNKNOWN")
			{
			cout << magentatxt 
				 //<< "Joint " 
				 //<< left << setw(2) 
				 //<< count 
				 //<< ":   " 
				 << whitetxt << setw(25) << left 
				 << joint->second->name << "," 
				 << setw(20) << left 
				 << joint_type << ","
				 << joint->second->axis.x << ", " 
				 << joint->second->axis.y << ", "
				 << setw(10) << left 
				 << joint->second->axis.z << ","
				 << setw(10) << left 
				 << joint->second->parent_to_joint_origin_transform.position.x << ", "
				 << setw(10) << left 
				 << joint->second->parent_to_joint_origin_transform.position.y << ", "
				 << setw(10) << left 
				 << joint->second->parent_to_joint_origin_transform.position.z << ", "
				 << setw(10) << left 
				 << joint->second->parent_to_joint_origin_transform.rotation.x << ", "
				 << setw(10) << left 
				 << joint->second->parent_to_joint_origin_transform.rotation.y << ", "
				 << setw(10) << left 
				 << joint->second->parent_to_joint_origin_transform.rotation.z << " "
				 <<endl;		
			 }
			 
	}
}
void printLinkList(shared_ptr<ModelInterface> robot)
{
	cout << bluetxt << "<-------------------- Link List -------------------->" << whitetxt << endl;
	int count=0;
	for (map<string,shared_ptr<Link> >::iterator link = robot->links_.begin();link != robot->links_.end(); link++)
	{
			count++;
			cout << magentatxt << "Link " << count << ": " << whitetxt << link->first << endl;
	}
}
void printRobotInfo(shared_ptr<ModelInterface> robot)
{
	shared_ptr<const Link> root_link = robot->getRoot();
	//if (!root_link) return -1;
	
	cout << bluetxt << "<----------------- Main Description ----------------->" << whitetxt << endl;
	cout << magentatxt << "Robot Name is: \t" << whitetxt << robot->name_ << endl;
	cout << magentatxt << "Root Link Name: " << whitetxt << root_link->name << endl;
	cout << magentatxt << "Total Joints: \t" << whitetxt << robot->joints_.size() << endl;
	cout << magentatxt << "Total Links: \t" << whitetxt << robot->links_.size() << endl;
}
void printLinkTree(shared_ptr<const Link> link,int level = 0)
{	
	level+=2;
	int count = 0;
	for (vector<shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
	{
		if (*child)
		{
			for(int j=0;j<level;j++) cout << " "; //indent
			cout << magentatxt << "child(" << (count++)+1 << "): " << whitetxt << (*child)->name << endl;
			// first grandchild
			printLinkTree(*child,level);
		}
		
		else
		{
			for(int j=0;j<level;j++) cout << " "; //indent
			cout << redtxt << "root link: " << link->name << " has a null child!" << *child << whitetxt << endl;
		}
	}
}
void printLinkBranch(shared_ptr<const Link> link, shared_ptr<ModelInterface> robot, bool sublink = false)
{
		string prev_child_link_name;
		
		for (vector<shared_ptr<Joint> >::const_iterator joint = link->child_joints.begin(); joint != link->child_joints.end(); joint++) 
		{
			string joint_type = jtypeToString((*joint)->type);

			//if((*joint)->type == 6 )
			//{
				//Set names
				string parent_link_name = (*joint)->parent_link_name;
				string child_link_name = (*joint)->child_link_name;
						
				//Assign next link
				shared_ptr<Link> child_link;
				robot->getLink(child_link_name, child_link);
			
				if (sublink==true){cout << ascii_arrow;}
				cout << greentxt << parent_link_name << whitetxt << " -> " << lightgreentxt << (*joint)->name << whitetxt;
			
				//Print branch
				if (child_link->child_joints.size()==1)
				{
					cout << " -> ";
					printLinkBranch(child_link, robot);
				}
				//Print sub-branch on a new line
				else if (child_link->child_joints.size()>1)
				{
					cout << endl;
					printLinkBranch(child_link, robot, true);
				}
				//Print last link of branch
				else {cout << " -> " << greentxt << child_link_name << whitetxt << endl;}
			//}
		}
}
void chainBegin(shared_ptr<ModelInterface> robot)
{
	for (map<string,shared_ptr<Joint> >::iterator joint = robot->joints_.begin();joint != robot->joints_.end(); joint++)
	{

		//Determine previous link joint
		boost::shared_ptr<const Link> parent_link = robot->getLink(joint->second->parent_link_name);
		boost::shared_ptr<Joint> next_joint;
		boost::shared_ptr<const Joint> prev_joint = parent_link->parent_joint;

		//Active Joint NOT "FIXED" AND "UNKNOWN"?
		if(joint->second->type != 6 && joint->second->type != 0 )
		{

			//Previous Joint is "FIXED"?
			if(prev_joint->type == 6)
			{
				//Print Parent Link (LIGHT GREEN)
				cout  << lightgreentxt << parent_link->name <<whitetxt ;


				next_joint=joint->second;

				//while joint type NOT "FIXED" cycle
				while(next_joint->type != 6)
				{

					boost::shared_ptr<Joint> joint = next_joint;
					boost::shared_ptr<const Link> link = robot->getLink(next_joint->child_link_name);

					//Print current Joint and Link name
 					cout << " -> " << greentxt << joint->name << whitetxt << " -> " << lightgreentxt << link->name << whitetxt;

 					//cycle child joints assign last active joint as next joint in chain
 					//****BUG**** = doesn't work with multiple active child joints on a single link
 					for (vector<shared_ptr<Joint> >::const_iterator child_joint = link->child_joints.begin(); child_joint != link->child_joints.end(); child_joint++)
 					{

 						//****BUG**** if joint active assign
 						if ((*child_joint)->type !=6)
 							next_joint = (*child_joint);
 					}

 					//if not active child joints
 					if (joint == next_joint)
 					{
 						cout <<endl;
 						break;
 					}
				}

			}

		}

	}
}
void chainEnd(shared_ptr<const Link> link, shared_ptr<ModelInterface> robot)
{

	for (vector<shared_ptr<Joint> >::const_iterator joint = link->child_joints.begin(); joint != link->child_joints.end(); joint++)
	{
		string child_link_name = (*joint)->child_link_name;

		shared_ptr<Link> child_link;
		robot->getLink(child_link_name, child_link);

		cout << (*joint)->name << child_link->name << endl;

	}



	//Determine ending Joints
	/*
	if(joint->second->type == 6 && parent_link && prev_joint )
	{
		//Determine if previous joint is fixed to determine start of chain
		if(prev_joint->type == 1)
		{
			//Print 1st fixed joint, link and 1st movable joint (start of chain pattern)
			cout << prev_joint->name << "\t" << parent_link->name << "\t" << joint->second->name << endl;
		}

	}
	*/
}

int main(int argc, char** argv)
{
	//Check to see if xml file provide
	if (argc != 2){
		cerr << redtxt << "Usage: full_info input.xml" << whitetxt << endl;
		return -1;
	}
	
	// get the entire file
	std::string xml_string;
	fstream xml_file(argv[1], std::fstream::in);
	
	while ( xml_file.good() )	
	{
		string line;
		getline( xml_file, line);
		xml_string += (line + "\n");
	}
	xml_file.close();
	
	//Check for good robot model, exit if invalid
	shared_ptr<ModelInterface> robot1 = parseURDF(xml_string);
	if (!robot1){
		cerr << redtxt << "ERROR: Model Parsing the xml failed" << whitetxt << endl;
		return -1;
	}	


	//Obtain KDL tree, exit if invalid
	KDL::Tree kdl_tree;
/*
	if (!kdl_parser::treeFromString(xml_string, kdl_tree)){
		cerr << redtxt << ("Failed to construct kdl tree") << whitetxt << endl;
		return -1;
    }

	else
	{
		int joints = kdl_tree.getNrOfJoints();
		cout << joints << endl;
	}
*/


	//Obtain root link
	shared_ptr<const Link> root_link = robot1->getRoot();
	if (!root_link) return -1;
	
	//Print robot information
	//printRobotInfo(robot1);
	//cout << magentatxt << "Total Segments: " << whitetxt << kdl_tree.getNrOfSegments() << endl;
	//printJointList(robot1);
	//printLinkList(robot1);
	//cout << bluetxt << "<------------------- Robot Tree ------------------->" << whitetxt << endl;
	//printLinkTree(root_link);
	//cout << bluetxt << "<----------------- Robot Branches ----------------->" << whitetxt << endl;
	//printLinkBranch(root_link, robot1);
	chainBegin(robot1);
	//chainEnd(parent_link, robot1);
}
