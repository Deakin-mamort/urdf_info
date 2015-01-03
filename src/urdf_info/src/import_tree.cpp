//Orocos KDL (Kinematic Dynamic Library)
#include "kdl/chain.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/tree.hpp"
#include "/opt/ros/indigo/include/kdl_parser/kdl_parser.hpp"

//URDF (Unified Robot Description Format) Library
#include "urdf_model/model.h"
#include "urdf_parser/urdf_parser.h"

//Standard Library
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

//Namespaces
using namespace boost;
using namespace std;

//Definitions

//Joint Type definitions
#define UNKNOWN 0
#define FIXED 6

//Text color definitions
#define redtxt	"\x1b[1;32m"
#define bluetxt "\x1b[1;34m"
#define magentatxt "\x1b[1;35m"
#define cyantxt "\x1b[1;36m"
#define whitetxt "\x1b[1;37m"
#define greentxt "\x1b[1;32m"
#define lightgreentxt "\x1b[0;32m"
#define ascii_arrow "\u2514 "

//Load file contents and return it in a string.
string getFileContent(char** filename)
{
	//Declare file steam and it's contents
	fstream xml_file(filename[1], fstream::in);
	string file_content;

	//Store file contents into string
	while ( xml_file.good() )
	{
		string line;
		getline( xml_file, line);
		file_content += (line + "\n");
	}
	xml_file.close();

	//Return file contents
	return file_content;
}

//Search URDF model and return root\leaf pairs of active chains
vector< pair< string, string > > getActiveRootLeaf(shared_ptr< urdf::ModelInterface > robot)
{

	vector< pair< string, string > > active_rootleaf_pairs;

	for (map< string, shared_ptr< urdf::Joint > >::iterator joint = robot->joints_.begin();joint != robot->joints_.end(); joint++)
	{

		//Determine previous link joint
		shared_ptr<const urdf::Link> parent_link = robot->getLink(joint->second->parent_link_name);
		shared_ptr<urdf::Joint> current_joint;
		shared_ptr<urdf::Joint> prev_joint = parent_link->parent_joint;

		//Is joint active? If active determine chain
		if(joint->second->type != FIXED && joint->second->type != UNKNOWN )
		{
			string root, leaf;

			//Is previous joint fixed? If fixed assign root of chain
			if(prev_joint->type == FIXED)
			{
				//Root is parent link of 1st active joint
				root = parent_link->name;
				//cout << root;

				//Assign starting joint
				current_joint=joint->second;

				//Search for leaf of active chain
				//***REFINE*** can be done just with for loop.
				//****BUG**** doesn't work with multiple active child joints on a single link
				while(current_joint->type != FIXED)
				{
					shared_ptr<urdf::Joint> joint = current_joint;
					shared_ptr<const urdf::Link> link = robot->getLink(current_joint->child_link_name);

					//Create interator for child joints


					//Search for end joint of chain
 					for (vector< shared_ptr< urdf::Joint > >::const_iterator child_joint = link->child_joints.begin(); child_joint != link->child_joints.end(); child_joint++)
 						if ((*child_joint)->type != FIXED) {current_joint = (*child_joint);}

 					//if no active child joints assign leaf as current joints child link
 					if (joint == current_joint)
 					{
 						leaf = current_joint->child_link_name;
 						//cout << " -> " << leaf << "\n";

 						//Assign active root and leaf pair to vector
 						active_rootleaf_pairs.push_back(std::make_pair(root, leaf));
 						break;
 					}
				}


			}



		}

	}

	return active_rootleaf_pairs;
}

//Create list of active chains from tree using base to active leafs
vector< KDL::Chain > getActiveChains(KDL::Tree tree, vector< pair< string, string > > rootleaf_pairs)
{
	std::vector< KDL::Chain> active_chains;

	for(vector< pair< string, string > >::iterator rootleaf = rootleaf_pairs.begin(); rootleaf !=rootleaf_pairs.end(); rootleaf++ )
	{
		KDL::Chain chain;

		if(tree.getChain("base", (*rootleaf).second, chain)){
				active_chains.push_back(chain);
		}
		else {cerr << "Can't obtain chain! \n";}
	}

	return active_chains;
}

//Solve active chains with Forward Kinematics printing cartesian position
vector< KDL::Frame > getActiveFrames(vector< KDL::Chain > active_chains)
{
	vector< KDL::Frame > active_frames;


	for(vector< KDL::Chain >::iterator chain = active_chains.begin(); chain != active_chains.end(); chain++ )
	{
		KDL::Frame cartPos;
		KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(*chain);
		unsigned int nj = (*chain).getNrOfJoints();
		KDL::JntArray jointpositions = KDL::JntArray(nj);

		if(!fksolver.JntToCart(jointpositions,cartPos)){
			active_frames.push_back(cartPos);
			//cout <<"\n" << "Rotation of End Effector \n" << cartPos.M << "\n";
			//cout <<"\n" << "Position of End Effector \n" << cartPos.p << "\n";
		}

		else {cerr << "Error: could not calculate forward kinematics. \n";}
	}

	return active_frames;
}

vector< pair< double, double > > getWorkspace(vector< KDL::Frame > active_frames)
{
	//Define min\max cartesian values
	double min_x=0, min_y=0, min_z=0;
	double max_x=0, max_y=0, max_z=0;

	//Define robot workspace
	vector< pair< double, double> > robot_workspace;

	for(vector< KDL::Frame >::iterator active_frame = active_frames.begin(); active_frame != active_frames.end(); active_frame++)
	{
		//Find max cartesian values
		if(active_frame->p[0] > max_x){max_x = active_frame->p[0];}
		if(active_frame->p[1] > max_y){max_y = active_frame->p[1];}
		if(active_frame->p[2] > max_z){max_z = active_frame->p[2];}

		//Find min cartesian values
		if(active_frame->p[0] < min_x){min_x = active_frame->p[0];}
		if(active_frame->p[1] < min_y){min_y = active_frame->p[1];}
		if(active_frame->p[2] < min_z){min_z = active_frame->p[2];}
	}

	robot_workspace.push_back(make_pair(min_x, max_x));
	robot_workspace.push_back(make_pair(min_y, max_y));
	robot_workspace.push_back(make_pair(min_z, max_z));

	return robot_workspace;
}


//Prints active chain information to terminal
void printChainInfo(vector< KDL::Frame> active_frames, vector< pair< string, string > > active_rootleaf_pairs, vector< KDL::Chain > active_chains)
{

	int chain_count=1;
	vector< KDL::Chain >::iterator active_chain = active_chains.begin();
	vector< KDL::Frame >::iterator active_frame = active_frames.begin();

	for(vector< pair< string, string> >::iterator active_rootleaf_pair = active_rootleaf_pairs.begin();
		active_rootleaf_pair != active_rootleaf_pairs.end();
		active_rootleaf_pair++)
	{

		//Print active chains information
		cout << bluetxt <<"----------------------- Active Chain " << chain_count << " --------------------------\n" << whitetxt;
		cout << cyantxt <<"Chain DOF:" << whitetxt << active_chain->getNrOfJoints() << "\n";
		cout << cyantxt <<"Active root:" << whitetxt << active_rootleaf_pair->first << "\n";
		cout << cyantxt <<"Active leaf:" << whitetxt << active_rootleaf_pair->second << "\n";

		//Print active chains frame information
		cout << cyantxt << "Active end effector position" << "\n\n" << whitetxt << active_frame->p << "\n\n";
		cout << cyantxt << "Active end effector rotation" << "\n\n" << whitetxt << active_frame->M << "\n\n";

		//Print full active chain joints and links
		//**UPDATE** Make full chain into function that returns string.
		cout << cyantxt << "Full Chain \n" << whitetxt;
		for(int count = 0; count!= active_chain->getNrOfSegments(); count++)
				{
					KDL::Segment segment = active_chain->getSegment(count);
					KDL::Joint joint = segment.getJoint() ;

					cout << greentxt << segment.getName() << whitetxt;
					if (count != active_chain->getNrOfSegments()-1)
						 cout << " -> " << lightgreentxt << joint.getName() << whitetxt << " -> ";

				}
		cout <<"\n\n";

		//increase iterators
		chain_count++;
		active_chain++;
		active_frame++;


	}

	cout << "-----------------------------------------------------------------\n";
}



//Print associated robot information to terminal
void printRobotInfo(shared_ptr< urdf::ModelInterface > robot, vector< KDL::Chain > active_chains, vector< pair< double, double> > robot_workspace)
{

	shared_ptr<const urdf::Link> root_link = robot->getRoot();

	cout << bluetxt << "---------------------- Main Description -------------------------\n" << whitetxt;
	cout << cyantxt << "Robot Name is: " << whitetxt << robot->name_ << "\n";
	cout << cyantxt << "Root Link Name: " << whitetxt << root_link->name << "\n";
	cout << cyantxt << "Total Joints: " << whitetxt << robot->joints_.size() << "\n";
	cout << cyantxt << "Total Links: " << whitetxt << robot->links_.size() << "\n";
	cout << cyantxt << "Total Active Link: " << whitetxt << active_chains.size() <<"\n";
	cout << cyantxt << "Robot Active Workspace:" << "\n"
					<< "Min:"
					<< "[" << robot_workspace[0].first <<",\t"
					<< robot_workspace[1].first <<",\t"
					<< robot_workspace[2].first <<" ]\n"
					<< "Max:"
					<< "[" << robot_workspace[0].second <<",\t"
					<< robot_workspace[1].second <<",\t"
					<< robot_workspace[2].second <<" ]\n";
}

int main(int argc, char** argv)
{
	vector< string > end_links;
	KDL::Tree kdl_tree;

	//Check if filename provided
	if (argc != 2){cerr << "Usage: full_info input.xml \n"; return -1;}

	//Obtain file contents
	string file_content = getFileContent(argv);

	//Assign URDF model from file contents
	shared_ptr< urdf::ModelInterface > robot1 = urdf::parseURDF(file_content);
	if (!robot1){cerr << "ERROR: Model Parsing the xml failed \n";return -1;}

	//Vector of active root/leaf pairs in URDF model
	vector< pair< string, string > > active_rootleaf_pairs = getActiveRootLeaf(robot1);

	//Assign KDL tree from file contents
	if (!kdl_parser::treeFromString(file_content, kdl_tree)) {cerr << "Failed to construct kdl tree \n";return -1;}

	//Vector of active chains
	vector< KDL::Chain > active_chains = getActiveChains(kdl_tree, active_rootleaf_pairs);

	//Vector of active frames
	vector< KDL::Frame > active_frames = getActiveFrames(active_chains);

	vector< pair< double, double> > robot_workspace = getWorkspace(active_frames);

	printRobotInfo(robot1, active_chains, robot_workspace);
	printChainInfo(active_frames, active_rootleaf_pairs, active_chains);

}

