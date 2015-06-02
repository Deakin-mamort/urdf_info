#include "urdf_model/model.h"
#include "urdf_parser/urdf_parser.h"

#include "kdl_parser/kdl_parser.hpp"
 
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>


#define redtxt	"\x1b[1;32m"
#define bluetxt "\x1b[1;34m"
#define magentatxt "\x1b[1;35m"
#define cyantxt "\x1b[1;36m"
#define whitetxt "\x1b[1;37m"
#define greentxt "\x1b[0;32m"
#define lightgreentxt "\x1b[1;32m"
#define ascii_arrow "\u2514 "

using namespace urdf;
using namespace std;
using namespace boost;
using namespace kdl_parser;

//Search robot model for active leafs
vector<string> getLeafs(shared_ptr<ModelInterface> robot)
{
	vector<string> leafs;
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
				//cout  << lightgreentxt << parent_link->name <<whitetxt ;

				next_joint=joint->second;

				//while joint type NOT "FIXED" cycle
				while(next_joint->type != 6)
				{

					boost::shared_ptr<Joint> joint = next_joint;
					boost::shared_ptr<const Link> link = robot->getLink(next_joint->child_link_name);

					//Print current Joint and Link name
 					//cout << " -> " << greentxt << joint->name << whitetxt << " -> " << lightgreentxt << link->name << whitetxt;

 					//cycle child joints assign last active joint as next joint in chain
 					//****BUG**** = doesn't work with multiple active child joints on a single link
 					for (vector<shared_ptr<Joint> >::const_iterator child_joint = link->child_joints.begin(); child_joint != link->child_joints.end(); child_joint++)
 					{
 						//****BUG**** if joint active assign
 						if ((*child_joint)->type !=6)
 							next_joint = (*child_joint);
 					}

 					//if no active child joints
 					if (joint == next_joint)
 					{
 						leafs.push_back(joint->child_link_name);
 						break;
 					}
				}

			}

		}

	}
	
	return leafs;
}

//Create chains from model root to active leafs
vector<KDL::Chain>  getChains(KDL::Tree kdl_tree, vector<string> leafs)
{
	vector<KDL::Chain> result;

	{
		KDL::Chain tmp_chain;
		bool error; 
		
		for(vector<string>::iterator it = leafs.begin(); it != leafs.end(); it++){
			error = kdl_tree.getChain("base", *it, tmp_chain); 									   
			result.push_back(tmp_chain);											
		}		  
	
	}
	
	return result;
}

//Find Cartesian co-ordinates for end-effectors
vector<KDL::Frame> getCarts(vector<string> leafs, vector<KDL::Chain> chains)
{
	vector<KDL::Frame> result;
	int count=0;
	
	for(vector<string>::iterator it = leafs.begin(); it != leafs.end(); it++)
	{
																		   
		KDL::Frame tmpFrame;
		
		KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chains[count]); 	  
		KDL::JntArray jointpositions(chains[count].getNrOfJoints());							  		
		bool k_status = fksolver.JntToCart(jointpositions, tmpFrame);								  
		
		result.push_back(tmpFrame);
		count++;
		//cout << "end-effector - " << *it << "\n" << cart_result.p << "\n\n";
	}
	
	return result;
}

//Determine max Cartesian co-ordinates in workspace
KDL::Frame workspaceMax(vector<KDL::Frame> effectors)
{
	KDL::Frame result;
	int count=0;
	
	for (vector<KDL::Frame>::iterator it = effectors.begin(); it != effectors.end(); it++){
		KDL::Frame temp = *it;
		
		//determine maximium catesian co-ordinates
		for (int i=0; i!=3; i++)
			{if( temp.p[i] > result.p[i]) {result.p[i] = temp.p[i];}}
	}
	
	return result;
}

//Determin min Catersian co-ordinates in workspace
KDL::Frame workspaceMin(vector<KDL::Frame> effectors)
{
	KDL::Frame result;
	int count=0;
	
	for (vector<KDL::Frame>::iterator it = effectors.begin(); it != effectors.end(); it++){
		KDL::Frame temp = *it;

		//determine minimium catesian co-ordinates
		for (int i=0; i!=3; i++)
			{if( temp.p[i] < result.p[i]) {result.p[i] = temp.p[i];}}
	}
	
	return result;
}

void checkMirrors(vector<KDL::Frame> effectors)
{
		//to check for mirrors we must iterate through all end effector locations and compare each of the co-ordinates
		//if a mirror is found the end effector name/number and axis needs to be recorded
	
		int count=1;
	
		for (vector<KDL::Frame>::iterator it1 = effectors.begin(); it1 != effectors.end(); it1++){
			KDL::Frame temp1 = *it1;
			
			for (int i=count; i!=effectors.size(); i++){
				KDL::Frame temp2 = effectors[count];
								
				for (int j=0; j!=3; j++){
					cout << temp1.p[j] << "\t" << temp2.p[j] << "\n";
					if( temp1.p[i] == temp2.p[i] ) {
						//	cout<< temp1.p[i] << "\t" << temp2.p[i] << "\n";
					}
					
				}
			}
			count++;
		}
}

void printEffectors(vector<KDL::Frame> effectors)
{
		for (vector<KDL::Frame>::iterator it = effectors.begin(); it != effectors.end(); it++){
			KDL::Frame temp = *it;
			cout << temp.p << "\n";
		}
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
	const string parse_string = xml_string;
 
	if (!treeFromString(parse_string, kdl_tree)){
		cerr << redtxt << ("Failed to construct kdl tree") << whitetxt << endl;
		return -1;
    }
	
	vector<string> leafs = getLeafs(robot1);
	vector<KDL::Chain> activeChains = getChains(kdl_tree, leafs);;
	vector<KDL::Frame> endEffectors = getCarts(leafs, activeChains);
	KDL::Frame max = workspaceMax(endEffectors);
	KDL::Frame min = workspaceMin(endEffectors);
	checkMirrors(endEffectors);
	printEffectors(endEffectors);
	
	//cout << "Max Workspace - " << max.p << "\n";
	//cout << "Min Workspace - " << min.p << "\n";
	
}
