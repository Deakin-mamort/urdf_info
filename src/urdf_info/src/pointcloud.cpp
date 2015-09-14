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
vector<string> getJoints(shared_ptr<ModelInterface> robot)
{
	vector<string> joints;
	for (map<string,shared_ptr<Joint> >::iterator joint = robot->joints_.begin();joint != robot->joints_.end(); joint++)
	{
		if(joint->second->type != 6){
		//	cout << joint->second->name << "\t" << joint->second->type << endl;
			joints.push_back(joint->second->child_link_name);
		}
	}

	return joints;
}


//Create chains from model root to active joints
vector<KDL::Chain>  getChains(shared_ptr<const Link> root, KDL::Tree kdl_tree, vector<string> joints)
{
	vector<KDL::Chain> result;
	
	{
		KDL::Chain tmp_chain;
		bool error; 
		
		for(vector<string>::iterator it = joints.begin(); it != joints.end(); it++){
			error = kdl_tree.getChain(root->name, *it, tmp_chain); 									   
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



void printEffectors(vector<KDL::Frame> effectors)
{
		for (vector<KDL::Frame>::iterator it = effectors.begin(); it != effectors.end(); it++){
			KDL::Frame temp = *it;
			cout << temp.p(0) << "," << setw(20) << temp.p(1) << "," << setw(20) << temp.p(2) << "\n";
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
	
	vector<string> joints = getJoints(robot1);
	shared_ptr<const Link> root_link = robot1->getRoot();
	
	vector<KDL::Chain> activeChains = getChains(root_link, kdl_tree, joints);;
	vector<KDL::Frame> endEffectors = getCarts(joints, activeChains);
	printEffectors(endEffectors);
	
}
