/**
 * @function GraspCmd.cpp
 */

#include "GraspCmd.h"

#include <string>
#include <boost/algorithm/string.hpp>

const std::string GraspCmd::LF_linkNames[] = { "Body_LF1", "Body_LF2", "Body_LF3" };

const double GraspCmd::LF_closedConfig[] = { 1.0, 1.0, 1.0 }; // ~ +63 degrees
const double GraspCmd::LF_openConfig[] = { 0.0, 0.0, 0.0 }; // ~ 0 degrees

const std::string GraspCmd::RF_linkNames[] = { "Body_RF1", "Body_RF2", "Body_RF3" };

// ~ +63 degrees
const double GraspCmd::RF_closedConfig[] = { 1.0, 1.0, 1.0 }; // ~ +63 degrees
const double GraspCmd::RF_openConfig[] = { 0.0, 0.0, 0.0 }; // ~ 0 degrees

/**
 * @function GraspCmd
 * @brief Constructor
 */
GraspCmd::GraspCmd() {

    this->jointNames.push_back("drchubo::LHY");
    this->jointNames.push_back("drchubo::LHR");
    this->jointNames.push_back("drchubo::LHP");
    this->jointNames.push_back("drchubo::LKP");
    this->jointNames.push_back("drchubo::LAP");
    this->jointNames.push_back("drchubo::LAR");

    this->jointNames.push_back("drchubo::RHY");
    this->jointNames.push_back("drchubo::RHR");
    this->jointNames.push_back("drchubo::RHP");
    this->jointNames.push_back("drchubo::RKP");
    this->jointNames.push_back("drchubo::RAP");
    this->jointNames.push_back("drchubo::RAR");

    this->jointNames.push_back("drchubo::LSP");
    this->jointNames.push_back("drchubo::LSR");
    this->jointNames.push_back("drchubo::LSY");
    this->jointNames.push_back("drchubo::LEP");
    this->jointNames.push_back("drchubo::LWY");
    this->jointNames.push_back("drchubo::LWP");
    this->jointNames.push_back("drchubo::LWR");

    this->jointNames.push_back("drchubo::RSP");
    this->jointNames.push_back("drchubo::RSR");
    this->jointNames.push_back("drchubo::RSY");
    this->jointNames.push_back("drchubo::REP");
    this->jointNames.push_back("drchubo::RWY");
    this->jointNames.push_back("drchubo::RWP");
    this->jointNames.push_back("drchubo::RWR");

    this->jointNames.push_back("drchubo::TSY");
    this->jointNames.push_back("drchubo::NKY");
    this->jointNames.push_back("drchubo::NKP");

    this->jointNames.push_back("drchubo::LF1");
    this->jointNames.push_back("drchubo::LF2");
    this->jointNames.push_back("drchubo::LF3");

    this->jointNames.push_back("drchubo::RF1");
    this->jointNames.push_back("drchubo::RF2");
    this->jointNames.push_back("drchubo::RF3");

    this->numJoints = this->jointNames.size();
}

/**
 * @function ~GraspCmd
 * @brief Destructor
 */
GraspCmd::~GraspCmd() {
}

/**
 * @function init
 * @brief init ROS Comm
 */
bool GraspCmd::init( const std::string &_master_url,
		     const std::string &_host_url,
		     const std::string &_node_name ) {

    std::map<std::string, std::string> remappings;
    remappings["__master"] = _master_url;
    remappings["__hostname"] = _host_url; 
  
    ros::init( remappings, _node_name );
    
    if( !ros::master::check() ) {
	printf("DID NOT INITIALIZE ROS! \n");
	return false; 	
    }

    ros::spinOnce();
    rosNode = new ros::NodeHandle();

    initJointCmdMsg();

    // Subscribe
    // Robot states
    robotState_sub = rosNode->subscribe( "/drchubo/robot_state",
					 1, 
					 &GraspCmd::robotState_cb,
					 this );

    // Publish
    // Joint commands
    this->jointCommand_pub = this->rosNode->advertise<robot_sim::JointCommands>("drchubo/joint_commands",
										1);

    ROS_INFO(" GraspCmd - Successful initialization");
}

/**
 * @function robotState_cb
 * @brief
 */
void GraspCmd::robotState_cb( const robot_sim::robotStateConstPtr &_msg ) {

}

/**
 * @function initJointCmdMsg
 */
void GraspCmd::initJointCmdMsg() {

    unsigned int n = this->numJoints;
    this->jointCmd_msg.position.resize(n);
    this->jointCmd_msg.velocity.resize(n);
    this->jointCmd_msg.effort.resize(n);
    this->jointCmd_msg.kp_position.resize(n);
    this->jointCmd_msg.ki_position.resize(n);
    this->jointCmd_msg.kd_position.resize(n);
    this->jointCmd_msg.kp_velocity.resize(n);
    this->jointCmd_msg.i_effort_min.resize(n);
    this->jointCmd_msg.i_effort_max.resize(n);
    this->jointCmd_msg.k_effort.resize(n);
    
    for ( unsigned int i = 0; i < n; ++i ) {
	std::vector<std::string> pieces;
	boost::split(pieces, this->jointNames[i], boost::is_any_of(":"));
	
	double val;
	this->rosNode->getParam("drchubo_controller/gains/" + pieces[2] +
				"/p", val);
	this->jointCmd_msg.kp_position[i] = val;
	
	this->rosNode->getParam("drchubo_controller/gains/" + pieces[2] +
				"/i", val);
	this->jointCmd_msg.ki_position[i] = val;
	
	this->rosNode->getParam("drchubo_controller/gains/" + pieces[2] +
				"/d", val);
	this->jointCmd_msg.kd_position[i] = val;
	
	this->rosNode->getParam("drchubo_controller/gains/" + pieces[2] +
				"/i_clamp", val);
	this->jointCmd_msg.i_effort_min[i] = -val;
	this->jointCmd_msg.i_effort_max[i] = val;
	
	this->jointCmd_msg.velocity[i]     = 0;
	this->jointCmd_msg.effort[i]       = 0;
	this->jointCmd_msg.kp_velocity[i]  = 0;
    }

}

/**
 * @function sendBasicCommand
 * @brief Open or close the hand(s)
 */
void GraspCmd::sendBasicCommand( int _type ) {

    if( _type == OPEN_LEFT ) {
	this->jointCmd_msg.header.stamp = ros::Time::now();
	this->jointCmd_msg.position[robot_sim::robotState::LF1] = LF_openConfig[0];
	this->jointCmd_msg.position[robot_sim::robotState::LF2] = LF_openConfig[1];
	this->jointCmd_msg.position[robot_sim::robotState::LF3] = LF_openConfig[2];

    } else if( _type == OPEN_RIGHT ) {
	this->jointCmd_msg.header.stamp = ros::Time::now();
	this->jointCmd_msg.position[robot_sim::robotState::RF1] = RF_openConfig[0];
	this->jointCmd_msg.position[robot_sim::robotState::RF2] = RF_openConfig[1];
	this->jointCmd_msg.position[robot_sim::robotState::RF3] = RF_openConfig[2];
    } else if( _type == OPEN_BOTH ) {
	this->jointCmd_msg.header.stamp = ros::Time::now();
	this->jointCmd_msg.position[robot_sim::robotState::LF1] = LF_openConfig[0];
	this->jointCmd_msg.position[robot_sim::robotState::LF2] = LF_openConfig[1];
	this->jointCmd_msg.position[robot_sim::robotState::LF3] = LF_openConfig[2];
	this->jointCmd_msg.position[robot_sim::robotState::RF1] = RF_openConfig[0];
	this->jointCmd_msg.position[robot_sim::robotState::RF2] = RF_openConfig[1];
	this->jointCmd_msg.position[robot_sim::robotState::RF3] = RF_openConfig[2];
    } else if( _type == CLOSED_LEFT ) {
	this->jointCmd_msg.header.stamp = ros::Time::now();
	this->jointCmd_msg.position[robot_sim::robotState::LF1] = LF_closedConfig[0];
	this->jointCmd_msg.position[robot_sim::robotState::LF2] = LF_closedConfig[1];
	this->jointCmd_msg.position[robot_sim::robotState::LF3] = LF_closedConfig[2];
    } else if( _type == CLOSED_RIGHT ) {
	this->jointCmd_msg.header.stamp = ros::Time::now();
	this->jointCmd_msg.position[robot_sim::robotState::RF1] = RF_closedConfig[0];
	this->jointCmd_msg.position[robot_sim::robotState::RF2] = RF_closedConfig[1];
	this->jointCmd_msg.position[robot_sim::robotState::RF3] = RF_closedConfig[2];
    } else if( _type == CLOSED_BOTH ) {
	this->jointCmd_msg.header.stamp = ros::Time::now();
	this->jointCmd_msg.position[robot_sim::robotState::LF1] = LF_closedConfig[0];
	this->jointCmd_msg.position[robot_sim::robotState::LF2] = LF_closedConfig[1];
	this->jointCmd_msg.position[robot_sim::robotState::LF3] = LF_closedConfig[2];
	this->jointCmd_msg.position[robot_sim::robotState::RF1] = RF_closedConfig[0];
	this->jointCmd_msg.position[robot_sim::robotState::RF2] = RF_closedConfig[1];
	this->jointCmd_msg.position[robot_sim::robotState::RF3] = RF_closedConfig[2];
	this->jointCmd_msg.position[robot_sim::robotState::LSP] = -0.6;
    } 
    printf("About to send \n");
    // Send
    this->jointCommand_pub.publish( this->jointCmd_msg );
    ros::spinOnce();

}
