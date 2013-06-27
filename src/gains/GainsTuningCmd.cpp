/**
 * @function GainsTuningCmd.cpp
 */

#include "GainsTuningCmd.h"

#include <string>
#include <boost/algorithm/string.hpp>


/**
 * @function GainsTuningCmd
 * @brief Constructor
 */
GainsTuningCmd::GainsTuningCmd() {

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
 * @function ~GainsTuningCmd
 * @brief Destructor
 */
GainsTuningCmd::~GainsTuningCmd() {
}

/**
 * @function init
 * @brief init ROS Comm
 */
bool GainsTuningCmd::init( const std::string &_master_url,
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
					 &GainsTuningCmd::robotState_cb,
					 this );

    // Publish
    // Joint commands
    this->jointCommand_pub = this->rosNode->advertise<robot_sim::JointCommands>("drchubo/joint_commands",
										1);

    ROS_INFO(" GainsTuningCmd - Successful initialization");
}

/**
 * @function robotState_cb
 * @brief
 */
void GainsTuningCmd::robotState_cb( const robot_sim::robotStateConstPtr &_msg ) {
    // Save state
    latestState = (*_msg);
}

/**
 * @function getGains
 */
std::vector<std::vector<double> > GainsTuningCmd::getGains() {
    
    // Spin and get the latest values from the callback
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    // Store 
    std::vector<std::vector<double> > gains;

    std::vector<float> Kp = latestState.kp_position;
    std::vector<float> Kd = latestState.kd_position;
    std::vector<float> Ki = latestState.ki_position;
    std::vector<float> Kpv = latestState.kp_velocity;

    for( int i = 0; i < this->numJoints; ++i ) {

	std::vector<double> Ks;
	Ks.push_back( (double) Kp[i] );
	Ks.push_back( (double) Kd[i] );
	Ks.push_back( (double) Ki[i] );
	Ks.push_back( (double) Kpv[i] );

	gains.push_back( Ks );
    }

    return gains;
}


/**
 * @function initJointCmdMsg
 */
void GainsTuningCmd::initJointCmdMsg() {

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
void GainsTuningCmd::sendBasicCommand( int _type ) {

    printf("About to send \n");
    // Send
    this->jointCommand_pub.publish( this->jointCmd_msg );
    ros::spinOnce();

}

/**
 * @function sendGainCommand
 */
void GainsTuningCmd::sendGainCommand( std::vector<std::vector<double> > _gains ) {

    // Build the messsage
    // Time stamp
    this->jointCmd_msg.header.stamp = ros::Time::now();
    // Same target positions
    std::copy( this->latestState.position.begin(),
	       this->latestState.position.end(),
	       this->jointCmd_msg.position.begin() );

    // Get the gains
    for( int i = 0; i < this->numJoints; ++i ) {
	this->jointCmd_msg.kp_position[i] = _gains[i][0];
	this->jointCmd_msg.kd_position[i] = _gains[i][1];
	this->jointCmd_msg.ki_position[i] = _gains[i][2];
	this->jointCmd_msg.kp_velocity[i] = _gains[i][3];
    }

    // Send
    this->jointCommand_pub.publish( this->jointCmd_msg );
    ros::spinOnce();
}
