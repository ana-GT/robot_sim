/**
 * @function GainsTuningCmd.h
 */

#pragma once

#include <ros/ros.h>

#include <robot_sim/JointCommands.h>
#include <robot_sim/robotState.h>

#include "GainsTuningCmd.h"

/**
 * @class GainsTuningCmd
 */
class GainsTuningCmd {
 public:
    GainsTuningCmd();
    ~GainsTuningCmd();
    
    bool init( const std::string &_master_url,
	       const std::string &_host_url,
	       const std::string &_node_name );

    void initJointCmdMsg();

    std::vector<std::vector<double> > getGains();

    enum {
	LEFT_LEG,
	RIGHT_LEG,
	BOTH_LEGS
    } LEG_MODE;

    void sendBasicCommand( int _type );
    void sendGainCommand( std::vector<std::vector<double> > _gains );

    // Callbacks
    void robotState_cb( const robot_sim::robotStateConstPtr &_msg );

    // Subscriber
    ros::Subscriber robotState_sub;

    // Publishers
    ros::Publisher jointCommand_pub;

    // Ros Node
    ros::NodeHandle* rosNode;
    
    // Standard message
    robot_sim::JointCommands jointCmd_msg;

    // Store latest state
    robot_sim::robotState latestState;

    // Constant stuff
    std::vector<std::string> jointNames;
    int numJoints;

};
