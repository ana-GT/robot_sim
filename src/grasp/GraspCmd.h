/**
 * @function GraspCmd.h
 */

#pragma once

#include <ros/ros.h>

#include <robot_sim/JointCommands.h>
#include <robot_sim/robotState.h>

/**
 * @class GraspCmd
 */
class GraspCmd {
 public:
    GraspCmd();
    ~GraspCmd();
    
    bool init( const std::string &_master_url,
	       const std::string &_host_url,
	       const std::string &_node_name );

    void initJointCmdMsg();

    enum {
	OPEN_LEFT,
	OPEN_RIGHT,
	OPEN_BOTH,
	CLOSED_LEFT,
	CLOSED_RIGHT,
	CLOSED_BOTH
    } BASIC_COMMAND_TYPE;

    void sendBasicCommand( int _type );

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

    // Constant stuff
    const static std::string LF_linkNames[];
    const static std::string RF_linkNames[];
    const static double LF_closedConfig[];
    const static double LF_openConfig[];

    const static double RF_closedConfig[];
    const static double RF_openConfig[];

    std::vector<std::string> jointNames;
    int numJoints;

};
