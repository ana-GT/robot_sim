/**
 * @file pubTest.cpp
 */

#include <ros/ros.h>
#include <robot_sim/JointCommands.h>
#include <robot_sim/robotState.h>

robot_sim::robotState rs;
robot_sim::JointCommands jc_msg;

void robotState_cb( const robot_sim::robotStateConstPtr &_msg );

int main( int argc, char* argv[] ) {

    ros::Publisher jc_pub;
    ros::Subscriber rs_sub;
    ros::init( argc, argv, "really" );
    ros::NodeHandle* node;
    node = new ros::NodeHandle("");
    
    rs_sub = node->subscribe( "drchubo/robot_state",
			      1, 
			      robotState_cb );

    jc_pub = node->advertise<robot_sim::JointCommands>("drchubo/joint_commands", 1, true );

    // Test

    // Get latest state
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    // Prepare message

    // Publish message
    jc_pub.publish( jc_msg );
    ros::spinOnce();
    ros::Duration(0.3).sleep();
}


void robotState_cb( const robot_sim::robotStateConstPtr &_msg ) {
    rs = (*_msg);
}
