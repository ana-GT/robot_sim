/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <string>
#include <algorithm>
#include <stdlib.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/common/Assert.hh>

#include "drchuboPlugin.h"

using std::string;

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(drchuboPlugin)

    /**
     * @function drchuboPlugin
     */
    drchuboPlugin::drchuboPlugin() {

    }

    /**
     * @function drchuboPlugin
     * @brief Destructor
     */
    drchuboPlugin::~drchuboPlugin() {
	event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
	this->rosNode->shutdown();
	this->rosQueue.clear();
	this->rosQueue.disable();
	this->callbackQueeuThread.join();
	delete this->rosNode;
    }

    /**
     * @function Load
     * @brief Load function before robot
     */
    void drchuboPlugin::Load( physics::ModelPtr _parent,
			       sdf::ElementPtr _sdf ) {
	printf("Loading drchuboPlugin \n");
	// Set the model
	this->model = _parent;
	
	// Get the world name.
	this->world = this->model->GetWorld();
	// JointController: built-in gazebo to control joints
	this->jointController = this->model->GetJointController();
	this->node = transport::NodePtr(new transport::Node());
	this->node->Init(this->world->GetName());

	std::string jointCmdName = std::string("~/") + this->model->GetName() + std::string("/joint_cmd");
	this->jointCmdPub = this->node->Advertise<msgs::JointCmd>(jointCmdName);
	// save sdf
	this->sdf = _sdf;

	// initialize update time, this will be the first update step for
	// UpdateStates as well - i.e. current setting skips the first
	// update.
	this->lastControllerUpdateTime = this->world->GetSimTime();
	// common::Time(2.0 * this->world->GetPhysicsEngine()->GetMaxStepSize());

	// init joints, hardcoded for drchubo
	this->jointNames.push_back("LHY");
	this->jointNames.push_back("LHR");
	this->jointNames.push_back("LHP");
	this->jointNames.push_back("LKP");
	this->jointNames.push_back("LAP");
	this->jointNames.push_back("LAR");

	this->jointNames.push_back("RHY");
	this->jointNames.push_back("RHR");
	this->jointNames.push_back("RHP");
	this->jointNames.push_back("RKP");
	this->jointNames.push_back("RAP");
	this->jointNames.push_back("RAR");

	this->jointNames.push_back("LSP");
	this->jointNames.push_back("LSR");
	this->jointNames.push_back("LSY");
	this->jointNames.push_back("LEP");
	this->jointNames.push_back("LWY");
	this->jointNames.push_back("LWP");
	this->jointNames.push_back("LWR");

	this->jointNames.push_back("RSP");
	this->jointNames.push_back("RSR");
	this->jointNames.push_back("RSY");
	this->jointNames.push_back("REP");
	this->jointNames.push_back("RWY");
	this->jointNames.push_back("RWP");
	this->jointNames.push_back("RWR");

	this->jointNames.push_back("TSY");
	this->jointNames.push_back("NKY");
	this->jointNames.push_back("NKP");

	// get pointers to joints from gazebo
	this->joints.resize(this->jointNames.size());
	for (unsigned int i = 0; i < this->joints.size(); ++i) {
	    this->joints[i] = this->model->GetJoint(this->jointNames[i]);
	    if (!this->joints[i]) {
		ROS_ERROR("drchubo robot expected joint[%s] not present, plugin not loaded",
			  this->jointNames[i].c_str());
		return;
	    }
	}

	// get effort limits from gazebo
	this->effortLimit.resize(this->jointNames.size());
	for (unsigned i = 0; i < this->effortLimit.size(); ++i) {
	    this->effortLimit[i] = this->joints[i]->GetEffortLimit(0);
	}
	// JointController: Publish messages to reset joint controller gains
	for (unsigned int i = 0; i < this->joints.size(); ++i) {
	    msgs::JointCmd msg;
	    msg.set_name( this->joints[i]->GetScopedName() );
	    msg.mutable_position()->set_target(0.0);
	    msg.mutable_position()->set_p_gain(0.0);
	    msg.mutable_position()->set_i_gain(0.0);
	    msg.mutable_position()->set_d_gain(0.0);
	    msg.mutable_position()->set_i_max(0.0);
	    msg.mutable_position()->set_i_min(0.0);
	    msg.mutable_position()->set_limit(0.0);
	}
	// initialize PID states: error terms	
	{
	    this->errorTerms.resize(this->joints.size());
	    for (unsigned i = 0; i < this->errorTerms.size(); ++i) {
		this->errorTerms[i].q_p = 0;
		this->errorTerms[i].d_q_p_dt = 0;
		this->errorTerms[i].k_i_q_i = 0;
		this->errorTerms[i].qd_p = 0;
	    }
	}
	// Initialize jointState message - ROS
	{

	    // We are not sending names due to the fact that there is an enum
	    // joint indices in drchuboState.msg.
	    this->drchuboState.position.resize(this->joints.size());
	    this->drchuboState.velocity.resize(this->joints.size());
	    this->drchuboState.effort.resize(this->joints.size());
	    this->drchuboState.kp_position.resize(this->joints.size());
	    this->drchuboState.ki_position.resize(this->joints.size());
	    this->drchuboState.kd_position.resize(this->joints.size());
	    this->drchuboState.kp_velocity.resize(this->joints.size());
	    this->drchuboState.i_effort_min.resize(this->joints.size());
	    this->drchuboState.i_effort_max.resize(this->joints.size());
	    this->drchuboState.k_effort.resize(this->joints.size());
	    
	    this->jointStates.name.resize(this->joints.size());
	    this->jointStates.position.resize(this->joints.size());
	    this->jointStates.velocity.resize(this->joints.size());
	    this->jointStates.effort.resize(this->joints.size());
	    
	    for (unsigned int i = 0; i < this->jointNames.size(); ++i)
		this->jointStates.name[i] = this->jointNames[i];
	}
	// Initialize jointCommands - ROS
	{
	    this->jointCommands.position.resize(this->joints.size());
	    this->jointCommands.velocity.resize(this->joints.size());
	    this->jointCommands.effort.resize(this->joints.size());
	    this->jointCommands.kp_position.resize(this->joints.size());
	    this->jointCommands.ki_position.resize(this->joints.size());
	    this->jointCommands.kd_position.resize(this->joints.size());
	    this->jointCommands.kp_velocity.resize(this->joints.size());
	    this->jointCommands.i_effort_min.resize(this->joints.size());
	    this->jointCommands.i_effort_max.resize(this->joints.size());
	    
	    this->ZeroJointCommands();
	}
	this->LoadROS();
    }

    /**
     * @function LoadROS
     * @brief Load ROS stuff related to the robot
     */
    void drchuboPlugin::LoadROS() {

	// initialize ros
	if (!ros::isInitialized()) {
	    gzerr << "Not loading plugin since ROS hasn't been "
		  << "properly initialized.  Try starting gazebo with ros plugin:\n"
		  << "  gazebo -s libgazebo_ros_api_plugin.so\n";
	    return;
	}
	
	// ros stuff
	this->rosNode = new ros::NodeHandle("");
	
	// publish multi queue
	this->pmq.startServiceThread();
	
	// **  ROS Parameters  **                                          
	// pull down controller parameters
	this->LoadPIDGainsFromParameter();
	
	double stepSize = this->world->GetPhysicsEngine()->GetMaxStepSize();
	if (math::equal(stepSize, 0.0)) {
	    stepSize = 0.001;
	    ROS_WARN("simulation step size is zero, something is wrong,"
		     "  Defaulting to step size of %f sec.", stepSize);
	}

	//  ** ROS Publishers **                                          
	// broadcasts the robot joint states
	this->pubJointStatesQueue = this->pmq.addPub<sensor_msgs::JointState>();
	this->pubJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
										 "drchubo/joint_states", 1);
	
	//broadcasts atlas states
	this->pubRobotStateQueue = this->pmq.addPub<robot_sim::robotState>();
	this->pubRobotState = this->rosNode->advertise<robot_sim::robotState>(
									      "drchubo/robot_state", 100);
	
	//  ** ROS Subscribers **                                          
	
	// ros topic subscribtions
	ros::SubscribeOptions jointCommandsSo =
	    ros::SubscribeOptions::create<robot_sim::JointCommands>(
								    "drchubo/joint_commands", 1,
								    boost::bind(&drchuboPlugin::SetJointCommands, this, _1),
								    ros::VoidPtr(), &this->rosQueue);
	// This subscription is TCP because the message is larger than a UDP datagram
	// and we have had reports of corrupted data, which we attribute to erroneous
	// demarshalling following packet loss.
	jointCommandsSo.transport_hints =
	    ros::TransportHints().reliable().tcpNoDelay(true);
	this->subJointCommands =
	    this->rosNode->subscribe(jointCommandsSo);
		
	//  ROS Custom callback queue                                
	this->callbackQueeuThread = boost::thread(boost::bind(&drchuboPlugin::RosQueueThread, this));
	
	//  Hook up to gazebo periodic updates                        
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&drchuboPlugin::UpdateStates, this));
    }
    
    /**
     * @function UpdateStates
     */
    void drchuboPlugin::UpdateStates() {
	common::Time curTime = this->world->GetSimTime();
	
	if (curTime > this->lastControllerUpdateTime) {
	    // gather robot state data and publish them
	    this->GetAndPublishRobotStates(curTime);	    
	    { 
		boost::mutex::scoped_lock lock(this->mutex);
		this->UpdatePIDControl((curTime - this->lastControllerUpdateTime).Double());
	    }
	    
	    this->lastControllerUpdateTime = curTime;
	}
    }

    /**
     * @function SetJointCommands
     */
    void drchuboPlugin::SetJointCommands( const robot_sim::JointCommands::ConstPtr &_msg ) {
	printf("Set joint commands start \n");
	boost::mutex::scoped_lock lock(this->mutex);
	
	this->jointCommands.header.stamp = _msg->header.stamp;
	
	if (_msg->position.size() == this->jointCommands.position.size())
	    std::copy(_msg->position.begin(), _msg->position.end(),
		      this->jointCommands.position.begin());
	else
	    ROS_DEBUG("JointCommands message contains different number of"
		      " elements position[%ld] than expected[%ld]",
		      _msg->position.size(), this->jointCommands.position.size());
	
	if (_msg->velocity.size() == this->jointCommands.velocity.size())
	    std::copy(_msg->velocity.begin(), _msg->velocity.end(),
		      this->jointCommands.velocity.begin());
	else
	    ROS_DEBUG("JointCommands message contains different number of"
		      " elements velocity[%ld] than expected[%ld]",
		      _msg->velocity.size(), this->jointCommands.velocity.size());
	
	if (_msg->effort.size() == this->jointCommands.effort.size())
	    std::copy(_msg->effort.begin(), _msg->effort.end(),
		      this->jointCommands.effort.begin());
	else
	    ROS_DEBUG("JointCommands message contains different number of"
		      " elements effort[%ld] than expected[%ld]",
      _msg->effort.size(), this->jointCommands.effort.size());
	
	if (_msg->kp_position.size() == this->drchuboState.kp_position.size())
	    std::copy(_msg->kp_position.begin(), _msg->kp_position.end(),
		      this->drchuboState.kp_position.begin());
	else
	    ROS_DEBUG("JointCommands message contains different number of"
		      " elements kp_position[%ld] than expected[%ld]",
		      _msg->kp_position.size(), this->drchuboState.kp_position.size());
	
	if (_msg->ki_position.size() == this->drchuboState.ki_position.size())
	    std::copy(_msg->ki_position.begin(), _msg->ki_position.end(),
		      this->drchuboState.ki_position.begin());
	else
	    ROS_DEBUG("JointCommands message contains different number of"
		      " elements ki_position[%ld] than expected[%ld]",
		      _msg->ki_position.size(), this->drchuboState.ki_position.size());
	
	if (_msg->kd_position.size() == this->drchuboState.kd_position.size())
	    std::copy(_msg->kd_position.begin(), _msg->kd_position.end(),
		      this->drchuboState.kd_position.begin());
	else
	    ROS_DEBUG("JointCommands message contains different number of"
		      " elements kd_position[%ld] than expected[%ld]",
		      _msg->kd_position.size(), this->drchuboState.kd_position.size());
	
	if (_msg->kp_velocity.size() == this->drchuboState.kp_velocity.size())
	    std::copy(_msg->kp_velocity.begin(), _msg->kp_velocity.end(),
		      this->drchuboState.kp_velocity.begin());
	else
	    ROS_DEBUG("JointCommands message contains different number of"
		      " elements kp_velocity[%ld] than expected[%ld]",
		      _msg->kp_velocity.size(), this->drchuboState.kp_velocity.size());
	
	if (_msg->i_effort_min.size() == this->drchuboState.i_effort_min.size())
	    std::copy(_msg->i_effort_min.begin(), _msg->i_effort_min.end(),
		      this->drchuboState.i_effort_min.begin());
	else
	    ROS_DEBUG("JointCommands message contains different number of"
		      " elements i_effort_min[%ld] than expected[%ld]",
		      _msg->i_effort_min.size(), this->drchuboState.i_effort_min.size());
	
	if (_msg->i_effort_max.size() == this->drchuboState.i_effort_max.size())
	    std::copy(_msg->i_effort_max.begin(), _msg->i_effort_max.end(),
		      this->drchuboState.i_effort_max.begin());
	else
	    ROS_DEBUG("JointCommands message contains different number of"
		      " elements i_effort_max[%ld] than expected[%ld]",
		      _msg->i_effort_max.size(), this->drchuboState.i_effort_max.size());
	printf("SEt joint commands end \n");
    }


    /**
     * @function ZeroJointCommands
     * @brief Set JointCommands to Zero
     */
    void drchuboPlugin::ZeroJointCommands() {
	boost::mutex::scoped_lock lock(this->mutex);
	
	for (unsigned i = 0; i < this->jointNames.size(); ++i) {
		this->jointCommands.position[i] = 0;
		this->jointCommands.velocity[i] = 0;
		this->jointCommands.effort[i] = 0;
		// store these directly on robotState, more efficient for pub later
		this->drchuboState.kp_position[i] = 0;
		this->drchuboState.ki_position[i] = 0;
		this->drchuboState.kd_position[i] = 0;
		this->drchuboState.kp_velocity[i] = 0;
		this->drchuboState.i_effort_min[i] = 0;
		this->drchuboState.i_effort_max[i] = 0;
		this->drchuboState.k_effort[i] = 0;
	}
    }

    /**
     * @function LoadPIDGainsFromParameter
     * @brief Get parameters from ROS param and set them in the plugin 
     */
    void drchuboPlugin::LoadPIDGainsFromParameter() {

	boost::mutex::scoped_lock lock(this->mutex);
	
	// pull down controller parameters
	for (unsigned int i = 0; i < this->joints.size(); ++i) {
	    char joint_ns[200] = "";
	    snprintf(joint_ns, sizeof(joint_ns), "drchubo_controller/gains/%s/",
		     this->joints[i]->GetName().c_str());
	    // this is so ugly
	    double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0;
	    string p_str = string(joint_ns)+"p";
	    string i_str = string(joint_ns)+"i";
	    string d_str = string(joint_ns)+"d";
	    string i_clamp_str = string(joint_ns)+"i_clamp";
	    if (!this->rosNode->getParam(p_str, p_val) ||
		!this->rosNode->getParam(i_str, i_val) ||
		!this->rosNode->getParam(d_str, d_val) ||
		!this->rosNode->getParam(i_clamp_str, i_clamp_val))
		{
		    ROS_ERROR("couldn't find a param for %s", joint_ns);
		    continue;
		}
	    // store these directly on drchuboState, more efficient for pub later
	    this->drchuboState.kp_position[i]  =  p_val;
	    this->drchuboState.ki_position[i]  =  i_val;
	    this->drchuboState.kd_position[i]  =  d_val;
	    this->drchuboState.i_effort_min[i] = -i_clamp_val;
	    this->drchuboState.i_effort_max[i] =  i_clamp_val;
	    // default k_effort is set to 1, controller relies on PID.
	    this->drchuboState.k_effort[i] = 255;
	}
    }
    
    
    /**
     * @function UpdatePIDControl
     */
    void drchuboPlugin::UpdatePIDControl(double _dt) {
	
	/// update pid with feedforward force
	for (unsigned int i = 0; i < this->joints.size(); ++i) {
	    // truncate joint position within range of motion
	    double positionTarget = math::clamp(
						this->jointCommands.position[i],
						this->joints[i]->GetLowStop(0).Radian(),
						this->joints[i]->GetHighStop(0).Radian());
	    
	    double q_p = positionTarget - this->drchuboState.position[i];
	    
	    if (!math::equal(_dt, 0.0))
		this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / _dt;
	    
	    this->errorTerms[i].q_p = q_p;
	    
	    
		
	    this->errorTerms[i].k_i_q_i = math::clamp(
						      this->errorTerms[i].k_i_q_i +
						      _dt * this->drchuboState.ki_position[i] * this->errorTerms[i].q_p,
						      static_cast<double>(this->drchuboState.i_effort_min[i]),
						      static_cast<double>(this->drchuboState.i_effort_max[i]));
	    
	    // convert k_effort to a double between 0 and 1
	    double k_effort =
		static_cast<double>(this->drchuboState.k_effort[i])/255.0;
	    
	    // use gain params to compute force cmd
	    double forceUnclamped =
		k_effort * (
			    this->drchuboState.kp_position[i] * this->errorTerms[i].q_p +
			    this->errorTerms[i].k_i_q_i +
			    this->drchuboState.kd_position[i] * this->errorTerms[i].d_q_p_dt +
			    this->jointCommands.velocity[i] +
			    this->jointCommands.effort[i]);
	    
	    // keep unclamped force for integral tie-back calculation
	    double forceClamped = math::clamp(forceUnclamped, -this->effortLimit[i],
					      this->effortLimit[i]);
	    
	    // apply force to joint
	    this->joints[i]->SetForce(0, forceClamped);
	    
	    // fill in jointState efforts
		this->drchuboState.effort[i] = forceClamped;
		this->jointStates.effort[i] = forceClamped;
		
	    }
    }


    /**
     * @function GetAndPublishRobotStates
     */
    void drchuboPlugin::GetAndPublishRobotStates(const common::Time &_curTime) {
	
	boost::mutex::scoped_lock lock(this->mutex);
	
	// populate drchuboState from robot
	this->drchuboState.header.stamp = ros::Time(_curTime.sec, _curTime.nsec);
	this->jointStates.header.stamp = this->drchuboState.header.stamp;
		

	for (unsigned int i = 0; i < this->joints.size(); ++i) {
	    
	    this->drchuboState.position[i] = this->joints[i]->GetAngle(0).Radian();
	    this->drchuboState.velocity[i] = this->joints[i]->GetVelocity(0);
	    // cached?
	    //   this->drchuboState.effort[i];
	    
	    this->jointStates.position[i] = this->joints[i]->GetAngle(0).Radian();
	    this->jointStates.velocity[i] = this->joints[i]->GetVelocity(0);
	    // cached?
	    //this->jointStates.effort[i];
	}
	
	// publish robot states
	this->pubJointStatesQueue->push(this->jointStates, this->pubJointStates);
	this->pubRobotStateQueue->push(this->drchuboState, this->pubRobotState);
    }

    /**
     * @function RosQueueThread
     */
    void drchuboPlugin::RosQueueThread() {
	static const double timeout = 0.01;
	
	while (this->rosNode->ok())
	    {
		this->rosQueue.callAvailable(ros::WallDuration(timeout));
	    }
    }

} // namespace gazebo
