/*
 * Copyright 2012 A.C.H.Q.
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

#include <map>
#include <string>
#include <stdlib.h>

#include "worldSimPlugin.h"

namespace gazebo {

    GZ_REGISTER_WORLD_PLUGIN(worldSimPlugin)

    /**
     * @function worldSimPlugin
     * @brief Constructor
     */
    worldSimPlugin::worldSimPlugin() {
	this->warpRobotWithCmdVel = false;
    }

    /**
     * @function ~worldSimPlugin
     * @brief Destructor
     */
    worldSimPlugin::~worldSimPlugin() {
	event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
	this->rosNode->shutdown();
	this->rosQueue.clear();
	this->rosQueue.disable();
	this->callbackQueueThread.join();
	delete this->rosNode;
    }

    /**
     * @function Load
     * @brief Load the controller
     */
    void worldSimPlugin::Load( physics::WorldPtr _parent, 
			       sdf::ElementPtr _sdf ) {
	// save pointers
	this->world = _parent;
	this->sdf = _sdf;
		
	// ros callback queue for processing subscription
	// this->deferredLoadThread = boost::thread(
	//   boost::bind(&worldSimPlugin::DeferredLoad, this));
	this->DeferredLoad();
    }
    
    /**
     * @function DeferredLoad
     * @brief Load the controller
     */
    void worldSimPlugin::DeferredLoad() {
	
	// initialize ros
	if ( !ros::isInitialized() ) {
	    gzerr << "Not loading worldSimPlugin since ROS hasn't been "
		  << "properly initialized.  Try starting gazebo with ros plugin:\n"
		  << "  gazebo -s libgazebo_ros_api_plugin.so\n";
	    return;
	}
	
	// ros stuff
	this->rosNode = new ros::NodeHandle("");
	
	// load WorldSimROSAPI
	this->LoadWorldSimROSAPI();
	
	// this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
	this->lastUpdateTime = this->world->GetSimTime().Double();
	this->robotCmdVel = geometry_msgs::Twist();
	
	// Load Robot
	this->drchubo.Load( this->world, this->sdf );

	// Load Vehicle
	this->vehicle.Load( this->world, this->sdf );

	// Setup ROS interfaces for robot
	this->LoadRobotROSAPI();

	// ros callback queue for processing subscription
	this->callbackQueueThread = boost::thread(
						  boost::bind(&worldSimPlugin::ROSQueueThread, this));
	
	// Mechanism for Updating every World Cycle
	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
									boost::bind(&worldSimPlugin::UpdateStates, this));
    }
    

    /**
     * @function LoadWorldSimROSAPI
     */
    void worldSimPlugin::LoadWorldSimROSAPI() {
	
	// ros subscription
	std::string robot_enter_car_topic_name = "drc_world/robot_enter_car";
	ros::SubscribeOptions robot_enter_car_so =
	    ros::SubscribeOptions::create<geometry_msgs::Pose>( robot_enter_car_topic_name, 100,
								boost::bind(&worldSimPlugin::RobotEnterCar, this, _1),
								ros::VoidPtr(), &this->rosQueue);
	this->subRobotEnterCar = this->rosNode->subscribe(robot_enter_car_so);
	
	std::string robot_exit_car_topic_name = "drc_world/robot_exit_car";
	ros::SubscribeOptions robot_exit_car_so =
	    ros::SubscribeOptions::create<geometry_msgs::Pose>( robot_exit_car_topic_name, 100,
								boost::bind(&worldSimPlugin::RobotExitCar, this, _1),
								ros::VoidPtr(), &this->rosQueue);
	this->subRobotExitCar = this->rosNode->subscribe(robot_exit_car_so);
    }
    
    
    /**
     * @function LoadRobotROSAPI
     */
    void worldSimPlugin::LoadRobotROSAPI() {
	
	// ros subscription
	std::string trajectory_topic_name = "drchubo/cmd_vel";
	ros::SubscribeOptions trajectory_so =
	    ros::SubscribeOptions::create<geometry_msgs::Twist>( trajectory_topic_name, 100,
								 boost::bind(&worldSimPlugin::SetRobotCmdVel, this, _1),
								 ros::VoidPtr(), &this->rosQueue );
	this->drchubo.subTrajectory = this->rosNode->subscribe(trajectory_so);
	
	std::string pose_topic_name = "drchubo/set_pose";
	ros::SubscribeOptions pose_so =
	    ros::SubscribeOptions::create<geometry_msgs::Pose>( pose_topic_name, 100,
								boost::bind(&worldSimPlugin::SetRobotPose, this, _1),
								ros::VoidPtr(), &this->rosQueue);
	
	this->drchubo.subPose = this->rosNode->subscribe(pose_so);
	
	std::string configuration_topic_name = "drchubo/configuration";
	ros::SubscribeOptions configuration_so =
	    ros::SubscribeOptions::create<sensor_msgs::JointState>( configuration_topic_name, 100,
								    boost::bind(&worldSimPlugin::SetRobotConfiguration, this, _1),
								    ros::VoidPtr(), &this->rosQueue );
	
	this->drchubo.subConfiguration = this->rosNode->subscribe(configuration_so);
	
	std::string mode_topic_name = "drchubo/mode";
	ros::SubscribeOptions mode_so =
      ros::SubscribeOptions::create<std_msgs::String>( mode_topic_name, 100,
						       boost::bind(&worldSimPlugin::SetRobotModeTopic, this, _1),
						       ros::VoidPtr(), &this->rosQueue);
	this->drchubo.subMode = this->rosNode->subscribe(mode_so);
	
}



    /**
     * @function SetRobotModeTopic
     * @brief Set robot modality
     */
    void worldSimPlugin::SetRobotModeTopic( const std_msgs::String::ConstPtr &_str ) {
	this->SetRobotMode(_str->data);
    }

    /**
     * @function SetRobotMode
     * @brief Set robot mode with code
     */
    void worldSimPlugin::SetRobotMode(const std::string &_str) {

	if (_str == "no_gravity") {

	    // stop warping robot
	    this->warpRobotWithCmdVel = false;
	    physics::Link_V links = this->drchubo.model->GetLinks();
	    for (unsigned int i = 0; i < links.size(); ++i) {
		links[i]->SetGravityMode(false);
	    }
	}
	else if (_str == "feet") {
	    // stop warping robot
	    this->warpRobotWithCmdVel = false;
	    physics::Link_V links = this->drchubo.model->GetLinks();
	    for (unsigned int i = 0; i < links.size(); ++i)
		{
		    if (links[i]->GetName() == "Body_LAR" || links[i]->GetName() == "Body_RAR")
			links[i]->SetGravityMode(true);
		    else
			links[i]->SetGravityMode(false);
		}

	}
	else if (_str == "nominal") {
	    // nominal
	    this->warpRobotWithCmdVel = false;
	    physics::Link_V links = this->drchubo.model->GetLinks();
	    for (unsigned int i = 0; i < links.size(); ++i)
		{
		    links[i]->SetGravityMode(true);
		}
	}
	else {
	    ROS_INFO("available modes for drchubo: no_gravity, feet,  nominal");
	}
    }

    /**
     * @function SetRobotCmdVel
     * @brief Set linear x/y velocity and angular yaw velocity
     */
    void worldSimPlugin::SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd) {

	if ( _cmd->linear.x == 0 && 
	     _cmd->linear.y == 0 && 
	     _cmd->angular.z == 0 ) {
	    this->warpRobotWithCmdVel = false;
	}
	else {
	    this->robotCmdVel = *_cmd;
	    this->warpRobotWithCmdVel = true;
	    this->lastUpdateTime = this->world->GetSimTime().Double();
	}
    }

    /**
     * @function SetRobotPose
     * @brief Defy the laws of gravity and teleport the robot to a new location
     */
    void worldSimPlugin::SetRobotPose( const geometry_msgs::Pose::ConstPtr &_pose ) {
	
	math::Quaternion q(_pose->orientation.w, _pose->orientation.x,
			   _pose->orientation.y, _pose->orientation.z);
	q.Normalize();
	math::Pose pose(math::Vector3(_pose->position.x,
				      _pose->position.y,
				      _pose->position.z), q);
	this->drchubo.model->SetWorldPose(pose);
    }
    


    /**
     * @function RobotEnterCar
     * @brief
     */
    void worldSimPlugin::RobotEnterCar( const geometry_msgs::Pose::ConstPtr &_pose ) {

	// Check if vehicle.model is loaded
	if (!this->vehicle.model) {
	    ROS_ERROR("vehicle model not found, cannot enter car.");
	    return;
	}

	math::Quaternion q(_pose->orientation.w, _pose->orientation.x,
			   _pose->orientation.y, _pose->orientation.z);
	q.Normalize();
	math::Pose pose(math::Vector3(_pose->position.x,
				      _pose->position.y,
				      _pose->position.z), q);
	
	// hardcoded offset of the robot when it's seated in the vehicle driver seat.
	this->drchubo.vehicleRelPose = math::Pose(math::Vector3(-0.06, 0.3, 2.02),
						  math::Quaternion());
	
	// turn physics off while manipulating things
	bool physics = this->world->GetEnablePhysicsEngine();
	bool paused = this->world->IsPaused();
	this->world->SetPaused(true);
	this->world->EnablePhysicsEngine(false);
	
	// set robot configuration
	this->drchuboCommandController.SetSeatingConfiguration(this->drchubo.model);
	ros::spinOnce();
	// give some time for controllers to settle
	// \todo: use joint state subscriber to check if goal is obtained
	gazebo::common::Time::MSleep(1000);
	ROS_INFO("Set robot seating-in-car configuration. Done");
	
	this->world->EnablePhysicsEngine(physics);
	this->world->SetPaused(paused);
	
	this->drchubo.model->SetLinkWorldPose(pose +
					      this->drchubo.vehicleRelPose + 
					      this->vehicle.model->GetWorldPose(),
					      this->drchubo.pinLink );
	
			
    }
    
    /**
     * @function RobotExitCar
     */
    void worldSimPlugin::RobotExitCar( const geometry_msgs::Pose::ConstPtr &_pose ) {
	// Check if drcVehicle.model is loaded
	if (!this->vehicle.model)
	    {
		ROS_ERROR("drc_vehicle model not found, cannot exit car.");
		return;
	    }
	math::Quaternion q(_pose->orientation.w, _pose->orientation.x,
			   _pose->orientation.y, _pose->orientation.z);
	q.Normalize();
	math::Pose pose(math::Vector3(_pose->position.x,
				      _pose->position.y,
				      _pose->position.z), q);
	
		
	// hardcoded offset of the robot when it's standing next to the vehicle.
	this->drchubo.vehicleRelPose = math::Pose(0.52, 1.7, 1.20, 0, 0, 0);
	
	// turn physics off while manipulating things
	bool physics = this->world->GetEnablePhysicsEngine();
	bool paused = this->world->IsPaused();
	this->world->SetPaused(true);
	this->world->EnablePhysicsEngine(false);
	// set robot configuration
	this->drchuboCommandController.SetStandingConfiguration(this->drchubo.model);
	ros::spinOnce();
	// give some time for controllers to settle
	// \todo: use joint state subscriber to check if goal is obtained
	gazebo::common::Time::MSleep(1000);
	ROS_INFO("set configuration done");
	
	this->world->EnablePhysicsEngine(physics);
	this->world->SetPaused(paused);
	
	// move model to new pose
	this->drchubo.model->SetLinkWorldPose(pose +
					      this->drchubo.vehicleRelPose + this->vehicle.model->GetWorldPose(),
					      this->drchubo.pinLink);
	
	gazebo::common::Time::MSleep(5000);
	

    }


////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void worldSimPlugin::UpdateStates()
{
  double curTime = this->world->GetSimTime().Double();

  if (curTime > this->lastUpdateTime)
  {

    double dt = curTime - this->lastUpdateTime;

    if (this->warpRobotWithCmdVel)
    {
      this->lastUpdateTime = curTime;
      math::Pose cur_pose = this->drchubo.pinLink->GetWorldPose();
      math::Pose new_pose = cur_pose;

      // increment x,y in cur_pose frame
      math::Vector3 cmd(this->robotCmdVel.linear.x,
                        this->robotCmdVel.linear.y, 0);
      cmd = cur_pose.rot.RotateVector(cmd);

      new_pose.pos = cur_pose.pos + cmd * dt;
      // prevent robot from drifting vertically
      new_pose.pos.z = this->drchubo.initialPose.pos.z;

      math::Vector3 rpy = cur_pose.rot.GetAsEuler();
      // decay non-yaw tilts
      rpy.x = 0;
      rpy.y = 0;
      rpy.z = rpy.z + this->robotCmdVel.angular.z * dt;

      new_pose.rot.SetFromEuler(rpy);

      // set this as the new anchor pose of the pin joint
      // UNCOMMENT THIS SOON!!!!!!!!!!!!!!!!!!!!!!1
     /*
      this->Teleport(this->atlas.pinLink,
                     this->atlas.pinJoint,
                     new_pose);
      */
    }
  }
}

    /**
     * @function ROSQueueThread
     */
    void worldSimPlugin::ROSQueueThread() {
	
	static const double timeout = 0.01;
	
	while (this->rosNode->ok()) {
	    this->rosQueue.callAvailable(ros::WallDuration(timeout));
	}
    }
    
    // ************************************
    // VEHICLE INFO 
    // ************************************

    /**
     * @function Load
     * @brief Load vehicle info from .world
     */
    void worldSimPlugin::Vehicle::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

	this->isInitialized = false;
	// load parameters
	if (_sdf->HasElement("vehicle") &&
	    _sdf->GetElement("vehicle")->HasElement("model_name")) {
	    this->model = _world->GetModel(_sdf->GetElement("vehicle")
					   ->GetValueString("model_name"));
	}
	else {
	    ROS_INFO("Can't find <vehicle><model_name> blocks. using default.");
	    this->model = _world->GetModel("vehicle");
	}
	
	if (!this->model) {
	    ROS_DEBUG("drc vehicle not found.");
	    return;
	}
	
	if ( _sdf->HasElement("vehicle") &&
	     _sdf->GetElement("vehicle")->HasElement("seat_link") ) {
	    this->seatLink = this->model->GetLink(_sdf->GetElement("vehicle")
						  ->GetValueString("seat_link"));
	}
	else {
	    ROS_INFO("Can't find <vehicle><seat_link> blocks, using default.");
	    this->seatLink = this->model->GetLink("chassis");
	}

	if (!this->seatLink) {
	    ROS_ERROR("vehicle seat link not found!!");
	    return;
	}

	// Note: hardcoded link by name: @todo: make this a pugin param
	this->initialPose = this->seatLink->GetWorldPose();
	this->isInitialized = true;
    }

    // *************************************
    // ROBOT FUNCTIONS
    // *************************************

    /**
     * @function Load
     */
    void worldSimPlugin::Robot::Load( physics::WorldPtr _world, 
				      sdf::ElementPtr _sdf ) {
	this->isInitialized = false;

	
	// load parameters
	if (_sdf->HasElement("robot") &&
	    _sdf->GetElement("robot")->HasElement("model_name")) {
	    this->model = _world->GetModel(_sdf->GetElement("robot")
					   ->GetValueString("model_name"));
	}
	else {
	    ROS_INFO("Can't find <drchubo><model_name> blocks. using default.");
	    this->model = _world->GetModel("drchubo");
	}
	
	if (!this->model) {
	    ROS_INFO("drchubo model not found!!");
	    return;
	}
	
	if (_sdf->HasElement("robot") &&
	    _sdf->GetElement("robot")->HasElement("pin_link")) {
	    this->pinLink = this->model->GetLink(_sdf->GetElement("robot")
						 ->GetValueString("pin_link"));
	}
	else
	    {
		ROS_INFO("Can't find <robot><pin_link> blocks, using default.");
		this->pinLink = this->model->GetLink("Body_Torso");
	    }
	
	if (!this->pinLink)
	    {
		ROS_ERROR("drchubo robot pin link not found!!");
		return;
	    }
	
	// Note: hardcoded link by name: @todo: make this a pugin param
	this->initialPose = this->pinLink->GetWorldPose();
	this->isInitialized = true;
    }





/**
 * @function SetRobotConfiguration
 */
void worldSimPlugin::SetRobotConfiguration( const sensor_msgs::JointState::ConstPtr &_cmd ) {

    // This function is planned but not yet implemented.
    ROS_ERROR("The atlas/configuration handler is not implemented.\n");
    
}

////////////////////////////////////////////////////////////////////////////////
worldSimPlugin::drchuboCommandController::drchuboCommandController() {

    // initialize ros
    if (!ros::isInitialized()) {

	gzerr << "Not loading AtlasCommandController since ROS hasn't been "
	      << "properly initialized.  Try starting Gazebo with"
	      << " ros plugin:\n"
	      << "  gazebo -s libgazebo_ros_api_plugin.so\n";
	return;
    }
    
    // ros stuff
    this->rosNode = new ros::NodeHandle("");
    
    // must match those inside AtlasPlugin
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


    unsigned int n = this->jointNames.size();
    this->jc.position.resize(n);
    this->jc.velocity.resize(n);
    this->jc.effort.resize(n);
    this->jc.kp_position.resize(n);
    this->jc.ki_position.resize(n);
    this->jc.kd_position.resize(n);
    this->jc.kp_velocity.resize(n);
    this->jc.i_effort_min.resize(n);
    this->jc.i_effort_max.resize(n);
    this->jc.k_effort.resize(n);
    
    for ( unsigned int i = 0; i < n; ++i ) {
	std::vector<std::string> pieces;
	boost::split(pieces, this->jointNames[i], boost::is_any_of(":"));
	
	double val;
	this->rosNode->getParam("drchubo_controller/gains/" + pieces[2] +
				"/p", val);
	this->jc.kp_position[i] = val;
	
	this->rosNode->getParam("drchubo_controller/gains/" + pieces[2] +
				"/i", val);
	this->jc.ki_position[i] = val;
	
	this->rosNode->getParam("drchubo_controller/gains/" + pieces[2] +
				"/d", val);
	this->jc.kd_position[i] = val;
	
	this->rosNode->getParam("drchubo_controller/gains/" + pieces[2] +
				"/i_clamp", val);
	this->jc.i_effort_min[i] = -val;
	this->jc.i_effort_max[i] = val;
	this->jc.k_effort[i] =  255;
	
	this->jc.velocity[i]     = 0;
	this->jc.effort[i]       = 0;
	this->jc.kp_velocity[i]  = 0;
    }

  this->pubDrchuboCommand =
    this->rosNode->advertise<robot_sim::JointCommands>(
    "/drchubo/drchubo_command", 1, true);


  // ros::SubscribeOptions jointStatesSo =
  //   ros::SubscribeOptions::create<sensor_msgs::JointState>(
  //   "/atlas/joint_states", 1,
  //   boost::bind(&AtlasCommandController::GetJointStates, this, _1),
  //   ros::VoidPtr(), this->rosNode->getCallbackQueue());
  // this->subJointStates =
  //   this->rosNode->subscribe(jointStatesSo);
}

/**
 * @function ~drchuboCommandController
 * @brief Destructor
 */
worldSimPlugin::drchuboCommandController::~drchuboCommandController() {

    this->rosNode->shutdown();
    delete this->rosNode;
}

/**
 * @function GetJointStates
 */
void worldSimPlugin::drchuboCommandController::GetJointStates( const sensor_msgs::JointState::ConstPtr &_js ) {
    /// \todo: implement joint state monitoring when setting configuration
}

/**
 * @function SetPIDStand
 */
void worldSimPlugin::drchuboCommandController::SetPIDStand( physics::ModelPtr _drchuboModel ) {

    // seated configuration
    this->jc.header.stamp = ros::Time::now();


  // StandPrep end pose --> Stand  pose
  this->jc.position[0]  =   2.438504816382192e-05;
  this->jc.position[1]  =   0.0015186156379058957;
  this->jc.position[2]  =   9.983908967114985e-06;
  this->jc.position[3]  =   -0.0010675729718059301;
  this->jc.position[4]  =   -0.0003740221436601132;
  this->jc.position[5]  =   0.06201673671603203;
  this->jc.position[6]  =  -0.2333149015903473;
  this->jc.position[7]  =   0.5181407332420349;
  this->jc.position[8]  =  -0.27610817551612854;
  this->jc.position[9]  =   -0.062101610004901886;
  this->jc.position[10] =  0.00035181696875952184;
  this->jc.position[11] =   -0.06218484416604042;
  this->jc.position[12] =  -0.2332201600074768;
  this->jc.position[13] =   0.51811283826828;
  this->jc.position[14] =  -0.2762000858783722;
  this->jc.position[15] =   0.06211360543966293;
  this->jc.position[16] =   0.29983898997306824;
  this->jc.position[17] =   -1.303462266921997;
  this->jc.position[18] =   2.0007927417755127;
  this->jc.position[19] =   0.49823325872421265;
  this->jc.position[20] =  0.0003098883025813848;
  this->jc.position[21] =   -0.0044272784143686295;
  this->jc.position[22] =   0.29982614517211914;
  this->jc.position[23] =   1.3034454584121704;
  this->jc.position[24] =   2.000779867172241;
  this->jc.position[25] =  -0.498238742351532;
  this->jc.position[26] =  0.0003156556049361825;
  this->jc.position[27] =   0.004448802210390568;


  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    this->jc.k_effort[i] =  255;

  // set joint positions
  std::map<std::string, double> jps;
  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    jps.insert(std::make_pair(this->jointNames[i], this->jc.position[i]));

  _drchuboModel->SetJointPositions(jps);

  // publish AtlasCommand
  this->pubDrchuboCommand.publish(jc);
}


/**
 * @function SetSeatingConfiguration
 * @brief
 */
void worldSimPlugin::drchuboCommandController::SetSeatingConfiguration( physics::ModelPtr _drchuboModel ) {

  // seated configuration
    this->jc.header.stamp = ros::Time::now();


    this->jc.position[0]  =   0.45;
    this->jc.position[1]  =   0.00;
    this->jc.position[2]  =  -1.60;
    this->jc.position[3]  =  1.60;
    this->jc.position[4]  =  -0.10;
    this->jc.position[5]  =  0.00;

    this->jc.position[6]  = -0.45;
    this->jc.position[7] =  0.00;
    this->jc.position[8] =  -1.60;
    this->jc.position[9] =  1.60;
    this->jc.position[10] =  -0.10;
    this->jc.position[11] =  0.00;

    this->jc.position[12] =   0.00;
    this->jc.position[13] =   0.00;
    this->jc.position[14] =   0.00;
    this->jc.position[15] =   1.50;
    this->jc.position[16] =   1.50;
    this->jc.position[17] =  -3.00;
    this->jc.position[18] =  0.00;

    this->jc.position[19] =   0.00;
    this->jc.position[20] =   0.00;
    this->jc.position[21] =   0.00;
    this->jc.position[22] =   1.50;
    this->jc.position[23] =  -1.50;
    this->jc.position[24] =  -3.00;
    this->jc.position[25] =  0.00;

    this->jc.position[26]  =   0.00;
    this->jc.position[27]  =   0.00;
    this->jc.position[28]  =   0.00;


    
    // set joint positions
    std::map<std::string, double> jps;
    for (unsigned int i = 0; i < this->jointNames.size(); ++i)
	jps.insert(std::make_pair(this->jointNames[i], this->jc.position[i]));
    
    _drchuboModel->SetJointPositions(jps);
    
    // publish AtlasCommand
    this->pubDrchuboCommand.publish(jc);
}

/**
 * @function SetStandingConfiguration
 * @brief
 */
void worldSimPlugin::drchuboCommandController::SetStandingConfiguration( physics::ModelPtr _drchuboModel ) {
 
    // standing configuration
    this->jc.header.stamp = ros::Time::now();
    this->jc.position[0]  =   0.00;
    this->jc.position[1]  =   0.00;
    this->jc.position[2]  =   0.00;
    this->jc.position[3]  =   0.00;
    this->jc.position[4]  =   0.00;
    this->jc.position[5]  =   0.00;
    this->jc.position[6]  =   0.00;
    this->jc.position[7]  =   0.00;
    this->jc.position[8]  =   0.00;
    this->jc.position[9]  =   0.00;
    this->jc.position[10] =   0.00;
    this->jc.position[11] =   0.00;
    this->jc.position[12] =   0.00;
    this->jc.position[13] =   0.00;
    this->jc.position[14] =   0.00;
    this->jc.position[15] =   0.00;
    this->jc.position[16] =   0.00;
    this->jc.position[17] =  -1.60;
    this->jc.position[18] =   0.00;
    this->jc.position[19] =   0.00;
    this->jc.position[20] =   0.00;
    this->jc.position[21] =   0.00;
    this->jc.position[22] =   0.00;
    this->jc.position[23] =   1.60;
    this->jc.position[24] =   0.00;
    this->jc.position[25] =   0.00;
    this->jc.position[26] =   0.00;
    this->jc.position[27] =   0.00;


  // set joint positions
  std::map<std::string, double> jps;
  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    jps.insert(std::make_pair(this->jointNames[i], this->jc.position[i]));

  _drchuboModel->SetJointPositions(jps);

  // publish AtlasCommand
  this->pubDrchuboCommand.publish(jc);
}
}
