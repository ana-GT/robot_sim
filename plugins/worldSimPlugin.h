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

#pragma once

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <robot_sim/JointCommands.h>

namespace gazebo {

    /**
     * @class worldSimPlugin
     */
    class worldSimPlugin : public WorldPlugin {
	/// \brief Constructor
    public: worldSimPlugin();
	
	/// \brief Destructor
    public: virtual ~worldSimPlugin();
	
	/// \brief Load the plugin.
	/// \param[in] _parent Pointer to parent world.
	/// \param[in] _sdf Pointer to sdf element.
    public: void Load( physics::WorldPtr _parent, 
		       sdf::ElementPtr _sdf );
	
	/// \brief Update the controller on every World::Update
    private: void UpdateStates();
	
	/// \brief Sets Atlas planar navigational command velocity
	/// \param[in] _cmd A Vector3, where:
	///   - x is the desired forward linear velocity, positive is robot-forward
	///     and negative is robot-back.
	///   - y is the desired lateral linear velocity, positive is robot-left
	///     and negative is robot-right.
	///   - z is the desired heading angular velocity, positive makes
	///     the robot turn left, and negative makes the robot turn right
    public: void SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd);
	
	/// \brief sets robot's absolute world pose
	/// \param[in] _cmd Pose command for the robot
    public: void SetRobotPose(const geometry_msgs::Pose::ConstPtr &_cmd);
	
	/// \brief sets robot's joint positions
	/// \param[in] _cmd configuration made of sensor_msgs::JointState message
	/// \todo: not yet implemented
    public: void SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr
				       &/*_cmd*/);
            
        
    /// \brief sets robot mode via ros topic
    /// \sa SetRobotMode(const std::string &_str)
    public: void SetRobotModeTopic(const std_msgs::String::ConstPtr &_str);

    /// \brief sets robot mode
    /// \param[in] _str sets robot mode by a string.  Supported modes are:
    ///  - "no_gravity" Gravity disabled for the robot.
    ///  - "nominal" Nominal "normal" physics.
    ///  - "pinned" Robot is pinned to inertial world by the pelvis.
    ///  - "feet" same as no_gravity except for r_foot and l_foot links.
    public: void SetRobotMode(const std::string &_str);


    /// \brief Robot Vehicle Interaction, put robot in driver's seat.
    /// \param[in] _pose Relative pose offset, Pose()::Zero provides default
    ///                 behavior.
    public: void RobotEnterCar(const geometry_msgs::Pose::ConstPtr &_pose);

    /// \brief Robot Vehicle Interaction, put robot outside driver's side door.
    /// \param[in] _pose Relative pose offset, Pose()::Zero provides default
    ///                 behavior.
    public: void RobotExitCar(const geometry_msgs::Pose::ConstPtr &_pose);

   
    /// \brief setup Robot ROS publication and sbuscriptions for the Robot
    /// These ros api describes Robot only actions
    private: void LoadRobotROSAPI();

    /// \brief setup ROS publication and sbuscriptions for VRC
    /// These ros api describes interactions between different models
    /// /atlas/cmd_vel - in pinned mode, the robot teleports based on
    ///                      messages from the cmd_vel
    private: void LoadWorldSimROSAPI();

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

      /// \brief ROS callback queue thread
  private: void ROSQueueThread();
      
      // ************************************************
      //   drchubo properties and states                                          
      // ************************************************
  private: class Robot {
	  
	  /// \brief Load the atlas portion of plugin.
	  /// \param[in] _parent Pointer to parent world.
	  /// \param[in] _sdf Pointer to sdf element.
      private: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
	  
      private: physics::ModelPtr model;
      private: physics::LinkPtr pinLink;
      private: physics::JointPtr pinJoint;

	  /// \brief keep initial pose of robot to prevent z-drifting when
	  /// teleporting the robot.
      private: math::Pose initialPose;
	  
	  /// \brief Pose of robot relative to vehicle.
      private: math::Pose vehicleRelPose;
	  
	  /// \brief Robot configuration when inside of vehicle.
      private: std::map<std::string, double> inVehicleConfiguration;

      /// \brief flag for successful initialization of robot
      private: bool isInitialized;

      private: ros::Subscriber subTrajectory;
      private: ros::Subscriber subPose;
      private: ros::Subscriber subConfiguration;
      private: ros::Subscriber subMode;

      friend class worldSimPlugin;
    } drchubo;


      // *********************************************
      //   DRC Vehicle properties and states                                    
      // *********************************************
  private: class Vehicle {

	  /// \brief Load the drc vehicle portion of plugin.
	  /// \param[in] _parent Pointer to parent world.
	  /// \param[in] _sdf Pointer to sdf element.
      private: void Load( physics::WorldPtr _parent, 
			  sdf::ElementPtr _sdf );
	  
      private: physics::ModelPtr model;
      private: math::Pose initialPose;
      private: physics::LinkPtr seatLink;
	  
	  /// \brief flag for successful initialization of vehicle
      private: bool isInitialized;
	  
	  friend class worldSimPlugin;
      } vehicle;
      

      // *********************************************
      //   Robot Joint Controller                                               
      // *********************************************
  private: class drchuboCommandController {

	  /// \brief Constructor, note atlas_controller is the name
	  /// of the controller loaded from yaml
      private: drchuboCommandController();
	  
      /// \brief Destructor
      private: ~drchuboCommandController();
	  
	  /// \brief subscriber to joint_states of the atlas robot
      private: void GetJointStates( const sensor_msgs::JointState::ConstPtr &_js );
	  
      /// \brief stand configuration with PID controller
      /// \param[in] pointer to atlas model
      private: void SetPIDStand( physics::ModelPtr _drchuboModel );
	  
      /// \brief sitting configuration of the robot when it enters the vehicle.
      /// \param[in] pointer to atlas model
      private: void SetSeatingConfiguration( physics::ModelPtr _drchuboModel );

      /// \brief standing configuration of the robot when it exits the vehicle.
      /// \param[in] _atlasModel pointer to atlas model
      private: void SetStandingConfiguration( physics::ModelPtr _drchuboModel );

	  /// \brief subscriber to joint_states
      private: ros::Subscriber subJointStates;

	  /// \brief publisher of joint_commands
      private: ros::Publisher pubJointCommand;
	  
	  /// \brief ros node handle
      private: ros::NodeHandle* rosNode;
	  
	  
	  /// \brief 
      private: robot_sim::JointCommands jc;

      /// \brief hardcoded joint names for atlas
      private: std::vector<std::string> jointNames;

      friend class worldSimPlugin;
    } drchuboCommandController;

      // **********************************
      //   Private variables                                                   
      // **********************************
  private: bool warpRobotWithCmdVel;
  private: double lastUpdateTime;
  private: geometry_msgs::Twist robotCmdVel;
            
      /// \brief Pointer to parent world.
  private: physics::WorldPtr world;
      
      /// \brief Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
      
      // default ros stuff
  private: ros::NodeHandle* rosNode;
  private: ros::CallbackQueue rosQueue;
  private: boost::thread callbackQueueThread;
      
  private: ros::Subscriber subRobotEnterCar;
  private: ros::Subscriber subRobotExitCar;

      // items below are used for deferred load in case ros is blocking
  private: sdf::ElementPtr sdf;
  private: boost::thread deferredLoadThread;
      

  };
    /** \} */
    /// @}
}
