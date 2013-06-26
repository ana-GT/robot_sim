/*
 * Copyright 2013 A.C.H.Q.
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

#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>

#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/Sensor.hh>

#include <robot_sim/robotState.h>
#include <robot_sim/JointCommands.h>

// Control stuff
#include <sensor_msgs/JointState.h>

#include "drcsim_plugins/PubQueue.h"

namespace gazebo
{
    /**
     * @class drchuboPlugin
     */
    class drchuboPlugin : public ModelPlugin
    {
	/// \brief Constructor
    public: drchuboPlugin();
	
	/// \brief Destructor
    public: virtual ~drchuboPlugin();
	
	/// \brief Load the controller
    public: void Load( physics::ModelPtr _parent, 
		       sdf::ElementPtr _sdf );
	
	/// \brief: Load ROS related stuff
    private: void LoadROS();
	
	/// \brief Update the controller
    private: void UpdateStates();

	/// \brief ROS callback queue thread
    private: void RosQueueThread();

    /// \brief get data from IMU for robot state
    /// \param[in] _curTime current simulation time
    private: void GetIMUState(const common::Time &_curTime);

    /// \brief get data from force torque sensor
    private: void GetForceTorqueSensorState(const common::Time &_curTime);

    private: void GetAndPublishRobotStates(const common::Time &_curTime);

    /// \brief pointer to gazebo world
    private: physics::WorldPtr world;

    /// \brief pointer to gazebo Atlas model
    private: physics::ModelPtr model;

    /// Pointer to the update event connections
    private: event::ConnectionPtr updateConnection;

    /// Throttle update rate
    private: common::Time lastControllerStatisticsTime;

    /// \brief A combined AtlasState, IMU and ForceTorqueSensors Message
    /// for accessing all these states synchronously.
    private: robot_sim::robotState drchuboState;

    /// \brief internal copy of sdf for the plugin
    private: sdf::ElementPtr sdf;

    // ROS internal stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue rosQueue;
    private: boost::thread callbackQueeuThread;


    /// \brief ROS publisher for atlas joint states
    private: ros::Publisher pubJointStates;
    private: PubQueue<sensor_msgs::JointState>::Ptr pubJointStatesQueue;

    /// \brief ROS publisher for robot state, currently it contains
    /// joint index enums
    /// atlas_msgs::AtlasState
    private: ros::Publisher pubRobotState;
    private: PubQueue<robot_sim::robotState>::Ptr pubRobotStateQueue;

    private: ros::Subscriber subJointCommands;


    /// \brief ros topic callback to update Joint Commands slowly.
    /// Control conmmands received through /atlas/joint_commands are
    /// not expected to be able to close near 1kHz.
    /// \param[in] _msg Incoming ros message
    private: void SetJointCommands(
      const robot_sim::JointCommands::ConstPtr &_msg);


    /// \brief Update PID Joint Servo Controllers
    /// \param[in] _dt time step size since last update
    private: void UpdatePIDControl(double _dt);

	// ************************************
	//  Some Helper Functions                                                 
	// ************************************
    private: void LoadPIDGainsFromParameter();
    private: void ZeroJointCommands();

    /// \brief keep a list of hard coded drchubo joint names.
    private: std::vector<std::string> jointNames;

    /// \brief internal bufferred controller states
    private: robot_sim::JointCommands jointCommands;
    private: sensor_msgs::JointState jointStates;

    // JointController: pointer to a copy of the joint controller in gazebo
    // \TODO: not yet functional
    private: physics::JointControllerPtr jointController;
    private: transport::NodePtr node;
    private: transport::PublisherPtr jointCmdPub;

    /// \brief Internal list of pointers to Joints
    private: physics::Joint_V joints;
    private: std::vector<double> effortLimit;

    /// \brief internal class for keeping track of PID states
    private: class ErrorTerms
      {
        /// error term contributions to final control output
        double q_p;
        double d_q_p_dt;
        double k_i_q_i;  // integral term weighted by k_i
        double qd_p;
        friend class drchuboPlugin;
      };
    private: std::vector<ErrorTerms> errorTerms;

    private: boost::mutex mutex;

	/// \brief: for keeping track of internal controller update rates.
    private: common::Time lastControllerUpdateTime;
	
	// \brief: ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;
	
	/// \brief Current joint cfm damping coefficient
    private: std::vector<double> lastJointCFMDamping;

	/// \brief: Current joint damping coefficient for the Model
    private: std::vector<double> jointDampingModel;
	
	/// \brief: Joint damping coefficient upper bound
    private: std::vector<double> jointDampingMax;

	/// \brief: Joint damping coefficient lower bounds
    private: std::vector<double> jointDampingMin;

    ////////////////////////////////////////////////////////////////////
    //                                                                //
    //   helper conversion functions                                  //
    //                                                                //
    ////////////////////////////////////////////////////////////////////
    /// \brief Conversion functions
    private: inline math::Pose ToPose(const geometry_msgs::Pose &_pose) const
    {
      return math::Pose(math::Vector3(_pose.position.x,
                                      _pose.position.y,
                                      _pose.position.z),
                        math::Quaternion(_pose.orientation.w,
                                         _pose.orientation.x,
                                         _pose.orientation.y,
                                         _pose.orientation.z));
    }

    /// \brief Conversion helper functions
    private: inline geometry_msgs::Pose ToPose(const math::Pose &_pose) const
    {
      geometry_msgs::Pose result;
      result.position.x = _pose.pos.x;
      result.position.y = _pose.pos.y;
      result.position.z = _pose.pos.y;
      result.orientation.w = _pose.rot.w;
      result.orientation.x = _pose.rot.x;
      result.orientation.y = _pose.rot.y;
      result.orientation.z = _pose.rot.z;
      return result;
    }

    /// \brief Conversion helper functions
    private: inline geometry_msgs::Quaternion ToQ(const math::Quaternion &_q)
      const
    {
      geometry_msgs::Quaternion result;
      result.w = _q.w;
      result.x = _q.x;
      result.y = _q.y;
      result.z = _q.z;
      return result;
    }

  };
}
