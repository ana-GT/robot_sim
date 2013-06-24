#
# This message has been carefully constructed to be less
# than 1500 in size when serialized, to accommodate transfer
# UDP.
#
# testing everything a robot needs
Header header

# Default joint indices used when publishing the
# JointCommands joint_states topic below
# For example, if you subscribe to this message, then
# msg.joint_states.position[robot_sim::robotState::back_lbz] gives back
# the position of the back_lbz.
int32 LSP  = 0
int32 LSR  = 1
int32 LSY  = 2
int32 LEP   = 3
int32 LWY = 4
int32 LWP = 5
int32 LWR = 6

int32 RSP = 7
int32 RSR = 8
int32 RSY = 9
int32 REP = 10
int32 RWY = 11
int32 RWP = 12
int32 RWR = 13

int32 LHY = 14
int32 LHR = 15
int32 LHP = 16
int32 LKP = 17
int32 LAP = 18
int32 LAR = 19

int32 RHY = 20
int32 RHR = 21
int32 RHP = 22
int32 RKP = 23
int32 RAP = 24
int32 RAR = 25

int32 TSY = 26
int32 NKY = 27
int32 NKP = 28

# repeating data from osrf_msgs/JointCommands as joint_states
float32[] position
float32[] velocity
float32[] effort
float32[] kp_position
float32[] ki_position
float32[] kd_position
float32[] kp_velocity
float32[] i_effort_min
float32[] i_effort_max

uint8[] k_effort       # k_effort can be an unsigned int 8value from 0 to 255, 
                       # at run time, a double between 0 and 1 is obtained
                       # by dividing by 255.0d.


#sensor_msgs/Imu imu 
#geometry_msgs/Quaternion orientation
#geometry_msgs/Vector3 angular_velocity
#geometry_msgs/Vector3 linear_acceleration

#atlas_msgs/ForceTorqueSensors force_torque_sensors
#geometry_msgs/Wrench l_foot
#geometry_msgs/Wrench r_foot
#geometry_msgs/Wrench l_hand
#geometry_msgs/Wrench r_hand
