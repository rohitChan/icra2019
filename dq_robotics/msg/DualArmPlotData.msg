float64 dt # Difference between time now and previous update

bool using_rel
bool using_right
bool using_position_control
float64 dt_total
sensor_msgs/JointState currentJointState
sensor_msgs/JointState desiredJointState

float64[] joint_limit_low
float64[] joint_limit_high
float64[] safe_joint_limit_low
float64[] safe_joint_limit_high
float64[] joint_velocity_limit

float64[] poseDesiredDQ_rel
geometry_msgs/Pose poseDesired_rel

float64[] poseNowDQ_rel
geometry_msgs/Pose poseNow_rel

float64[] poseDotNowDQ_rel
geometry_msgs/Pose poseDotNow_rel

float64[] poseDotDesiredDQ_rel
geometry_msgs/Pose poseDotDesired_rel

float64[] poseDesiredDQ_right
geometry_msgs/Pose poseDesired_right

float64[] poseNowDQ_right
geometry_msgs/Pose poseNow_right

float64[] poseDotNowDQ_right
geometry_msgs/Pose poseDotNow_right

float64[] poseDotDesiredDQ_right
geometry_msgs/Pose poseDotDesired_right

float64[] qDotPosition_cmd
float64[] qDotNullSpace_cmd

float64[] poseError

float64[] screwError_right
float64 normScrewError_right

float64 normScrewError_rel
float64[] screwError_rel

float64 normError_jointLimit
float64 Kp
float64 K_jl

