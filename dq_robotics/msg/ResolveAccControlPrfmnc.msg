geometry_msgs/PoseStamped desiredPose # this can be relative desire pose
geometry_msgs/PoseStamped currentPose # this can be relative desire pose
geometry_msgs/TwistStamped desiredVel # this can be relative desire pose
geometry_msgs/TwistStamped currentVel # this can be relative desire pose
float64 timeStamp
float64 timeKDLJacDot
float64 timeDQJacDot
int32 controlMode
float64 norm_error 