#include <ros/ros.h>
#include <dq_robotics/DQoperations.h>
#include <dq_robotics/ResolveAccControlPrfmnc.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdexcept> //for range_error
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
std::ofstream result_file;
geometry_msgs::PoseStamped desiredPose_gMsgPose, currentPose_gMsgPose;
geometry_msgs::TwistStamped desiredVel_gMsgTwist, currentVel_gMsgTwist; 
// void ManipulatorModDQ::joint_state_callback(const sensor_msgs::JointState& jointStateConsPtr)
double timeStamp, x_current, y_current, z_current, qw_current, qx_current, qy_current, qz_current, x_desired, y_desired, z_desired, qw_desired, qx_desired, qy_desired, qz_desired, x_curr_transVel, y_curr_transVel, z_curr_transVel, x_curr_rotVel, y_curr_rotVel, z_curr_rotVel, x_des_transVel, y_des_transVel, z_des_transVel, x_des_rotVel, y_des_rotVel, z_des_rotVel, normError, timeKDLJacDot, timeDQJacDot;

int controllerMode;
// geometry_msgs/PoseStamped desiredPose # this can be relative desire pose
// geometry_msgs/PoseStamped currentPose # this can be relative desire pose
// geometry_msgs/TwistStamped desiredVel # this can be relative desire pose
// geometry_msgs/TwistStamped currentVel # this can be relative desire pose
// float64 timeStamp
// int32 controlMode
// float64 norm_error 

void chatterCallback(const dq_robotics::ResolveAccControlPrfmnc& msg)
{
	timeStamp = msg.timeStamp;

	x_current = msg.currentPose.pose.position.x;
	y_current = msg.currentPose.pose.position.y;
	z_current = msg.currentPose.pose.position.z;
	qw_current = msg.currentPose.pose.orientation.w;
	qx_current = msg.currentPose.pose.orientation.x;
	qy_current = msg.currentPose.pose.orientation.y;
	qz_current = msg.currentPose.pose.orientation.z;

	x_desired = msg.desiredPose.pose.position.x;
	y_desired = msg.desiredPose.pose.position.y;
	z_desired = msg.desiredPose.pose.position.z;
	qw_desired = msg.desiredPose.pose.orientation.w;
	qx_desired = msg.desiredPose.pose.orientation.x;
	qy_desired = msg.desiredPose.pose.orientation.y;
	qz_desired = msg.desiredPose.pose.orientation.z;

	x_curr_transVel = msg.currentVel.twist.linear.x;
	y_curr_transVel = msg.currentVel.twist.linear.y;
	z_curr_transVel = msg.currentVel.twist.linear.z;
	x_curr_rotVel = msg.currentVel.twist.angular.x;
	y_curr_rotVel = msg.currentVel.twist.angular.y;
	z_curr_rotVel = msg.currentVel.twist.angular.z;

	x_des_transVel = msg.desiredVel.twist.linear.x;
	y_des_transVel = msg.desiredVel.twist.linear.y;
	z_des_transVel = msg.desiredVel.twist.linear.z;
	x_des_rotVel = msg.desiredVel.twist.angular.x;
	y_des_rotVel = msg.desiredVel.twist.angular.y;
	z_des_rotVel = msg.desiredVel.twist.angular.z;
	normError = msg.norm_error;
	controllerMode = msg.controlMode;
	timeKDLJacDot = msg.timeKDLJacDot;
	timeDQJacDot = msg.timeDQJacDot;

	result_file << timeStamp  << "," << x_current << ","  << y_current  << "," << z_current << "," << qw_current << "," << qx_current << "," << qy_current << "," << qz_current << "," << x_desired << "," << y_desired << "," << z_desired << "," << qw_desired << "," << qx_desired << "," << qy_desired << "," << qz_desired << "," << x_curr_transVel << "," << y_curr_transVel << "," << z_curr_transVel << "," << x_curr_rotVel << "," << y_curr_rotVel << "," << z_curr_rotVel << "," << x_des_transVel << "," << y_des_transVel << "," <<  z_des_transVel << "," << x_des_rotVel << "," << y_des_rotVel << "," << z_des_rotVel << "," << normError << "," << controllerMode << "," << timeKDLJacDot << "," << timeDQJacDot << ", \n";	

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "resultAnalyzer");
	ros::NodeHandle node_;
	ros::Subscriber sub = node_.subscribe("/dq_robotics/trajCntrlResults", 1000, chatterCallback);

	// <dq_robotics::ResolveAccControlPrfmnc>("/dq_robotics/trajCntrlResults
	std::string result_file_name, result_file_path;
	result_file_name = "quatVec_z_axis_0.01_pitch.csv";
	result_file_path = ros::package::getPath("dq_robotics");
	result_file_path.append("/src/resolvedAccControl/result/");
	result_file_path.append(result_file_name);
	std::cout << "result_file_name: " << result_file_path << std::endl;
	
	result_file.open(result_file_path);
	if(result_file.is_open())
	{
		std::cout << "file is open" << std::endl;
	}
	result_file << "time" << "," << "x_current"  << "," << "y_current"  << "," << "z_current" << "," << "qw_current" <<  "," << "qx_current" <<  "," << "qy_current" <<  "," << "qz_current" <<  "," << "x_desired"  << "," << "y_desired"  << "," << "z_desired" << "," << "qw_desired" <<  "," << "qx_desired" <<  "," << "qy_desired" <<  "," << "qz_desired" <<  ","  << "x_curr_transVel"  << "," << "y_curr_transVel"  << "," << "z_curr_transVel" << "," << "x_curr_rotVel"  << "," << "y_curr_rotVel"  << "," << "z_curr_rotVel" << "," << "x_des_transVel"  << "," << "y_des_transVel"  << "," << "z_des_transVel" << "," << "x_des_rotVel"  << "," << "y_des_rotVel"  << "," << "z_des_rotVel"  << "," << "normError" << "," << "controllerMode" << "," << "timeKDLJacDot" << "," << "timeDQJacDot" << ", \n";

	ros::spin();
	return 0;
}