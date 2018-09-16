#ifndef MANIPULATOR_DQ_MOD_H
#define MANIPULATOR_DQ_MOD_H
 
#include <ros/ros.h>
#include <ros/package.h>
#include <dq_robotics/DQoperations.h>
// #include <baxter_trajectory_interface/GetIKSolution.h>
// #include <baxter_trajectory_interface/GetJointTrajectory.h>
// #include <baxter_trajectory_interface/ControlSystemStates.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <urdf/model.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <math.h>
#include "baxter_core_msgs/JointCommand.h"
#include <sensor_msgs/JointState.h>

using namespace Eigen;

class ManipulatorModDQ
{
private:
	ros::NodeHandle rh;
	// baxter_trajectory_interface::ControlSystemStates state;
	std::vector<RowVector3d> u, p, u_right, p_right, u_left, p_left, u_baxter, p_baxter;
	Matrix<double,8,1> pe_init_left, pe_init_right, pe_now, pe_init, pe_now_dot_dq;
	MatrixXd jacobian, jacobian_8d, joint_limit_jacobian, jointLimit_repulsive_function, pe_now_htm, pe_init_htm;
	RowVectorXd q_init, q, q_vel, q_dual, q_baxter, q_vel_baxter;
	std::vector<int> joint_type, joint_type_left, joint_type_right, joint_type_baxter;
	int joint_size, joint_size_right, joint_size_left, joint_size_baxter; 
	std::string root_name, tip_name_left, tip_name_right, tip_name;
	std::vector<std::string> joint_names, joint_names_left, joint_names_right, joint_names_baxter;
	std::vector<double> velocity_limit, velocity_limit_ordered, velocity_limit_ordered_left, velocity_limit_ordered_right, velocity_limit_ordered_baxter, joint_low_limit, joint_high_limit, joint_low_limit_ordered, joint_low_limit_ordered_left, joint_low_limit_ordered_right, joint_low_limit_ordered_baxter, joint_high_limit_ordered, joint_high_limit_ordered_left, joint_high_limit_ordered_right, joint_high_limit_ordered_baxter, min_safe, min_safe_left, min_safe_right, min_safe_baxter, max_safe, max_safe_left, max_safe_right, max_safe_baxter, joint_mid_point, joint_mid_point_left, joint_mid_point_right, joint_mid_point_baxter, vel_desired_control;
	
	std::vector<Matrix<double,8,1> > fkm_current;
	// urdf::Model robot_model;
	std::string xml;
	ros::Time last_time, current_time;
	double fraction_jointLimit, dt, totalTime, sleepTimeout;

	ros::Publisher left_cmd_pub, right_cmd_pub;
	baxter_core_msgs::JointCommand cmd_left, cmd_right;
	ros::Subscriber joint_state_sub;
public:
	void joint_state_callback(const sensor_msgs::JointState& jointStateConsPtr);
	bool initialize_baxter();
	bool getRobotParams_baxter();
	bool loadModel_baxter();
	bool readJoints_baxter(std::string tip_name, std::vector<std::string>  &joint_names_new, urdf::Model &robot_model);
	void getCurrentJointState_baxter();
	// void fkmDual();
	// void jacobianDual();
	// void jacobianDual_8d();
	// void getDQderivative();
	
	void mapJointName2JointCmds();
	void robotParams(std::vector<RowVector3d> &u_, std::vector<RowVector3d> &p_, Matrix<double,8,1> &pe_init_left_, Matrix<double,8,1> &pe_init_right_, std::vector<double> &joint_high_limit_, std::vector<double> &joint_low_limit_, std::vector<double> &velocity_limit_, std::vector<double> &max_safe_, std::vector<double> &min_safe_, std::vector<std::string> &joint_names_, std::vector<int> &joint_type_);
	void currentRobotState(ros::Time &current_time_, RowVectorXd &q_, RowVectorXd &q_vel_);
	void sendJointCommandBaxter(std::vector<double> jointCmds_, int mode, double sleepTimeout_, bool velocity_control=true);
	// void initialize_leftArm();
	// void initialize_rightArm();
	// void robot_state_right();
	// void robot_state_left();
	ManipulatorModDQ();
	~ManipulatorModDQ();
};
#endif