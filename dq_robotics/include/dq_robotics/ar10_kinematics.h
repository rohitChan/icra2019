#ifndef AR10_KINEMATICS_H
#define AR10_KINEMATICS_H

#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdexcept> //for range_error
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <complex>
#include <cmath>
#include <vector>
#include <cstddef>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <ros/package.h>
#include <dq_robotics/dq_controller.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <dq_robotics/IkHand.h>
#include <ros/package.h>

using namespace Eigen;

struct slider_params
{
	double b, c, init_displacement, total_displacement, total_sliderCmd, init_sliderCmd, current_sliderCmd, alpha, alpha_cal;
};	

struct four_bar_params
{
	double p, qq, g, h, theta, alpha, psi, theta_init;
};

struct finger_params
{
	slider_params proximal_phalanx, middle_phalanx;
	four_bar_params distal_phalanx;
	std::vector<std::string> finger_joint_names, finger_frame_names;
	std::vector<int> sliderOrderInjointState;
	std::vector<double> slider_position_input, slider_position_output;
	std::vector<RowVector3d> u, p;
	RowVectorXd q, q_init;
	Matrix<double,8,1> finger_ee_init, finger_ee_current, frame_planarBaseToBase, frame_rot0ToBase_init, frame_rot1ToBase_init, frame_rot2ToBase_init, frame_rot0ToBase_current, frame_rot1ToBase_current, frame_rot2ToBase_current;
	std::vector<int> finger_joint_type;
	std::vector<Matrix<double,8,1>> dq_frames_stack_init, dq_frames_stack_current;
	std::vector<Matrix<double,8,1> > screwDispalcementArray;
	MatrixXd finger_jacobian_simple, finger_jacobian_coupled;
	std::vector<int> joint_type;
	int joint_size;
	bool is_thumb;
};

class AR10Kinematics
{
	private:
		ros::NodeHandle n;
		Matrix<double,8,1> frame_0, frame_1, frame_2, frame_3, frame_hand_base;
		finger_params index_finger, thumb;
		ros::Publisher right_ar10Hand_pub, left_ar10Hand_pub;
		ros::Subscriber jointCmds_subscriber, fkm_subscriber;
		sensor_msgs::JointState right_hand_cmds, left_hand_cmds;
		float durationSleep;
		std::string base_frame, result_file_name, path;
	  tf2_ros::StaticTransformBroadcaster static_broadcaster;
	  ros::ServiceServer finger_ik_service;
	  double slider_cmd_tolerance, dt, Kp, mu;
	  std::vector<geometry_msgs::TransformStamped> static_transformStamped_array;
  	geometry_msgs::TransformStamped hand_base_static_transformStamped;
  	int joint_size_coupled;
		std::vector<RowVectorXd>  u_relative, p_relative;
		Matrix<double,8,1> ee_relative;
		RowVectorXd q_relative, q_init_relative;

	  // std::ofstream result_file;
  	
		//double b_1, c_1, s_1, b_2, c_2, s_2, a_3, b_3, g_3, h_3, theta_1_diff, theta_2_diff, theta_3_diff;  

	public:
		AR10Kinematics();
		~AR10Kinematics();		
		void init_AR10Kinematics();
		static void getThetaFromSlider(slider_params &sp);
		static void getAlphaFromTheta4Bar(four_bar_params &four_bar);
		static double getSliderFromTheta(slider_params sp);
		std::vector<int> getSlidersList(std::vector<std::string> joint_names);
		bool get_index_param();
		bool getControllerParms();
		void init_fingers();
		void init_thumb();
		bool get_thumb_param();
		void init_index_finger();
		void init_index_finger_2();
		void getSliderPosition(finger_params &finger);
		void computeJointRotation(finger_params &finger);
		void fkmFinger(finger_params &finger);
		void Ar10KinematicsLoop();
		void Ar10KinematicsLoop_finger(finger_params &finger);
		void publishFramesAsTf(finger_params &finger, std::vector<geometry_msgs::TransformStamped> &static_transformStamped_array);
		// void fkmCallback(const sensor_msgs::JointState& msg);
		bool computeHandIKCallback(dq_robotics::IkHand::Request& hand_ik_req, 
									dq_robotics::IkHand::Response& hand_ik_res);
		void getJacobianCoupled(finger_params &finger);
		void update_relative();
		bool ik_relative(dq_robotics::IkHand::Request hand_ik_req, dq_robotics::IkHand::Response &hand_ik_res);
		bool importHandState(std::string finger_name, Matrix<double,8,1>& pe_init, Matrix<double,8,1>& pe_now, MatrixXd& jacobian, RowVectorXd& q, RowVectorXd& q_init);
		bool updateFingerVariables(std::string finger_name);
		void init_AR10IKService();
		bool sendJointCmds_AR10(std::vector<double> jointCmds, std::string fingerName);
	// int joint_size;
};
#endif

