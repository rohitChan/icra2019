#include <dq_robotics/DQoperations.h>
#include <dq_robotics/baxter_dq.h>
#include <dq_robotics/dq_controller.h>
#include <dq_robotics/DualArmPlotData.h>
#include <dq_robotics/baxter_poseControl_server.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <math.h>
#include "baxter_core_msgs/JointCommand.h"
#include "dq_robotics/BaxterControl.h"
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <dynamic_reconfigure/server.h>
#include <dq_robotics/kdl_controllerConfig.h>
#include <dq_robotics/jacDotSolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "baxter_core_msgs/JointCommand.h"
#include <dq_robotics/ResolveAccControlPrfmnc.h>
// ros::NodeHandle node_;
	// 

bool start_controller, reset_robot, reset_controller;
KDL::ChainJntToJacDotSolver *jacDotKDL;
KDL::ChainJntToJacSolver  *jacKDL;
KDL::Jacobian jac, jdot;
double K_p, K_d, K_i, total_time, time_now_total, theta_init, theta_final, norm_error, traj_pitch;
ros::WallTime start_, end_, start_kdl, end_kdl, start_dq, end_dq, start_total, end_total;
std::string robot_desc_string;	
KDL::ChainIdSolver_RNE *idsolver;
KDL::Tree tree;
KDL::Chain chain;
KDL::Vector g;
KDL::Rotation rot_correction;
Matrix4d crclCntr_dsrdTraj, startPose_dsrdTraj;
Matrix<double,8,1> pe_init_right, pe_now_right, crclCntr_dsrdTraj_dq, startPose_dsrdTraj_dq, lineTraj_dq_cc, pose_desired_right, vel_desired_right, acc_desired_right, acc_cmd_right;
RowVectorXd q_right, q_vel_right, qdd_cmd, vel_now;
MatrixXd jacobian_6d_right, jacobian_6d_dot_right, jacKDL_eigen, jacDotKDL_eigen;
BaxterPoseControlServer* baxter_controller;
double time_now, theta, theta_d, theta_dd, time_dq_jacDot, time_kdl_jacDot;
RowVector3d v_e, w_e;
KDL::JntArray q_kdl;
KDL::JntArray dq_kdl;
KDL::JntArray v_kdl;
KDL::JntArray torque_kdl;
KDL::Wrenches fext;
ros::Publisher right_cmd_pub ;
baxter_core_msgs::JointCommand cmd;
geometry_msgs::PoseStamped desired_pose;
ros::Publisher pose_publisher;
int controller_type;
int joint_size;
dq_robotics::ResolveAccControlPrfmnc resultMsg;
ros::Publisher result_topic;
geometry_msgs::PoseStamped desiredPose_gMsgPose, currentPose_gMsgPose;
geometry_msgs::TwistStamped desiredVel_gMsgTwist, currentVel_gMsgTwist; 
RowVectorXd vel_cart_now, vel_cart_desired;

void initializeTraj()
{
	lineTraj_dq_cc << 0, 0, 0, 1, 0, 0, 0, 0;
	crclCntr_dsrdTraj << 1, 0, 0, 0.6,
						 0, 1, 0, -0.3,
						 0, 0, 1, 0.2,
						 0, 0, 0, 1;
	crclCntr_dsrdTraj_dq = DQoperations::htm2DQ(crclCntr_dsrdTraj);
	startPose_dsrdTraj << -1, 0, 0, 0,
						   0, 1, 0, -0.15,
						   0, 0, -1, 0,
						   0, 0, 0, 1;
	startPose_dsrdTraj = crclCntr_dsrdTraj*startPose_dsrdTraj;
	startPose_dsrdTraj_dq = DQoperations::htm2DQ(startPose_dsrdTraj);
}

void resetRobot()
{
	cmd.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
	for(unsigned int i=0;i < joint_size;i++)
	{
		cmd.command[i] = 0;
	}		
	if (right_cmd_pub.getNumSubscribers())	
	{
		right_cmd_pub.publish(cmd);    
	}
    ros::Duration(2.0).sleep();
    cmd.mode = baxter_core_msgs::JointCommand::RAW_POSITION_MODE;
    cmd.command[0] = 1.035053536625683;
    cmd.command[1] = -0.9276748814737039;
    cmd.command[2] = -0.33824276372873374;
    cmd.command[3] = 1.5082866096883332;
    cmd.command[4] = 0.25080585881926515;
    cmd.command[5] = 1.0895098545956152;
    cmd.command[6] = -0.07209709703061444;
    int count_temp =0; 
    while(reset_robot)	
	{
		if (right_cmd_pub.getNumSubscribers())	
		{
			right_cmd_pub.publish(cmd);    
		}
		ros::spinOnce();
		// count_temp = count_temp+1;
	}
	ros::Duration(2.0).sleep();
	cmd.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
	// reset_robot =false;
	// time_now = 0;
	// start_ = ros::WallTime::now();
	// end_ = ros::WallTime::now();
    // cmd.names.push_back("right_s0");    -0.13185736132604742
    // cmd.names.push_back("right_s1");    -0.4816721820621366
    // cmd.names.push_back("right_e0");    0.6068619915741271
    // cmd.names.push_back("right_e1");    1.911764079997937
    // cmd.names.push_back("right_w0");    -0.4816721820621366
    // cmd.names.push_back("right_w1");    1.304727973182544
    // cmd.names.push_back("right_w2");    -1.0247054998116605
}

bool initializeAccController(ros::NodeHandle &node_)
{
	reset_robot = false;
	reset_controller = true;
	start_controller = false;	
	time_dq_jacDot =0;
	time_kdl_jacDot =0;
	controller_type = 0;
	rot_correction = KDL::Rotation(0.0000000,  0.0000000, -1.0000000, 0.7080644,  0.7061478,  0.0000000, 0.7061478, -0.7080644,  0.0000000).Inverse() ;
	// rot_correction = rot_correction.SetInverse();
	start_ = ros::WallTime::now();
	start_kdl = ros::WallTime::now();
	start_dq = ros::WallTime::now();
	ros::Duration(1.0).sleep();
	pose_publisher = node_.advertise<geometry_msgs::PoseStamped>("dq_robotics/desired_pose", 1000);
	desired_pose.header.frame_id = "base";
	desired_pose.header.stamp = ros::Time::now();
	right_cmd_pub = node_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);
	cmd.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
	cmd.names.push_back("right_s0");
	cmd.names.push_back("right_s1");
	cmd.names.push_back("right_e0");
	cmd.names.push_back("right_e1");
	cmd.names.push_back("right_w0");
	cmd.names.push_back("right_w1");
	cmd.names.push_back("right_w2");
	cmd.command.resize(cmd.names.size());
	time_now = 0;


	if(!node_.getParam("/robot_description",robot_desc_string))
	            {
			ROS_ERROR("Could not find '/robot_description'.");
			return false;
	}
	if (!kdl_parser::treeFromString(robot_desc_string,tree))
	{
		ROS_ERROR("Failed to construct KDL tree.");
		return false;
	}


	if (!tree.getChain("base","right_hand",chain)) 
	{
		ROS_ERROR("Failed to get chain from KDL tree.");
		return false;
	}
	node_.param("/gazebo/gravity_x",g[0],0.0);
	node_.param("/gazebo/gravity_y",g[1],0.0);
	// node_.param("/gazebo/gravity_z",g[2],-9.8); // put it to zero to use baxter gravity comps. Try both
	node_.param("/gazebo/gravity_z",g[2], 0.0); // put it to zero to use baxter gravity comps. Try both
	start_controller =false;			
	std::cout << "chain.getNrOfJoints(): " << chain.getNrOfJoints() << std::endl; 
	std::cout << "chain.getNrOfSegments(): " << chain.getNrOfSegments() << std::endl; 
	fext.resize(chain.getNrOfSegments());
	for(unsigned int i=0;i < fext.size();i++) fext[i].Zero();
	joint_size = chain.getNrOfJoints(); 
	q_kdl.resize(joint_size);
	dq_kdl.resize(joint_size);
	v_kdl.resize(joint_size);	
	jac.resize(joint_size);
	jdot.resize(joint_size);
	torque_kdl.resize(joint_size);
	// jacKDL=new KDL::ChainJntToJacSolver (chain);	
	if((jacKDL=new KDL::ChainJntToJacSolver (chain)) == NULL)
	{
		ROS_ERROR("Failed to create ChainJntToJacSolver.");
		return false;
	}
	if((jacDotKDL=new KDL::ChainJntToJacDotSolver (chain)) == NULL)
	{
		ROS_ERROR("Failed to create ChainJntToJacDotSolver.");
		return false;
	}	
	jacDotKDL->setRepresentation(0);
	if (!baxter_controller->BaxterPoseControlServer::initializeController())
	{
		ROS_ERROR("The robot can not be initialized.");
		return 0;
	}	
	if((idsolver=new KDL::ChainIdSolver_RNE(chain,g)) == NULL)
	{
		ROS_ERROR("Failed to create ChainIDSolver_RNE.");
		return false;
	}	
	initializeTraj();
	return 1;
}

void dynReconfgr_callback(dq_robotics::kdl_controllerConfig &config, uint32_t level) 
{
	if (config.start_controller)
		start_controller = true;
	else 
		start_controller = false;
	if (config.reset_robot)
		reset_robot = true;	
	else
		reset_robot = false;
	if (config.reset_controller)
		reset_controller = true;
	else	
		reset_controller = false;
	K_p = config.accCntrl_K_p;
	K_d = config.accCntrl_K_d;
	K_i = config.accCntrl_K_i;
	traj_pitch = config.accCntrl_traj_pitch;
	total_time = config.accCntrl_total_time;
	theta_init = config.accCntrl_theta_init;
	theta_final = config.accCntrl_theta_final;
	controller_type = config.controller_type;
  	ROS_INFO("Reconfigure Request: K_p: %f, K_d: %f, K_i: %f, total_time: %f", 
            K_p, 
            K_d, 
            K_i, 
            total_time);
}

void getCubicTheta()
{
	theta = (theta_init - 3.0 * (time_now * time_now) * (theta_init - theta_final)
	        / (total_time * total_time)) + 2.0 * pow(time_now, 3.0) *
	(theta_init - theta_final) / pow(total_time, 3.0);
	std::cout << "time_now: " << time_now << "theta: " << theta << std::endl;
	/*  theta_d = -(6*time_now*(theta_init - theta_final))/total_time^2 - (6*time_now^2*(theta_init - theta_final))/total_time^3; */
	theta_d = 6.0 * (time_now * time_now) * (theta_init - theta_final) /
	pow(total_time, 3.0) - 6.0 * time_now * (theta_init - theta_final) /
	(total_time * total_time);

	/*  theta_dd = -(6*(theta_init - theta_final))/total_time^2 - (12*time_now*(theta_init - theta_final))/total_time^3; */
	theta_dd = 12.0 * time_now * (theta_init - theta_final) / pow(total_time, 3.0) - 6.0 * (theta_init - theta_final) / (total_time*total_time);
}

void updateManipulatorState()
{
	// ROS_INFO("updateManipulatorState");
	std::string arm ="right";
	baxter_controller->BaxterPoseControlServer::update_manipulator();
	start_dq = ros::WallTime::now();
	if(!baxter_controller->BaxterPoseControlServer::importManipulatorState_accControl(arm, pe_init_right, pe_now_right, q_right, q_vel_right, jacobian_6d_right, jacobian_6d_dot_right))
		std::cout << arm << " arm can not be updated" << std::endl;
	end_dq = ros::WallTime::now();
	time_dq_jacDot = (end_dq - start_dq).toSec();
}

void getDesiredTraj()
{
	// ROS_INFO("getDesiredTraj");
	getCubicTheta();
	Matrix<double,8,1> cc_startinPose_dq, l_startingPose, l_baseFrame; 	
	cc_startinPose_dq = DQoperations::classicConjDQ( DQoperations::mulDQ(DQoperations::classicConjDQ(crclCntr_dsrdTraj_dq), startPose_dsrdTraj_dq));
	l_startingPose = DQoperations::mulDQ(cc_startinPose_dq , DQoperations::mulDQ(lineTraj_dq_cc, DQoperations::classicConjDQ(cc_startinPose_dq )));
	l_baseFrame = DQoperations::mulDQ(startPose_dsrdTraj_dq, DQoperations::mulDQ(l_startingPose, DQoperations::classicConjDQ(startPose_dsrdTraj_dq)));
 	RowVector3d axis, moment;
 	axis << l_startingPose(1), l_startingPose(2), l_startingPose(3);
 	moment << l_startingPose(5), l_startingPose(6), l_startingPose(7);
 	double d_traj = traj_pitch*theta;
 	double d_traj = 0;
	pose_desired_right = DQoperations::screw2DQ(theta, axis, d_traj, moment);
	pose_desired_right = DQoperations::mulDQ(startPose_dsrdTraj_dq, pose_desired_right);
	vel_desired_right = l_baseFrame*theta_d;
	acc_desired_right = l_baseFrame*theta_dd;
	desired_pose.pose = DQoperations::DQ2geometry_msgsPose(pose_desired_right);
	pose_publisher.publish(desired_pose);
}
Matrix<double,8,1> getAd_e_star(Matrix<double,8,1>  pose_error, Matrix<double,8,1> dq)
{	
	pose_error = DQoperations::mulDQ(DQoperations::classicConjDQ(pose_error), DQoperations::mulDQ(dq, pose_error));
	return pose_error;
}

Matrix<double,8,1> getCross_wc_adEVelDesired(RowVectorXd vel , Matrix<double,8,1> adEStarVelDesired)
{
	VectorXd adEStarVelDesired_6Vec;
	RowVectorXd adEStarVelDesired_6d = DQoperations::Matrix8d2RowVector6d(adEStarVelDesired);
	adEStarVelDesired_6Vec = DQoperations::crossProductOp_6d(vel)*adEStarVelDesired_6d.transpose();
	adEStarVelDesired = DQoperations::RowVector6d2Matrix8d(adEStarVelDesired_6Vec);
	return adEStarVelDesired;
}

void getNewControlLaw()
{
	RowVector3d ee_position_now, ee_position_desired;
	Matrix4d htm_desired, htm_current;
	ROS_INFO("NEW CONTROL LAW");
	// ROS_INFO("NEW CONTROL LAW");
	// std::cout << "jacobian_6d_dot_right" << std::endl;
	// std::cout << jacobian_6d_dot_right << std::endl;
	htm_current = DQoperations::dq2HTM(pe_now_right);
	htm_desired = DQoperations::dq2HTM(pose_desired_right);
	// ROS_INFO("1");
	ee_position_now << htm_current(0,3), htm_current(1,3), htm_current(2,3);

	ee_position_desired << htm_desired(0,3), htm_desired(1,3), htm_desired(2,3);
	Matrix<double,8,1> pose_error, ad_e_star_acc_desired, ad_e_star_vel_desired, cross_wc_adEVelDesired;
	pose_error=DQoperations::mulDQ(pose_desired_right, DQoperations::classicConjDQ(pe_now_right));	
	// ROS_INFO("getControlLaw");
	// get cmd acc for kdl
	Matrix<double,8,1> error_dq, vel_now_8d;
	norm_error = DQoperations::get_error_screw(pe_now_right, pose_desired_right,  v_e, w_e);
	error_dq << 0, w_e(0), w_e(1), w_e(2), 0, v_e(0), v_e(1), v_e(2);
	// ROS_INFO("getControlLaw_1");
	// Notice negative sign in K_p term, because the twist error is from curr to des in base frame
	// std::cout << "vel_now: " << jacobian_6d_right*q_vel_right.transpose() << std::endl;
		// ROS_INFO("2");
	vel_now  = jacobian_6d_right*q_vel_right.transpose();
		// ROS_INFO("3");
	vel_cart_now  = vel_now;
	vel_cart_now  = DQoperations::spatial2CartVel(vel_cart_now, ee_position_now);
			// ROS_INFO("4");
	vel_cart_desired = DQoperations::Matrix8d2RowVector6d(vel_desired_right);
			// ROS_INFO("5");
	vel_cart_desired  = DQoperations::spatial2CartVel(vel_cart_desired, ee_position_desired);
	vel_now_8d << 0, vel_now(0), vel_now(1), vel_now(2), 0, vel_now(3), vel_now(4), vel_now(5); 
	std::cout << "vel_now_8d: " << vel_now_8d.transpose() << std::endl;
	ad_e_star_acc_desired = getAd_e_star(pose_error, acc_desired_right);
	ad_e_star_vel_desired = getAd_e_star(pose_error, vel_desired_right);
	cross_wc_adEVelDesired = getCross_wc_adEVelDesired(vel_now ,ad_e_star_vel_desired);
	acc_cmd_right = acc_desired_right + K_d*(ad_e_star_vel_desired - vel_now_8d) + K_p*(error_dq) + cross_wc_adEVelDesired + ad_e_star_acc_desired;
	// acc_cmd_right = K_d*(vel_desired_right - vel_now_8d) - K_p*(error_dq);
	std::cout << "acc_cmd_right: " << acc_cmd_right.transpose() << std::endl;
	// ROS_INFO("getControlLaw_2, acc_cmd_right.rows(): %d, acc_cmd_right.cols(): %d ", acc_cmd_right.rows(), acc_cmd_right.cols());
	RowVectorXd acc_cmd_6d = RowVectorXd::Zero(6);
	
	acc_cmd_6d << acc_cmd_right(1,0), acc_cmd_right(2,0), acc_cmd_right(3,0), acc_cmd_right(5,0), acc_cmd_right(6,0), acc_cmd_right(7,0);
	std::cout << "acc_cmd_6d : " << acc_cmd_6d << std::endl;
	// ROS_INFO("getControlLaw_3");
	MatrixXd jacobian_8d = MatrixXd::Zero(8, jacobian_6d_right.cols());
	// ROS_INFO("getControlLaw_4");
	jacobian_8d.block(1, 0, 3, jacobian_8d.cols()) = jacobian_6d_right.block(0, 0,  3, jacobian_8d.cols());
	jacobian_8d.block(5, 0, 3, jacobian_8d.cols()) = jacobian_6d_right.block(3, 0,  3, jacobian_8d.cols());	
	// ROS_INFO("getControlLaw_5");
	double mu = 0.0001;
	MatrixXd jac_inv_damped =  DQoperations::invDamped_8d(jacobian_8d, mu);
	// std::cout << "acc_cart1: " << std::endl;
	// std::cout << acc_cmd_6d.transpose() << std::endl;
	// std::cout << "acc_cart2: " << std::endl;
	// std::cout << - jacobian_6d_dot_right*q_vel_right.transpose() << std::endl;	
	// ROS_INFO("getControlLaw_7");
	RowVectorXd acc_term_8d= RowVectorXd::Zero(8);
	acc_term_8d = DQoperations::twistEigen2DQEigen(acc_cmd_6d.transpose() - jacobian_6d_dot_right*q_vel_right.transpose()); 
	std::cout << "jdotQdot: " << std::endl;
	std::cout << (jacobian_6d_dot_right*q_vel_right.transpose()).transpose() << std::endl;
	qdd_cmd = jac_inv_damped*acc_term_8d.transpose();
	std::cout << "qdd_cmd: " << qdd_cmd << std::endl;
	for(unsigned int i=0;i < joint_size;i++)
	{
		q_kdl(i) = q_right(i);
		dq_kdl(i) = q_vel_right(i);
		v_kdl(i) = qdd_cmd(i);
	}
}



void getControlLaw()
{
	// ROS_INFO("getControlLaw");
	// get cmd acc for kdl
	Matrix<double,8,1> error_dq, vel_now_8d;
	norm_error = DQoperations::get_error_screw(pe_now_right, pose_desired_right,  v_e, w_e);
	error_dq << 0, w_e(0), w_e(1), w_e(2), 0, v_e(0), v_e(1), v_e(2);
	// ROS_INFO("getControlLaw_1");
	// Notice negative sign in K_p term, because the twist error is from curr to des in base frame
	// std::cout << "vel_now: " << jacobian_6d_right*q_vel_right.transpose() << std::endl;
	vel_now  = jacobian_6d_right*q_vel_right.transpose();
	vel_now_8d << 0, vel_now(0), vel_now(1), vel_now(2), 0, vel_now(3), vel_now(4), vel_now(5); 
	std::cout << "vel_now_8d: " << vel_now_8d.transpose() << std::endl;
	acc_cmd_right = acc_desired_right + K_d*(vel_desired_right - vel_now_8d) + K_p*(error_dq);
	// acc_cmd_right = K_d*(vel_desired_right - vel_now_8d) - K_p*(error_dq);
	std::cout << "acc_cmd_right: " << acc_cmd_right.transpose() << std::endl;
	// ROS_INFO("getControlLaw_2, acc_cmd_right.rows(): %d, acc_cmd_right.cols(): %d ", acc_cmd_right.rows(), acc_cmd_right.cols());
	RowVectorXd acc_cmd_6d = RowVectorXd::Zero(6);
	
	acc_cmd_6d << acc_cmd_right(1,0), acc_cmd_right(2,0), acc_cmd_right(3,0), acc_cmd_right(5,0), acc_cmd_right(6,0), acc_cmd_right(7,0);
	std::cout << "acc_cmd_6d : " << acc_cmd_6d << std::endl;
	// ROS_INFO("getControlLaw_3");
	MatrixXd jacobian_8d = MatrixXd::Zero(8, jacobian_6d_right.cols());
	// ROS_INFO("getControlLaw_4");
	jacobian_8d.block(1, 0, 3, jacobian_8d.cols()) = jacobian_6d_right.block(0, 0,  3, jacobian_8d.cols());
	jacobian_8d.block(5, 0, 3, jacobian_8d.cols()) = jacobian_6d_right.block(3, 0,  3, jacobian_8d.cols());	
	// ROS_INFO("getControlLaw_5");
	double mu = 0.0001;
	MatrixXd jac_inv_damped =  DQoperations::invDamped_8d(jacobian_8d, mu);
	// std::cout << "acc_cart1: " << std::endl;
	// std::cout << acc_cmd_6d.transpose() << std::endl;
	// std::cout << "acc_cart2: " << std::endl;
	// std::cout << - jacobian_6d_dot_right*q_vel_right.transpose() << std::endl;	
	// ROS_INFO("getControlLaw_7");
	RowVectorXd acc_term_8d= RowVectorXd::Zero(8);
	acc_term_8d = DQoperations::twistEigen2DQEigen(acc_cmd_6d.transpose() - jacobian_6d_dot_right*q_vel_right.transpose()); 
	std::cout << "jdotQdot: " << std::endl;
	std::cout << (jacobian_6d_dot_right*q_vel_right.transpose()).transpose() << std::endl;
	qdd_cmd = jac_inv_damped*acc_term_8d.transpose();
	std::cout << "qdd_cmd: " << qdd_cmd << std::endl;
	for(unsigned int i=0;i < joint_size;i++)
	{
		q_kdl(i) = q_right(i);
		dq_kdl(i) = q_vel_right(i);
		v_kdl(i) = qdd_cmd(i);
	}
}

void getKDLjacDot()
{
	start_kdl = ros::WallTime::now();
	for(unsigned int i=0;i < joint_size;i++)
	{
		q_kdl(i) = q_right(i);
		dq_kdl(i) = q_vel_right(i);
	}		
	if(jacDotKDL->JntToJacDot (KDL::JntArrayVel(q_kdl, dq_kdl), jdot) < 0)				
					ROS_ERROR("KDL jacobian dot solver failed.");			
	jacDotKDL_eigen = jdot.data;
	std::cout << "jacDot" << std::endl;
	std::cout << jacDotKDL_eigen << std::endl;
	end_kdl = ros::WallTime::now();
	time_kdl_jacDot = (end_kdl - start_kdl).toSec();	
}

void getCartControlLaw()
{
	ROS_INFO("1");
	RowVector3d ee_position_now, ee_position_desired;
	Matrix4d htm_desired, htm_current;
	htm_current = DQoperations::dq2HTM(pe_now_right);
	htm_desired = DQoperations::dq2HTM(pose_desired_right);
	ee_position_now << htm_current(0,3), htm_current(1,3), htm_current(2,3);
	ee_position_desired << htm_desired(0,3), htm_desired(1,3), htm_desired(2,3);
	norm_error = DQoperations::get_error_screw(pe_now_right, pose_desired_right,  v_e, w_e);
	// Matrix<double,8,1> error_dq; 
	// error_dq << 0, w_e(0), w_e(1), w_e(2), 0, v_e(0), v_e(1), v_e(2);
	// std::cout << "error_dq: " << error_dq << std::endl;
	RowVectorXd kdl_error_pose;
	if (controller_type==1)
		kdl_error_pose = DQoperations::spatial2CartPoseError(pose_desired_right, pe_now_right);
	else if (controller_type==2)
		kdl_error_pose = DQoperations::spatial2CartPoseError_quatVec(pose_desired_right, pe_now_right);
	std::cout << "kdl_error_pose: " << kdl_error_pose << std::endl;
	vel_now  = jacobian_6d_right*q_vel_right.transpose();
	vel_cart_now  = vel_now;
	
	// ROS_INFO("2");
	// vel_cart_now  = DQoperations::Matrix8d2RowVector6d(vel_cart_now);
	// ROS_INFO("3");	
	vel_cart_now  = DQoperations::spatial2CartVel(vel_cart_now, ee_position_now);		
	std::cout << "vel_cart_now: " << vel_cart_now << std::endl;
	vel_cart_desired = DQoperations::Matrix8d2RowVector6d(vel_desired_right);
	vel_cart_desired = DQoperations::spatial2CartVel(vel_cart_desired, ee_position_desired);
	std::cout << "vel_cart_desired: " << vel_cart_desired << std::endl;		
	RowVectorXd kdl_error_vel = vel_cart_desired - vel_cart_now;
	std::cout << "acc_desired_right: " << acc_desired_right.transpose() << std::endl;	
	RowVectorXd kdl_acc_desired = DQoperations::Matrix8d2RowVector6d(acc_desired_right);
	std::cout << "kdl_acc_desired1: " << kdl_acc_desired << std::endl;	
	kdl_acc_desired = DQoperations::spatial2CartAcc(acc_desired_right, DQoperations::Matrix8d2RowVector6d(vel_desired_right), ee_position_desired);
	std::cout << "kdl_acc_desired: " << kdl_acc_desired.transpose() << std::endl;			
	RowVectorXd kdl_acc_cmd = kdl_acc_desired + K_d*(vel_cart_desired - vel_cart_now) + K_p*(kdl_error_pose);
	// RowVectorXd kdl_acc_cmd = K_d*(vel_cart_desired - vel_cart_now) + K_p*(kdl_error_pose);
	// RowVectorXd kdl_acc_cmd =  K_p*(kdl_error_pose);
	// kdl_acc_cmd << kdl_acc_cmd(3), kdl_acc_cmd(4), kdl_acc_cmd(5), kdl_acc_cmd(0), kdl_acc_cmd(1), kdl_acc_cmd(2);
	std::cout << "kdl_acc_cmd: " << std::endl;		
	std::cout << kdl_acc_cmd << std::endl;
	for(unsigned int i=0;i < joint_size;i++)
	{
		q_kdl(i) = q_right(i);
		dq_kdl(i) = q_vel_right(i);
	}		
	if(jacKDL->JntToJac (q_kdl, jac) < 0)
			ROS_ERROR("KDL jacobian solver failed.");
	// jac.changeBase (rot_correction);
	// std::cout << "jacobianKDL: " << std::endl;	
	// std::cout << jac.data << std::endl;
	start_kdl = ros::WallTime::now();
	if(jacDotKDL->JntToJacDot (KDL::JntArrayVel(q_kdl, dq_kdl), jdot) < 0)				
					ROS_ERROR("KDL jacobian dot solver failed.");			
	jacKDL_eigen = jac.data;
	end_kdl = ros::WallTime::now();
	time_kdl_jacDot = (end_kdl - start_kdl).toSec();		

	MatrixXd jac_temp = jacKDL_eigen;
	jacKDL_eigen.block(3, 0, 3, joint_size) = jac_temp.block(0, 0, 3, joint_size); 
	jacKDL_eigen.block(0, 0, 3, joint_size) = jac_temp.block(3, 0, 3, joint_size); 
	
	jacDotKDL_eigen = jdot.data;
	jac_temp = jacDotKDL_eigen;
	jacDotKDL_eigen.block(3, 0, 3, joint_size) = jac_temp.block(0, 0, 3, joint_size); 
	jacDotKDL_eigen.block(0, 0, 3, joint_size) = jac_temp.block(3, 0, 3, joint_size); 	
	// std::cout << "jacDotKDL_eigen: " << std::endl;	
	// std::cout << jacDotKDL_eigen << std::endl;		
	double mu = 0.0001;
	// ROS_INFO("8");	
	// MatrixXd jacobian_8d = MatrixXd::Zero(8, jacobian_6d_right.cols());
	// // ROS_INFO("getControlLaw_4");
	// jacobian_8d.block(1, 0, 3, jacobian_8d.cols()) = jacKDL_eigen.block(0, 0,  3, jacobian_8d.cols());
	// jacobian_8d.block(5, 0, 3, jacobian_8d.cols()) = jacKDL_eigen.block(3, 0,  3, jacobian_8d.cols());		
	// jacobian_8d =  DQoperations::invDamped_8d(jacobian_8d, mu);
	// std::cout << "jacobian_8d_damped: " << std::endl;	
	// std::cout << jacobian_8d << std::endl;

	MatrixXd jac_inv_damped =  DQoperations::invDamped_8d(jacKDL_eigen, mu);
	// std::cout << "jacobian_6d_damped: " << std::endl;	
	// std::cout << jac_inv_damped << std::endl;

	VectorXd acc_term= RowVectorXd::Zero(6);
	acc_term = (kdl_acc_cmd.transpose() - jacDotKDL_eigen*q_vel_right.transpose()); 
	// acc_term = (kdl_acc_cmd.transpose() ); 
	std::cout << "acc_term: " << acc_term.transpose() << std::endl;
	// std::cout << "jac_inv_damped: " << jac_inv_damped << std::endl;
	acc_term = jac_inv_damped*acc_term;
	ROS_INFO("10");
	qdd_cmd = acc_term.transpose();
	std::cout << "qdd_cmd: " << qdd_cmd << std::endl;	
	for(unsigned int i=0;i < joint_size;i++)
	{
		v_kdl(i) = qdd_cmd(i);
	}
}

void compileResults()
{
	resultMsg.desiredPose.header.stamp = ros::Time::now();	
	resultMsg.desiredPose.pose = DQoperations::DQ2geometry_msgsPose(pose_desired_right);
	resultMsg.currentPose.header.stamp = ros::Time::now();	
	resultMsg.currentPose.pose = DQoperations::DQ2geometry_msgsPose(pe_now_right);	
	resultMsg.desiredVel.header.stamp = ros::Time::now();
	resultMsg.currentVel.header.stamp = ros::Time::now();
	// if(controller_type != 0)
	// {	
		resultMsg.currentVel.twist =  DQoperations::Rowvector2geometry_msgsTwist(vel_cart_now);
		resultMsg.desiredVel.twist =  DQoperations::Rowvector2geometry_msgsTwist(vel_cart_desired);
	// }
	// else
	// {
	// 	resultMsg.currentVel.twist =  DQoperations::Rowvector2geometry_msgsTwist(vel_now);
	// 	resultMsg.desiredVel.twist =  DQoperations::Rowvector2geometry_msgsTwist(vel_desired_right);
	// }	
	// resultMsg.desiredPose = DQoperations::DQToDouble(pose_desired_right);
	// resultMsg.currentPose = DQoperations::DQToDouble(pe_now_right);
	// resultMsg.desiredVel = DQoperations::RowVectorToDouble(vel_desired_right);
	// resultMsg.currentVel = DQoperations::RowVectorToDouble(vel_now);

	// resultMsg.timeStamp = time_now_total;
	resultMsg.timeStamp = time_now;
	resultMsg.timeKDLJacDot = time_kdl_jacDot;
	resultMsg.timeDQJacDot = time_dq_jacDot;
	resultMsg.norm_error = norm_error;
	resultMsg.controlMode = controller_type;
// 	float64 timeKDLJacDot
// float64 timeDQJacDot
	// std::vector<double>  DQoperations::DQToDouble(Matrix<double,8,1> dq_eigen)
	// Matrix<double,8,1> DQoperations::returnDoubleToDQ(std::vector<double> dq_double)
	// RowVectorXd DQoperations::doubleVector2Rowvector(std::vector<double> doubleVector)
}

void updateTime()
{
	time_now = 0;
	start_ = ros::WallTime::now();
	end_ = ros::WallTime::now();
	// start_controller =false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "resolveAccControl");
	ros::NodeHandle node_;
	result_topic = node_.advertise<dq_robotics::ResolveAccControlPrfmnc>("/dq_robotics/trajCntrlResults", 1000);
	dynamic_reconfigure::Server<dq_robotics::kdl_controllerConfig> server;
	dynamic_reconfigure::Server<dq_robotics::kdl_controllerConfig>::CallbackType f;
	f = boost::bind(&dynReconfgr_callback, _1, _2);
	server.setCallback(f);
	start_controller =false;
	baxter_controller = new BaxterPoseControlServer();
	if (!initializeAccController(node_))
	{
		ROS_ERROR("The acc controller can not be initialized.");
		return 0;
	}
	while (!start_controller)
	{
		ros::Duration(1.0).sleep();
		ROS_INFO("waiting for controller to start...");
		ros::spinOnce();
	}
	// ros::spinOnce();	
	start_ = ros::WallTime::now();
	start_total = ros::WallTime::now();
	start_controller =false;			
	// updating manipulator

	int count = 0;
	ROS_INFO("starting acceleration control");
	// getDesiredTraj();
	if(controller_type ==0 )
		std::cout << "DQ based controller selected" << std::endl;
	else if(controller_type ==1)
		std::cout << "KDL quaternion error based controller selected" << std::endl;
	else if (controller_type ==2)	
		std::cout << "KDL quaternion VECTOR error based controller selected" << std::endl;
	time_now = 0;
	time_now_total = 0;
	start_ = ros::WallTime::now();
	start_total = ros::WallTime::now();
	end_ = ros::WallTime::now();
	end_total = ros::WallTime::now();
	getDesiredTraj();	
	while (ros::ok())
	{		
		if(reset_robot)
		{
			resetRobot();
			updateTime();
			// start_controller =false;
		}

		else if(!reset_robot && !reset_controller && (time_now < total_time))
		{
			updateManipulatorState();
			// getKDLjacDot();
			if (time_now < total_time)
			{
				getDesiredTraj();
			
			}	// getCartControlLaw();
			if(controller_type ==0 )
				getNewControlLaw();
			else if(controller_type ==1 || controller_type ==2)
				getCartControlLaw();
			if(idsolver->CartToJnt(q_kdl,dq_kdl,v_kdl,fext,torque_kdl) < 0)
				ROS_ERROR("KDL inverse dynamics solver failed.");		
			std::cout << "computed_torque: " ; 
			for(unsigned int i=0;i < joint_size;i++)
			{
				std::cout << torque_kdl(i) << ", ";
				cmd.command[i] = torque_kdl(i);
			}		
			std::cout << std::endl;
		    right_cmd_pub.publish(cmd);
	
		    // loop_rate.sleep();		
			end_ = ros::WallTime::now();
			double time_last= time_now;
			time_now = (end_ - start_).toSec();
			count = count +1;
			ROS_INFO("count: %d", count);
			ROS_INFO("time_now: %f", time_now);
			ROS_INFO("time_iteration: %f", (time_now - time_last));
			compileResults();
			result_topic.publish(resultMsg);
			end_total = ros::WallTime::now();
			time_now_total = (end_total - start_total).toSec();
			// reset_robot =false;		
			// reset_controller =false;	
			// ros::spinOnce();
		}
		else if (!reset_robot && reset_controller)
		{
			updateTime(); 
			// ros::Duration(1.0).sleep();
		}
		std::cout << "reset_robot: " << reset_robot << "reset_controller: " << reset_controller << std::endl;	
		// reset_controller =false;	
		ros::spinOnce();
	}


	// ROS_INFO("starting regulation now.");
	// start_ = ros::WallTime::now();
	// end_ = ros::WallTime::now();
	// time_now = 0;
	// while (ros::ok())
	// {
	// 	updateManipulatorState();
	// 	// getControlLaw_regulation();
	// 	getControlLaw();
	// 	for(unsigned int i=0;i < joint_size;i++)
	// 	{
	// 		q_kdl(i) = q_right(i);
	// 		dq_kdl(i) = q_vel_right(i);
	// 		v_kdl(i) = qdd_cmd(i);
	// 	}
	// 	// std::cout << "q_kdl: " < q_kdl.data << std::endl;
	// 	if(idsolver->CartToJnt(q_kdl,dq_kdl,v_kdl,fext,torque_kdl) < 0)
	// 		ROS_ERROR("KDL inverse dynamics solver failed.");		
	// 	std::cout << "computed_torque: " ; 
	// 	for(unsigned int i=0;i < joint_size;i++)
	// 	{
	// 		std::cout << torque_kdl(i) << ", ";
	// 		cmd.command[i] = torque_kdl(i);
	// 	}		
	// 	std::cout << std::endl;
	//     right_cmd_pub.publish(cmd);
	//     ros::spinOnce();
	//     // loop_rate.sleep();		
	// 	end_ = ros::WallTime::now();
	// 	time_now = (end_ - start_).toSec();
	// 	ROS_INFO("time_now: %f", time_now);		
	// }
	// 	ros::Duration(1.0).sleep();
	// 	ros::Duration(1.0).sleep();
	// 	end_ = ros::WallTime::now();

	// // print results
	// 	double execution_time = (end_ - start_).toNSec() * 1e-6;
	// 	ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
	return 0;
}

	// get current joint position and velocity
	// make your control law
	// 	get kp, kv and kd from dynamic reconfigure: done
	// 	get desired acc, vel and position..generate circular trajectory
	// 	get current position and velocity from the update_accControl thing you made
	// 	compute control law like this
	// 		v.data=ddqr.data+Kp*(qr.data-q.data)+Kd*(dqr.data-dq.data);
	// 	get torque from kdl like this
	// 		if(idsolver->CartToJnt(q,dq,v,fext,torque) < 0)
	// 	        ROS_ERROR("KDL inverse dynamics solver failed.");
	// 	send torque cmd. 
	//  repeat