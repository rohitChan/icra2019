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
#include "baxter_core_msgs/JointCommand.h"
// ros::NodeHandle node_;
bool dynRcnfgr_toggle;
double K_p, K_d, K_i, total_time, theta_init, theta_final, norm_error;
ros::WallTime start_, end_;
std::string robot_desc_string;	
KDL::ChainIdSolver_RNE *idsolver;
KDL::Tree tree;
KDL::Chain chain;
KDL::Vector g;
Matrix4d crclCntr_dsrdTraj, startPose_dsrdTraj;
Matrix<double,8,1> pe_init_right, pe_now_right, crclCntr_dsrdTraj_dq, startPose_dsrdTraj_dq, lineTraj_dq_cc, pose_desired_right, vel_desired_right, acc_desired_right, acc_cmd_right;
RowVectorXd q_right, q_vel_right, qdd_cmd;
MatrixXd jacobian_6d_right, jacobian_6d_dot_right;
BaxterPoseControlServer* baxter_controller;
double time_now, theta, theta_d, theta_dd;
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


void initializeTraj()
{
	lineTraj_dq_cc << 0, 0, 0, 1, 0, 0, 0, 0;
	crclCntr_dsrdTraj << 1, 0, 0, 0.5,
						 0, 1, 0, 0,
						 0, 0, 1, 0.2,
						 0, 0, 0, 1;
	crclCntr_dsrdTraj_dq = DQoperations::htm2DQ(crclCntr_dsrdTraj);
	startPose_dsrdTraj << -1, 0, 0, 0,
						   0, 1, 0, -0.3,
						   0, 0, -1, 0,
						   0, 0, 0, 1;
	startPose_dsrdTraj = crclCntr_dsrdTraj*startPose_dsrdTraj;
	startPose_dsrdTraj_dq = DQoperations::htm2DQ(startPose_dsrdTraj);
}


bool initializeAccController(ros::NodeHandle &node_)
{
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
	dynRcnfgr_toggle = false;

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
	node_.param("/gazebo/gravity_z",g[2],0.0); // put it to zero to use baxter gravity comps. Try both
	
	if (!baxter_controller->BaxterPoseControlServer::initializeController())
	{
		ROS_ERROR("The robot can not be initialized.");
		return 0;
	}	
	return 1;
}

void dynReconfgr_callback(dq_robotics::kdl_controllerConfig &config, uint32_t level) 
{
	if (config.start_controller)
		dynRcnfgr_toggle = true;
	K_p = config.accCntrl_K_p;
	K_d = config.accCntrl_K_d;
	K_i = config.accCntrl_K_i;
	total_time = config.accCntrl_total_time;
	theta_init = config.accCntrl_theta_init;
	theta_final = config.accCntrl_theta_final;
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
	if(!baxter_controller->BaxterPoseControlServer::importManipulatorState_accControl(arm, pe_init_right, pe_now_right, q_right, q_vel_right, jacobian_6d_right, jacobian_6d_dot_right))
		std::cout << arm << " arm can not be updated" << std::endl;
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
 	double d_traj = 0.05*theta;
	pose_desired_right = DQoperations::screw2DQ(theta, axis, d_traj, moment);
	pose_desired_right = DQoperations::mulDQ(startPose_dsrdTraj_dq, pose_desired_right);
	vel_desired_right = l_baseFrame*theta_d;
	acc_desired_right = l_baseFrame*theta_dd;
	desired_pose.pose = DQoperations::DQ2geometry_msgsPose(pose_desired_right);
	pose_publisher.publish(desired_pose);
}

void getControlLaw_regulation()
{
	vel_desired_right = RowVectorXd::Zero(8);
	acc_desired_right = RowVectorXd::Zero(8);
	// ROS_INFO("getControlLaw");
	// get cmd acc for kdl
	Matrix<double,8,1> error_dq, vel_now_8d;
	norm_error = DQoperations::get_error_screw(pe_now_right, pose_desired_right,  v_e, w_e);
	error_dq << 0, w_e(0), w_e(1), w_e(2), 0, v_e(0), v_e(1), v_e(2);
	// ROS_INFO("getControlLaw_1");
	// Notice negative sign in K_p term, because the twist error is from curr to des in base frame
	// std::cout << "vel_now: " << jacobian_6d_right*q_vel_right.transpose() << std::endl;
	RowVectorXd vel_now  = jacobian_6d_right*q_vel_right.transpose();
	vel_now_8d << 0, vel_now(0), vel_now(1), vel_now(2), 0, vel_now(3), vel_now(4), vel_now(5); 
	std::cout << "vel_now_8d: " << vel_now_8d.transpose() << std::endl;
	acc_cmd_right = acc_desired_right + K_d*(vel_desired_right - vel_now_8d) + K_p*(error_dq);
	std::cout << "acc_cmd_right: " << acc_cmd_right.transpose() << std::endl;
	// ROS_INFO("getControlLaw_2, acc_cmd_right.rows(): %d, acc_cmd_right.cols(): %d ", acc_cmd_right.rows(), acc_cmd_right.cols());
	RowVectorXd acc_cmd_6d = RowVectorXd::Zero(6);
	
	acc_cmd_6d << acc_cmd_right(1,0), acc_cmd_right(2,0), acc_cmd_right(3,0), acc_cmd_right(5,0), acc_cmd_right(6,0), acc_cmd_right(7,0);
	std::cout << "acc_cmd_6d : " << acc_cmd_6d.transpose() << std::endl;
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
	qdd_cmd = jac_inv_damped*acc_term_8d.transpose();
	std::cout << "qdd_cmd: " << qdd_cmd << std::endl;
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
	RowVectorXd vel_now  = jacobian_6d_right*q_vel_right.transpose();
	vel_now_8d << 0, vel_now(0), vel_now(1), vel_now(2), 0, vel_now(3), vel_now(4), vel_now(5); 
	std::cout << "vel_now_8d: " << vel_now_8d.transpose() << std::endl;
	acc_cmd_right = acc_desired_right + K_d*(vel_desired_right - vel_now_8d) + K_p*(error_dq);
	// acc_cmd_right = K_d*(vel_desired_right - vel_now_8d) - K_p*(error_dq);
	std::cout << "acc_cmd_right: " << acc_cmd_right.transpose() << std::endl;
	// ROS_INFO("getControlLaw_2, acc_cmd_right.rows(): %d, acc_cmd_right.cols(): %d ", acc_cmd_right.rows(), acc_cmd_right.cols());
	RowVectorXd acc_cmd_6d = RowVectorXd::Zero(6);
	
	acc_cmd_6d << acc_cmd_right(1,0), acc_cmd_right(2,0), acc_cmd_right(3,0), acc_cmd_right(5,0), acc_cmd_right(6,0), acc_cmd_right(7,0);
	std::cout << "acc_cmd_6d : " << acc_cmd_6d.transpose() << std::endl;
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
	qdd_cmd = jac_inv_damped*acc_term_8d.transpose();
	std::cout << "qdd_cmd: " << qdd_cmd << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kdl_controller");
	ros::NodeHandle node_;

	dynamic_reconfigure::Server<dq_robotics::kdl_controllerConfig> server;
	dynamic_reconfigure::Server<dq_robotics::kdl_controllerConfig>::CallbackType f;
	f = boost::bind(&dynReconfgr_callback, _1, _2);
	server.setCallback(f);
	dynRcnfgr_toggle =false;
	baxter_controller = new BaxterPoseControlServer();
	if (!initializeAccController(node_))
	{
		ROS_ERROR("The acc controller can not be initialized.");
		return 0;
	}
	// updating manipulator
	initializeTraj();
	start_ = ros::WallTime::now();
	ros::Duration(1.0).sleep();

	start_ = ros::WallTime::now();
	if((idsolver=new KDL::ChainIdSolver_RNE(chain,g)) == NULL)
	{
		ROS_ERROR("Failed to create ChainIDSolver_RNE.");
		return false;
	}
	ROS_INFO("starting acceleration control");
	while (!dynRcnfgr_toggle)
	{
		ROS_INFO("dyn reconf waiting...");
		ros::spinOnce();
		// dynRcnfgr_toggle =true;
	}
	ros::spinOnce();	
	dynRcnfgr_toggle =false;			
	start_ = ros::WallTime::now();
	std::cout << "chain.getNrOfJoints(): " << chain.getNrOfJoints() << std::endl; 
	std::cout << "chain.getNrOfSegments(): " << chain.getNrOfSegments() << std::endl; 
	fext.resize(chain.getNrOfSegments());
	for(unsigned int i=0;i < fext.size();i++) fext[i].Zero();
	int joint_size = chain.getNrOfJoints(); 
	q_kdl.resize(joint_size);
	dq_kdl.resize(joint_size);
	v_kdl.resize(joint_size);	
	torque_kdl.resize(joint_size);	

	start_ = ros::WallTime::now();
	end_ = ros::WallTime::now();

	while (time_now < total_time)
	{

		// while (!dynRcnfgr_toggle)
		// {
		// 	ROS_INFO("dyn reconf waiting...2");
		// 	ros::spinOnce();
		// 	// dynRcnfgr_toggle =true;
		// }
		dynRcnfgr_toggle =false;
		updateManipulatorState();
		getDesiredTraj();
		getControlLaw();	
		for(unsigned int i=0;i < joint_size;i++)
		{
			q_kdl(i) = q_right(i);
			dq_kdl(i) = q_vel_right(i);
			v_kdl(i) = qdd_cmd(i);
		}
		// std::cout << "q_kdl: " < q_kdl.data << std::endl;
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
	    ros::spinOnce();
	    // loop_rate.sleep();		
		end_ = ros::WallTime::now();
		time_now = (end_ - start_).toSec();
		ROS_INFO("time_now: %f", time_now);
	}


	ROS_INFO("starting regulation now.");
	start_ = ros::WallTime::now();
	end_ = ros::WallTime::now();
	time_now = 0;
	while (ros::ok())
	{
		updateManipulatorState();
		getControlLaw_regulation();
		for(unsigned int i=0;i < joint_size;i++)
		{
			q_kdl(i) = q_right(i);
			dq_kdl(i) = q_vel_right(i);
			v_kdl(i) = qdd_cmd(i);
		}
		// std::cout << "q_kdl: " < q_kdl.data << std::endl;
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
	    ros::spinOnce();
	    // loop_rate.sleep();		
		end_ = ros::WallTime::now();
		time_now = (end_ - start_).toSec();
		ROS_INFO("time_now: %f", time_now);		
	}
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