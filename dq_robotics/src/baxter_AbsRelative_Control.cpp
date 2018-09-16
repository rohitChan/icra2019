#include <dq_robotics/baxter_poseControl_server.h>
#include <dq_robotics/ar10_kinematics.h>
#include <dq_robotics/Grasp.h>
#include <dq_robotics/BaxterControl.h>
#include <dq_robotics/ControllerPerformance.h>
// #include <baxter_trajectory_interface/baxter_poseControl_server.h>
#define PI 3.14159265

double mu, Kp, K_jl, Kp_position, K_jl_position, sleepTimeout, dt, timeStep, norm_error_const, time_start;
Matrix<double,8,1> pe_init_baxterRight, pe_now_baxterRight, pe_init_baxterLeft, pe_now_baxterLeft, pe_init_thumbRight, pe_init_indexRight,  pe_now_thumbRight, pe_now_indexRight, pe_desired_thumb, pe_desired_index, tf_rightHand2handBase, p_abs_2;
MatrixXd jacobian_baxterRight, jacobian_baxterLeft, jacobian_thumbRight, jacobian_indexRight;
RowVectorXd q_baxterRight, q_baxterLeft, q_thumbRight, q_indexRight, q_init_thumbRight, q_init_indexRight;
std::vector<double> joint_velocity_limit_baxterRight, joint_velocity_limit_baxterLeft, joint_velocity_limit_baxter;
BaxterPoseControlServer* baxter_controller;
AR10Kinematics*  ar10;
Matrix<double,8,1> tfRight_baxterBase2handBase;
Matrix<double,8,1> p_right_stick, p_left_stick, p_rel_stick;
ros::Publisher result_topics;
dq_robotics::ControllerPerformance resultMsg;
bool getControllerParams()
{
	p_abs_2 << 1, 0, 0, 0, 0, 0, 0, 0 ;
	std::string paramName="Kp";
	if(!ros::param::get(paramName, Kp))
	{
		std::cout << "controller param Kp not found" << std::endl;
		return 0;
	}

	paramName="Kp_position";
	if(!ros::param::get(paramName, Kp_position))
	{
		std::cout << "controller param Kp_position not found" << std::endl;
		return 0;
	}	

	paramName="timeStep";
	if(!ros::param::get(paramName, timeStep))
	{
		std::cout << "controller param timeStep not found" << std::endl;
		return 0;
	}

	paramName="K_jl";
	if(!ros::param::get(paramName, K_jl))
	{
		std::cout << "controller param K_jl not found" << std::endl;
		return 0;
	}

	paramName="K_jl_position";
	if(!ros::param::get(paramName, K_jl_position))
	{
		std::cout << "controller param K_jl_position not found" << std::endl;
		return 0;
	}	

	paramName="mu";
	if(!ros::param::get(paramName, mu))
	{
		std::cout << "controller param mu not found" << std::endl;
		return 0;
	}

	paramName="sleepTimeout";
	if(!ros::param::get(paramName, sleepTimeout))
	{
		std::cout << "controller param sleepTimeout not found" << std::endl;
		return 0;
	}

	paramName="norm_error_const";
	if(!ros::param::get(paramName, norm_error_const))
	{
		std::cout << "controller param norm_error_const not found" << std::endl;
		return 0;
	}

	ROS_INFO("Controller parameters loaded");
	return 1;
}


MatrixXd getJointLimitJacobian(RowVectorXd q, std::vector<double> upper_limit_ordered, std::vector<double>  low_limit_ordered, double fraction)
{
	int noOfJoints=q.size();
	MatrixXd joint_limit_jacobian=MatrixXd::Zero(noOfJoints, noOfJoints);
	// joint_limit_jacobian.resize(noOfJoints);
	double max_safe, min_safe;
	for (int i=0; i <noOfJoints; i++)
	{
		max_safe=upper_limit_ordered[i]- (upper_limit_ordered[i]- low_limit_ordered[i])*fraction;
		min_safe=low_limit_ordered[i] + (upper_limit_ordered[i]- low_limit_ordered[i])*fraction;
		if(q(i)>=max_safe)
			joint_limit_jacobian(i,i)=(q(i)-max_safe)*(2*upper_limit_ordered[i]-max_safe-q(i))/((upper_limit_ordered[i]-q(i))*(upper_limit_ordered[i]-q(i)));
		else if(q(i)<min_safe)
			joint_limit_jacobian(i,i)=(min_safe-q(i))*(2*low_limit_ordered[i]- q(i) -min_safe)/((q(i) - low_limit_ordered[i])*(q(i) - low_limit_ordered[i]));
		else
			joint_limit_jacobian(i,i)=0;		
	}
	return joint_limit_jacobian;

}


bool update_baxter()
{
	// ros::WallTime updateRIght_time_start = ros::WallTime::now();
	if(baxter_controller->BaxterPoseControlServer::updateManipulatorVariables("all"))
	{
		if(baxter_controller->BaxterPoseControlServer::importManipulatorState("right", pe_init_baxterRight, pe_now_baxterRight, jacobian_baxterRight, q_baxterRight, joint_velocity_limit_baxterRight))
		{
			if(baxter_controller->BaxterPoseControlServer::importManipulatorState("left", pe_init_baxterLeft, pe_now_baxterLeft, jacobian_baxterLeft, q_baxterLeft, joint_velocity_limit_baxterLeft))
			{
				// ros::WallTime updateRIght_time_end = ros::WallTime::now();
				// ros::WallDuration updateRIght_time_dur = updateRIght_time_end  - updateRIght_time_start;
				// std::cout << "updateRIght duration_server: " << updateRIght_time_dur.toSec() << std::endl;
				return 1;
			}
		}
	}
		// std::cout << "Here 4" << std::endl;

	// 	if(baxter_controller->BaxterPoseControlServer::importManipulatorState("right", pe_init_baxterRight, pe_now_baxterRight, jacobian_baxterRight, q_baxterRight, joint_velocity_limit_baxterRight))
	// 	{
	// 		ros::WallTime updateLeft_time_start = ros::WallTime::now();
	// 		if(baxter_controller->BaxterPoseControlServer::updateManipulatorVariables("left"))
	// 		{
	// 			ros::WallTime updateLeft_time_end = ros::WallTime::now();
	// 			ros::WallDuration updateLeft_time_dur = updateLeft_time_end  - updateLeft_time_start;
	// 			std::cout << "updateLeft duration_server: " << updateLeft_time_dur.toSec() << std::endl;		
	// 			if(baxter_controller->BaxterPoseControlServer::importManipulatorState("left", pe_init_baxterLeft, pe_now_baxterLeft, jacobian_baxterLeft, q_baxterLeft, joint_velocity_limit_baxterLeft))
	// 				return 1;
	// 			else
	// 			{
	// 				std::cout << "cannot get left arm state" << std::endl;
	// 				return 0;
	// 			}
	// 		}
	// 		else
	// 		{
	// 			std::cout << "cannot update left arm " << std::endl;
	// 			return 0;
	// 		}				
	// 	}
	// 	else
	// 	{
	// 		std::cout << "cannot get right arm state" << std::endl;
	// 		return 0;
	// 	}
	// }
	// else
	// {
	// 	std::cout << "cannot update left arm" << std::endl;
	// 	return 0;
	// }
}

MatrixXd relativeJacobian_inAbsFrame(MatrixXd jacobian_ref, MatrixXd jacobian_tool, Matrix<double,8,1> absPose_baseFrame)
{
	// ROS_INFO("here 23");
	MatrixXd relJacobian, relJacobian_8d, jacobian_6d;
	relJacobian = MatrixXd::Zero(6, (jacobian_ref.cols() + jacobian_tool.cols()));
	relJacobian_8d = MatrixXd::Zero(8, (jacobian_ref.cols() + jacobian_tool.cols()));
	jacobian_6d = MatrixXd::Zero(6, jacobian_ref.cols());
	jacobian_6d.block(0, 0, 3, jacobian_ref.cols()) = jacobian_ref.block(5, 0,  3, jacobian_ref.cols());
	jacobian_6d.block(3, 0, 3, jacobian_ref.cols()) = jacobian_ref.block(1, 0,  3, jacobian_ref.cols());

	Matrix<double,8,1> line_transform_tf = DQoperations::classicConjDQ(absPose_baseFrame);
	for (int i = 0; i < (jacobian_ref.cols()); i++)
	{
		// std::cout << "before transformation: " << DQoperations::transformLine6dVector(jacobian_6d.col(i), DQoperations::classicConjDQ(p_ee_ref)) << std::endl;
		relJacobian.col(i) << -DQoperations::transformLine6dVector(jacobian_6d.col(i).transpose(), line_transform_tf).transpose();
		// std::cout << relJacobian.col(i).transpose()  << std::endl;		
		// relJacobian.col(i + jacobian_ref.cols()) = DQoperations::transformLine6dVector(jacobian_6d.col(i), DQoperations::classicConjDQ(p_ee_ref));
	}
	// ROS_INFO("here 25");
	jacobian_6d.block(0, 0, 3, jacobian_tool.cols()) = jacobian_tool.block(5, 0,  3, jacobian_tool.cols());
	jacobian_6d.block(3, 0, 3, jacobian_tool.cols()) = jacobian_tool.block(1, 0,  3, jacobian_tool.cols());	
	// ROS_INFO("here 26");
	for (int i = 0; i < (jacobian_tool.cols()); i++)
	{
		// std::cout << "before transformation: " << DQoperations::transformLine6dVector(jacobian_6d.col(i), DQoperations::classicConjDQ(p_ee_ref)) << std::endl;		
		relJacobian.col(i+jacobian_ref.cols()) = DQoperations::transformLine6dVector(jacobian_6d.col(i).transpose(), line_transform_tf).transpose();
		// std::cout << "after transformation: " << DQoperations::transformLine6dVector(jacobian_6d.col(i), DQoperations::classicConjDQ(p_ee_ref)) << std::endl;
		// std::cout << relJacobian.col(i+jacobian_ref.cols()).transpose()  << std::endl;		
	}

	relJacobian_8d = DQController::jacobianDual_8d((jacobian_ref.cols() + jacobian_tool.cols()), relJacobian);
	relJacobian = relJacobian_8d;
	relJacobian_8d.block(0, 0, 4, relJacobian.cols()) = relJacobian.block(4, 0, 4, relJacobian.cols());
	relJacobian_8d.block(4, 0, 4, relJacobian.cols()) = relJacobian.block(0, 0, 4, relJacobian.cols()) ; 
		// std::cout << "relJacobian_8d: " << std::endl;
		// std::cout << relJacobian_8d << std::endl;	
	// ROS_INFO("here 27");
	return relJacobian_8d;
}




MatrixXd relativeJacobian(MatrixXd jacobian_ref, MatrixXd jacobian_tool, Matrix<double,8,1> p_ee_ref, Matrix<double,8,1> p_ee_tool)
{
	// ROS_INFO("here 23");
	MatrixXd relJacobian, relJacobian_8d, jacobian_6d;
	relJacobian = MatrixXd::Zero(6, (jacobian_ref.cols() + jacobian_tool.cols()));
	relJacobian_8d = MatrixXd::Zero(8, (jacobian_ref.cols() + jacobian_tool.cols()));
	jacobian_6d = MatrixXd::Zero(6, jacobian_ref.cols());
	jacobian_6d.block(0, 0, 3, jacobian_ref.cols()) = jacobian_ref.block(5, 0,  3, jacobian_ref.cols());
	jacobian_6d.block(3, 0, 3, jacobian_ref.cols()) = jacobian_ref.block(1, 0,  3, jacobian_ref.cols());
	// RowVector3d p_rel;

	// Matrix4d pose_ref =  DQoperations::dq2HTM(p_ee_ref);
	// Matrix4d pose_tool =  DQoperations::dq2HTM(p_ee_tool);
	// p_rel << (pose_ref(0,3) - pose_tool(0,3)), (pose_ref(1,3) - pose_tool(1,3)), (pose_ref(2,3) - pose_tool(2,3));
	// ROS_INFO("here 24");
	
	// std::cout << "relJacobian: " <<  std::endl;
	for (int i = 0; i < (jacobian_ref.cols()); i++)
	{
		// std::cout << "before transformation: " << DQoperations::transformLine6dVector(jacobian_6d.col(i), DQoperations::classicConjDQ(p_ee_ref)) << std::endl;
		relJacobian.col(i) << -DQoperations::transformLine6dVector(jacobian_6d.col(i).transpose(), DQoperations::classicConjDQ(p_ee_ref)).transpose();
		// std::cout << relJacobian.col(i).transpose()  << std::endl;		
		// relJacobian.col(i + jacobian_ref.cols()) = DQoperations::transformLine6dVector(jacobian_6d.col(i), DQoperations::classicConjDQ(p_ee_ref));
	}
	// ROS_INFO("here 25");
	jacobian_6d.block(0, 0, 3, jacobian_tool.cols()) = jacobian_tool.block(5, 0,  3, jacobian_tool.cols());
	jacobian_6d.block(3, 0, 3, jacobian_tool.cols()) = jacobian_tool.block(1, 0,  3, jacobian_tool.cols());	
	// ROS_INFO("here 26");
	for (int i = 0; i < (jacobian_tool.cols()); i++)
	{
		// std::cout << "before transformation: " << DQoperations::transformLine6dVector(jacobian_6d.col(i), DQoperations::classicConjDQ(p_ee_ref)) << std::endl;		
		relJacobian.col(i+jacobian_ref.cols()) = DQoperations::transformLine6dVector(jacobian_6d.col(i).transpose(), DQoperations::classicConjDQ(p_ee_ref)).transpose();
		// std::cout << "after transformation: " << DQoperations::transformLine6dVector(jacobian_6d.col(i), DQoperations::classicConjDQ(p_ee_ref)) << std::endl;
		// std::cout << relJacobian.col(i+jacobian_ref.cols()).transpose()  << std::endl;		
	}

	relJacobian_8d = DQController::jacobianDual_8d((jacobian_ref.cols() + jacobian_tool.cols()), relJacobian);
	relJacobian = relJacobian_8d;
	relJacobian_8d.block(0, 0, 4, relJacobian.cols()) = relJacobian.block(4, 0, 4, relJacobian.cols());
	relJacobian_8d.block(4, 0, 4, relJacobian.cols()) = relJacobian.block(0, 0, 4, relJacobian.cols()) ; 
		// std::cout << "relJacobian_8d: " << std::endl;
		// std::cout << relJacobian_8d << std::endl;	
	// ROS_INFO("here 27");
	return relJacobian_8d;
}

MatrixXd absoluteJacobian(MatrixXd jacobian_ref, MatrixXd jacobian_tool, Matrix<double,8,1> p_ee_ref, Matrix<double,8,1> p_ee_tool)
{
	MatrixXd absJacobian, absJacobian_8d, jacobian_6d;
	absJacobian = MatrixXd::Zero(6, (jacobian_ref.cols() + jacobian_tool.cols()));
	absJacobian_8d = MatrixXd::Zero(8, (jacobian_ref.cols() + jacobian_tool.cols()));
	jacobian_6d = MatrixXd::Zero(6, jacobian_ref.cols());
	// ROS_INFO("here 8");
	jacobian_6d.block(0, 0, 3, jacobian_ref.cols()) = jacobian_ref.block(1, 0,  3, jacobian_ref.cols());
	// ROS_INFO("here 9");	
	jacobian_6d.block(3, 0, 3, jacobian_ref.cols()) = jacobian_ref.block(5, 0,  3, jacobian_ref.cols());
		// std::cout << "jacobian_6d_ref: " << std::endl;
		// std::cout << jacobian_6d << std::endl;		
	RowVector3d p_rel;

	Matrix4d pose_ref =  DQoperations::dq2HTM(p_ee_ref);
	Matrix4d pose_tool =  DQoperations::dq2HTM(p_ee_tool);
	p_rel << (pose_ref(0,3) - pose_tool(0,3)), (pose_ref(1,3) - pose_tool(1,3)), (pose_ref(2,3) - pose_tool(2,3));
	// ROS_INFO("here 10");	
	for (int i = 0; i < jacobian_ref.cols(); i++)
	{
		RowVector3d vec = ((jacobian_ref.col(i)).tail(3)).transpose();
		vec = vec.cross(p_rel);
		absJacobian.col(i) << (((jacobian_6d.col(i)).head(3))/2 + vec.transpose()/4), ((jacobian_6d.col(i)).tail(3))/2;
	}
	// ROS_INFO("here 11");	
	jacobian_6d = MatrixXd::Zero(6, jacobian_tool.cols());
	jacobian_6d.block(0, 0, 3, jacobian_tool.cols()) = jacobian_tool.block(1, 0,  3, jacobian_tool.cols());
	jacobian_6d.block(3, 0, 3, jacobian_tool.cols()) = jacobian_tool.block(5, 0,  3, jacobian_tool.cols());	
	// std::cout << "jacobian_6d_tool: " << std::endl;
	// std::cout << jacobian_6d << std::endl;		
// ROS_INFO("here 12");	
	for (int i = 0; i < jacobian_tool.cols(); i++)
	{
		RowVector3d vec= ((jacobian_tool.col(i)).tail(3)).transpose(); 
		vec = vec.cross(-p_rel);		
		// std::cout << "vec: " << (((jacobian_6d.col(i)).head(3)).transpose())/2 << std::endl;
		// std::cout << "vec: " << (((jacobian_6d.col(i)).tail(3))/2 + vec.transpose()/4).transpose() << std::endl;
		RowVectorXd tem_vec1, tem_vec2, tem_vec;
		tem_vec1 = (((jacobian_6d.col(i)).tail(3)).transpose())/2 ;
		tem_vec2 = (((jacobian_6d.col(i)).head(3))/2 + vec.transpose()/4).transpose();
		// std::cout << "tem_vec1: " << tem_vec1 << std::endl;		
		// std::cout << "tem_vec2: " << tem_vec2 << std::endl;		
		// tem_vec <<  (((jacobian_6d.col(i)).head(3)))/2, (((jacobian_6d.col(i)).tail(3))/2 + vec.transpose()/4).transpose();
		tem_vec = RowVectorXd::Zero(6);
		tem_vec <<  tem_vec2, tem_vec1;
		// std::cout << "tem_vec: " << tem_vec << std::endl;		
		absJacobian.col(i+jacobian_ref.cols()) << tem_vec.transpose();
		// std::cout << "absJacobian.col(" << (i+jacobian_ref.cols()) << "):" << std::endl;
		// std::cout << absJacobian.col(i+jacobian_ref.cols()) << std::endl;
	}
	// ROS_INFO("here 12");	
	// 	std::cout << "absJacobian: " << std::endl;
	// 	std::cout << absJacobian << std::endl;	
	absJacobian_8d = DQController::jacobianDual_8d(absJacobian.cols(), absJacobian);
	// ROS_INFO("here 13");		
		// std::cout << "absJacobian_8d: " << std::endl;
		// std::cout << absJacobian_8d << std::endl;
	return absJacobian_8d;
}

// void getRelPosesFromAbsolute(Matrix<double,8,1> dq, Matrix<double,8,1> p_ee_r_1, Matrix<double,8,1>  p_ee_t_1, Matrix<double,8,1> p_ee_r_2, Matrix<double,8,1>  p_ee_t_2)
// {
// 	double theta_rel, d_rel;
// 	RowVector3d l_rel, m_rel;
// 	Matrix<double,8,1> p_ee_halfRot_1, p_ee_halfRot_2;
// 	p_ee_halfRot << 1, 0, 0, 0, 0, 0, 0, 0 ;
// 	RowVector4d q_abs_rot, q_abs_trans, q_abs_rot_2, q_abs_trans_2; 
// 	p_rel = dq;
// 	RowVector4d trans, rot, trans_pos, trans_neg;
// 	DQoperations::dq2rotAndTransQuat(dq, rot, trans);
// 	trans_pos = trans/2;
// 	trans_neg = -trans/2;
// 	std::cout << "p_rel: " << p_rel.transpose() << std::endl;	


// 	if (p_rel(0,0) < 0  )
// 	{
// 		p_rel = -p_rel;
// 	}
// 	if(p_rel(0,0) > 0.9995)
// 	{
// 		p_rel << 1, 0, 0, 0, p_rel(4,0), p_rel(5,0), p_rel(6,0), p_rel(7,0);
// 		std::cout << "relative orientation is UNITY" << std::endl;
// 		std::cout << "p_rel: " << p_rel.transpose() <<  std::endl;
// 	}
// 	else 	
// 	{
// 		DQoperations::dq2screw(p_rel, theta_rel, d_rel, l_rel, m_rel); 
// 		std::cout << "constructing abs rot::" << std::endl;
// 		p_ee_halfRot_1 = DQoperations::screw2DQ(theta_rel/2, l_rel, 0, m_rel);
// 		p_ee_halfRot_2 = DQoperations::screw2DQ(theta_rel/2, -l_rel, 0, m_rel);
// 		std::cout << "p_ee_halfRot_1: " << p_ee_halfRot.transpose() << std::endl;
// 		std::cout << "p_ee_halfRot_2: " << p_abs_2.transpose() << std::endl;
// 		std::cout << "theta_rel/2: " << theta_rel/2 << std::endl;
// 		std::cout << "l_rel: " << l_rel << std::endl;
// 		std::cout << "m_rel: " << m_rel << std::endl;
// 		std::cout << "d_rel: " << d_rel << std::endl;
// 	}		
// 	RowVector4d rot_temp;
// 	rot_temp << p_ee_halfRot_1(0,0), p_ee_halfRot_1(1,0), p_ee_halfRot_1(2,0), p_ee_halfRot_1(3,0);
// 	std::cout << "rot_temp: " << rot_temp << std::endl;
// 	p_ee_r_1 = DQoperations::rotTrans2dq(rot_temp, trans);

// }

void getAbsoluteRelativePose(Matrix<double,8,1> p_ee_r, Matrix<double,8,1>  p_ee_t, Matrix<double,8,1> &p_abs, Matrix<double,8,1>  &p_rel)
{
	// std::cout << "getAbsoluteRelativePose coming through" << std::endl;
	// std::cout << "p_ee_r: " << p_ee_r.transpose() << std::endl;
	// std::cout << "p_ee_t: " << p_ee_t.transpose() << std::endl;
	double theta_rel, d_rel;
	RowVector3d l_rel, m_rel;
	Matrix<double,8,1> p_ee_halfRot;
	p_ee_halfRot << 1, 0, 0, 0, 0, 0, 0, 0 ;
	RowVector4d q_abs_rot, q_abs_trans, q_abs_rot_2, q_abs_trans_2; 
	p_rel = DQoperations::mulDQ(DQoperations::classicConjDQ(p_ee_r), p_ee_t);

	// std::cout << "p_rel: " << p_rel.transpose() << std::endl;	

	// if (l_rel(0) < 0 && fabs(theta_rel/2) < 1.56  )
	// {
	// 	std::cout << "hacking here_1" << std::endl;
	// 	l_rel = - l_rel;
	// 	theta_rel = - theta_rel;
	// }
	// else 	
	if (p_rel(0,0) < 0  )
	{
		p_rel = -p_rel;
	}
	if(p_rel(0,0) > 0.9995)
	{
		p_rel << 1, 0, 0, 0, p_rel(4,0), p_rel(5,0), p_rel(6,0), p_rel(7,0);
		// std::cout << "relative orientation is UNITY" << std::endl;
		// std::cout << "p_rel: " << p_rel.transpose() <<  std::endl;
	}
	else 	
	{
		DQoperations::dq2screw(p_rel, theta_rel, d_rel, l_rel, m_rel); 
		// if (l_rel(0) < 0)
		// {
		// 	std::cout << "hacking here_2" << std::endl;
		// 	l_rel = - l_rel;
		// 	theta_rel = - theta_rel;
		// }
		// if (fabs(theta_rel/2) > 1.55 && fabs(theta_rel/2) < 1.58 )
		// {
		// // if (theta_rel/2 < 0)
		// // 	theta_rel = - M_PI;
		// // else
		// 	theta_rel =  M_PI;
		// }
		std::cout << "constructing abs rot::" << std::endl;
		p_ee_halfRot = DQoperations::screw2DQ(theta_rel/2, l_rel, 0, m_rel);
		p_abs_2 = DQoperations::screw2DQ(theta_rel/2, -l_rel, 0, m_rel);
		// std::cout << "p_ee_halfRot: " << p_ee_halfRot.transpose() << std::endl;
		// std::cout << "p_ee_halfRot_2: " << p_abs_2.transpose() << std::endl;
		// std::cout << "theta_rel/2: " << theta_rel/2 << std::endl;
		// std::cout << "l_rel: " << l_rel << std::endl;
		// std::cout << "m_rel: " << m_rel << std::endl;
		// std::cout << "d_rel: " << d_rel << std::endl;
	}		


	// if((theta_rel/2 > 1.56) && (theta_rel/2 < 1.58) && (l_rel(0) < 0))
	// {
	// 	std::cout << "hacking here" << std::endl;
	// 	// theta_rel = - theta_rel;
	// 	l_rel = -l_rel;
	// 	std::cout << "theta_rel/2: " << theta_rel/2 << std::endl;
	// 	std::cout << "l_rel: " << l_rel << std::endl;		
	// }
	// p_ee_halfRot = DQoperations::screw2DQ(theta_rel/2, l_rel, 0, m_rel);

	// std::cout << "p_ee_r.head(4): " << p_ee_r.head(4) << std::endl;	
	q_abs_rot = DQoperations::multQuat( p_ee_r.head(4), p_ee_halfRot.head(4));
	q_abs_rot_2 = DQoperations::multQuat( p_ee_r.head(4), p_abs_2.head(4));
	
	// std::cout << "q_abs_rot: " << q_abs_rot << std::endl;	
	// std::cout << "q_abs_rot_2: " << q_abs_rot_2 << std::endl;	
	// std::cout << DQoperations::multQuat(DQoperations::conjQuat(p_ee_r.head(4)), q_abs_rot)  << std::endl;		
	// std::cout << "q_abs_from_t: " << std::endl;	
	// std::cout << DQoperations::conjQuat(DQoperations::multQuat(DQoperations::conjQuat(p_ee_t.head(4)), q_abs_rot))  << std::endl;			

	q_abs_trans =  DQoperations::multQuat((DQoperations::multQuat(p_ee_r.tail(4), DQoperations::conjQuat(p_ee_r.head(4))) + DQoperations::multQuat(p_ee_t.tail(4), DQoperations::conjQuat(p_ee_t.head(4)))), q_abs_rot)/2;
	q_abs_trans_2 =  DQoperations::multQuat((DQoperations::multQuat(p_ee_r.tail(4), DQoperations::conjQuat(p_ee_r.head(4))) + DQoperations::multQuat(p_ee_t.tail(4), DQoperations::conjQuat(p_ee_t.head(4)))), q_abs_rot_2)/2;
	// std::cout << "q_abs_trans: " << std::endl;	
	// std::cout << q_abs_trans  << std::endl;	
	// std::cout << "q_abs_rot: " << std::endl;	
	// std::cout << q_abs_rot  << std::endl;		
	p_abs << q_abs_rot(0), q_abs_rot(1), q_abs_rot(2), q_abs_rot(3), q_abs_trans(0), q_abs_trans(1), q_abs_trans(2), q_abs_trans(3);
	p_abs_2 << q_abs_rot_2(0), q_abs_rot_2(1), q_abs_rot_2(2), q_abs_rot_2(3), q_abs_trans_2(0), q_abs_trans_2(1), q_abs_trans_2(2), q_abs_trans_2(3);
	// std::cout << "p_abs: " << std::endl;	
	// std::cout << p_abs.transpose()  << std::endl;	
	// std::cout << "p_abs_2: " << std::endl;	
	// std::cout << p_abs_2.transpose()  << std::endl;		
	Matrix4d p_r_htm = DQoperations::dq2HTM(p_ee_r);
	Matrix4d p_t_htm = DQoperations::dq2HTM(p_ee_t);
	Matrix4d p_abs_htm = DQoperations::dq2HTM(p_abs);
	Matrix4d p_rel_htm = DQoperations::dq2HTM(p_rel);
	// std::cout << "p_r_htm: " << std::endl;	
	// std::cout << p_r_htm  << std::endl;	
	// std::cout << "p_t_htm: " << std::endl;	
	// std::cout << p_t_htm  << std::endl;	
	// std::cout << "p_abs_htm: " << std::endl;	
	// std::cout << p_abs_htm  << std::endl;	
	// std::cout << "p_abs_htm_2"  << std::endl;	
	// std::cout << DQoperations::dq2HTM(p_abs_2)  << std::endl;	
	// std::cout << "p_rel_htm: " << std::endl;	
	// std::cout << p_rel_htm  << std::endl;	

}

RowVectorXd getScrewError_8d_control(Matrix<double,8,1> pose_now, Matrix<double,8,1> pose_desired)
{
	// if(pose_now(0,0)<0)
	// {
	// 	pose_now= -pose_now;
	// }
	RowVectorXd screw_error=RowVectorXd::Zero(8);
	RowVector3d v_e, w_e;	
	Matrix<double,8,1> pose_error;
	pose_error=DQoperations::mulDQ(pose_now, DQoperations::classicConjDQ(pose_desired));
	// std::cout << "pose_error:  " << pose_error.transpose() << std::endl;
	// std::cout << "pose_now: " << pose_now.transpose() << std::endl;
	// std::cout << "pose_ee_desired: " << pose_ee_desired.transpose() << std::endl;
	// std::cout << "pose_error: " << pose_error.transpose() << std::endl;
	RowVector3d l_e, m_e;
	double theta_e, d_e;
	// DQoperations::dq2screw(pose_error, theta_e, d_e, l_e, m_e);
	double eps_theta=0.05; /*0.1 degrees*/ 
	double sr, sd, theta;
	RowVector3d vr, vd;
	// std::cout << "dq(dq2screw): " << dq.transpose() << std::endl;
	// std::cout << "dq(0,0): " << dq(0,0) << std::endl;

	Matrix<double,8,1> dq = pose_error;
	// if(dq(0,0)<0)
	// {
	// 	dq=-dq;
	// 	std::cout << "dq(dq2screw): " << dq.transpose() << std::endl;
	// }	
	sr=dq(0);

	if(sr > 1.0)
		sr =1.0 ;
	else if(sr < -1.0)
		sr=-1;

	vr << dq(1), dq(2), dq(3);
	sd=dq(4);
	vd << dq(5), dq(6), dq(7);
	theta_e=2*acos(sr);
	// std::cout << "sr: " << sr << std::endl;
	theta_e=DQoperations::normalizeAngle(theta_e);
	// if(theta_e > M_PI)
	// {
	// 	theta_e = 2*M_PI - theta_e;
			
	// }	
	// if(theta_e < -M_PI)
	// {
	// 	theta_e = -(2*M_PI - theta_e);
			
	// }		
	double absTheta=fabs(theta_e);
	// std::cout << "abs(theta_e):  " << absTheta << std::endl;
	if (absTheta > eps_theta && absTheta < (2*M_PI -eps_theta))
	{
		// std::cout << "check:: theta_e:  " << absTheta << "eps_theta: "  << eps_theta << "(2*M_PI -eps_theta): "  << (2*M_PI -eps_theta) << std::endl;
		// std::cout << "definitly here, vr: " << std::endl;
		// std::cout << "vr: " <<  vr << std::endl;
		// std::cout << "vr.norm " <<  vr.norm() << std::endl;
		// std::cout << "vr/vr.norm " <<  vr/vr.norm() << std::endl;
		l_e=vr/vr.norm();
		d_e=-sd*(2/(vr.norm()));
		m_e=(vd-sr*0.5*d_e*l_e)/vr.norm();
	}	
	else
	{
		// std::cout << "why do you have to come here:  " << std::endl;
		// std::cout << "check:: theta_e:  " << absTheta << "eps_theta: "  << eps_theta << "(2*M_PI -eps_theta): "  << (2*M_PI -eps_theta) << std::endl;
		RowVector4d qrr, qdd, tt;
		RowVector3d t;
		qrr << dq(0), dq(1), dq(2), dq(3);	
		qdd << dq(4), dq(5), dq(6), dq(7);
		tt=2*DQoperations::multQuat(qdd, DQoperations::conjQuat(qrr));
		t << tt(1), tt(2), tt(3);  
		d_e=t.norm();
		l_e=t/d_e;
		m_e << 0,0,0;			
	}
	// 	std::cout << "theta_e:" << theta_e << std::endl;
	// std::cout << "l_e:" << l_e << std::endl;
	// std::cout << "d_e:" << d_e << std::endl;
	// std::cout << "m_e:" << m_e << std::endl;	
	// DQoperations::dq2screw(pose_error, theta_e, d_e, l_e, m_e);



	v_e = theta_e*m_e + d_e*l_e;
	w_e = theta_e*l_e;

 	// double norm_error_position=DQoperations::get_error_screw_param(pose_now, pose_desired,  v_e, w_e);
 	screw_error << 0, v_e(0), v_e(1), v_e(2), 0, w_e(0), w_e(1), w_e(2);
 	// std::cout << "screwError(getScrewError_8d): " << 0 << v_e(0) << v_e(1) << v_e(2) << 0 << w_e(0) << w_e(1) << w_e(2) << std::endl;
 	return screw_error;
}

bool coop_TaskSpaceControlCallback(dq_robotics::BaxterControl::Request& baxterControl_req, 
														  dq_robotics::BaxterControl::Response& baxterControl_res)
{
	// ROS_INFO("here 1");
	ros::WallTime start = ros::WallTime::now();
	RowVectorXd screw_error_abs, screw_error_abs_2, screw_error_rel, combined_screw_error, q_dot, q, jointCmds_baxterRight, jointCmds_baxterLeft;
	// MatrixXd rel_jacobian, abs_jacobian, combined_jacobian;
	// combined_jacobian = MatrixXd::Zero(16, (jacobian_baxterRight.cols() + jacobian_baxterLeft.cols()));
	Matrix<double,8,1> desiredAbsPose, tf_abs_wrt_ref, temp, desiredRelPose, p_abs_now, p_rel_now, pe_now_baxterRight_vs, pe_now_baxterLeft_vs, starting_abs_pose;
	double normError =1;
	DQoperations::doubleToDQ(desiredAbsPose, baxterControl_req.desiredAbsPose);
	DQoperations::doubleToDQ(desiredRelPose, baxterControl_req.desiredRelPose);	
	// ROS_INFO("here 2");
	if(baxterControl_req.virtualSticks)
	{
		std::cout << "******************this is the correct function**************************" << std::endl;
		// ros::WallTime update_time = ros::WallTime::now();
		if(!update_baxter())
		{
			std::cout << "The arms could not be updated" << std::endl;
			return 0;
		}
		// ros::WallTime update_time_end = ros::WallTime::now();
		// ros::WallDuration update_time_dur = update_time_end  - update_time; 
		// std::cout << "update_time loop duration_server: " << update_time_dur.toSec() << std::endl;
		
		// ros::WallTime vsConstruct_time_start = ros::WallTime::now();
		 
		if (!baxterControl_req.newTask)
		{
			pe_now_baxterRight_vs = DQoperations::mulDQ(pe_now_baxterRight, p_right_stick);
			pe_now_baxterLeft_vs = DQoperations::mulDQ(pe_now_baxterLeft, p_left_stick);
		}
		else if (baxterControl_req.newTask)
		{
			// vsConstruct_time_start = ros::WallTime::now();
			starting_abs_pose = DQoperations::returnDoubleToDQ(baxterControl_req.starting_abs_pose);
			// std::cout << "starting_abs_pose:  " << starting_abs_pose.transpose() << std::endl;
			// std::cout << "pe_now_baxterRight:  " << pe_now_baxterRight.transpose() << std::endl;
			// std::cout << "pe_now_baxterLeft:  " << pe_now_baxterLeft.transpose() << std::endl;
			p_right_stick = DQoperations::mulDQ(DQoperations::classicConjDQ(pe_now_baxterRight), starting_abs_pose) ;
			p_left_stick = DQoperations::mulDQ(DQoperations::classicConjDQ(pe_now_baxterLeft), starting_abs_pose)	;
			// std::cout << "p_right_stick:  " << p_right_stick.transpose() << std::endl;			
			// std::cout << "p_left_stick:  " << p_left_stick.transpose() << std::endl;			
			pe_now_baxterRight_vs = DQoperations::mulDQ(pe_now_baxterRight, p_right_stick);
			pe_now_baxterLeft_vs = DQoperations::mulDQ(pe_now_baxterLeft, p_left_stick);
			// getAbsoluteRelativePose(pe_now_baxterRight_vs, pe_now_baxterLeft_vs, p_abs_now, p_rel_stick);
			// std::cout << "Is the relative pose of the stick ends Identity!! p_rel_stick_htm: " << std::endl;	
			// std::cout << DQoperations::dq2HTM(p_rel_stick) << std::endl;
			// ros::WallTime vsConstruct_time_end = ros::WallTime::now();
			// ros::WallDuration vsConstruct_time_dur = vsConstruct_time_end  - vsConstruct_time_start;
			// std::cout << "vsConstruct duration_server: " << vsConstruct_time_dur.toSec() << std::endl; 
		} 
		// ros::WallTime absRelPoseCalc_time_start = ros::WallTime::now();
		getAbsoluteRelativePose(pe_now_baxterRight_vs, pe_now_baxterLeft_vs, p_abs_now, p_rel_now);
		// ros::WallTime absRelPoseCalc_time_end = ros::WallTime::now();
		// ros::WallDuration absRelPoseCalc_time_dur = absRelPoseCalc_time_end  - absRelPoseCalc_time_start;
		// std::cout << "absRelPoseCalc duration_server: " << absRelPoseCalc_time_dur.toSec() << std::endl; 
		// p_rel_now = DQoperations::mulDQ(DQoperations::classicConjDQ(pe_now_baxterRight_vs), pe_now_baxterLeft_vs);
		// getAbsoluteRelativePose(pe_now_baxterRight, pe_now_baxterLeft, p_abs_now, p_rel_now);
		// tf_abs_wrt_ref = DQoperations::mulDQ(DQoperations::classicConjDQ(pe_now_baxterRight_vs), desiredAbsPose) ;
		// std::cout << "desiredRelPose_beforeTF:  " << desiredRelPose.transpose() << std::endl;
		// temp = DQoperations::dq2twist(desiredRelPose) ;
		// std::cout << "temp:  " << temp.transpose() << std::endl;
		// // temp.block(0,0,4,1) = desiredRelPose.block(4,0,4,1);
		// // temp.block(4,0,4,1) = desiredRelPose.block(0,0,4,1);
		// temp = DQoperations::transformLine(temp, tf_abs_wrt_ref);
		// temp.block(0,0,4,1) = desiredRelPose.block(4,0,4,1);
		// temp.block(4,0,4,1) = desiredRelPose.block(0,0,4,1);
		// desiredRelPose = DQoperations::mulDQ(DQoperations::mulDQ(p_right_stick, DQoperations::twist2dq(temp)), DQoperations::classicConjDQ(p_left_stick)) ; 
		// desiredRelPose = temp;		
		// ros::WallTime screwErrCalc_time_start = ros::WallTime::now();
		// std::cout << "It is the absolute " << std::endl;
		// std::cout << "p_abs_now:  " << p_abs_now.transpose() << std::endl;
		// std::cout << "desiredRelPose:  " << desiredRelPose.transpose() << std::endl;
		// std::cout << "desiredRelPose_htm:  "  << std::endl;
		// std::cout << DQoperations::dq2HTM(desiredRelPose) << std::endl;
		screw_error_abs= getScrewError_8d_control(p_abs_now, desiredAbsPose);
		screw_error_abs_2= getScrewError_8d_control(p_abs_2, desiredAbsPose);
		if(screw_error_abs.norm() > screw_error_abs_2.norm())
		{
			screw_error_abs = screw_error_abs_2;
			p_abs_now = p_abs_2;
		}
		std::cout << "It is the relative " << std::endl;		
		screw_error_rel= DQController::getScrewError_8d(p_rel_now, desiredRelPose);
		// std::cout << "rowvector screw_error_rel_beforeTF: " << screw_error_rel << std::endl;
		// Matrix<double,8,1> screwError_rel, tf_ref_wrt_abs;
		// screwError_rel << screw_error_rel.tail(4).transpose(), screw_error_rel.head(4).transpose();
		// std::cout << "Matrix screwError_rel_beforeTF: " << screwError_rel.transpose() << std::endl;
		// // screwError_rel << screw_error_rel;
		// tf_ref_wrt_abs = DQoperations::mulDQ(DQoperations::classicConjDQ(p_abs_now), pe_now_baxterRight);
		// screwError_rel = DQoperations::transformLine(screwError_rel, tf_ref_wrt_abs);
		// std::cout << "Matrix screwError_rel: " << screwError_rel.transpose() << std::endl;
		// screw_error_rel << screwError_rel(4, 0), screwError_rel(5, 0), screwError_rel(6, 0), screwError_rel(7, 0), screwError_rel(0, 0), screwError_rel(1, 0), screwError_rel(2, 0), screwError_rel(3, 0);		
		// std::cout << "screw_error_rel: " << screw_error_rel << std::endl;
		// std::cout << "screw_error_abs: " << screw_error_abs << std::endl;

		combined_screw_error = RowVectorXd::Zero(screw_error_abs.cols()+screw_error_rel.cols());
		combined_screw_error << screw_error_abs.row(0), screw_error_rel.row(0);

		// std::cout << "combined_screw_error: " << combined_screw_error << std::endl;
		// ros::WallTime screwErrCalc_time_end = ros::WallTime::now();
		// ros::WallDuration screwErrCalc_time_dur = screwErrCalc_time_end  - screwErrCalc_time_start;
		// std::cout << "screwErrCalc duration_server: " << screwErrCalc_time_dur.toSec() << std::endl; 
		// ros::WallTime jacCalc_time_start = ros::WallTime::now();		
		MatrixXd abs_jacobian =  absoluteJacobian(jacobian_baxterRight, jacobian_baxterLeft, pe_now_baxterRight, pe_now_baxterLeft);

		MatrixXd rel_jacobian = relativeJacobian(jacobian_baxterRight, jacobian_baxterLeft, pe_now_baxterRight_vs, pe_now_baxterLeft);
		// MatrixXd rel_jacobian =  relativeJacobian_inAbsFrame(jacobian_baxterRight, jacobian_baxterLeft, pe_now_baxterRight_vs);
		MatrixXd combined_jacobian = MatrixXd::Zero(16, (jacobian_baxterRight.cols() + jacobian_baxterLeft.cols()));
		combined_jacobian.block(0, 0, abs_jacobian.rows(), abs_jacobian.cols()) = abs_jacobian.block(0, 0, abs_jacobian.rows(), abs_jacobian.cols());
		combined_jacobian.block(abs_jacobian.rows(), 0, rel_jacobian.rows(), rel_jacobian.cols()) = rel_jacobian.block(0, 0, rel_jacobian.rows(), rel_jacobian.cols());
		// ros::WallTime jacCalc_time_end = ros::WallTime::now();
		// ros::WallDuration jacCalc_time_dur = jacCalc_time_end  - jacCalc_time_start;
		// std::cout << "jacCalc duration_server: " << jacCalc_time_dur.toSec() << std::endl; 

		// ros::WallTime robotCmd_time_start = ros::WallTime::now();		
		q_dot= DQController::calculateControlVel(Kp, mu, combined_screw_error, combined_jacobian, (combined_jacobian.cols()));

		std::cout << "q_dot: "  << std::endl;	
		std::cout << q_dot << std::endl;	

		q.resize(q_dot.size());
		q << q_baxterRight, q_baxterLeft;
		q = q + q_dot*timeStep; // HERE YOU CAN USE (time_now-time_last) in place of timeStep
		std::cout << "q: " << std::endl;			
		std::cout << q << std::endl;			
		baxter_controller->sendBaxterJointCmds("combined", DQoperations::dqEigenToDQdouble(q), 0);
		// baxter_controller->sendBaxterVelocityCmds("combined", DQoperations::dqEigenToDQdouble(q_dot));
		normError = combined_screw_error.norm();
		baxterControl_res.result = 1;
		baxterControl_res.normError = normError;
		time_start =ros::Time::now().toSec();
		resultMsg.desiredAbsPose = DQoperations::DQToDouble(desiredAbsPose);
		resultMsg.currentAbsPose = DQoperations::DQToDouble(p_abs_now);
		resultMsg.desiredRelPose = DQoperations::DQToDouble(desiredRelPose);
		resultMsg.currentRelPose = DQoperations::DQToDouble(p_rel_now);
		resultMsg.del_q = DQoperations::dqEigenToDQdouble(q_dot);
		resultMsg.joint_state_left = DQoperations::dqEigenToDQdouble(q_baxterLeft);
		resultMsg.joint_state_right = DQoperations::dqEigenToDQdouble(q_baxterRight);
		resultMsg.norm_error_abs = screw_error_abs.norm();
		resultMsg.norm_error_rel = screw_error_rel.norm();
		resultMsg.timeStamp = time_start;
		result_topics.publish(resultMsg);

		// ros::WallTime robotCmd_time_end = ros::WallTime::now();
		// ros::WallDuration robotCmd_time_dur = robotCmd_time_end  - robotCmd_time_start;
		// std::cout << "robotCmd duration_server: " << robotCmd_time_dur.toSec() << std::endl;		
		ros::WallTime end = ros::WallTime::now();
		ros::WallDuration dur = end - start;
		std::cout << "master loop duration_server: " << dur.toSec() << std::endl;

		return true;
	}

	if(baxterControl_req.tracking && !baxterControl_req.virtualSticks)
	{
		if(!update_baxter())
		{
			std::cout << "The arms could not be updated" << std::endl;
			return 0;
		}
		getAbsoluteRelativePose(pe_now_baxterRight, pe_now_baxterLeft, p_abs_now, p_rel_now);
		std::cout << "It is the absolute " << std::endl;
		std::cout << "p_abs_now:  " << p_abs_now.transpose() << std::endl;
		screw_error_abs= getScrewError_8d_control(p_abs_now, desiredAbsPose);
		screw_error_abs_2= getScrewError_8d_control(p_abs_2, desiredAbsPose);
		if(screw_error_abs.norm() > screw_error_abs_2.norm())
			screw_error_abs = screw_error_abs_2;
		std::cout << "It is the relative " << std::endl;		
		screw_error_rel= DQController::getScrewError_8d(p_rel_now, desiredRelPose);
		std::cout << "screw_error_rel: " << screw_error_rel << std::endl;
		std::cout << "screw_error_abs: " << screw_error_abs << std::endl;

		combined_screw_error = RowVectorXd::Zero(screw_error_abs.cols()+screw_error_rel.cols());
		combined_screw_error << screw_error_abs.row(0), screw_error_rel.row(0);

		std::cout << "combined_screw_error: " << combined_screw_error << std::endl;

		MatrixXd abs_jacobian =  absoluteJacobian(jacobian_baxterRight, jacobian_baxterLeft, pe_now_baxterRight, pe_now_baxterLeft);

		MatrixXd rel_jacobian = relativeJacobian(jacobian_baxterRight, jacobian_baxterLeft, pe_now_baxterRight, pe_now_baxterLeft);

		MatrixXd combined_jacobian = MatrixXd::Zero(16, (jacobian_baxterRight.cols() + jacobian_baxterLeft.cols()));
		combined_jacobian.block(0, 0, abs_jacobian.rows(), abs_jacobian.cols()) = abs_jacobian.block(0, 0, abs_jacobian.rows(), abs_jacobian.cols());
		combined_jacobian.block(abs_jacobian.rows(), 0, rel_jacobian.rows(), rel_jacobian.cols()) = rel_jacobian.block(0, 0, rel_jacobian.rows(), rel_jacobian.cols());
		q_dot= DQController::calculateControlVel(Kp, mu, combined_screw_error, combined_jacobian, (combined_jacobian.cols()));

		std::cout << "q_dot: "  << std::endl;	
		std::cout << q_dot << std::endl;	

		q.resize(q_dot.size());
		q << q_baxterRight, q_baxterLeft;
		q = q + q_dot*timeStep; // HERE YOU CAN USE (time_now-time_last) in place of timeStep
		std::cout << "q: " << std::endl;			
		std::cout << q << std::endl;			
		baxter_controller->sendBaxterJointCmds("combined", DQoperations::dqEigenToDQdouble(q), 0);
		// baxter_controller->sendBaxterVelocityCmds("combined", DQoperations::dqEigenToDQdouble(q_dot));
		normError = combined_screw_error.norm();
		baxterControl_res.result = 1;
		baxterControl_res.normError = normError;
		return true;
  }
	else
	{
	  while (normError > norm_error_const)
	  {
	  	if(!update_baxter())
			{
				std::cout << "The arms could not be updated" << std::endl;
				return 0;
			}
			// ROS_INFO("here 3");
			getAbsoluteRelativePose(pe_now_baxterRight, pe_now_baxterLeft, p_abs_now, p_rel_now);
			// ROS_INFO("here 4");		
			std::cout << "It is the absolute " << std::endl;
			std::cout << "p_abs_now:  " << p_abs_now.transpose() << std::endl;
			// screw_error_abs= DQController::getScrewError_8d(p_abs_now, desiredAbsPose);
			screw_error_abs= getScrewError_8d_control(p_abs_now, desiredAbsPose);
			screw_error_abs_2= getScrewError_8d_control(p_abs_2, desiredAbsPose);
			if(screw_error_abs.norm() > screw_error_abs_2.norm())
				screw_error_abs = screw_error_abs_2;
			std::cout << "It is the relative " << std::endl;		
			screw_error_rel= DQController::getScrewError_8d(p_rel_now, desiredRelPose);
			std::cout << "screw_error_rel: " << screw_error_rel << std::endl;
			std::cout << "screw_error_abs: " << screw_error_abs << std::endl;
			// std::cout << "screw_error_abs.rows(), cols: " << screw_error_abs.rows() << screw_error_abs.cols() << std::endl;
			// std::cout << "screw_error_rel.rows(), cols: " << screw_error_rel.rows() << screw_error_rel.cols() << std::endl;
			

			combined_screw_error = RowVectorXd::Zero(screw_error_abs.cols()+screw_error_rel.cols());
			combined_screw_error << screw_error_abs.row(0), screw_error_rel.row(0);
			// std::cout << "screw_error_abs: " << screw_error_abs << std::endl;		
			// std::cout << "screw_error_rel: " << screw_error_rel << std::endl;
			std::cout << "combined_screw_error: " << combined_screw_error << std::endl;
			// abs_jacobian =   MatrixXd::Zero(8, (jacobian_baxterRight.cols() + jacobian_baxterLeft.cols()));
			MatrixXd abs_jacobian =  absoluteJacobian(jacobian_baxterRight, jacobian_baxterLeft, pe_now_baxterRight, pe_now_baxterLeft);
				// ROS_INFO("here 6");
			MatrixXd rel_jacobian = relativeJacobian(jacobian_baxterRight, jacobian_baxterLeft, pe_now_baxterRight, pe_now_baxterLeft);

			// std::cout << "abs_jacobian: " << std::endl;
			// std::cout << abs_jacobian << std::endl;
			// std::cout << "rel_jacobian: " << std::endl;
			// std::cout << rel_jacobian << std::endl;

			// std::cout << "combined_jacobian.rows(): " << combined_jacobian.rows() <<  "::combined_jacobian.cols(): " << combined_jacobian.cols() << std::endl;
			// std::cout << "rel_jacobian.rows(): " << rel_jacobian.rows() <<  "::rel_jacobian.cols(): " << rel_jacobian.cols() << std::endl;
			// std::cout << "abs_jacobian.rows(): " << abs_jacobian.rows() <<  "::abs_jacobian.cols(): " << abs_jacobian.cols() << std::endl;

			// ROS_INFO("here 7");
			MatrixXd combined_jacobian = MatrixXd::Zero(16, (jacobian_baxterRight.cols() + jacobian_baxterLeft.cols()));
			combined_jacobian.block(0, 0, abs_jacobian.rows(), abs_jacobian.cols()) = abs_jacobian.block(0, 0, abs_jacobian.rows(), abs_jacobian.cols());
			// ROS_INFO("here 17");
			combined_jacobian.block(abs_jacobian.rows(), 0, rel_jacobian.rows(), rel_jacobian.cols()) = rel_jacobian.block(0, 0, rel_jacobian.rows(), rel_jacobian.cols());
	// ROS_INFO("here 18");
			q_dot= DQController::calculateControlVel(Kp, mu, combined_screw_error, combined_jacobian, (combined_jacobian.cols()));
			// q_dot= DQController::calculateControlVel(Kp, mu, screw_error_abs, abs_jacobian, (abs_jacobian.cols()));
			// q_dot= DQController::calculateControlVel(Kp, mu, screw_error_rel, rel_jacobian, (rel_jacobian.cols()));
			std::cout << "q_dot: "  << std::endl;	
			std::cout << q_dot << std::endl;	
			// q_dot.head(7)=-q_dot.head(7);
			// RowVectorXd joint_vel_lim_right = DQoperations::doubleVector2Rowvector(joint_velocity_limit_baxterRight);
			// RowVectorXd joint_vel_lim_left = DQoperations::doubleVector2Rowvector(joint_velocity_limit_baxterLeft);
			// RowVectorXd joint_velocity_limit = RowVectorXd::Zero(joint_vel_lim_right.cols()+joint_vel_lim_left.cols());
			// std::cout << "joint_vel_lim_right: "  << std::endl;		
			// std::cout << joint_vel_lim_right  << std::endl;		
			// std::cout << "joint_vel_lim_left: "  << std::endl;		
			// std::cout << joint_vel_lim_left  << std::endl;		
			// std::cout << "joint_velocity_limit: "  << std::endl;		
			// std::cout << joint_velocity_limit  << std::endl;		

			// joint_velocity_limit << joint_vel_lim_right, joint_vel_lim_left;			
			// std::cout << "joint_velocity_limit: "  << std::endl;		
			// std::cout << joint_velocity_limit  << std::endl;		
			// joint_velocity_limit_baxter = DQoperations::dqEigenToDQdouble(joint_velocity_limit);
			// // std::cout << "q_dot(before normalization): "  << std::endl;	
			// q_dot= DQController::normalizeControlVelocity( q_dot, joint_velocity_limit_baxter);
			// std::cout << "q_dot(after normalization): " << std::endl;		
			// std::cout << q_dot  << std::endl;		
			q.resize(q_dot.size());
			q << q_baxterRight, q_baxterLeft;
			q = q + q_dot*timeStep; // HERE YOU CAN USE (time_now-time_last) in place of timeStep
			std::cout << "q: " << std::endl;			
			std::cout << q << std::endl;			
			baxter_controller->sendBaxterJointCmds("combined", DQoperations::dqEigenToDQdouble(q), 0);
			normError = combined_screw_error.norm();
			// normError = screw_error_abs.norm();
			// normError = screw_error_rel.norm();
			// std::cout << "normError(abs, rel, combo): " << screw_error_abs.norm() << screw_error_rel.norm() << combined_screw_error.norm() << std::endl;
			// std::cout << normError << std::endl;
			// std::cout << "normError" << normError << std::endl;
	  }
	  baxterControl_res.result = 1;
	  baxterControl_res.normError = normError;

	  return true;
	}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "baxter_ar10_kinematics");
	ros::NodeHandle n;
	result_topics = n.advertise<dq_robotics::ControllerPerformance>("/dq_robotics/results", 1000);
	double time_start =ros::Time::now().toSec();
  ros::Duration(0.5).sleep();
  time_start =ros::Time::now().toSec();
	Matrix4d htm_rightHand2handBase = Matrix4d::Identity();
	htm_rightHand2handBase << -0.0002037,  1.0000000,  0.0000000, 0,
													  -1.0000000, -0.0002037,  0.0000000, 0, 
													   0.0000000,  0.0000000,  1.0000000, 0.01,
													   0,					 0,					 0, 				 1;
	tf_rightHand2handBase = DQoperations::htm2DQ(htm_rightHand2handBase);

	if (!getControllerParams())
		std::cout << "Controller params not found for COMBO controller." << std::endl;


	baxter_controller = new BaxterPoseControlServer();
	if (!baxter_controller->BaxterPoseControlServer::initializeController())
	{
		ROS_ERROR("The robot can not be initialized.");
		return 0;
	}
	if(!update_baxter())
	{
		std::cout << "The arms could not be updated" << std::endl;
		return 0;
	}
	ros::ServiceServer service = n.advertiseService("baxter/coopTaskSpaceControlCallback", coop_TaskSpaceControlCallback);
	ros::spin();

	return 0;
}