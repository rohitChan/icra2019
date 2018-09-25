// #include <dq_robotics/baxter_poseControl_server.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <dq_robotics/baxter_poseControl_server.h>
using namespace Eigen;


template<typename _Matrix_Type_>
	_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
	{
	    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
	}



BaxterPoseControlServer::BaxterPoseControlServer(){}

BaxterPoseControlServer::~BaxterPoseControlServer(){}

void BaxterPoseControlServer::resetTime()
{
	time_now=0;
	time_last=0;	
}


dq_robotics::BaxterControl::Response BaxterPoseControlServer::rightArmControl(dq_robotics::BaxterControl::Request baxterControl_req)
{
	BaxterPoseControlServer::update_right();
	Matrix<double,8,1> desiredAbsPose, desiredAbsVelocity;
	DQoperations::doubleToDQ(desiredAbsPose, baxterControl_req.desiredAbsPose);
	DQoperations::doubleToDQ(desiredAbsVelocity, baxterControl_req.desiredAbsVelocity);


	dq_robotics::BaxterControl::Response res;
	if (baxterControl_req.useVisualPose_single==true)
		DQoperations::doubleToDQ(desiredAbsPose, baxterControl_req.desiredAbsPose_vision);
	RowVectorXd q_dot= BaxterPoseControlServer::jointVelocity4velocityControl(Kp, mu, u_right, p_right, pose_now_right,  pe_init_right, joint_type_right, desiredAbsPose, desiredAbsVelocity, fkm_matrix_right, norm_error_right);
	
	// MatrixXd htm_right_current;
	// htm_right_current= DQoperations::dq2HTM(pose_now_right);
	// std::cout << "htm_right_current: " << htm_right_current << std::endl;

	// MatrixXd htm_right_desired;
	// htm_right_desired= DQoperations::dq2HTM(desiredAbsPose);
	// std::cout << "htm_right_desired: " << htm_right_desired << std::endl;
	std::vector<double> jointCmds;
	if (baxterControl_req.velocity_control==true)
	{
		q_dot=DQController::normalizeControlVelocity( q_dot, joint_velocity_limit_right);		
		DQoperations::dqEigenToDQdouble(q_dot, jointCmds);
	}
	else
	{
		RowVectorXd q(q_dot.size());
		q=q_right+q_dot*timeStep; // HERE YOU CAN USE (time_now-time_last) in place of timeStep
		std::vector<double> jointPositionCmds;
		DQoperations::dqEigenToDQdouble(q, jointCmds);
	}
	// pose_error_abs=DQoperations::mulDQ(pose_now_right, DQoperations::classicConjDQ(desiredAbsPose));
	remapJointCmds4RightArm(jointCmds);
	baxter->ManipulatorModDQ::sendJointCommandBaxter(jointCmds, 1, sleepTimeout, baxterControl_req.velocity_control);
	

	time_last=current_time.toSec();
	BaxterPoseControlServer::update_manipulator();
	BaxterPoseControlServer::update_right();
	time_now=current_time.toSec();

	res.timeNow= time_now;
	res.timeLast=time_last;
	res.timeStamp=current_time;
	res.currentAbsPose = DQoperations::DQToDouble(pose_now_right);
	res.normError=norm_error_right;
	return res;
}

void BaxterPoseControlServer::sendBaxterVelocityCmds(std::string hand, std::vector<double> jointCmds)
{
	bool velocity_control =1; 
	if (!hand.compare("right"))
	{
		remapJointCmds4RightArm(jointCmds);
		baxter->ManipulatorModDQ::sendJointCommandBaxter(jointCmds, 1, sleepTimeout, velocity_control);
	}
	else if(!hand.compare("combined"))
	{
		std::vector<double> jointCmds_new ;
		for (int i=0; i<14; i++)
		{
			jointCmds_new.push_back(jointCmds[i]);
		}
		baxter->ManipulatorModDQ::sendJointCommandBaxter(jointCmds_new, 3, sleepTimeout, velocity_control);
	}
}

void BaxterPoseControlServer::sendBaxterJointCmds(std::string hand, std::vector<double> jointCmds, bool velocity_control)
{
	// bool velocity_control =1; 
	if (!hand.compare("right"))
	{
		remapJointCmds4RightArm(jointCmds);
		baxter->ManipulatorModDQ::sendJointCommandBaxter(jointCmds, 1, sleepTimeout, velocity_control);
	}
	else if(!hand.compare("combined"))
	{
		std::vector<double> jointCmds_new ;
		for (int i=0; i<14; i++)
		{
			jointCmds_new.push_back(jointCmds[i]);
		}
		baxter->ManipulatorModDQ::sendJointCommandBaxter(jointCmds_new, 3, sleepTimeout, velocity_control);
	}
}

dq_robotics::BaxterControl::Response BaxterPoseControlServer::leftArmControl(dq_robotics::BaxterControl::Request baxterControl_req)
{
	BaxterPoseControlServer::update_left();	
	Matrix<double,8,1> desiredAbsPose, desiredAbsVelocity;
	DQoperations::doubleToDQ(desiredAbsPose, baxterControl_req.desiredAbsPose);
	DQoperations::doubleToDQ(desiredAbsVelocity, baxterControl_req.desiredAbsVelocity);
 
	dq_robotics::BaxterControl::Response res;

	if (baxterControl_req.useVisualPose_single==true)
		DQoperations::doubleToDQ(desiredAbsPose, baxterControl_req.desiredAbsPose_vision);
	RowVectorXd q_dot= BaxterPoseControlServer::jointVelocity4velocityControl(Kp, mu, u_left, p_left, pose_now_left,  pe_init_left, joint_type_left, desiredAbsPose, desiredAbsVelocity, fkm_matrix_left, norm_error_left);

	std::vector<double> jointCmds;
	if (baxterControl_req.velocity_control==true)
	{
		q_dot=DQController::normalizeControlVelocity( q_dot, joint_velocity_limit_left);		
		DQoperations::dqEigenToDQdouble(q_dot, jointCmds);
	}
	else
	{
		RowVectorXd q(q_dot.size());
		q=q_left+q_dot*timeStep; // HERE YOU CAN USE (time_now-time_last) in place of timeStep
		std::vector<double> jointPositionCmds;
		DQoperations::dqEigenToDQdouble(q, jointCmds);
	}
	
	remapJointCmds4LeftArm(jointCmds);
	
	baxter->ManipulatorModDQ::sendJointCommandBaxter(jointCmds, 2, sleepTimeout, baxterControl_req.velocity_control);
	

	time_last=current_time.toSec();
	BaxterPoseControlServer::update_manipulator();
	BaxterPoseControlServer::update_left();
	time_now=current_time.toSec();

	res.timeNow= time_now;
	res.timeLast=time_last;
	res.timeStamp=current_time;
	res.currentAbsPose = DQoperations::DQToDouble(pose_now_left);
	res.normError=norm_error_left;
	return res;
}

dq_robotics::BaxterControl::Response BaxterPoseControlServer::relativeArmControl(dq_robotics::BaxterControl::Request baxterControl_req)
{
	ROS_INFO ("Are we in relative!");
	BaxterPoseControlServer::update_relative();	
	Matrix<double,8,1> desiredRelPose, desiredRelVelocity;
	DQoperations::doubleToDQ(desiredRelPose, baxterControl_req.desiredRelPose);
	DQoperations::doubleToDQ(desiredRelVelocity, baxterControl_req.desiredRelVelocity);
 
	dq_robotics::BaxterControl::Response res;
	if (baxterControl_req.useVisualPose_relative==true)
		DQoperations::doubleToDQ(desiredRelPose, baxterControl_req.desiredRelPose_vision);
	RowVectorXd q_dot= BaxterPoseControlServer::jointVelocity4velocityControl(Kp, mu, u_relative, p_relative, pose_now_relative,  pe_init_relative, joint_type_relative, desiredRelPose, desiredRelVelocity, fkm_matrix_relative, norm_error_relative);

	std::vector<double> jointCmds;
	if (baxterControl_req.velocity_control==true)
	{
		q_dot=DQController::normalizeControlVelocity( q_dot, joint_velocity_limit_relative);		
		DQoperations::dqEigenToDQdouble(q_dot, jointCmds);
	}
	else
	{
		RowVectorXd q(q_dot.size());
		q=q_relative+q_dot*timeStep; // HERE YOU CAN USE (time_now-time_last) in place of timeStep
		std::vector<double> jointPositionCmds;
		DQoperations::dqEigenToDQdouble(q, jointCmds);
	}
	
	remapJointCmds4RelativeArm(jointCmds);
	
	baxter->ManipulatorModDQ::sendJointCommandBaxter(jointCmds, 3, sleepTimeout, baxterControl_req.velocity_control);
	

	time_last=current_time.toSec();
	BaxterPoseControlServer::update_manipulator();
	BaxterPoseControlServer::update_relative();
	time_now=current_time.toSec();

	res.timeNow= time_now;
	res.timeLast=time_last;
	res.timeStamp=current_time;
	res.currentRelPose = DQoperations::DQToDouble(pose_now_relative);
	res.normError=norm_error_relative;
	return res;	
}

dq_robotics::BaxterControl::Response BaxterPoseControlServer::rightOrientationRelativeArmControl(dq_robotics::BaxterControl::Request baxterControl_req)
{
	// ROS_INFO ("Are we in rightRelative!");
	BaxterPoseControlServer::update_rightRelative();	
	Matrix<double,8,1> desiredRelPose, desiredRelVelocity, desiredAbsPose, desiredAbsVelocity;
	DQoperations::doubleToDQ(desiredRelPose, baxterControl_req.desiredRelPose);
	DQoperations::doubleToDQ(desiredAbsPose, baxterControl_req.desiredAbsPose);
	desiredAbsPose=DQoperations::classicConjDQ(desiredAbsPose);
	DQoperations::doubleToDQ(desiredRelVelocity, baxterControl_req.desiredRelVelocity);
	DQoperations::doubleToDQ(desiredAbsVelocity, baxterControl_req.desiredAbsVelocity);
 
	dq_robotics::BaxterControl::Response res;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	pe_init_rightRelative_relative=pe_init_relative;
	pe_init_rightRelative_right=DQoperations::classicConjDQ(pe_init_right);

	if (baxterControl_req.useVisualPose_relative==true)
	DQoperations::doubleToDQ(desiredRelPose, baxterControl_req.desiredRelPose_vision);

	if (baxterControl_req.useVisualPose_single==true)
	{
		DQoperations::doubleToDQ(desiredAbsPose, baxterControl_req.desiredAbsPose_vision);
		desiredAbsPose= DQoperations::classicConjDQ(desiredAbsPose);		
	}

	pe_init_rightRelative_right=DQoperations::classicConjDQ(pe_init_right);
	MatrixXd jacobian_8d_rightRelative_relative, jacobian_8d_rightRelative_right;
	jacobian_8d_rightRelative_relative= DQController::jacobianDual(u_rightRelative_relative, p_rightRelative_relative, pe_init_rightRelative_relative, joint_type_rightRelative_relative, fkm_matrix_rightRelative_relative);	
	jacobian_8d_rightRelative_relative= DQController::jacobianDual_8d(joint_size_rightRelative_relative, jacobian_8d_rightRelative_relative);	

	// ROS_INFO("jacobian_8d_rightRelative_relative: rows: %d, cols: %d", jacobian_8d_rightRelative_relative.rows(), jacobian_8d_rightRelative_relative.cols());

	jacobian_8d_rightRelative_right= MatrixXd::Zero(8,joint_size_rightRelative_right);
	for (int i=0; i<7; i++)
	{
		jacobian_8d_rightRelative_right.col(i)=jacobian_8d_rightRelative_relative.col(i);
	}
	// std::cout << "jacobian_8d: " << std::endl;
	// std::cout << jacobian_8d << std::endl;
	RowVectorXd screw_error_rightRelative_relative, screw_error_rightRelative_right; 
	screw_error_rightRelative_relative= DQController::getScrewError_8d(pose_now_rightRelative_relative, desiredRelPose);
	screw_error_rightRelative_right= DQController::getScrewError_8d(pose_now_rightRelative_right, desiredAbsPose);

	norm_error=(screw_error_rightRelative_relative.norm()+screw_error_rightRelative_right.norm())/2;

	MatrixXd combined_jacobian_8d= MatrixXd::Zero(12,joint_size_rightRelative_relative);
	combined_jacobian_8d.block(0, 0, jacobian_8d_rightRelative_relative.rows(), jacobian_8d_rightRelative_relative.cols())=jacobian_8d_rightRelative_relative;
	combined_jacobian_8d.block(jacobian_8d_rightRelative_relative.rows(), 0, 4, jacobian_8d_rightRelative_right.cols())=jacobian_8d_rightRelative_right.block(4,0, 4, jacobian_8d_rightRelative_right.cols());

	RowVectorXd combined_screw_error =RowVectorXd::Zero(screw_error_rightRelative_relative.cols()+4);
	// ROS_INFO("1");
	// std::cout << "screw_error_rel.size: " << screw_error_rel.rows() << ":" << screw_error_rel.cols() << std::endl; 
	// std::cout << "screw_error_right.size: " << screw_error_right.rows() << ":" << screw_error_right.cols() << std::endl; 
	combined_screw_error << screw_error_rightRelative_relative.row(0), screw_error_rightRelative_right(4), screw_error_rightRelative_right(5), screw_error_rightRelative_right(6), screw_error_rightRelative_right(7);
	
	RowVectorXd q_dot= DQController::calculateControlVel(Kp, mu, combined_screw_error, combined_jacobian_8d, joint_size_rightRelative_relative);

	std::vector<double> jointCmds;
	if (baxterControl_req.velocity_control==true)
	{
		q_dot=DQController::normalizeControlVelocity( q_dot, joint_velocity_limit_rightRelative_relative);		
		DQoperations::dqEigenToDQdouble(q_dot, jointCmds);
	}
	else
	{
		RowVectorXd q(q_dot.size());
		q=q_rightRelative_relative+q_dot*timeStep; // HERE YOU CAN USE (time_now-time_last) in place of timeStep
		std::vector<double> jointPositionCmds;
		DQoperations::dqEigenToDQdouble(q, jointCmds);
	}
	
	remapJointCmds4RelativeArm(jointCmds);
	
	baxter->ManipulatorModDQ::sendJointCommandBaxter(jointCmds, 5, sleepTimeout, baxterControl_req.velocity_control);	

	time_last=current_time.toSec();
	BaxterPoseControlServer::update_manipulator();
	BaxterPoseControlServer::update_rightRelative();
	time_now=current_time.toSec();

	res.timeNow= time_now;
	res.timeLast=time_last;
	res.timeStamp=current_time;
	res.currentRelPose = DQoperations::DQToDouble(pose_now_rightRelative_relative);
	res.currentAbsPose = DQoperations::DQToDouble(DQoperations::classicConjDQ(pose_now_rightRelative_right));
	res.normError=norm_error;
	return res;	
}

dq_robotics::BaxterControl::Response BaxterPoseControlServer::rightRelativeArmControl(dq_robotics::BaxterControl::Request baxterControl_req)
{
	// ROS_INFO ("Are we in rightRelative!");
	BaxterPoseControlServer::update_rightRelative();	
	Matrix<double,8,1> desiredRelPose, desiredRelVelocity, desiredAbsPose, desiredAbsVelocity;
	DQoperations::doubleToDQ(desiredRelPose, baxterControl_req.desiredRelPose);
	DQoperations::doubleToDQ(desiredAbsPose, baxterControl_req.desiredAbsPose);
	desiredAbsPose=DQoperations::classicConjDQ(desiredAbsPose);
	DQoperations::doubleToDQ(desiredRelVelocity, baxterControl_req.desiredRelVelocity);
	DQoperations::doubleToDQ(desiredAbsVelocity, baxterControl_req.desiredAbsVelocity);
 
	dq_robotics::BaxterControl::Response res;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	pe_init_rightRelative_relative=pe_init_relative;
	pe_init_rightRelative_right=DQoperations::classicConjDQ(pe_init_right);

	if (baxterControl_req.useVisualPose_relative==true)
		DQoperations::doubleToDQ(desiredRelPose, baxterControl_req.desiredRelPose_vision);

	if (baxterControl_req.useVisualPose_single==true)
	{
		DQoperations::doubleToDQ(desiredAbsPose, baxterControl_req.desiredAbsPose_vision);
		desiredAbsPose= DQoperations::classicConjDQ(desiredAbsPose);		
	}

	pe_init_rightRelative_right=DQoperations::classicConjDQ(pe_init_right);
	MatrixXd jacobian_8d_rightRelative_relative, jacobian_8d_rightRelative_right;
	jacobian_8d_rightRelative_relative= DQController::jacobianDual(u_rightRelative_relative, p_rightRelative_relative, pe_init_rightRelative_relative, joint_type_rightRelative_relative, fkm_matrix_rightRelative_relative);	
	jacobian_8d_rightRelative_relative= DQController::jacobianDual_8d(joint_size_rightRelative_relative, jacobian_8d_rightRelative_relative);	

	// ROS_INFO("jacobian_8d_rightRelative_relative: rows: %d, cols: %d", jacobian_8d_rightRelative_relative.rows(), jacobian_8d_rightRelative_relative.cols());

	jacobian_8d_rightRelative_right= MatrixXd::Zero(8,joint_size_rightRelative_right);
	for (int i=0; i<7; i++)
	{
		jacobian_8d_rightRelative_right.col(i)=jacobian_8d_rightRelative_relative.col(i);
	}
	// std::cout << "jacobian_8d: " << std::endl;
	// std::cout << jacobian_8d << std::endl;
	RowVectorXd screw_error_rightRelative_relative, screw_error_rightRelative_right; 
	screw_error_rightRelative_relative= DQController::getScrewError_8d(pose_now_rightRelative_relative, desiredRelPose);
	screw_error_rightRelative_right= DQController::getScrewError_8d(pose_now_rightRelative_right, desiredAbsPose);

	norm_error=(screw_error_rightRelative_relative.norm()+screw_error_rightRelative_right.norm())/2;

	MatrixXd combined_jacobian_8d= MatrixXd::Zero(16,joint_size_rightRelative_relative);
	combined_jacobian_8d.block(0, 0, jacobian_8d_rightRelative_relative.rows(), jacobian_8d_rightRelative_relative.cols())=jacobian_8d_rightRelative_relative;
	combined_jacobian_8d.block(jacobian_8d_rightRelative_relative.rows(), 0, jacobian_8d_rightRelative_right.rows(), jacobian_8d_rightRelative_right.cols())=jacobian_8d_rightRelative_right;

	RowVectorXd combined_screw_error =RowVectorXd::Zero(screw_error_rightRelative_relative.cols()+screw_error_rightRelative_right.cols());
	// ROS_INFO("1");
	// std::cout << "screw_error_rel.size: " << screw_error_rel.rows() << ":" << screw_error_rel.cols() << std::endl; 
	// std::cout << "screw_error_right.size: " << screw_error_right.rows() << ":" << screw_error_right.cols() << std::endl; 
	combined_screw_error << screw_error_rightRelative_relative.row(0), screw_error_rightRelative_right.row(0);
	
	RowVectorXd q_dot= DQController::calculateControlVel(Kp, mu, combined_screw_error, combined_jacobian_8d, joint_size_rightRelative_relative);

	std::vector<double> jointCmds;
	if (baxterControl_req.velocity_control==true)
	{
		q_dot=DQController::normalizeControlVelocity( q_dot, joint_velocity_limit_rightRelative_relative);		
		DQoperations::dqEigenToDQdouble(q_dot, jointCmds);
	}
	else
	{
		RowVectorXd q(q_dot.size());
		q=q_rightRelative_relative+q_dot*timeStep; // HERE YOU CAN USE (time_now-time_last) in place of timeStep
		DQoperations::dqEigenToDQdouble(q, jointCmds);
	}
	
	remapJointCmds4RelativeArm(jointCmds);
	
	baxter->ManipulatorModDQ::sendJointCommandBaxter(jointCmds, 4, sleepTimeout, baxterControl_req.velocity_control);	
	
	time_last=current_time.toSec();
	BaxterPoseControlServer::update_manipulator();
	BaxterPoseControlServer::update_rightRelative();
	time_now=current_time.toSec();

	res.timeNow= time_now;
	res.timeLast=time_last;
	res.timeStamp=current_time;
	res.currentRelPose = DQoperations::DQToDouble(pose_now_rightRelative_relative);
	res.currentAbsPose = DQoperations::DQToDouble(DQoperations::classicConjDQ(pose_now_rightRelative_right));
	res.normError=norm_error;
	return res;	
}

void BaxterPoseControlServer::remapJointCmds4RightArm(std::vector<double>  &jointCmds)
{
	std::vector<double> jointCmds_new ;
	for (int i=0; i<7; i++)
	{
		jointCmds_new.push_back(jointCmds[i]);
	}
	for (int i=0; i<7; i++)
	{
		jointCmds_new.push_back(0.0);
	}
	jointCmds.clear();
	jointCmds.resize(14);
	jointCmds=jointCmds_new;
}

void BaxterPoseControlServer::remapJointCmds4LeftArm(std::vector<double>  &jointCmds)
{
	std::vector<double> jointCmds_new ;
	for (int i=0; i<7; i++)
	{
		jointCmds_new.push_back(0.0);
	}
	for (int i=0; i<7; i++)
	{
		jointCmds_new.push_back(jointCmds[i]);
	}
	jointCmds.clear();
	jointCmds.resize(14);
	jointCmds=jointCmds_new;
}

void BaxterPoseControlServer::remapJointCmds4RelativeArm(std::vector<double>  &jointCmds)
{
	std::vector<double> jointCmds_new ;
	jointCmds_new.clear();
	for (int i=0; i<14; i++)
	{
		jointCmds_new.push_back(0.0);
	}
	for (int i=0; i<7; i++)
	{
		jointCmds_new[i] =-jointCmds[6-i];	
		jointCmds_new[i+7]=jointCmds[i+7];
	}
	jointCmds.clear();
	jointCmds.resize(14);
	jointCmds=jointCmds_new;
}

RowVectorXd BaxterPoseControlServer::jointVelocity4velocityControl(double Kp, double mu, std::vector<RowVector3d> u, std::vector<RowVector3d> p, Matrix<double,8,1> pose_now, Matrix<double,8,1> pe_init, std::vector<int> joint_type, Matrix<double,8,1> pose_desired, Matrix<double,8,1> cart_vel_desired, std::vector<Matrix<double,8,1> >  fkm_matrix, double& norm_error)
{
	
	int joint_size=u.size();
	// std::cout << "fkm_matrix: " << std::endl;

	// for (int i=0; i<u.size(); i++)
	// {
	// 	std::cout << fkm_matrix[i] << std::endl;
	// 	std::cout << "u_left[i]: " << u[i] << std::endl;
	// 	std::cout << "p_left[i]: " << p[i] << std::endl;		
	// }

// std::cout << "pose_now: " << pose_now << std::endl;
// std::cout << "pose_desired: " << pose_desired << std::endl;
// std::cout << "pe_init: " << pe_init << std::endl;
	MatrixXd jacobian_8d;
	jacobian_8d= DQController::jacobianDual(u, p, pe_init, joint_type, fkm_matrix);	
	jacobian_8d= DQController::jacobianDual_8d(joint_size, jacobian_8d);
	// std::cout << "jacobian_8d: " << std::endl;
	// std::cout << jacobian_8d << std::endl;
	RowVectorXd screw_error= DQController::getScrewError_8d(pose_now, pose_desired);

	norm_error=screw_error.norm();
	// RowVectorXd q_dot= DQController::calculateControlVel_velcityFF(Kp, mu, screw_error, jacobian_8d, joint_size, cart_vel_desired);
	RowVectorXd q_dot= DQController::calculateControlVel(Kp, mu, screw_error, jacobian_8d, joint_size);
	// std::cout << "q_dot: " << q_dot << std::endl;
	return q_dot;

}


bool BaxterPoseControlServer::baxterControlServerCallback(dq_robotics::BaxterControl::Request& baxterControl_req, 
														  dq_robotics::BaxterControl::Response& baxterControl_res)
{	
	BaxterPoseControlServer::update_manipulator();
	bool request_manipulator_infoService = baxterControl_req.request_manipulator_infoService;
	if(request_manipulator_infoService)
	{
		int infoService_mode=baxterControl_req.infoService_mode;
		switch(infoService_mode)
		{
			case 1: 
				update_right();
				baxterControl_res.currentAbsPose = DQoperations::DQToDouble(pose_now_right);
				break;
			case 2: 
				update_left();
				baxterControl_res.currentAbsPose = DQoperations::DQToDouble(pose_now_left);			
				break;
			case 3: 
				update_relative();
				baxterControl_res.currentRelPose = DQoperations::DQToDouble(pose_now_relative);
				break;
			case 4: 
				update_rightRelative();
				baxterControl_res.currentAbsPose=DQoperations::DQToDouble(DQoperations::classicConjDQ(pose_now_right));
				baxterControl_res.currentRelPose = DQoperations::DQToDouble(pose_now_relative);
				break;
			case 5: 
				update_rightRelative();
				baxterControl_res.currentAbsPose=DQoperations::DQToDouble(DQoperations::classicConjDQ(pose_now_right));
				baxterControl_res.currentRelPose = DQoperations::DQToDouble(pose_now_relative);
				break;			
			case 6: 
				resetTime();
				break;							
		}
		baxterControl_res.timeNow= time_now;
		baxterControl_res.timeLast=time_last;
		baxterControl_res.timeStamp=current_time;
		return 1;	
	}

	int controllerMode = baxterControl_req.controlMode;
	switch(controllerMode)
	{
		case 1: baxterControl_res = rightArmControl(baxterControl_req);
			break;
		case 2: baxterControl_res = leftArmControl(baxterControl_req);
			break;
		case 3: baxterControl_res = relativeArmControl(baxterControl_req);
			break;
		case 4: baxterControl_res = rightRelativeArmControl(baxterControl_req);
			break;
		case 5: baxterControl_res = rightOrientationRelativeArmControl(baxterControl_req);
			break;			
	}
	return 1;
}

bool BaxterPoseControlServer::initializeController()
{
	if (BaxterPoseControlServer::getControllerParams())
	{
		baxter= new ManipulatorModDQ();
		baxter->ManipulatorModDQ::initialize_baxter();
		baxter->ManipulatorModDQ::robotParams(u_baxter, p_baxter, pe_init_left, pe_init_right, joint_high_limit_baxter, joint_low_limit_baxter, velocity_limit_baxter, max_safe_baxter, min_safe_baxter, joint_names_baxter, joint_type_baxter);
		BaxterPoseControlServer::intializeMode_right();
		BaxterPoseControlServer::intializeMode_left();
		BaxterPoseControlServer::intializeMode_relative();
		BaxterPoseControlServer::intializeMode_rightRelative();
		// controllerService = rh.advertiseService("baxterControlService", &BaxterPoseControlServer::baxterControlServerCallback, this);
		return 1;
	}
	else return 0;
}

bool BaxterPoseControlServer::getControllerParams()
{
	dt_total=0;
	use_VM=false;

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

	paramName="use_VM";
	int value;
	if(ros::param::get(paramName, value))
	{
		if(value)
			use_VM=true;
	}
	else
	{
		std::cout << "controller param use_VM not found" << std::endl;
		return 0;
	}

	paramName="dualArm";
	if(ros::param::get(paramName, value))
	{
		if(value)
			dualArm=true;
	}
	else
	{
		std::cout << "controller param dualArm not found" << std::endl;
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

void BaxterPoseControlServer::intializeMode_right()
{
	joint_size_right=7;	
	u_right.resize(7);
	p_right.resize(7);
	joint_velocity_limit_right.resize(7);
	joint_high_limit_right.resize(7);
	joint_low_limit_right.resize(7);
	joint_max_safe_limit_right.resize(7);
	joint_min_safe_limit_right.resize(7);
	joint_names_right.resize(7);
	joint_type_right.resize(7);
	for (int i=0; i<7; i++)
	{
		u_right[i] =u_baxter[i];	
		p_right[i] =p_baxter[i];
		joint_velocity_limit_right[i] =velocity_limit_baxter[i];
		joint_high_limit_right[i] =joint_high_limit_baxter[i];
		joint_low_limit_right[i] =joint_low_limit_baxter[i];
		joint_max_safe_limit_right[i] =max_safe_baxter[i];
		joint_min_safe_limit_right[i] =min_safe_baxter[i];
		joint_names_right[i] =joint_names_baxter[i];
		joint_type_right[i] =joint_type_baxter[i];
	}
	fkm_matrix_right.resize(joint_size_right);
}


void BaxterPoseControlServer::intializeMode_left()
{
	joint_size_left=7;
	u_left.resize(7);
	p_left.resize(7);
	joint_velocity_limit_left.resize(7);
	joint_high_limit_left.resize(7);
	joint_low_limit_left.resize(7);
	joint_max_safe_limit_left.resize(7);
	joint_min_safe_limit_left.resize(7);
	joint_names_left.resize(7);
	joint_type_left.resize(7);
	for (int i=0; i<7; i++)
	{
		u_left[i] =u_baxter[i+7];	
		p_left[i] =p_baxter[i+7];
		joint_velocity_limit_left[i] =velocity_limit_baxter[i+7];
		joint_high_limit_left[i] =joint_high_limit_baxter[i+7];
		joint_low_limit_left[i] =joint_low_limit_baxter[i+7];
		joint_max_safe_limit_left[i] =max_safe_baxter[i+7];
		joint_min_safe_limit_left[i] =min_safe_baxter[i+7];
		joint_names_left[i] =joint_names_baxter[i+7];
		joint_type_left[i] =joint_type_baxter[i+7];
	}
	fkm_matrix_left.resize(joint_size_left);
}


void BaxterPoseControlServer::intializeMode_relative()
{
	Matrix<double,8,1> transform_base2Right= DQoperations::classicConjDQ(pe_init_right);
	pe_init_relative= DQoperations::mulDQ(transform_base2Right, pe_init_left);
	// std::cout << "pe_init_relative:" << pe_init_relative.transpose() << std::endl;
	// MatrixXd htm_relative;
	// htm_relative= DQoperations::dq2HTM(pe_init_relative);
	// std::cout << "htm_relative: " << htm_relative << std::endl;
	
	joint_size_relative=14;
	u_relative.resize(14);
	p_relative.resize(14);
	joint_velocity_limit_relative.resize(14);
	joint_high_limit_relative.resize(14);
	joint_low_limit_relative.resize(14);
	joint_max_safe_limit_relative.resize(14);
	joint_min_safe_limit_relative.resize(14);
	joint_names_relative.resize(14);
	joint_type_relative.resize(14);
	for (int i=0; i<7; i++)
	{
		u_relative[i] =u_baxter[6-i];	
		u_relative[i+7] =u_baxter[i+7];	

		p_relative[i] =p_baxter[6-i];
		p_relative[i+7] =p_baxter[i+7];	
		
		joint_velocity_limit_relative[i] =velocity_limit_baxter[6-i];
		joint_velocity_limit_relative[i+7] =velocity_limit_baxter[i+7];
		
		joint_high_limit_relative[i] =-joint_low_limit_baxter[6-i];
		joint_high_limit_relative[i+7] =joint_high_limit_baxter[i+7];
		
		joint_low_limit_relative[i] =-joint_high_limit_baxter[6-i];
		joint_low_limit_relative[i+7] =joint_low_limit_baxter[i+7];
		
		joint_max_safe_limit_relative[i] =-min_safe_baxter[6-i];
		joint_max_safe_limit_relative[i+7] =max_safe_baxter[i+7];
		
		joint_min_safe_limit_relative[i] =-max_safe_baxter[6-i];
		joint_min_safe_limit_relative[i+7] =min_safe_baxter[i+7];
		
		joint_names_relative[i] =joint_names_baxter[6-i];
		joint_names_relative[i+7] =joint_names_baxter[i+7];
		
		joint_type_relative[i] =joint_type_baxter[6-i];
		joint_type_relative[i+7] =joint_type_baxter[i+7];

	}


	for (int i=0; i<14; i++)
	{
		u_relative[i]= DQoperations::transformLineVector(u_relative[i], transform_base2Right);
		// std::cout << "u_relative[" << i << "]:" << u_relative[i] << std::endl;		
		p_relative[i]= DQoperations::transformPoint(p_relative[i], transform_base2Right);
		// std::cout << "p_relative[" << i << "]:" << p_relative[i] << std::endl;
	}	

	fkm_matrix_relative.resize(joint_size_relative);
}

void BaxterPoseControlServer::intializeMode_rightRelative()
{
	joint_size_rightRelative_relative=14;
	joint_size_rightRelative_right=7;
	pe_init_rightRelative_relative=pe_init_relative;
	pe_init_rightRelative_right=DQoperations::classicConjDQ(pe_init_right);


	u_rightRelative_relative.resize(14);
	p_rightRelative_relative.resize(14);	
	joint_velocity_limit_rightRelative_relative.resize(14);
	joint_high_limit_rightRelative_relative.resize(14);
	joint_low_limit_rightRelative_relative.resize(14);
	joint_max_safe_limit_rightRelative_relative.resize(14);
	joint_min_safe_limit_rightRelative_relative.resize(14);
	joint_names_rightRelative_relative.resize(14);
	joint_type_rightRelative_relative.resize(14);

	u_rightRelative_right.resize(7);
	p_rightRelative_right.resize(7);	
	joint_velocity_limit_rightRelative_right.resize(7);
	joint_high_limit_rightRelative_right.resize(7);
	joint_low_limit_rightRelative_right.resize(7);
	joint_max_safe_limit_rightRelative_right.resize(7);
	joint_min_safe_limit_rightRelative_right.resize(7);
	joint_names_rightRelative_right.resize(7);
	joint_type_rightRelative_right.resize(7);

	u_rightRelative_relative = u_relative;
	p_rightRelative_relative = p_relative;	
	joint_velocity_limit_rightRelative_relative = joint_velocity_limit_relative;
	joint_high_limit_rightRelative_relative = joint_high_limit_relative;
	joint_low_limit_rightRelative_relative = joint_low_limit_relative;
	joint_max_safe_limit_rightRelative_relative = joint_max_safe_limit_relative;
	joint_min_safe_limit_rightRelative_relative = joint_min_safe_limit_relative;
	joint_names_rightRelative_relative = joint_names_relative;
	joint_type_rightRelative_relative = joint_type_relative;

	for (int i=0; i<7; i++)
	{
		u_rightRelative_right[i] = u_relative[i];
		p_rightRelative_right[i] = p_relative[i];	
		joint_velocity_limit_rightRelative_right[i] = joint_velocity_limit_relative[i];
		joint_high_limit_rightRelative_right[i] = joint_high_limit_relative[i];
		joint_low_limit_rightRelative_right[i] = joint_low_limit_relative[i];
		joint_max_safe_limit_rightRelative_right[i] = joint_max_safe_limit_relative[i];
		joint_min_safe_limit_rightRelative_right[i] = joint_min_safe_limit_relative[i];
		joint_names_rightRelative_right[i] = joint_names_relative[i];
		joint_type_rightRelative_right[i] = joint_type_relative[i];
	}
	fkm_matrix_rightRelative_relative.resize(joint_size_rightRelative_relative);
	fkm_matrix_rightRelative_right.resize(joint_size_rightRelative_right);
}

void BaxterPoseControlServer::update_manipulator()
{
	baxter->ManipulatorModDQ::getCurrentJointState_baxter();
	baxter->ManipulatorModDQ::currentRobotState(current_time, q_baxter, q_vel_baxter);
}

// std::vector<Matrix<double,8,1> > DQController::fkmDual(std::vector<RowVector3d> u,std::vector<RowVector3d> p, RowVectorXd q, std::vector<int> joint_type)
// {
// 	int joint_size=joint_type.size();
// // ROS_INFO("joint_size=%d", joint_size);
// 	RowVectorXd q_dual;	
// 	q_dual=RowVectorXd::Zero(joint_size*2);
// // ROS_INFO("11");
// 	for (int j=0; j< joint_size; j++)
// 	{
// 		if(joint_type[j]==0)
// 			q_dual(2*j)=q(j);
// 		else
// 			q_dual(2*j+1)=q(j);
// 		// ROS_INFO("j=%d", j);
// 	}
// // ROS_INFO("00");
// 	std::vector<Matrix<double,8,1> > fkm_current;
// 	fkm_current.clear();
// 	fkm_current.resize(joint_size);
// // ROS_INFO("22");
// 	for (int i=0;i<joint_size; i++)
// 	{
// 		double theta_i=q_dual[2*i];
// 		RowVector3d u_i=u[i];
// 		RowVector3d p_i=p[i];
// 		double d_i=q_dual[2*i+1];
// 		Matrix<double,8,1> screwDispalcementArray_i;
// 		screwDispalcementArray_i= DQoperations::screw2DQ(theta_i, u_i, d_i, p_i.cross(u_i));
// 		// std::cout << "screwDispalcementArray_" << i << ": " << screwDispalcementArray_i.transpose() << std::endl;
// 		if (i==0)
// 			fkm_current[i]=screwDispalcementArray_i;
// 		else
// 			fkm_current[i]=DQoperations::mulDQ(fkm_current[i-1],screwDispalcementArray_i);
// 		// ROS_INFO("i=%d",i);
// 		// std::cout << "fkm_current_source_" << i << ": " << fkm_current[i].transpose() << std::endl;
// 	}

// 	return fkm_current;
// }



void BaxterPoseControlServer::update_rightAcc()
{
	// update_manipulator();
	q_right=q_baxter.head(7);
	q_vel_right=q_vel_baxter.head(7);
	fkm_matrix_right=DQController::fkmDual(u_right, p_right, q_right, joint_type_right);
	pose_now_right=DQoperations::mulDQ(fkm_matrix_right[joint_size_right-1], pe_init_right);
	jacobian_8d_right= DQController::jacobianDual(u_right, p_right, pe_init_right, joint_type_right, fkm_matrix_right);	
	jacobian_8d_right= DQController::jacobianDual_8d(joint_size_right, jacobian_8d_right);	

	jacobian_6d_right = MatrixXd::Zero(6, jacobian_8d_right.cols());
	jacobian_6d_right.block(0, 0, 3, jacobian_8d_right.cols()) = jacobian_8d_right.block(5, 0,  3, jacobian_8d_right.cols());
	jacobian_6d_right.block(3, 0, 3, jacobian_8d_right.cols()) = jacobian_8d_right.block(1, 0,  3, jacobian_8d_right.cols());	
	link_velocity_right = DQController::linkVelocites(jacobian_6d_right, q_vel_right);
	jacobian_6d_dot_right = DQController::getJacobianDot(link_velocity_right, jacobian_6d_right);
}

void BaxterPoseControlServer::update_right()
{
	q_right=q_baxter.head(7);

	q_vel_right=q_vel_baxter.head(7);
		// ROS_INFO("3");	
	// ros::WallTime fkm_time_start = ros::WallTime::now();
	fkm_matrix_right=DQController::fkmDual(u_right, p_right, q_right, joint_type_right);
		// ROS_INFO("4");

	pose_now_right=DQoperations::mulDQ(fkm_matrix_right[joint_size_right-1], pe_init_right);
	// ros::WallTime fkm_time_end = ros::WallTime::now();
	// ros::WallDuration fkm_time_dur = fkm_time_end  - fkm_time_start;
	// std::cout << "fkm duration_server: " << fkm_time_dur.toSec() << std::endl;	
			// ROS_INFO("5");

	// MatrixXd htm_right;
	// htm_right= DQoperations::dq2HTM(pose_now_right);
	// std::cout << "htm_right: " << htm_right << std::endl;
} 

void BaxterPoseControlServer::update_leftAcc()
{
	update_manipulator();
	q_left=q_baxter.tail(7);
	q_vel_left=q_vel_baxter.tail(7);
	fkm_matrix_left=DQController::fkmDual(u_left, p_left, q_left, joint_type_left);
	pose_now_left=DQoperations::mulDQ(fkm_matrix_left[joint_size_left-1], pe_init_left);
	jacobian_8d_left= DQController::jacobianDual(u_left, p_left, pe_init_left, joint_type_left, fkm_matrix_left);	
	jacobian_8d_left= DQController::jacobianDual_8d(joint_size_left, jacobian_8d_left);	

	jacobian_6d_left = MatrixXd::Zero(6, jacobian_8d_left.cols());
	jacobian_6d_left.block(0, 0, 3, jacobian_8d_left.cols()) = jacobian_8d_left.block(5, 0,  3, jacobian_8d_left.cols());
	jacobian_6d_left.block(3, 0, 3, jacobian_8d_left.cols()) = jacobian_8d_left.block(1, 0,  3, jacobian_8d_left.cols());	
	link_velocity_left = DQController::linkVelocites(jacobian_6d_left, q_vel_left);
	jacobian_6d_dot_left = DQController::getJacobianDot(link_velocity_left, jacobian_6d_left);
}


void BaxterPoseControlServer::update_left()
{
	q_left=q_baxter.tail(7);
	q_vel_left=q_vel_baxter.tail(7);
	// std::cout << "q_left: " << q_left << std::endl;
	// std::cout << "q_vel_left: " << q_vel_left << std::endl;	
	// std::vector<Matrix<double,8,1> > fkm_matrix_left;

	fkm_matrix_left=DQController::fkmDual(u_left, p_left, q_left, joint_type_left);
	pose_now_left=DQoperations::mulDQ(fkm_matrix_left[joint_size_left-1], pe_init_left);
	// MatrixXd htm_left;
	// htm_left= DQoperations::dq2HTM(pose_now_left);
	// std::cout << "htm_left: " << htm_left << std::endl;

}

void BaxterPoseControlServer::update_relative()
{
	RowVectorXd q_temp;

	q_relative.resize(14);	
	q_temp = q_baxter.head(7);
	q_relative << -q_temp.reverse() , q_baxter.tail(7);
	// std::cout << "q_relative: " << q_relative << std::endl;

	q_vel_relative.resize(14);
	q_temp = q_vel_baxter.head(7);
	q_vel_relative << -q_temp.reverse() , q_vel_baxter.tail(7);
	// std::cout << "q_left: " << q_left << std::endl;
	// std::cout << "q_vel_left: " << q_vel_left << std::endl;	
	// std::vector<Matrix<double,8,1> > fkm_matrix_left;

	fkm_matrix_relative=DQController::fkmDual(u_relative, p_relative, q_relative, joint_type_relative);
	pose_now_relative=DQoperations::mulDQ(fkm_matrix_relative[joint_size_relative-1], pe_init_relative);
	// MatrixXd htm_left;
	// htm_left= DQoperations::dq2HTM(pose_now_left);
	// std::cout << "htm_left: " << htm_left << std::endl;

}

void BaxterPoseControlServer::update_rightRelative()
{
	RowVectorXd q_temp;

	q_rightRelative_relative.resize(14);	
	q_rightRelative_right.resize(7);

	q_temp = q_baxter.head(7);
	q_rightRelative_relative << -q_temp.reverse() , q_baxter.tail(7);
	q_rightRelative_right << -q_temp.reverse();
	// std::cout << "q_relative: " << q_relative << std::endl;

	q_vel_rightRelative_relative.resize(14);
	q_vel_rightRelative_right.resize(7);

	q_temp = q_vel_baxter.head(7);
	q_vel_rightRelative_relative << -q_temp.reverse() , q_vel_baxter.tail(7);
	q_vel_rightRelative_right << -q_temp.reverse();
	// std::cout << "q_left: " << q_left << std::endl;
	// std::cout << "q_vel_left: " << q_vel_left << std::endl;	
	// std::vector<Matrix<double,8,1> > fkm_matrix_left;


	fkm_matrix_rightRelative_right=DQController::fkmDual(u_rightRelative_right, p_rightRelative_right, q_rightRelative_right, joint_type_rightRelative_right);
	pose_now_rightRelative_right=DQoperations::mulDQ(fkm_matrix_rightRelative_right[joint_size_rightRelative_right-1], pe_init_rightRelative_right);

	fkm_matrix_rightRelative_relative=DQController::fkmDual(u_rightRelative_relative, p_rightRelative_relative, q_rightRelative_relative, joint_type_rightRelative_relative);
	pose_now_rightRelative_relative=DQoperations::mulDQ(fkm_matrix_rightRelative_relative[joint_size_rightRelative_relative-1], pe_init_rightRelative_relative);

	// MatrixXd htm_left;
	// htm_left= DQoperations::dq2HTM(pose_now_left);
	// std::cout << "htm_left: " << htm_left << std::endl;

}

bool BaxterPoseControlServer::updateManipulatorVariables(std::string arm)
{
	// std::cout << "Here 5" << std::endl;
	// ros::WallTime robotStat_time_start = ros::WallTime::now();
	ros::WallTime jacobReal_time_start = ros::WallTime::now();
	update_manipulator();
	// ros::WallTime robotStat_time_end = ros::WallTime::now();
	// ros::WallDuration robotStat_time_dur = robotStat_time_end  - robotStat_time_start;
	// std::cout << "robotStat duration_server: " << robotStat_time_dur.toSec() << std::endl; 	
	if(!arm.compare("all"))
	{
		// ros::WallTime jacobReal_time_start = ros::WallTime::now();
		update_right();		
		jacobian_8d_right= DQController::jacobianDual(u_right, p_right, pe_init_right, joint_type_right, fkm_matrix_right);	
		jacobian_8d_right= DQController::jacobianDual_8d(joint_size_right, jacobian_8d_right);	
		ros::WallTime jacobReal_time_end = ros::WallTime::now();
		ros::WallDuration jacobReal_time_dur = jacobReal_time_end  - jacobReal_time_start;
		// jacobReal_time_start = ros::WallTime::now();		
		std::cout << "jacobReal_time_dur duration_server_RIGHT: " << jacobReal_time_dur.toSec() << std::endl; 		
		
		// jacobReal_time_start = ros::WallTime::now();			
		update_left();		
		jacobian_8d_left= DQController::jacobianDual(u_left, p_left, pe_init_left, joint_type_left, fkm_matrix_left);	
		jacobian_8d_left= DQController::jacobianDual_8d(joint_size_left, jacobian_8d_left);		
		jacobReal_time_end = ros::WallTime::now();
		jacobReal_time_dur = jacobReal_time_end  - jacobReal_time_start;
		std::cout << "jacobReal_time_dur duration_server_TOTAL: " << jacobReal_time_dur.toSec() << std::endl; 					
		
		return 1;
	}
	// std::cout << "Here 6" << std::endl;
	if(!arm.compare("right"))
	{
		// std::cout << "Here 7" << std::endl;
		// ros::WallTime manipUpdate_time_start = ros::WallTime::now();
		update_right();		
		// ros::WallTime manipUpdate_time_end = ros::WallTime::now();
		// ros::WallDuration manipUpdate_time_dur = manipUpdate_time_end  - manipUpdate_time_start;
		// std::cout << "manipUpdate duration_server: " << manipUpdate_time_dur.toSec() << std::endl;		
		// std::cout << "Here 8" << std::endl;
		ros::WallTime jacobSingle_time_start = ros::WallTime::now();		
		jacobian_8d_right= DQController::jacobianDual(u_right, p_right, pe_init_right, joint_type_right, fkm_matrix_right);	
		jacobian_8d_right= DQController::jacobianDual_8d(joint_size_right, jacobian_8d_right);
		ros::WallTime jacobSingle_time_end = ros::WallTime::now();
		ros::WallDuration jacobSingle_time_dur = jacobSingle_time_end  - jacobSingle_time_start;
		std::cout << "jacobSingle duration_server: " << jacobSingle_time_dur.toSec() << std::endl; 		
		return 1;
	}
	if(!arm.compare("left"))
	{
		// std::cout << "Here 9" << std::endl;
		update_left();		
		jacobian_8d_left= DQController::jacobianDual(u_left, p_left, pe_init_left, joint_type_left, fkm_matrix_left);	
		jacobian_8d_left= DQController::jacobianDual_8d(joint_size_left, jacobian_8d_left);
		return 1;
	}
	else return 0;
}

void BaxterPoseControlServer::initializeBaxterIKService()
{
	controllerService = rh.advertiseService("baxterControlService", &BaxterPoseControlServer::baxterControlServerCallback, this);	
}

bool BaxterPoseControlServer::importJointLimits(std::string arm, RowVectorXd &joint_high_limit, RowVectorXd &joint_low_limit, RowVectorXd &joint_max_safe_limit, RowVectorXd &joint_min_safe_limit)
{	
	if(!arm.compare("right"))
	{
		for (int i=0; i<7; i++)
		{
			joint_min_safe_limit(i) = this->joint_min_safe_limit_right[i];
			joint_max_safe_limit(i) = this->joint_max_safe_limit_right[i];
			joint_low_limit(i) = this->joint_low_limit_right[i];
			joint_high_limit(i)= this->joint_high_limit_right[i];			
		}
		return true;		
	}
	return false;
} 

bool BaxterPoseControlServer::importManipulatorState_accControl(std::string arm, Matrix<double,8,1>& pe_init, Matrix<double,8,1>& pe_now, RowVectorXd& q, RowVectorXd& q_vel, MatrixXd& jacobian_6d, MatrixXd& jacobian_6d_dot)
{
	// update_manipulator();
	// updateManipulatorVariables(arm);	
	if(!arm.compare("right"))
	{
		update_rightAcc();
		pe_init= this->pe_init_right;
		pe_now= this->pose_now_right;
		jacobian_6d = this->jacobian_6d_right;
		q = this-> q_right;
		q_vel = this-> q_vel_right;
		jacobian_6d_dot = this->jacobian_6d_dot_right;
		return 1;
	}
	else if(!arm.compare("left"))
	{
		update_leftAcc();
		pe_init= this->pe_init_left;
		pe_now= this->pose_now_left;
		jacobian_6d = this->jacobian_6d_left;
		q = this-> q_left;
		q_vel = this-> q_vel_left;
		jacobian_6d_dot = this->jacobian_6d_dot_left;
		return 1;
	}
	else return 0;
}



bool BaxterPoseControlServer::importManipulatorState(std::string arm, Matrix<double,8,1>& pe_init, Matrix<double,8,1>& pe_now, MatrixXd& jacobian, RowVectorXd& q, std::vector<double> joint_velocity_limit)
{
	// updateManipulatorVariables(arm);	
	if(!arm.compare("right"))
	{
		pe_init= this->pe_init_right;
		pe_now= this->pose_now_right;
		jacobian = this->jacobian_8d_right;
		q = this-> q_right;
		joint_velocity_limit = this->joint_velocity_limit_right;
		return 1;
	}
	else if(!arm.compare("left"))
	{
		pe_init= this->pe_init_left;
		pe_now= this->pose_now_left;
		jacobian = this->jacobian_8d_left;
		q = this-> q_left;
		joint_velocity_limit = this->joint_velocity_limit_left;
		return 1;
	}
	else return 0;
}

void BaxterPoseControlServer::Run()
{	
	ROS_INFO("BaxterPoseControlServer...spinning");
    ros::spin();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "baxter_controller");
	ros::NodeHandle n;
	BaxterPoseControlServer* baxter_controller= new BaxterPoseControlServer();
	if (!baxter_controller->BaxterPoseControlServer::initializeController())
	{
		ROS_ERROR("The robot can not be initialized.");
		return 0;
	}
	baxter_controller->initializeBaxterIKService();
	baxter_controller->BaxterPoseControlServer::Run();
	return 1;
	// ros::ServiceServer controllerService = n.advertiseService("baxterControlService", &BaxterPoseControlServer::baxterControlServerCallback, baxter_controller);
	// baxter_controller->BaxterPoseControlServer::update_manipulator();
}

	// using_rel=1;
	// using_right=1;
	// manipulator_right->ManipulatorDQ::updateManipulator_right();
	// // dt=manipulator_right->ManipulatorDQ::getSamplingTime();
	// pose_now_right= manipulator_right->ManipulatorDQ::poseNow();	
	// jacobian_8d_right=manipulator_right->ManipulatorDQ::jacobian_dq_8d();	
	// screw_error_right = DQPositionController::getScrewError(pose_now_right, pose_desired_right);

	// manipulator_rel->ManipulatorDQ::updateManipulator_dualArm();
	// q_now=manipulator_rel->ManipulatorDQ::currentJointPosition();	
	// q_dot_now=manipulator_rel->ManipulatorDQ::currentJointVelocity();		
	// dt=manipulator_rel->ManipulatorDQ::getSamplingTime();
	// pose_now_rel= manipulator_rel->ManipulatorDQ::poseNow();	
	// jacobian_8d_rel=manipulator_rel->ManipulatorDQ::jacobian_dq_8d();	
	// screw_error_rel = DQPositionController::getScrewError(pose_now_rel, pose_desired_rel);

	// RowVectorXd combined_screw_error =RowVectorXd::Zero(screw_error_rel.cols()+screw_error_right.cols());
	// // ROS_INFO("1");
	// // std::cout << "screw_error_rel.size: " << screw_error_rel.rows() << ":" << screw_error_rel.cols() << std::endl; 
	// // std::cout << "screw_error_right.size: " << screw_error_right.rows() << ":" << screw_error_right.cols() << std::endl; 
	// combined_screw_error << screw_error_rel.row(0), screw_error_right.row(0);
	// norm_error=combined_screw_error.norm();
	// // ROS_INFO("2");
	// MatrixXd combined_jacobian_8d =MatrixXd::Zero((jacobian_8d_rel.rows()+jacobian_8d_right.rows()), (jacobian_8d_rel.cols()));

	// // ROS_INFO("3");
	// combined_jacobian_8d.block(0, 0, jacobian_8d_rel.rows(), jacobian_8d_rel.cols())=jacobian_8d_rel;
	// combined_jacobian_8d.block(jacobian_8d_rel.rows(), 0, jacobian_8d_right.rows(), jacobian_8d_right.cols())=jacobian_8d_right;
	// jacobian_8d_control=combined_jacobian_8d;
	// joint_size_control=joint_size_rel;
	// screw_error_control=combined_screw_error;
	// getControllerParam
	// initialize_controller();
	// std::vector<RowVector3d> u, p;
	// Matrix<double,8,1> pe_init_left, pe_init_right, pe_init_leftRef, pe_init_torsoRef;
	// std::vector<double> joint_high_limit, joint_low_limit, velocity_limit, max_safe, min_safe, joint_names;
	// std::vector<std::string> joint_names;
	// std::vector<int> joint_type;
	// ros::Time current_time, last_time;
	// RowVectorXd q, q_vel;
	// std::vector<Matrix<double,8,1> > fkm_matrix;
	// baxter->ManipulatorModDQ::robotParams(u, p, pe_init_left, pe_init_right, joint_high_limit, joint_low_limit, velocity_limit, max_safe, min_safe, joint_names, joint_type);

	// intialize_right(u, p, joint_high_limit, joint_low_limit, velocity_limit, max_safe, min_safe, joint_names, joint_type);
	// intialize_left(u, p, joint_high_limit, joint_low_limit, velocity_limit, max_safe, min_safe, joint_names, joint_type);
	// intialize_rel(u, p, pe_init_left, pe_init_right, joint_high_limit, joint_low_limit, velocity_limit, max_safe, min_safe, joint_names, joint_type);
	// intialize_rightRelative(u, p, pe_init_left, pe_init_right, joint_high_limit, joint_low_limit, velocity_limit, max_safe, min_safe, joint_names, joint_type);

	// // dq_robotics::BaxterControl::Request baxterControl_req; 
	// // dq_robotics::BaxterControl::Response baxterControl_res; 

	// ros::ServiceServer service = n.advertiseService("baxterControlService", baxterControlService_process);
	// ROS_INFO("Ready to take cartesian poses for BaxterControl.");
