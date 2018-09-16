#ifndef DQController_H
#define DQController_H

#include <dq_robotics/DQoperations.h>
#include <dq_robotics/baxter_dq.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <math.h>

class DQController
{

public:
	static std::vector<Matrix<double,8,1> > fkmDual(std::vector<RowVector3d> u,std::vector<RowVector3d> p, RowVectorXd q, std::vector<int> joint_type);
	static MatrixXd jacobianDual(std::vector<RowVector3d> u, std::vector<RowVector3d> p, Matrix<double,8,1> pe_init, std::vector<int> joint_type, std::vector<Matrix<double,8,1> > fkm_current);
	static MatrixXd jacobianDual_8d(int joint_size, MatrixXd jacobian);
	// void getDQderivative();
	static RowVectorXd getScrewError_8d(Matrix<double,8,1> pose_now, Matrix<double,8,1> pose_desired);
	static RowVectorXd calculateControlVel(double Kp, double mu, RowVectorXd screwError_8d, MatrixXd jacobian_8d, int joint_size);
	static RowVectorXd calculateControlVel_velcityFF(double Kp, double mu, RowVectorXd screwError_8d, MatrixXd jacobian_8d, int joint_size, Matrix<double,8,1> cart_velocity_desired);
	static RowVectorXd normalizeControlVelocity( RowVectorXd q_dot, std::vector<double> joint_velocity_limit);
	static RowVectorXd jointVelocity4velocityControl(double Kp, double mu, std::vector<RowVector3d> u, std::vector<RowVector3d> p, Matrix<double,8,1> pose_now, Matrix<double,8,1> pe_init, std::vector<int> joint_type, Matrix<double,8,1> pose_desired, Matrix<double,8,1> cart_vel_desired, std::vector<Matrix<double,8,1> >  fkm_matrix, double& norm_error);
	static MatrixXd linkVelocites(MatrixXd jacobian_6d, RowVectorXd joint_velocities);
	static MatrixXd getJacobianDot(MatrixXd link_velocity, MatrixXd jacobain_6d);
	DQController();
	~DQController();
};
#endif