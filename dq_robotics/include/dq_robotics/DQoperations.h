#ifndef DQ_OPERATIONS_H
#define DQ_OPERATIONS_H

 #define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <stdexcept> //for range_error
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <complex>
#include <cmath>
#include <vector>
#include <cstddef>
#include <Eigen/SVD>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <time.h>  
using namespace Eigen;


class DQoperations
{
private:
	// int joint_size;


public:
	// const double PI  =3.141592653589793238463;
	static Matrix3d crossProductOp_3d(Vector3d vector);
	static MatrixXd crossProductOp_6d(VectorXd vector);
	static RowVector4d multQuat(RowVector4d p, RowVector4d q);
	static Matrix<double,8,1> mulDQ(Matrix<double,8,1> p, Matrix<double,8,1> q);
	static RowVector4d conjQuat(RowVector4d p);
	static Matrix<double,8,1> classicConjDQ(Matrix<double,8,1> dq);
	static Matrix<double,8,1> dualConjDQ(Matrix<double,8,1> dq);
	static Matrix<double,8,1> combinedConjDQ(Matrix<double,8,1> dq);
	static Matrix<double,8,1> screw2DQ(double theta, RowVector3d axis, double d, RowVector3d moment);
	static void dq2screw(Matrix<double,8,1> dq, double &theta_e, double &d_e, RowVector3d &l_e, RowVector3d &m_e); /*The screwResult DQ contains the screw parameter as [theta d l m] */
	static Matrix4d dq2HTM(Matrix<double,8,1> dq);
	static void dq2rotAndTransQuat(Matrix<double,8,1> dq, RowVector4d &rot, RowVector4d &trans);
	static Matrix<double,8,1>  htm2DQ(Matrix4d htm);
	static std::vector<Matrix<double,8,1> > fkm_dual(RowVectorXd q, std::vector<RowVector3d> u, std::vector<RowVector3d> p);
	static std::vector<Matrix<double,8,1> > fkm_revolute_only(RowVectorXd q, std::vector<RowVector3d> u, std::vector<RowVector3d> p);
	static MatrixXd jacobian_revolute_only(RowVectorXd q /*q is defined as [q1 q2 ...]*/, std::vector<RowVector3d> u, std::vector<RowVector3d> p , Matrix<double,8,1> pose_ee_init);
	// static double get_error_screw_param(RowVectorXd q /*q is defined as [q1 q2 ...]*/, std::vector<RowVector3d> u, std::vector<RowVector3d> p , Matrix<double,8,1> pose_ee_init, Matrix<double,8,1> pose_ee_desired, RowVector3d &v_e, RowVector3d &w_e);
	static double get_error_screw_param(Matrix<double,8,1> pose_now, Matrix<double,8,1> pose_ee_desired,  RowVector3d &v_e, RowVector3d &w_e);
	static MatrixXd jacobian_dual_vm(RowVectorXd q /*q is defined as [q1 q2 ...]*/, RowVectorXd joint_type, std::vector<RowVector3d> u, std::vector<RowVector3d> p , Matrix<double,8,1> pose_ee_init);
	static void dqEigenToDQdouble(RowVectorXd dq_eigen, std::vector<double> &dq_double);
	static void doubleToDQ(Matrix<double,8,1> &dq_eigen, std::vector<double> dq_double);
	static std::vector<double> dqEigenToDQdouble(RowVectorXd dq_eigen);
	static Matrix<double,8,1> returnDoubleToDQ(std::vector<double> & dq_double)
	{
		Matrix<double,8,1>  dq_eigen;
		for (int i=0; i< dq_double.size(); i++)
		{
			dq_eigen(i)=dq_double[i];
		}
		return dq_eigen;
	};

	static std::vector<double>  DQToDouble(Matrix<double,8,1> dq_eigen);
	static Matrix<double,8,1> inverseDQ(Matrix<double,8,1> dq); 
	static Matrix<double,8,1>  transformLine(Matrix<double,8,1> line, Matrix<double,8,1> transform);
	static RowVectorXd  transformLineVector(RowVectorXd lineVector, Matrix<double,8,1> transform);
	static RowVectorXd  transformLine6dVector(RowVectorXd lineVector, Matrix<double,8,1> transform);
	static RowVector3d  transformPoint(RowVector3d point, Matrix<double,8,1> transform);
	static double normalizeAngle(double theta);
	static RowVectorXd doubleVector2Rowvector(std::vector<double> doubleVector)
	{
		RowVectorXd eigenDouble=RowVectorXd::Zero(doubleVector.size());
		for (int i=0; i< doubleVector.size(); i++)
		{
			eigenDouble(i)=doubleVector[i];
		}
		return eigenDouble;
	};
	static std::vector<double>  RowVectorToDouble(RowVectorXd eigenVector);
	static Matrix<double,8,1> sclerp(Matrix<double,8,1> pose_now, Matrix<double,8,1> &pose_intermediate, Matrix<double,8,1> pose_desired, double tau);
	static Matrix<double,8,1>  preGraspFromGraspPose(Matrix<double,8,1> grasp_location, double distance, RowVector3d approach_direction);
	static geometry_msgs::Pose DQ2geometry_msgsPose(Matrix<double,8,1> pose_now);
	static geometry_msgs::Twist Rowvector2geometry_msgsTwist(RowVectorXd vel);
	static RowVectorXd Matrix8d2RowVector8d(Matrix<double,8,1> matrix8d);
	static RowVectorXd Matrix8d2RowVector6d(Matrix<double,8,1> matrix8d);
	static Matrix4d htmFromGeometryMsgPose(geometry_msgs::Pose pose);
	static Matrix<double,8,1>  dqFromGeometryMsgPose(geometry_msgs::Pose pose);
	static Matrix<double,8,1> dq2twist(Matrix<double,8,1> dq) ;
	static Matrix<double,8,1> twist2dq(Matrix<double,8,1> dq_twist);
	static Matrix<double,8,1> rotTrans2dq(RowVector4d rot, RowVector4d trans);
	static MatrixXd transformJacobian(MatrixXd jacobian_8d, Matrix<double,8,1> dq);
	static MatrixXd invDamped_8d(MatrixXd jacobian_8d, double mu);
	static RowVectorXd twistEigen2DQEigen(RowVectorXd twist);
	static RowVectorXd DQEigen2twistEigen(RowVectorXd DQtwist);
	static RowVectorXd spatial2CartVel(RowVectorXd screwVel, RowVector3d ee_pose);
	static RowVectorXd spatial2CartAcc(RowVectorXd screwAcc, RowVectorXd screwVel, RowVector3d ee_pose);
	static double get_error_screw(Matrix<double,8,1> pose_now, Matrix<double,8,1> pose_ee_desired,  RowVector3d &v_e, RowVector3d &w_e);
	static RowVectorXd spatial2CartPoseError(Matrix<double,8,1>  desired_pose, Matrix<double,8,1>  current_pose);
	static RowVectorXd spatial2CartPoseError_quatVec(Matrix<double,8,1>  desired_pose, Matrix<double,8,1>  current_pose);	
	static Matrix<double,8,1> RowVector6d2Matrix8d(RowVectorXd rowVector6d);
	DQoperations();
	~DQoperations();
};
#endif