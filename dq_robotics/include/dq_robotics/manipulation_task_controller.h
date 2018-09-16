#ifndef MANIPULATION_TASK_CONTROLLER_H
#define MANIPULATION_TASK_CONTROLLER_H


#include <ros/ros.h>
#include <ros/package.h>
#include <dq_robotics/baxter_poseControl_server.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/thread.hpp>
#include "tf/transform_datatypes.h"
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <dq_robotics/ControllerPerformance.h>
#include <ar10_local/ar10_joint_cmds.h>

using namespace Eigen;

class ManipulationTaskController
{
private:
	dq_robotics::BaxterControl srv;
	ros::NodeHandle nh;
	ros::ServiceClient pose_control_client;
	ros::Publisher controller_error_publisher, hand_cmds_publisher;
	std::string right_object_name, right_ref_frame, left_object_name, left_ref_frame; 
	ar10_local::ar10_joint_cmds hand_cmds_msg;
	// boost::thread thr;
	void publishPose2Rviz(float rate, Matrix<double,8,1> pose, std::string ref_frame);
public:
	ros::Publisher rviz_pub;
	// boost::thread* thr;
	bool ToPreGraspToGrasp(Matrix<double,8,1> grasp_location, std::string reference_frame, double distance, RowVectorXd approach_direction, int controller_mode, bool use_velocity_control);
	bool go2Pose_withPrecision(Matrix<double,8,1> pose_desired_abs, Matrix<double,8,1> pose_desired_rel, double error_threshold, int controller_mode, bool use_velocity_control);
	bool go2Pose_constVelocity(Matrix<double,8,1> pose_desired_abs , Matrix<double,8,1> pose_desired_rel, int controller_mode, int total_time, bool use_velocity_control);
	bool doRotationOrTranslation(bool rotation, Matrix<double,8,1> initial_pose, RowVectorXd axis, double initial_vm_jp, double final_vm_jp, double vm_velocity, int controller_mode, bool use_velocity_control, Matrix<double,8,1> pose_right, Matrix<double,8,1> pose_relative, bool vm_relative,  Matrix<double,8,1>& final_pose_commanded);	
	void init_task_controller();
	bool bimanual_pouringTask();
	void getHTMfromTFTransform(Matrix4d &htm, tf::StampedTransform transform);
	ManipulationTaskController();
	~ManipulationTaskController();
};

#endif