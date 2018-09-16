#include <dq_robotics/manipulation_task_controller.h>

using namespace Eigen;

ManipulationTaskController::ManipulationTaskController()
{
}
ManipulationTaskController::~ManipulationTaskController(){}
// ManipulationTaskController::doRotationOrTranslation(false, pre_grasp_pose, axis, 0, 0.1, 0.02, controller_mode, use_velocity_control)
bool ManipulationTaskController::doRotationOrTranslation(bool rotation, Matrix<double,8,1> initial_pose, RowVectorXd axis, double initial_vm_jp, double final_vm_jp, double vm_velocity, int controller_mode, bool use_velocity_control, Matrix<double,8,1> pose_right, Matrix<double,8,1> pose_relative, bool vm_relative,  Matrix<double,8,1>& final_pose_commanded)
{
	Matrix<double,8,1> next_pose;
	dq_robotics::BaxterControl::Request motion_request;
	double normError=1;
  std::vector<RowVector3d> u_abs, p_abs;
  std::vector<int> joint_type_abs;
  RowVectorXd q_vm_abs, q_init_abs;
  u_abs.resize(1);
  p_abs.resize(1);
  q_init_abs.resize(1);
  q_vm_abs.resize(1);
  joint_type_abs.resize(1);
	std::vector<Matrix<double,8,1> > fkm_matrix_abs;
	fkm_matrix_abs.resize(1);	  

  u_abs[0]<< axis(0), axis(1), axis(2);
  p_abs[0]<< axis(3), axis(4), axis(5);
  if (rotation)
	  joint_type_abs[0]=0;
	else
		joint_type_abs[0]=1;

  q_init_abs[0]=initial_vm_jp;

	double time_now=0, time_begin=0, time_last=0, time_diff=0;
  ros::Duration(0.5).sleep();
  time_begin=ros::Time::now().toSec();
  ros::Duration(0.5).sleep();
  time_begin=ros::Time::now().toSec();
  // std::cout << "time_begin: " << time_begin << std::endl;    
  time_last=ros::Time::now().toSec()-time_begin; 

	motion_request.velocity_control=use_velocity_control;
	motion_request.useVisualPose_single=false;
	motion_request.useVisualPose_relative=false;        
	motion_request.controlMode=controller_mode;
  q_vm_abs[0]=initial_vm_jp;
  // ROS_INFO("doRotationOrTranslation");
  // double time_begin=ros::Time::now().toSec();
  while (q_vm_abs[0] < final_vm_jp)
  {
  	time_now=ros::Time::now().toSec()-time_begin;  	
  	std::cout << "q_vm_abs[0]: " << q_vm_abs[0] << std::endl; 
  	// std::cout << "final_vm_jp: " << final_vm_jp << std::endl; 
		// time_now=ros::Time::now().toSec();  	
    q_vm_abs[0]=vm_velocity*time_diff+q_init_abs[0];
    fkm_matrix_abs=DQController::fkmDual(u_abs, p_abs, q_vm_abs, joint_type_abs);
    next_pose=DQoperations::mulDQ(fkm_matrix_abs[p_abs.size()-1], initial_pose);
    std::cout << "htm_next_pose: " << std::endl;
    std::cout << DQoperations::dq2HTM(next_pose) << std::endl;
    motion_request.desiredAbsPose= DQoperations::DQToDouble(pose_right);	  	
    if(vm_relative )
		{
			motion_request.desiredRelPose= DQoperations::DQToDouble(next_pose);
			motion_request.desiredAbsPose= DQoperations::DQToDouble(pose_right);
		}	    
		else
		{
			motion_request.desiredRelPose= DQoperations::DQToDouble(pose_relative);
			motion_request.desiredAbsPose= DQoperations::DQToDouble(next_pose);			
		}
    srv.request=motion_request;  
    if (pose_control_client.call(srv))
    {
      ROS_INFO("normError: %f", srv.response.normError);
      normError=srv.response.normError;
      // prepareMsg();
      // controller_error_publisher.publish(controlOutputMsg);      
    }
    else
    {
      ROS_ERROR("Failed to call service baxterControlService");
      return 0;
    }
    time_diff=time_diff+(time_now - time_last) ;      	   

		// std::cout << "time_diff: " << time_diff << std::endl;    
		// std::cout << "time_last: " << time_last << std::endl;    
		// std::cout << "time_now2: " << time_now << std::endl;    

    time_last=time_now;		
  }
  final_pose_commanded=next_pose;  	
  return 1;
}

void ManipulationTaskController::publishPose2Rviz(float rate, Matrix<double,8,1> pose, std::string ref_frame)
{
	geometry_msgs::PoseStamped rviz_pose;
  rviz_pose.pose=DQoperations::DQ2geometry_msgsPose(pose);
  rviz_pose.header.frame_id="base";
  rviz_pose.header.stamp=ros::Time::now();
  ros::Rate loop_rate(rate);
  while(ros::ok())
	{  
		rviz_pub.publish(rviz_pose);
		loop_rate.sleep();
	}	
}



bool ManipulationTaskController::ToPreGraspToGrasp(Matrix<double,8,1> grasp_location, std::string reference_frame, double distance, RowVectorXd approach_direction, int controller_mode, bool use_velocity_control)
{
  ROS_INFO("ToPreGraspToGrasp");

  dq_robotics::BaxterControl::Request go2pregrasp_req;
  double pre_grasp_distance=0.1, q_vel_vm=0;
  RowVectorXd axis(6);
	std::cout << "grasp_location: " << std::endl;
	std::cout << DQoperations::dq2HTM(grasp_location) << std::endl;
	RowVector3d grasp_position, grasp_axis_vector;
	grasp_axis_vector << approach_direction(0), approach_direction(1), approach_direction(2);
	grasp_position << approach_direction(3), approach_direction(4), approach_direction(5);
	grasp_position = grasp_position.cross(grasp_axis_vector);
  axis << grasp_axis_vector, grasp_position;
	std::cout << "axis_before: " << axis << std::endl;
	axis=  DQoperations::transformLine6dVector(axis, grasp_location);
  std::cout << "axis_after: " << axis << std::endl;
  // std::string ref_frame="/base";
	boost::thread thr1(boost::bind(&ManipulationTaskController::publishPose2Rviz,  this, 10, grasp_location, reference_frame));
	// ros::Duration(5).sleep();
  Matrix<double,8,1> pre_grasp_pose= DQoperations::preGraspFromGraspPose(grasp_location, distance, grasp_axis_vector);
  Matrix<double,8,1>  pose_desired_rel;

	boost::thread thr2(boost::bind(&ManipulationTaskController::publishPose2Rviz,  this, 10, pre_grasp_pose, reference_frame));
	// thr.join();
  // ManipulationTaskController::publishPose2Rviz(10, pre_grasp_pose, ref_frame);

  // boost::thread thr = new boost::thread(boost::bind(&ManipulationTaskController::publishPose2Rviz, this));
 //  rviz_pose.pose=DQoperations::DQ2geometry_msgsPose(pre_grasp_pose);
 //  rviz_pose.header.frame_id="base";
 //  rviz_pose.header.stamp=ros::Time::now();
 //  ros::Rate loop_rate(10);
 //  while(ros::ok())
	// {  
	// 	rviz_pub.publish(rviz_pose);
	// 	loop_rate.sleep();
	// }
	// if(ManipulationTaskController::go2Pose_constVelocity(pre_grasp_pose, pose_desired_rel, 1, 50, false))
	ROS_INFO("Rough approach starts...");
	if(!ManipulationTaskController::go2Pose_withPrecision(pre_grasp_pose, pose_desired_rel, 0.5, controller_mode, true))
		return 0;  
	ROS_INFO("Rough approach over");
	ROS_INFO("Precise approach starts...");
	if(!ManipulationTaskController::go2Pose_withPrecision(pre_grasp_pose, pose_desired_rel, 0.25, controller_mode, true))
		return 0;	
	else	ROS_INFO("Precise approach Over. Pre-grasp to grasp start will start now."); 
		// std::cout << "pre_grasp_pose: " << std::endl;
		// std::cout << DQoperations::dq2HTM(pre_grasp_pose) << std::endl;
		// publish2Rviz("base", pre_grasp_pose)
	 //  std::cout <<  DQoperations::dq2HTM(pre_grasp_pose) << std::endl;
	distance=distance+0.05;
	Matrix<double,8,1> final_pose_commanded;
	if (!ManipulationTaskController::doRotationOrTranslation(false, pre_grasp_pose, axis, 0, distance, 0.02, controller_mode, use_velocity_control, pre_grasp_pose, pre_grasp_pose, false, final_pose_commanded))
	return 0;
	else ROS_INFO("Grasp position achieved."); 

	return 1;  
}


bool ManipulationTaskController::go2Pose_withPrecision(Matrix<double,8,1> pose_desired_abs, Matrix<double,8,1> pose_desired_rel, double error_threshold, int controller_mode, bool use_velocity_control)
{
	dq_robotics::BaxterControl::Request controllerServ_req;
  controllerServ_req.velocity_control=use_velocity_control;
  controllerServ_req.useVisualPose_single=false;
  controllerServ_req.useVisualPose_relative=false;        
  controllerServ_req.controlMode=controller_mode;
  controllerServ_req.desiredAbsPose= DQoperations::DQToDouble(pose_desired_abs); 
  controllerServ_req.desiredRelPose= DQoperations::DQToDouble(pose_desired_rel);
  srv.request=controllerServ_req;
  double normError=1;
  // ROS_INFO("Home to pre-grasp: BEGIN");
  while (normError>error_threshold)
  {
    if (pose_control_client.call(srv))
    {
      ROS_INFO("normError: %f", srv.response.normError);
      normError=srv.response.normError;
      // prepareMsg();
      // controller_error_publisher.publish(controlOutputMsg);
    }
    else
    {
      ROS_ERROR("Failed to call service baxterControlService");
      return 0;
    }
  }
  return 1;  
}

bool ManipulationTaskController::go2Pose_constVelocity(Matrix<double,8,1> pose_desired_abs , Matrix<double,8,1> pose_desired_rel, int controller_mode, int total_time, bool use_velocity_control)
{
      // std::cout << "time: " << std::endl;
  dq_robotics::BaxterControl::Request inforServ_req, controllerServ_req;
  double normError=1;
  inforServ_req.request_manipulator_infoService=true;
  inforServ_req.infoService_mode=controller_mode;
  Matrix<double,8,1> pose_current_rel, pose_current_abs;
  srv.request=inforServ_req;

  if (pose_control_client.call(srv))
  {
    pose_current_abs=DQoperations::returnDoubleToDQ(srv.response.currentAbsPose);
    if (controller_mode!=1 && controller_mode!=2)
    {     
      pose_current_rel=DQoperations::returnDoubleToDQ(srv.response.currentRelPose);
    }
  }  
  else
  {
    ROS_ERROR("Failed to get manipulator info from baxterControlService service.");
    return 1;
  }  

  controllerServ_req.velocity_control=use_velocity_control;
  controllerServ_req.useVisualPose_single=false;
  controllerServ_req.useVisualPose_relative=false;        
  controllerServ_req.controlMode=controller_mode;

  double time_now=0, time_begin=0, time_last=0, time_diff=0;
  time_begin=ros::Time::now().toSec();
  ros::Duration(0.5).sleep();
  time_begin=ros::Time::now().toSec();
  // std::cout << "time_begin: " << time_begin << std::endl;    
  time_last=ros::Time::now().toSec()-time_begin;  	
  // time_last=time_now;
  while (time_diff<total_time)
  {
		// std::cout << "ros::Time::now().toSec(): " << ros::Time::now().toSec() << std::endl;    
		// std::cout << "time_begin: " << time_begin << std::endl;    
    time_now=ros::Time::now().toSec()-time_begin;  	
		// std::cout << "time_now1: " << time_now << std::endl;    
    double tau=(time_diff/total_time);
    Matrix<double,8,1> pose_desired_abs_i;
    Matrix<double,8,1> velocity_abs= DQoperations::sclerp(pose_current_abs, pose_desired_abs_i, pose_desired_abs, tau);
    // publish2Rviz("base", pose_desired_abs_i);
    controllerServ_req.desiredAbsPose= DQoperations::DQToDouble(pose_desired_abs_i); 
    controllerServ_req.desiredAbsVelocity= DQoperations::DQToDouble(velocity_abs);  	
    if (controller_mode!=1 && controller_mode!=2)
    { 
      Matrix<double,8,1> pose_desired_rel_i;
			Matrix<double,8,1> velocity_rel= DQoperations::sclerp(pose_current_rel, pose_desired_rel_i, pose_desired_rel, tau);    	
      // publish2Rviz("/right_arm", "/left_arm");
      controllerServ_req.desiredRelPose= DQoperations::DQToDouble(pose_desired_rel_i);         
      controllerServ_req.desiredRelVelocity= DQoperations::DQToDouble(velocity_rel);         
    }
    srv.request=controllerServ_req;
    if (pose_control_client.call(srv))
    {
      ROS_INFO("normError: %f", srv.response.normError);
      normError=srv.response.normError;
      // std::cout << "time: " << time_diff << std::endl;
      // prepareMsg();
      // controller_error_publisher.publish(controlOutputMsg);
    }
    // std::cout << "time: " << time_diff << std::endl;
    else
    {
      ROS_ERROR("Failed to call position controller service baxterControlService");
      return 1;
    }
    time_diff=time_diff+(time_now - time_last) ;      	   

		std::cout << "time_diff: " << time_diff << std::endl;    
		std::cout << "time_last: " << time_last << std::endl;    
		std::cout << "time_now2: " << time_now << std::endl;    

    time_last=time_now;		
  }        
}

void ManipulationTaskController::getHTMfromTFTransform(Matrix4d &htm, tf::StampedTransform transform)
{
  tf::Quaternion quat=transform.getRotation();
  Eigen::Quaterniond q;
  q.x()=-quat.x();
  q.y()=-quat.y();
  q.z()=-quat.z();
  q.w()=-quat.w();
  Eigen::Matrix3d rot = q.normalized().toRotationMatrix();

  tf::Vector3 trans=transform.getOrigin();
  Eigen::RowVector4d translation;
  translation << trans.getX(), trans.getY(), trans.getZ(), 1;

  htm.setIdentity();
  htm.block<3,3>(0,0) = rot;
  htm.rightCols<1>() =translation ;
}

bool ManipulationTaskController::bimanual_pouringTask()
{
	hand_cmds_msg.shortcutCmds=1;
	hand_cmds_msg.hand=0;
	ROS_INFO("opening right hand");
	double count=0;
	while(count<2)
	{
		hand_cmds_publisher.publish(hand_cmds_msg);
		count=count+1;
		ros::Duration(.4).sleep();	
	}	
	ros::Duration(3).sleep();	
	ROS_INFO("right hand should be open");

	hand_cmds_msg.shortcutCmds=1;
	hand_cmds_msg.hand=1;
	ROS_INFO("opening left hand");
	count =0;
	while(count<2)
	{
		hand_cmds_publisher.publish(hand_cmds_msg);
		count=count+1;
		ros::Duration(.4).sleep();	
	}
	ros::Duration(3).sleep();		
	ROS_INFO("left hand should be open");
	Matrix4d right_object=Matrix4d::Identity(), left_object=Matrix4d::Identity(), rel_pose_rotation=Matrix4d::Identity();
	Matrix4d htm_right=Matrix4d::Identity(), htm_left=Matrix4d::Identity() ;
  Matrix<double,8,1> right_grasp_location, left_grasp_location;
  tf::TransformListener listener;
	tf::StampedTransform transform;
  try{
    ros::Time now = ros::Time(0);
    listener.waitForTransform("base", "/bottle_0_ft1",
                              now, ros::Duration(4.0));
    listener.lookupTransform("base", "/bottle_0_ft1",
                             now, transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }
  getHTMfromTFTransform(htm_right, transform);
  htm_right(0,3)=htm_right(0,3)-0.03;
  htm_right(1,3)=htm_right(1,3)-0.03;
  htm_right(2,3)=htm_right(2,3)+0.04;

  right_grasp_location=  DQoperations::htm2DQ(htm_right);
  std::cout << "right_grasp_location:" << std::endl;
  std::cout << right_grasp_location << std::endl;

	RowVectorXd right_lift_axis(6), left_lift_axis(6);
	right_lift_axis << 0, 0, 1, htm_right(0, 3), htm_right(1, 3), htm_right(2, 3);  
	double lift_distance=0.3, lift_velocity=0.03;

	try{
    ros::Time now = ros::Time(0);
    listener.waitForTransform("base", "/glass_0_ft2",
                              now, ros::Duration(4.0));
    listener.lookupTransform("base", "/glass_0_ft2",
                             now, transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }
  getHTMfromTFTransform(htm_left, transform);

  htm_left(0,3)=htm_left(0,3)-0.05;  
  htm_left(1,3)=htm_left(1,3)+0.01;  
  htm_left(2,3)=htm_left(2,3)+0.03;  

 	left_grasp_location=  DQoperations::htm2DQ(htm_left);
  std::cout << "left_grasp_location:" << std::endl;
  std::cout << left_grasp_location << std::endl; 	

	left_lift_axis << 0, 0, 1, htm_left(0, 3), htm_left(1, 3), htm_left(2, 3);  

	std::string reference_frame="base";
	double distance=0.1;
	RowVectorXd approach_direction(6);
	approach_direction << 0, 1, 0, htm_right(0, 3), htm_right(1, 3), htm_right(2, 3);
	int controller_mode=1;
	bool use_velocity_control=false;

	if (ManipulationTaskController::ToPreGraspToGrasp(right_grasp_location, reference_frame, distance, approach_direction, controller_mode, use_velocity_control))
	{
		ROS_INFO("Right Grasp configuration achieved.");	
	}
	else return 0;
	hand_cmds_msg.shortcutCmds=2;
	hand_cmds_msg.hand=0;
	count =0;
	ROS_INFO("Closing right hand... ");
	while(count<2)
	{
		hand_cmds_publisher.publish(hand_cmds_msg);
		count=count+1;
		ros::Duration(.4).sleep();	
	}	
	ros::Duration(3).sleep();		
	ROS_INFO("Right hand should be closed");
	Matrix<double,8,1> final_pose_commanded_right, final_pose_commanded_left;
	if(ManipulationTaskController::doRotationOrTranslation(false, right_grasp_location, right_lift_axis, 0, lift_distance, lift_velocity, controller_mode, false, right_grasp_location, right_grasp_location, false, final_pose_commanded_right))
		ROS_INFO("Right glass lift completed.");
	else return 0;

	approach_direction << 0, 1, 0, htm_left(0, 3), htm_left(1, 3), htm_left(2, 3);
	controller_mode=2;
	if (ManipulationTaskController::ToPreGraspToGrasp(left_grasp_location, reference_frame, distance, approach_direction, controller_mode, use_velocity_control))
	{
		ROS_INFO("Left Grasp configuration achieved.");	
	}
	else return 0;

	hand_cmds_msg.shortcutCmds=2;
	hand_cmds_msg.hand=1;	
	count =0;
	ROS_INFO("Closing left hand... ");
	while(count<2)
	{
		hand_cmds_publisher.publish(hand_cmds_msg);
		count=count+1;
		ros::Duration(.4).sleep();	
	}	
	ros::Duration(3).sleep();			
	ROS_INFO("Left hand should be closed");
// return 0;
	if(ManipulationTaskController::doRotationOrTranslation(false, left_grasp_location, left_lift_axis, 0, lift_distance, lift_velocity, controller_mode, false, left_grasp_location, right_grasp_location, false, final_pose_commanded_left ))
		ROS_INFO("Left glass lift completed.");
	else return 0;

	// right_lift_axis.resize(6);
	// right_lift_axis << -1, 0, 0, 0, 0, 0;
	// controller_mode=1;
	// if (!ManipulationTaskController::doRotationOrTranslation(false, final_pose_commanded_right, right_lift_axis, 0, 0.1, 0.02, controller_mode, false, final_pose_commanded_right, final_pose_commanded_right, false,  final_pose_commanded_right))
	// 	return 0;

	// left_lift_axis.resize(6);
	// left_lift_axis << 0, 1, 0, 0, 0, 0;
	// controller_mode=2;
	// if (!ManipulationTaskController::doRotationOrTranslation(false, final_pose_commanded_left, left_lift_axis, 0, 0.1, 0.02, controller_mode, false, final_pose_commanded_left, final_pose_commanded_left, false, final_pose_commanded_left))
	// 	return 0;


	Matrix4d htm_right_desired;
	htm_right_desired << 		0, -1, 0, 0.679,
													0, 0, 1, -0.05,
													-1, 0, 0, 0.387,
													0, 0, 0, 1;
	// htm_right_desired << 		0, -1, 0, 0.479,
	// 												0, 0, 1, -0.146,
	// 												-1, 0, 0, 0.387,
	// 												0, 0, 0, 1;													


	Matrix<double,8,1> dq_right_desired=DQoperations::htm2DQ(htm_right_desired);
	controller_mode=1;
  if (!ManipulationTaskController::go2Pose_withPrecision(dq_right_desired, dq_right_desired, 0.2, controller_mode, true))
  	return 0;

	Matrix4d htm_relative_desired;
	htm_relative_desired << -1, 0, 0, -0.15,
													0, 1, 0, -0.09,
													0, 0, -1, 0.073,
													0, 0, 0, 1;
	Matrix<double,8,1> dq_relative_desired=DQoperations::htm2DQ(htm_relative_desired);
	// controller_mode=4;
 //  if (!ManipulationTaskController::go2Pose_withPrecision(dq_right_desired, dq_relative_desired, 0.5, controller_mode, true))
 //  	return 0;
	controller_mode=4;
  if (!ManipulationTaskController::go2Pose_withPrecision(dq_right_desired, dq_relative_desired, 0.15, controller_mode, false))
  	return 0;
	left_lift_axis.resize(6);
	left_lift_axis << 0, 0, -1, -0.15, -0.09, 0.073;
	

	// bool ManipulationTaskController::doRotationOrTranslation(bool rotation, Matrix<double,8,1> initial_pose, RowVectorXd axis, double initial_vm_jp, double final_vm_jp, double vm_velocity, int controller_mode, bool use_velocity_control, Matrix<double,8,1> pose_right, Matrix<double,8,1> pose_relative, bool vm_relative,  Matrix<double,8,1>& final_pose_commanded)
	controller_mode=4;
	if (!ManipulationTaskController::doRotationOrTranslation(true, dq_relative_desired, left_lift_axis, 0, 2.4, 0.2, controller_mode, false, dq_right_desired, dq_relative_desired, true, final_pose_commanded_left))
		return 0;

	ROS_INFO("Rotating back");
	left_lift_axis << 0, 0, 1, -0.15, -0.09, 0.073;
	if (!ManipulationTaskController::doRotationOrTranslation(true, final_pose_commanded_left, left_lift_axis, 0, 2.4, 0.2, controller_mode, false, dq_right_desired, dq_relative_desired, true, final_pose_commanded_left))
		return 0;


 //  hand_cmds_msg.shortcutCmds=1;
	// hand_cmds_msg.hand=0;
	// ROS_INFO("opening right hand");
	// count=0;
	// while(count<2)
	// {
	// 	hand_cmds_publisher.publish(hand_cmds_msg);
	// 	count=count+1;
	// 	ros::Duration(.4).sleep();	
	// }	
	// ros::Duration(3).sleep();	
	// ROS_INFO("right hand should be open");

	// hand_cmds_msg.shortcutCmds=1;
	// hand_cmds_msg.hand=1;
	// ROS_INFO("opening left hand");
	// count =0;
	// while(count<2)
	// {
	// 	hand_cmds_publisher.publish(hand_cmds_msg);
	// 	count=count+1;
	// 	ros::Duration(.4).sleep();	
	// }
	// ros::Duration(3).sleep();		
	// ROS_INFO("left hand should be open");
	// bool ManipulationTaskController::go2Pose_withPrecision(Matrix<double,8,1> pose_desired_abs, Matrix<double,8,1> pose_desired_rel, double error_threshold, int controller_mode, bool use_velocity_control)

	// reference_frame="/right_hand";
	// controller_mode=4;
	// // bool ManipulationTaskController::go2Pose_withPrecision(right_grasp_location, rel_pose_4rotation, 0.08, controller_mode,  use_velocity_control)
	// if (ManipulationTaskController::go2Pose_withPrecision(right_grasp_location, rel_pose_4rotation, 0.1, controller_mode,  use_velocity_control))
	// {
	// 	ROS_INFO("Relative pre-rotation configuration achieved.");	
	// }	
	// else return 0;

	// controller_mode=5;
	// RowVectorXd axis(6);
	// axis << -1, 0, 0, 0, 0.4, 0.2;
	// if (ManipulationTaskController::doRotationOrTranslation(true, rel_pose_4rotation, axis, 0.0, 1.6, 0.2, controller_mode, false, right_grasp_location, rel_pose_4rotation, true))
		// if (!ManipulationTaskController::doRotationOrTranslation(true, dq_relative_desired, left_lift_axis, 0, 1.6, 0.02, controller_mode, false, final_pose_commanded_right, dq_relative_desired, true, final_pose_commanded_left))
	// 	ROS_INFO("Rotation achieved.");
	// else return 0;
	return 1;
}

// void setDesirePoseRelative(int controller_mode, std::string object_name, std::string ref_frame)
// {

// }

void ManipulationTaskController::init_task_controller()
{
	pose_control_client = nh.serviceClient<dq_robotics::BaxterControl>("baxterControlService");
	rviz_pub = nh.advertise<geometry_msgs::PoseStamped>( "/rviz_pub", 0 );
	hand_cmds_publisher = nh.advertise<ar10_local::ar10_joint_cmds>( "ar10/desired_joint_cmds", 0 );
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "baxter_controller");
	ros::NodeHandle n;
	ManipulationTaskController* baxter_task_controller= new ManipulationTaskController();
	baxter_task_controller->ManipulationTaskController::init_task_controller();
	
	if (baxter_task_controller->ManipulationTaskController::bimanual_pouringTask())
		ROS_INFO("bimanual_pouringTask done!");
	return 1;
	// ros::ServiceServer controllerService = n.advertiseService("baxterControlService", &BaxterPoseControlServer::baxterControlServerCallback, baxter_controller);
	// baxter_controller->BaxterPoseControlServer::update_manipulator();
}
