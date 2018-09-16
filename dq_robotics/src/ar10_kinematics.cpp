#include <dq_robotics/ar10_kinematics.h>
// #include <baxter_trajectory_interface/baxter_poseControl_server.h>
#define PI 3.14159265

AR10Kinematics::AR10Kinematics(){}

void AR10Kinematics::init_AR10Kinematics()
{

	std::string paramName="result_file_name";
	if(!ros::param::get(paramName, result_file_name))
	{
		std::cout << "result_file_name not found" << std::endl;
	}
	// result_file_name="iser2018_result.csv";
	path = ros::package::getPath("ar10_kinematics");
	path.append("/results/");
	path.append(result_file_name);
	base_frame="ar10_right_mod/base";
	right_ar10Hand_pub = n.advertise<sensor_msgs::JointState>("/ar10_right/ar10/right/joint_states", 1000);
	// finger_ik_service=n.advertiseService("/ar10_right/finger_ik_sever", &AR10Kinematics::computeHandIKCallback, this);

	Matrix4d mat = Matrix4d::Identity();
	mat(2,3)=-0.0021;
	frame_hand_base = DQoperations::htm2DQ(mat);
  hand_base_static_transformStamped.header.stamp = ros::Time::now();
  hand_base_static_transformStamped.header.frame_id = "ar10_right/circuit_support";
	hand_base_static_transformStamped.child_frame_id = "ar10_right_mod/hand_base";	 
	
	hand_base_static_transformStamped.transform.translation.x=mat(0,3);
	hand_base_static_transformStamped.transform.translation.y=mat(1,3);
	hand_base_static_transformStamped.transform.translation.z=mat(2,3);
  hand_base_static_transformStamped.transform.rotation.x = frame_hand_base(1);
  hand_base_static_transformStamped.transform.rotation.y = frame_hand_base(2);
  hand_base_static_transformStamped.transform.rotation.z = frame_hand_base(3);
  hand_base_static_transformStamped.transform.rotation.w = frame_hand_base(0);		
  joint_size_coupled = 2;
};

AR10Kinematics::~AR10Kinematics(){};


void AR10Kinematics::getJacobianCoupled(finger_params &finger)
{
			std::cout << "ar10_here_1" << std::endl;
	finger.finger_jacobian_coupled=MatrixXd::Zero(8, 2);
	finger.finger_jacobian_simple= DQController::jacobianDual(finger.u, finger.p, finger.finger_ee_init, finger.finger_joint_type, finger.screwDispalcementArray);	
	finger.finger_jacobian_simple= DQController::jacobianDual_8d(finger.p.size(), finger.finger_jacobian_simple);
	finger.finger_jacobian_coupled.col(0)= finger.finger_jacobian_simple.col(0);
	double coupling_term = (finger.distal_phalanx.p*sin(finger.distal_phalanx.psi - finger.distal_phalanx.theta) - finger.distal_phalanx.h*sin(finger.distal_phalanx.theta + finger.distal_phalanx.alpha - finger.distal_phalanx.psi))/(finger.distal_phalanx.h*sin(finger.distal_phalanx.theta + finger.distal_phalanx.alpha - finger.distal_phalanx.psi));
	finger.finger_jacobian_coupled.col(1) = finger.finger_jacobian_simple.col(1) + coupling_term*finger.finger_jacobian_simple.col(2);
}

bool AR10Kinematics::getControllerParms()
{
	double value;	
	
	std::string paramName="ar10_dt";

	if(!ros::param::get(paramName, value))
	{
		std::cout << "ar10 controller param: ar10_dt not found" << std::endl;
		return 0;
	}
	else
		dt = value;

	paramName="ar10_Kp";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "ar10 controller param: ar10_Kp not found" << std::endl;
		return 0;
	}
	else
		Kp = value;

	paramName="ar10_mu";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "ar10 controller param: ar10_mu not found" << std::endl;
		return 0;
	}
	else
		mu = value;

	paramName="ar10_slider_cmd_tolerance";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "ar10 controller param: ar10_slider_cmd_tolerance not found" << std::endl;
		return 0;
	}
	else
		slider_cmd_tolerance = value;
	
	return 1;
}

// void AR10Kinematics::update_relative()
// {
// 	u_relative.resize(5);
// 	p_relative.resize(5);
// 	Ar10KinematicsLoop();
// 	ee_relative = DQoperations::mulDQ(DQoperations::classicConjDQ(index_finger.finger_ee_current), thumb.finger_ee_current);
// 	u_relative[0]= DQoperations::transformLineVector(index_finger.u[2], DQoperations::classicConjDQ(index_finger.finger_ee_current));
// 	u_relative[1]= DQoperations::transformLineVector(index_finger.u[1], DQoperations::classicConjDQ(index_finger.finger_ee_current));
// 	u_relative[2]= DQoperations::transformLineVector(index_finger.u[0], DQoperations::classicConjDQ(index_finger.finger_ee_current));
// 	u_relative[3]= DQoperations::transformLineVector(thumb.u[0], DQoperations::classicConjDQ(index_finger.finger_ee_current));
// 	u_relative[4]= DQoperations::transformLineVector(thumb.u[1], DQoperations::classicConjDQ(index_finger.finger_ee_current));

// 	p_relative[0]= DQoperations::transformPoint(index_finger.p[2], DQoperations::classicConjDQ(index_finger.finger_ee_current));
// 	p_relative[1]= DQoperations::transformPoint(index_finger.p[1], DQoperations::classicConjDQ(index_finger.finger_ee_current));
// 	p_relative[2]= DQoperations::transformPoint(index_finger.p[0], DQoperations::classicConjDQ(index_finger.finger_ee_current));
// 	p_relative[3]= DQoperations::transformPoint(thumb.p[0], DQoperations::classicConjDQ(index_finger.finger_ee_current));
// 	p_relative[4]= DQoperations::transformPoint(thumb.p[1], DQoperations::classicConjDQ(index_finger.finger_ee_current));
// 	q_relative << index_finger.q(2), index_finger.q(1), index_finger.q(0), thumb.q(0), thumb.q(1);  
// }

bool AR10Kinematics::ik_relative(dq_robotics::IkHand::Request hand_ik_req, dq_robotics::IkHand::Response &hand_ik_res)
{
	Ar10KinematicsLoop();

	finger_params	index_ik, thumb_ik;
	index_ik = index_finger; 
	thumb_ik = thumb;

	right_hand_cmds.position.resize(4);
	right_hand_cmds.name.resize(4);

	right_hand_cmds.name[0]= "servo0";
	right_hand_cmds.name[1]= "servo1";
	right_hand_cmds.name[2]= "servo8";
	right_hand_cmds.name[3]= "servo9";

	ee_relative = DQoperations::mulDQ(DQoperations::classicConjDQ(index_finger.finger_ee_current), thumb.finger_ee_current);
	Matrix<double,8,1> desiredPose;
	DQoperations::doubleToDQ(desiredPose, hand_ik_req.desiredPose);

 // controller

	RowVectorXd q, q_dot, screw_error, screw_error_position;
	q.resize(4);
	q_dot.resize(4);
	q << -index_ik.q(1), -index_ik.q(0), thumb_ik.q(0), thumb_ik.q(1);
	// q_dot.resize(5);
	// q << q_relative(0), q_relative(1), q_relative(2), q_relative(3), q_relative(4);

	std::vector<Matrix<double,8,1> > screwDispalcementArray;

	// std::cout << "result_file_name: " << path <<  std::endl;
	// std::ofstream result_file(path, std::ios::app);
	// result_file.open(result_file_name);
	Matrix4d error_htm, current_htm, desired_htm;
	current_htm =DQoperations::dq2HTM(ee_relative);
	desired_htm =DQoperations::dq2HTM(desiredPose);



 //  if(result_file.is_open())
 //  {
 //  	std::cout << "file is open" << std::endl;
 //  }
	// result_file << "current_dq_pose:" << "," << finger_ik.finger_ee_current(0,0) << "," << finger_ik.finger_ee_current(1,0) << "," << finger_ik.finger_ee_current(2,0) << "," << finger_ik.finger_ee_current(3,0) << "," << finger_ik.finger_ee_current(4,0) << "," << finger_ik.finger_ee_current(5,0) << "," << finger_ik.finger_ee_current(6,0) << "," << finger_ik.finger_ee_current(7,0) << ", \n";  
	// result_file << "deisred_dq_pose:" << "," << desiredPose(0,0) << "," << desiredPose(1,0) << "," << desiredPose(2,0) << "," << desiredPose(3,0) << "," << desiredPose(4,0) << "," << desiredPose(5,0) << "," << desiredPose(6,0) << "," << desiredPose(7,0) << ", \n";

	// result_file << "desired_xy:" << "," << desired_htm(1,3) << "," << desired_htm(2,3) << ", \n";
	// result_file << "init_xy:" << "," << current_htm(1,3) << "," << current_htm(2,3) << ", \n";


	// result_file << "time" << "," << "x_current"  << "," << "y_current"  << "," << "norm_error" << "," << "error_x" <<  "," << "error_y" << ", \n";
	Matrix<double,8,1> pose_error;	
	ros::Time end = ros::Time::now(), begin = ros::Time::now();
	double secs =0;
	
	ros::WallDuration sleep_time(dt);
	double norm_error=1;
	MatrixXd jacobian_relative, jacobian_relative_position;

	while(secs<60)
	{
		jacobian_relative = MatrixXd::Zero(8,4);	
		jacobian_relative_position = MatrixXd::Zero(4,4);	
		current_htm =DQoperations::dq2HTM(ee_relative);
		pose_error =DQoperations::mulDQ(ee_relative, DQoperations::classicConjDQ(desiredPose));		
		screw_error= DQController::getScrewError_8d(ee_relative, desiredPose);
		screw_error_position= screw_error.head(4);
		// norm_error=screw_error.norm();
		norm_error=screw_error_position.norm();
	
		getJacobianCoupled(index_ik);
		thumb_ik.finger_jacobian_simple= DQController::jacobianDual(thumb_ik.u, thumb_ik.p, thumb_ik.finger_ee_init, thumb_ik.finger_joint_type, thumb_ik.screwDispalcementArray);	
		thumb_ik.finger_jacobian_simple= DQController::jacobianDual_8d(thumb_ik.p.size(), thumb_ik.finger_jacobian_simple);	

		// std::cout << "index_finger.finger_ee_current:" << std::endl;
		// std::cout << index_finger.finger_ee_current << std::endl;
		
		// std::cout << "jacob_rel_beforeTF:" << std::endl;
		// std::cout << index_ik.finger_jacobian_coupled.col(0).transpose() << std::endl;
		// std::cout << index_ik.finger_jacobian_coupled.col(1).transpose() << std::endl;
		// std::cout << thumb_ik.finger_jacobian_simple.col(0).transpose() << std::endl;
		// std::cout << thumb_ik.finger_jacobian_simple.col(1).transpose() << std::endl;



		jacobian_relative.col(0) = DQoperations::transformLine(index_ik.finger_jacobian_coupled.col(1), DQoperations::classicConjDQ(index_finger.finger_ee_current));
		jacobian_relative.col(1) = DQoperations::transformLine(index_ik.finger_jacobian_coupled.col(0), DQoperations::classicConjDQ(index_finger.finger_ee_current));				
		jacobian_relative.col(2) = DQoperations::transformLine(thumb_ik.finger_jacobian_simple.col(0), DQoperations::classicConjDQ(index_finger.finger_ee_current));
		jacobian_relative.col(3) = DQoperations::transformLine(thumb_ik.finger_jacobian_simple.col(1), DQoperations::classicConjDQ(index_finger.finger_ee_current));
		jacobian_relative_position = jacobian_relative.block(0,0,jacobian_relative.rows()/2, jacobian_relative.cols()); 
		// std::cout << "jacob_rel:" << std::endl;
		// std::cout << jacobian_relative.col(0).transpose() << std::endl;
		// std::cout << jacobian_relative.col(1).transpose() << std::endl;
		// std::cout << jacobian_relative.col(2).transpose() << std::endl;
		// std::cout << "jacobian_relative:" << std::endl;
		// std::cout << jacobian_relative << std::endl;

		// std::cout << "jacobian_relative_position:" << std::endl;
		// std::cout << jacobian_relative_position << std::endl;
		// q_dot= DQController::calculateControlVel(Kp, mu, screw_error, jacobian_relative, 4);

		q_dot= DQController::calculateControlVel(Kp, mu, screw_error_position, jacobian_relative_position, 4);
	
		// std::cout << "############# q_dot: " << q_dot << std::endl;
		// std::cout << "virtual_finger.finger_jacobian_coupled: " << virtual_finger.finger_jacobian_coupled << std::endl;
		q = q + q_dot*dt;
		index_ik.proximal_phalanx.alpha_cal = index_ik.q_init(0) - q(1); 
		index_ik.proximal_phalanx.current_sliderCmd= getSliderFromTheta(index_ik.proximal_phalanx);
		index_ik.middle_phalanx.alpha_cal = index_ik.q_init(1) - q(0); 
		index_ik.middle_phalanx.current_sliderCmd= getSliderFromTheta(index_ik.middle_phalanx);

		thumb_ik.proximal_phalanx.alpha_cal = thumb_ik.q_init(0) + q(2); 
		thumb_ik.proximal_phalanx.current_sliderCmd= getSliderFromTheta(thumb_ik.proximal_phalanx);
		thumb_ik.middle_phalanx.alpha_cal = thumb_ik.q_init(1) + q(3); 
		thumb_ik.middle_phalanx.current_sliderCmd= getSliderFromTheta(thumb_ik.middle_phalanx);


		right_hand_cmds.position[0]= thumb_ik.proximal_phalanx.current_sliderCmd;
		right_hand_cmds.position[1]= thumb_ik.middle_phalanx.current_sliderCmd;
		right_hand_cmds.position[2]= index_ik.proximal_phalanx.current_sliderCmd;
		right_hand_cmds.position[3]= index_ik.middle_phalanx.current_sliderCmd;
		
		// right_hand_cmds.position[0]= 0.5;
		// right_hand_cmds.position[1]= 0.5;
		// right_hand_cmds.position[2]= 0.5;
		// right_hand_cmds.position[3]= 0.5;

		// std::cout << "right_hand_cmds" << std::endl;
		// for(int ii = 0; ii<right_hand_cmds.position.size(); ii++)
		// {
		// 	std::cout << right_hand_cmds.position[ii] << std::endl;
		// }

		do
		{
			// sleep_time.sleep();
			right_ar10Hand_pub.publish(right_hand_cmds);
			sleep_time.sleep();
		}
		while (!right_ar10Hand_pub.getNumSubscribers());

		sleep_time.sleep();
		std::cout << "norm_error: " << norm_error << std::endl;
	// Writing to  csv :result
		error_htm = DQoperations::dq2HTM(pose_error);
		end = ros::Time::now();
		secs = (end-begin).toSec();
		 // std::cout << "secs: " << secs << std::endl;
		// result_file << secs << "," << current_htm(1,3) << "," << current_htm(2,3)  << "," << norm_error << "," << (desired_htm(1,3) - current_htm(1,3)) <<  "," << (desired_htm(2,3) - current_htm(2,3)) << ", \n";

		Ar10KinematicsLoop();
		index_ik = index_finger; 
		thumb_ik = thumb;
		ee_relative = DQoperations::mulDQ(DQoperations::classicConjDQ(index_finger.finger_ee_current), thumb.finger_ee_current);
		q << -index_ik.q(1), -index_ik.q(0), thumb_ik.q(0), thumb_ik.q(1);
	}
	index_finger = index_ik; 
	thumb = thumb_ik;	
	hand_ik_res.jointPosition.clear();
	hand_ik_res.jointPosition.push_back(index_ik.middle_phalanx.current_sliderCmd);
	hand_ik_res.jointPosition.push_back(index_ik.proximal_phalanx.current_sliderCmd);
	hand_ik_res.jointPosition.push_back(thumb_ik.proximal_phalanx.current_sliderCmd);
	hand_ik_res.jointPosition.push_back(thumb_ik.middle_phalanx.current_sliderCmd);
	return true;
}

bool AR10Kinematics::sendJointCmds_AR10(std::vector<double> jointCmds, std::string fingerName)
{
	right_hand_cmds.name.resize(2);
	right_hand_cmds.position.resize(2);
	finger_params	finger_ik;
	// std::cout << "right_hand_cmds: " << std::endl;
	if(!fingerName.compare("index"))
	{
		right_hand_cmds.name[0]= "servo8";
		right_hand_cmds.name[1]= "servo9";
		// std::cout << right_hand_cmds.position[0] << std::endl;
		// std::cout << right_hand_cmds.position[1] << std::endl;
		finger_ik = index_finger;
	}	
	else if(!fingerName.compare("thumb"))
	{
		right_hand_cmds.name[0]= "servo0";
		right_hand_cmds.name[1]= "servo1";
		finger_ik = thumb;
	}
	else return 0;
	// right_hand_cmds.position = jointCmds;
	std::cout << "jointCmds(sendJointCmds_AR10): " << jointCmds[0] << jointCmds[1] << std::endl;
	finger_ik.proximal_phalanx.alpha_cal = finger_ik.q_init(0) + jointCmds[0]; 
	finger_ik.proximal_phalanx.current_sliderCmd= getSliderFromTheta(finger_ik.proximal_phalanx);
	finger_ik.middle_phalanx.alpha_cal = finger_ik.q_init(1) + jointCmds[1]; 
	finger_ik.middle_phalanx.current_sliderCmd= getSliderFromTheta(finger_ik.middle_phalanx);

	right_hand_cmds.position[0]= finger_ik.proximal_phalanx.current_sliderCmd;
	right_hand_cmds.position[1]= finger_ik.middle_phalanx.current_sliderCmd;


	ros::WallDuration sleep_time(0.1);
	do
	{
		// sleep_time.sleep();
		right_ar10Hand_pub.publish(right_hand_cmds);
		sleep_time.sleep();
	}
	while (!right_ar10Hand_pub.getNumSubscribers());

	if(!fingerName.compare("index"))
	{
		index_finger = finger_ik;
	}	
	else if(!fingerName.compare("thumb"))
	{
		thumb = finger_ik;
	}
	std::cout << "return from  sendJointCmds_AR10: " << std::endl; 
	return 1;
}

bool AR10Kinematics::computeHandIKCallback(dq_robotics::IkHand::Request& hand_ik_req, 
									dq_robotics::IkHand::Response& hand_ik_res)
{	
	if(hand_ik_req.finger!=4 && hand_ik_req.finger!=5 && hand_ik_req.finger!=0)
		return 0;
	if(hand_ik_req.finger==0)
	{
		return ik_relative(hand_ik_req, hand_ik_res);
	}
	finger_params	finger_ik;
	right_hand_cmds.position.resize(2);
	right_hand_cmds.name.resize(2);


	if(hand_ik_req.finger == 4)
	{
		right_hand_cmds.name[0]= "servo8";
		right_hand_cmds.name[1]= "servo9";
		finger_ik = index_finger;		
	}	

	if(hand_ik_req.finger == 5)
	{
		right_hand_cmds.name[0]= "servo0";
		right_hand_cmds.name[1]= "servo1";
		finger_ik = thumb;
	}		
	//Controller params
	double norm_error=1;
	// double dt=0.1, Kp=0.5, mu=0.001;
	// slider_cmd_tolerance= 0.1;
	//update the finger
// std::cout << "hello _1" << std::endl;	
	finger_ik.slider_position_output.clear();
	getSliderPosition(finger_ik);
// std::cout << "hello _2" << std::endl;		
	computeJointRotation(finger_ik);
	fkmFinger(finger_ik)	;	
// std::cout << "hello _3" << std::endl;	
	//get request
	Matrix<double,8,1> desiredPose;
	DQoperations::doubleToDQ(desiredPose, hand_ik_req.desiredPose);

 // controller

	RowVectorXd q, q_dot, screw_error;
	q.resize(2);
	q_dot.resize(2);
	q << finger_ik.q(0), finger_ik.q(1);

	std::vector<Matrix<double,8,1> > screwDispalcementArray;

	std::cout << "result_file_name: " << path <<  std::endl;
	std::ofstream result_file(path, std::ios::app);
	// result_file.open(result_file_name);
	Matrix4d error_htm, current_htm, desired_htm;
	current_htm =DQoperations::dq2HTM(finger_ik.finger_ee_current);
	desired_htm =DQoperations::dq2HTM(desiredPose);
  if(result_file.is_open())
  {
  	std::cout << "file is open" << std::endl;
  }
	

	result_file << "time" << "," << "x_current"  << "," << "y_current"  << "," << "norm_error" << "," << "error_x" <<  "," << "error_y" << ", \n";
	Matrix<double,8,1> pose_error;	
	ros::Time end = ros::Time::now(), begin = ros::Time::now();
	double secs =0;
	
	ros::WallDuration sleep_time(0.5);
	while(secs<60)
	{
		current_htm =DQoperations::dq2HTM(finger_ik.finger_ee_current);
		pose_error =DQoperations::mulDQ(finger_ik.finger_ee_current, DQoperations::classicConjDQ(desiredPose));		
		screw_error= DQController::getScrewError_8d(finger_ik.finger_ee_current, desiredPose);
		norm_error=screw_error.norm();
		if(!finger_ik.is_thumb)
		{
			getJacobianCoupled(finger_ik);
			q_dot= DQController::calculateControlVel(Kp, mu, screw_error, finger_ik.finger_jacobian_coupled, joint_size_coupled);
		}
		else
		{
			finger_ik.finger_jacobian_simple= DQController::jacobianDual(finger_ik.u, finger_ik.p, finger_ik.finger_ee_init, finger_ik.finger_joint_type, finger_ik.screwDispalcementArray);	
			finger_ik.finger_jacobian_simple= DQController::jacobianDual_8d(finger_ik.p.size(), finger_ik.finger_jacobian_simple);	
			q_dot= DQController::calculateControlVel(Kp, mu, screw_error, finger_ik.finger_jacobian_simple, joint_size_coupled);		
		}		
	
		// std::cout << "q_dot: " << q_dot << std::endl;
		// std::cout << "virtual_finger.finger_jacobian_coupled: " << virtual_finger.finger_jacobian_coupled << std::endl;
		q = q + q_dot;
		finger_ik.proximal_phalanx.alpha_cal = finger_ik.q_init(0) + q(0); 
		finger_ik.proximal_phalanx.current_sliderCmd= getSliderFromTheta(finger_ik.proximal_phalanx);
		finger_ik.middle_phalanx.alpha_cal = finger_ik.q_init(1) + q(1); 
		finger_ik.middle_phalanx.current_sliderCmd= getSliderFromTheta(finger_ik.middle_phalanx);

		right_hand_cmds.position[0]= finger_ik.proximal_phalanx.current_sliderCmd;
		right_hand_cmds.position[1]= finger_ik.middle_phalanx.current_sliderCmd;

		do
		{
			// sleep_time.sleep();
			right_ar10Hand_pub.publish(right_hand_cmds);
			sleep_time.sleep();
		}
		while (!right_ar10Hand_pub.getNumSubscribers());

		sleep_time.sleep();
		std::cout << "norm_error: " << norm_error << std::endl;
	// Writing to  csv :result
		error_htm = DQoperations::dq2HTM(pose_error);
		end = ros::Time::now();
		secs = (end-begin).toSec();
		 // std::cout << "secs: " << secs << std::endl;
		result_file << secs << "," << current_htm(1,3) << "," << current_htm(2,3)  << "," << norm_error << "," << (desired_htm(1,3) - current_htm(1,3)) <<  "," << (desired_htm(2,3) - current_htm(2,3)) << ", \n";

		Ar10KinematicsLoop_finger(finger_ik);
	}

	result_file.close();
	// if(virtual_finger.proximal_phalanx.current_sliderCmd < virtual_finger.proximal_phalanx.init_sliderCmd)
	// 	virtual_finger.proximal_phalanx.init_sliderCmd;
	// if(virtual_finger.middle_phalanx.current_sliderCmd < virtual_finger.middle_phalanx.init_sliderCmd)
	// 	virtual_finger.middle_phalanx.init_sliderCmd;	
	hand_ik_res.jointPosition.clear();
	hand_ik_res.jointPosition.push_back(finger_ik.proximal_phalanx.current_sliderCmd);
	hand_ik_res.jointPosition.push_back(finger_ik.middle_phalanx.current_sliderCmd);
	
	if(hand_ik_req.finger == 4)
	{
		index_finger = finger_ik;		
	}	

	if(hand_ik_req.finger == 5)
	{
		thumb = finger_ik;
	}		
	// command the sliders
	// std::cout << "q_result: " << q << std::endl;
	return true;
}


std::vector<int> AR10Kinematics::getSlidersList(std::vector<std::string> joint_names)
{
	std::vector<int> joint_order;
	joint_order.clear();
	// std::cout << "joint_names.size(): " << joint_names.size()  << std::endl;
	// const sensor_msgs::JointStateConstPtr joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/ar10_right/joint_states");
	const sensor_msgs::JointStateConstPtr joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/ar10/right/servo_positions");
	
	for (int j=0; j < joint_names.size(); j++)
	{
		// std::cout << "joint_names[" << j << "]: " << joint_names[j]  << std::endl;
		for (int i=0; i < joint_state->name.size(); i++)
		{
			if (joint_state->name[i].compare(joint_names[j])==0)
		  {
			  joint_order.push_back(i);
				// std::cout << "sliderOrderInjointState: " << i  << std::endl;			  	
		  }
		}
	}
	// std::cout << "joint_order[0]: " << joint_order[0]  << std::endl;
	return joint_order;
}

bool AR10Kinematics::get_thumb_param()
{

	thumb.u.resize(2);
	thumb.p.resize(2);

	std::vector<double> value;	
	value.clear();
	value.resize(3);

	std::string paramName="thumb_right_u0";

	if(!ros::param::get(paramName, value))
	{
		std::cout << "right thumb param: thumb_right_u0 not found" << std::endl;
		return 0;
	}
	else
		thumb.u[0] << value[0], value[1], value[2];

	paramName="thumb_right_u1";
	value.clear();
	value.resize(3);
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right thumb param: thumb_right_u1 not found" << std::endl;
		return 0;
	}
	else
		thumb.u[1] << value[0], value[1], value[2];	

	paramName="thumb_right_p0";
	value.clear();
	value.resize(3);
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right thumb param: thumb_right_p0 not found" << std::endl;
		return 0;
	}
	else
		thumb.p[0] << value[0], value[1], value[2];		

	paramName="thumb_right_p1";
	value.clear();
	value.resize(3);
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right thumb param: thumb_right_p1 not found" << std::endl;
		return 0;
	}
	else
		thumb.p[1] << value[0], value[1], value[2];

	value.clear();
	value.resize(8);
	paramName="thumb_right_frame0";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right thumb param: thumb_right_frame0 not found" << std::endl;
		return 0;
	}
	else
		thumb.frame_rot0ToBase_init << value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7];

	value.clear();
	value.resize(8);
	paramName="thumb_right_frame1";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right thumb param: thumb_right_frame1 not found" << std::endl;
		return 0;
	}
	else
		thumb.frame_rot1ToBase_init << value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7];	

	value.clear();
	value.resize(8);
	paramName="thumb_right_ee_init";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right thumb param: thumb_right_ee_init not found" << std::endl;
		return 0;
	}
	else
		thumb.finger_ee_init << value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7];	

	thumb.finger_joint_type.clear();						
	thumb.finger_joint_type.resize(thumb.joint_size);
	paramName="thumb_right_joint_type";
	if(!ros::param::get(paramName, thumb.finger_joint_type))
	{
		std::cout << "right thumb param: finger_joint_type not found" << std::endl;
		return 0;
	}

	thumb.finger_joint_names.clear();
	thumb.finger_joint_names.resize(thumb.joint_size);
	paramName="thumb_right_joint_names";
	if(!ros::param::get(paramName, thumb.finger_joint_names))
	{
		std::cout << "thumb_right_joint_names param not found" << std::endl;
		return 0;
	}

}

void AR10Kinematics::init_thumb()
{
	// define parameters for all joints FROM THE SERVO FEEDBACK	
	thumb.joint_size=2;
	thumb.is_thumb= true;
	double final_displacement, final_slider_cmd;

	thumb.proximal_phalanx.c=1.186*.01;
	thumb.proximal_phalanx.b=5.236*.01;
	thumb.proximal_phalanx.init_displacement = 4.246*.01;
	final_displacement= 5.960*.01;
	thumb.proximal_phalanx.total_displacement=(final_displacement - thumb.proximal_phalanx.init_displacement);
	final_slider_cmd = 2.170;
	thumb.proximal_phalanx.init_sliderCmd=0.896;	
	thumb.proximal_phalanx.total_sliderCmd=final_slider_cmd - thumb.proximal_phalanx.init_sliderCmd; 

	thumb.middle_phalanx.c=1.2*.01;
	thumb.middle_phalanx.b=4.94*.01;
	thumb.middle_phalanx.init_displacement = 4.148*.01;
	final_displacement= 5.120*.01;
	thumb.middle_phalanx.total_displacement=(final_displacement - thumb.middle_phalanx.init_displacement);
	final_slider_cmd = 1.394;
	thumb.middle_phalanx.init_sliderCmd=0.303;	
	thumb.middle_phalanx.total_sliderCmd=final_slider_cmd - thumb.middle_phalanx.init_sliderCmd; 	
	
	thumb.finger_frame_names.resize(thumb.joint_size+1);
	thumb.finger_frame_names[0]="ar10_right_mod/thumb/middle_phalanx";
	thumb.finger_frame_names[1]="ar10_right_mod/thumb/distal_phalanx";
	thumb.finger_frame_names[2]="ar10_right_mod/thumb/thumbTip";
	
	if(!get_thumb_param())
		ROS_INFO("Thumb params not found.");
	thumb.dq_frames_stack_current.clear();
	thumb.dq_frames_stack_init.clear();
	thumb.dq_frames_stack_init.push_back(thumb.frame_rot0ToBase_init);
	thumb.dq_frames_stack_init.push_back(thumb.frame_rot1ToBase_init);
	thumb.dq_frames_stack_init.push_back(thumb.finger_ee_init);
	thumb.dq_frames_stack_current = thumb.dq_frames_stack_init; 

	thumb.sliderOrderInjointState=	getSlidersList(thumb.finger_joint_names);

	thumb.q_init.resize(thumb.joint_size);
	thumb.q.resize(thumb.joint_size);
	
	getThetaFromSlider(thumb.proximal_phalanx);
	getThetaFromSlider(thumb.middle_phalanx);

	thumb.q_init << thumb.proximal_phalanx.alpha, thumb.middle_phalanx.alpha;
	thumb.q = thumb.q_init;
}

bool AR10Kinematics::get_index_param()
{
	index_finger.joint_size = 3;
	index_finger.u.resize(3);
	index_finger.p.resize(3);

	std::vector<double> value;	
	value.clear();
	value.resize(3);

	std::string paramName="index_right_u0";

	if(!ros::param::get(paramName, value))
	{
		std::cout << "right index param: thumb_right_u0 not found" << std::endl;
		return 0;
	}
	else
		index_finger.u[0] << value[0], value[1], value[2];
	
	paramName="index_right_u1";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right index param: index_right_u1 not found" << std::endl;
		return 0;
	}
	else
		index_finger.u[1] << value[0], value[1], value[2];

	paramName="index_right_u2";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right index param: index_right_u2 not found" << std::endl;
		return 0;
	}
	else
		index_finger.u[2] << value[0], value[1], value[2];

	paramName="index_right_p0";

	if(!ros::param::get(paramName, value))
	{
		std::cout << "right index param: thumb_right_p0 not found" << std::endl;
		return 0;
	}
	else
		index_finger.p[0] << value[0], value[1], value[2];
	
	paramName="index_right_p1";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right index param: index_right_p1 not found" << std::endl;
		return 0;
	}
	else
		index_finger.p[1] << value[0], value[1], value[2];

	paramName="index_right_p2";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right index param: index_right_p2 not found" << std::endl;
		return 0;
	}
	else
		index_finger.p[2] << value[0], value[1], value[2];	

	index_finger.q_init.resize(3);
	index_finger.q.resize(3);
	paramName="index_right_q_init";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right index param: index_right_q_init not found" << std::endl;
		return 0;
	}
	else
		index_finger.q_init << value[0], value[1], value[2];
	index_finger.q = index_finger.q_init;

	value.clear();
	value.resize(8);
	paramName="index_right_frame_0";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right index param: index_right_frame_0 not found" << std::endl;
		return 0;
	}
	else
		index_finger.frame_rot0ToBase_init << value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7];

	value.clear();
	value.resize(8);
	paramName="index_right_frame_1";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right index param: index_right_frame_1 not found" << std::endl;
		return 0;
	}
	else
		index_finger.frame_rot1ToBase_init << value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7];

	value.clear();
	value.resize(8);
	paramName="index_right_frame_2";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right index param: index_right_frame_2 not found" << std::endl;
		return 0;
	}
	else
		index_finger.frame_rot2ToBase_init << value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7];	

	value.clear();
	value.resize(8);
	paramName="index_right_ee_init";
	if(!ros::param::get(paramName, value))
	{
		std::cout << "right index param: index_right_ee_init not found" << std::endl;
		return 0;
	}
	else
		index_finger.finger_ee_init << value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7];		


	index_finger.finger_joint_type.clear();						
	index_finger.finger_joint_type.resize(index_finger.joint_size);
	paramName="index_right_joint_type";
	if(!ros::param::get(paramName, index_finger.finger_joint_type))
	{
		std::cout << "right index param: index_right_joint_type not found" << std::endl;
		return 0;
	}

	index_finger.finger_joint_names.clear();
	index_finger.finger_joint_names.resize(2);
	paramName="index_right_joint_names";
	if(!ros::param::get(paramName, index_finger.finger_joint_names))
	{
		std::cout << "right index param: index_right_joint_names param not found" << std::endl;
		return 0;
	}
		// std::cout << "finger_joint_names[0]: " << index_finger.finger_joint_names[0] << std::endl;
		// std::cout << "finger_joint_names[1]: " << index_finger.finger_joint_names[1] << std::endl;
}
void AR10Kinematics::init_index_finger_2()
{
	index_finger.is_thumb= false;
	index_finger.proximal_phalanx.c=1.3*.01;
	index_finger.proximal_phalanx.b=5.22*.01;
	index_finger.proximal_phalanx.init_displacement=4.2*.01;
	double final_displacement= 6*.01, final_slider_cmd=1.67;
	index_finger.proximal_phalanx.total_displacement=(final_displacement - index_finger.proximal_phalanx.init_displacement);
// FROM THE SERVO FEEDBACK	
	// index_finger.proximal_phalanx.total_sliderCmd=1.536;
 	index_finger.proximal_phalanx.init_sliderCmd=0.134;
	index_finger.proximal_phalanx.total_sliderCmd= final_slider_cmd - index_finger.proximal_phalanx.init_sliderCmd; 
// FROM GUI COMMAND	
	// index_finger.proximal_phalanx.total_sliderCmd=1.37; 
	// index_finger.proximal_phalanx.init_sliderCmd=0.2;	

	index_finger.middle_phalanx.c=1.4*.01;
	index_finger.middle_phalanx.b=4.98*.01;
	index_finger.middle_phalanx.init_displacement=4.15*.01;
	final_displacement= 5.75*.01;
	final_slider_cmd =    1.604;
	index_finger.middle_phalanx.total_displacement=(final_displacement - index_finger.middle_phalanx.init_displacement);

// FROM THE SERVO FEEDBACK	
	// index_finger.middle_phalanx.total_sliderCmd=1.105; 
	index_finger.middle_phalanx.init_sliderCmd = 0.499;
	index_finger.middle_phalanx.total_sliderCmd= final_slider_cmd - index_finger.middle_phalanx.init_sliderCmd;
// FROM GUI COMMAND	
	// index_finger.middle_phalanx.total_sliderCmd=1.03;
	// index_finger.middle_phalanx.init_sliderCmd=0.54;	


	index_finger.distal_phalanx.p = 2.97;
	index_finger.distal_phalanx.qq = 3.47;
	index_finger.distal_phalanx.g = 0.84;
	index_finger.distal_phalanx.h = 0.72;
	index_finger.distal_phalanx.theta_init = 3.32939008;
	index_finger.distal_phalanx.theta=index_finger.distal_phalanx.theta_init;

	if(!get_index_param())
		ROS_INFO("index_finger params not found.");
	index_finger.dq_frames_stack_current.clear();
	index_finger.dq_frames_stack_init.clear();
	index_finger.dq_frames_stack_init.push_back(index_finger.frame_rot0ToBase_init);
	index_finger.dq_frames_stack_init.push_back(index_finger.frame_rot1ToBase_init);
	index_finger.dq_frames_stack_init.push_back(index_finger.frame_rot2ToBase_init);
	index_finger.dq_frames_stack_init.push_back(index_finger.finger_ee_init);
	index_finger.dq_frames_stack_current = index_finger.dq_frames_stack_init; 

	index_finger.sliderOrderInjointState=	getSlidersList(index_finger.finger_joint_names);

	index_finger.finger_frame_names.clear();
	index_finger.finger_frame_names.push_back("ar10_right_mod/indexFinger/planar_rot_0");
	index_finger.finger_frame_names.push_back("ar10_right_mod/indexFinger/planar_rot_1");
	index_finger.finger_frame_names.push_back("ar10_right_mod/indexFinger/planar_rot_2");
	index_finger.finger_frame_names.push_back("ar10_right_mod/indexFinger/fingerTip");
}

void AR10Kinematics::init_index_finger()
{
	index_finger.is_thumb= false;
	index_finger.proximal_phalanx.c=1.3*.01;
	index_finger.proximal_phalanx.b=5.22*.01;
	index_finger.proximal_phalanx.init_displacement=4.2*.01;
	double final_displacement= 6*.01, final_slider_cmd=1.67;
	index_finger.proximal_phalanx.total_displacement=(final_displacement - index_finger.proximal_phalanx.init_displacement);
// FROM THE SERVO FEEDBACK	
	// index_finger.proximal_phalanx.total_sliderCmd=1.536;
 	index_finger.proximal_phalanx.init_sliderCmd=0.134;
	index_finger.proximal_phalanx.total_sliderCmd= final_slider_cmd - index_finger.proximal_phalanx.init_sliderCmd; 
// FROM GUI COMMAND	
	// index_finger.proximal_phalanx.total_sliderCmd=1.37; 
	// index_finger.proximal_phalanx.init_sliderCmd=0.2;	

	index_finger.middle_phalanx.c=1.4*.01;
	index_finger.middle_phalanx.b=4.98*.01;
	index_finger.middle_phalanx.init_displacement=4.15*.01;
	final_displacement= 5.75*.01;
	final_slider_cmd =    1.604;
	index_finger.middle_phalanx.total_displacement=(final_displacement - index_finger.middle_phalanx.init_displacement);

// FROM THE SERVO FEEDBACK	
	// index_finger.middle_phalanx.total_sliderCmd=1.105; 
	index_finger.middle_phalanx.init_sliderCmd = 0.499;
	index_finger.middle_phalanx.total_sliderCmd= final_slider_cmd - index_finger.middle_phalanx.init_sliderCmd;
// FROM GUI COMMAND	
	// index_finger.middle_phalanx.total_sliderCmd=1.03;
	// index_finger.middle_phalanx.init_sliderCmd=0.54;	


	index_finger.distal_phalanx.p = 2.97;
	index_finger.distal_phalanx.qq = 3.47;
	index_finger.distal_phalanx.g = 0.84;
	index_finger.distal_phalanx.h = 0.72;
	index_finger.distal_phalanx.theta_init = 3.32939008;
	index_finger.distal_phalanx.theta=index_finger.distal_phalanx.theta_init;

	index_finger.q_init.resize(3);
	index_finger.q.resize(3);
	index_finger.u.resize(3);
	index_finger.p.resize(3);

	index_finger.q_init << 0.59044489, 0.81437063, 4.1102504;

	index_finger.q=index_finger.q_init;
	
	Matrix4d tf_handBase_circSupport=Matrix4d::Identity(), tf_indexPlanarBase_handBase, tf_h2_h1=Matrix4d::Identity(), tf_h3_h2=Matrix4d::Identity(), tf_h4_h3=Matrix4d::Identity(), tf_h5_h4=Matrix4d::Identity();
	
	tf_handBase_circSupport(2,3)=-0.0021;

	tf_indexPlanarBase_handBase << 0, 0, -1, -0.0565,
																	-1, 0, 0, 0.014,
																	0, 1, 0, 0.08,
																	0, 0, 0, 1;
	
	tf_h2_h1(0,3) = 0.68*.01;
	tf_h2_h1(1,3) = 5.51*.01;
	tf_h2_h1(2,3) = 0*.01;

	tf_h3_h2(0,3) = -0.92*.01;
	tf_h3_h2(1,3) = 4.42*.01;
	tf_h3_h2(2,3) = 0*.01;

	tf_h4_h3(0,3) = -1.14*.01;
	tf_h4_h3(1,3) = 2.75*.01;
	tf_h4_h3(2,3) = 0*.01;

	tf_h5_h4(0,3) = -2.15*.01;
	tf_h5_h4(1,3) = 2.03*.01;
	tf_h5_h4(2,3) = -1*.01;

	// Matrix4d tf_indexPlanarBase_circSupport = tf_handBase_circSupport*tf_indexPlanarBase_handBase;
	Matrix4d tf_indexH2_handBase = tf_indexPlanarBase_handBase*tf_h2_h1;
	// std::cout << "frame_rot0ToBase_init:" << std::endl;
	// std::cout << tf_indexH2_handBase << std::endl;
	index_finger.frame_rot0ToBase_init=DQoperations::htm2DQ(tf_indexH2_handBase);

	Matrix4d tf_indexH3_handBase = tf_indexH2_handBase*tf_h3_h2;
	// std::cout << "frame_rot1ToBase_init:" << std::endl;
	// std::cout << tf_indexH3_handBase << std::endl;
	index_finger.frame_rot1ToBase_init=DQoperations::htm2DQ(tf_indexH3_handBase);	


	Matrix4d tf_indexH4_handBase = tf_indexH3_handBase*tf_h4_h3;
	// std::cout << "frame_rot2ToBase_init:" << std::endl;
	// std::cout << tf_indexH4_handBase << std::endl;	
	index_finger.frame_rot2ToBase_init=DQoperations::htm2DQ(tf_indexH4_handBase);		

	Matrix4d tf_indexH5_handBase = tf_indexH4_handBase*tf_h5_h4;
	// std::cout << "finger_ee_init:" << std::endl;
	// std::cout << tf_indexH5_handBase << std::endl;
	index_finger.finger_ee_init= DQoperations::htm2DQ(tf_indexH5_handBase);

	index_finger.finger_joint_type.resize(3);
	index_finger.finger_joint_type[0]=0;
	index_finger.finger_joint_type[1]=0;
	index_finger.finger_joint_type[2]=0;

	index_finger.p[0] << tf_indexH2_handBase(0,3), tf_indexH2_handBase(1,3), tf_indexH2_handBase(2,3); 
	index_finger.p[1] << tf_indexH3_handBase(0,3), tf_indexH3_handBase(1,3), tf_indexH3_handBase(2,3);
	index_finger.p[2] << tf_indexH4_handBase(0,3), tf_indexH4_handBase(1,3), tf_indexH4_handBase(2,3);

	index_finger.u[0] << tf_indexH2_handBase(0,2), tf_indexH2_handBase(1,2), tf_indexH2_handBase(2,2); 
	index_finger.u[1] << tf_indexH3_handBase(0,2), tf_indexH3_handBase(1,2), tf_indexH3_handBase(2,2);
	index_finger.u[2] << tf_indexH4_handBase(0,2), tf_indexH4_handBase(1,2), tf_indexH4_handBase(2,2);

	index_finger.finger_joint_names.clear();
	index_finger.finger_joint_names.push_back("servo8");
	index_finger.finger_joint_names.push_back("servo9");
	index_finger.sliderOrderInjointState=	getSlidersList(index_finger.finger_joint_names);
	// std::cout << "HELLO_sliderOrderInjointState[0]: " << index_finger.sliderOrderInjointState[0]  << std::endl;
	index_finger.finger_frame_names.clear();
	index_finger.finger_frame_names.push_back("ar10_right_mod/hand_base");
	index_finger.finger_frame_names.push_back("ar10_right_mod/indexFinger/planar_base");
	index_finger.finger_frame_names.push_back("ar10_right_mod/indexFinger/planar_rot_0");
	index_finger.finger_frame_names.push_back("ar10_right_mod/indexFinger/planar_rot_1");
	index_finger.finger_frame_names.push_back("ar10_right_mod/indexFinger/planar_rot_2");
	index_finger.finger_frame_names.push_back("ar10_right_mod/indexFinger/fingerTip");

	index_finger.dq_frames_stack_current.clear();
	index_finger.dq_frames_stack_init.clear();
	index_finger.dq_frames_stack_init.push_back(DQoperations::htm2DQ(tf_handBase_circSupport));
	index_finger.dq_frames_stack_init.push_back(DQoperations::htm2DQ(tf_indexPlanarBase_handBase));
	index_finger.dq_frames_stack_init.push_back(DQoperations::htm2DQ(tf_indexH2_handBase));
	index_finger.dq_frames_stack_init.push_back(DQoperations::htm2DQ(tf_indexH3_handBase));
	index_finger.dq_frames_stack_init.push_back(DQoperations::htm2DQ(tf_indexH4_handBase));
	index_finger.dq_frames_stack_init.push_back(index_finger.finger_ee_init);

	index_finger.dq_frames_stack_current= index_finger.dq_frames_stack_init;
}

void AR10Kinematics::init_fingers()
{
	if(!getControllerParms())
		ROS_WARN("AR10 controller params not found.");

	init_thumb();
	// std::cout << "hello" << std::endl;
	init_index_finger_2();
	// std::cout << "hello2" << std::endl;
}

double AR10Kinematics::getSliderFromTheta(slider_params sp)
{
	double sliderCmd = sqrt(sp.b*sp.b + sp.c*sp.c - 2*sp.b*sp.c*cos(sp.alpha_cal));
	sliderCmd =  (sliderCmd - sp.init_displacement)*(sp.total_sliderCmd/sp.total_displacement) + sp.init_sliderCmd;
	return sliderCmd;
}

void AR10Kinematics::getThetaFromSlider(slider_params &sp)
{
	if (sp.current_sliderCmd < sp.init_sliderCmd)
		sp.current_sliderCmd = sp.init_sliderCmd;

	double current_displacement=(sp.total_displacement/sp.total_sliderCmd)*(sp.current_sliderCmd - sp.init_sliderCmd) + sp.init_displacement;
	double cosTheta = (sp.c*sp.c + sp.b*sp.b - current_displacement*current_displacement)/(2*sp.b*sp.c);
	double theta = acos(cosTheta);
	sp.alpha=theta;
}


void AR10Kinematics::getAlphaFromTheta4Bar(four_bar_params &four_bar)
{
	double A = 2*four_bar.p*four_bar.qq*cos(four_bar.theta) - 2*four_bar.g*four_bar.qq;
	double B = 2*four_bar.p*four_bar.qq*sin(four_bar.theta);
	double C = four_bar.g*four_bar.g + four_bar.qq*four_bar.qq + four_bar.p*four_bar.p - four_bar.h*four_bar.h - 2*four_bar.p*four_bar.g*cos(four_bar.theta);
	four_bar.psi = atan2(B, A) - acos(C/(sqrt(A*A + B*B)));
	four_bar.psi = (four_bar.psi > 0 ? four_bar.psi : (2*PI + four_bar.psi));

	four_bar.alpha = atan2((four_bar.qq*sin(four_bar.psi) - four_bar.p*sin(four_bar.theta)), (four_bar.g + four_bar.qq*cos(four_bar.psi) - four_bar.p*cos(four_bar.theta))) - four_bar.theta;
	four_bar.alpha = (four_bar.alpha > 0 ? four_bar.alpha : (2*PI + four_bar.alpha));	
}

void AR10Kinematics::getSliderPosition(finger_params &finger)
{
	std::cout << "here in getSliderPosition" << std::endl;
	// const sensor_msgs::JointStateConstPtr joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/ar10_right/joint_states");
	const sensor_msgs::JointStateConstPtr joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/ar10/right/servo_positions");
	// std::cout << "finger.sliderOrderInjointState[0]: " << finger.sliderOrderInjointState[0]  << std::endl;
	finger.proximal_phalanx.current_sliderCmd = joint_state->position[finger.sliderOrderInjointState[0]];			
	// std::cout << "finger.proximal_phalanx.current_sliderCmd: " << finger.proximal_phalanx.current_sliderCmd  << std::endl;

	// std::cout << "finger.sliderOrderInjointState[1]: " << finger.sliderOrderInjointState[1]  << std::endl;		
	finger.middle_phalanx.current_sliderCmd = joint_state->position[finger.sliderOrderInjointState[1]];
	// std::cout << "finger.middle_phalanx.current_sliderCmd: " << finger.middle_phalanx.current_sliderCmd  << std::endl;			
}

void AR10Kinematics::computeJointRotation(finger_params &finger)
{

	AR10Kinematics::getThetaFromSlider(finger.proximal_phalanx);	
	finger.q(0)=finger.proximal_phalanx.alpha - finger.q_init(0);

	AR10Kinematics::getThetaFromSlider(finger.middle_phalanx);
	finger.q(1)=finger.middle_phalanx.alpha - finger.q_init(1);
	if(!finger.is_thumb)
	{
		finger.distal_phalanx.theta=finger.distal_phalanx.theta_init+finger.q(1);
		AR10Kinematics::getAlphaFromTheta4Bar(finger.distal_phalanx);	
		finger.q(2)=finger.distal_phalanx.alpha - finger.q_init(2);
		// std::cout << "finger.q: " << finger.q  << std::endl;					
	}
	// else
	// 	std::cout << "finger.q(1): " << finger.q(1)  << std::endl;		
}

void AR10Kinematics::fkmFinger(finger_params &finger)
{
	finger.screwDispalcementArray.resize(finger.q.size());
	finger.dq_frames_stack_current.resize(finger.q.size()+1);
	// if(finger.is_thumb)
	// {
	// 	std::cout << "finger.u.size(): " << finger.u.size() << std::endl;		
	// 	std::cout << "finger.p.size(): " << finger.p.size() << std::endl;		
	// 	std::cout << "finger.q.size(): " << finger.q.size() << std::endl;		
	// 	std::cout << "finger.finger_joint_type.size(): " << finger.finger_joint_type.size() << std::endl;		
	// }	
  finger.screwDispalcementArray= DQController::fkmDual(finger.u, finger.p, finger.q /*q is defined as [q1 q2 ...]*/, finger.finger_joint_type);

	finger.frame_rot0ToBase_current=finger.screwDispalcementArray[0];
  finger.frame_rot0ToBase_current=DQoperations::mulDQ(finger.frame_rot0ToBase_current, finger.frame_rot0ToBase_init);
  finger.dq_frames_stack_current[0] = finger.frame_rot0ToBase_current;
	
	finger.frame_rot1ToBase_current=finger.screwDispalcementArray[1];
  finger.frame_rot1ToBase_current=DQoperations::mulDQ(finger.frame_rot1ToBase_current, finger.frame_rot1ToBase_init);
	finger.dq_frames_stack_current[1] = finger.frame_rot1ToBase_current;

	if(!finger.is_thumb)
	{
		// std::cout << "finger.screwDispalcementArray[2]:" << std::endl;
		// std::cout << DQoperations::dq2HTM(finger.screwDispalcementArray[2]) << std::endl;
		
		finger.frame_rot2ToBase_current=finger.screwDispalcementArray[2];
	  finger.frame_rot2ToBase_current=DQoperations::mulDQ(finger.frame_rot2ToBase_current, finger.frame_rot2ToBase_init);
		finger.dq_frames_stack_current[2]= finger.frame_rot2ToBase_current;  

	  finger.finger_ee_current=finger.screwDispalcementArray[2];
	  finger.finger_ee_current=DQoperations::mulDQ(finger.finger_ee_current, finger.finger_ee_init);
		finger.dq_frames_stack_current[3]= finger.finger_ee_current;  		
	}	
	else
	{
	  finger.finger_ee_current=finger.screwDispalcementArray[1];
	  finger.finger_ee_current=DQoperations::mulDQ(finger.finger_ee_current, finger.finger_ee_init);
	  finger.dq_frames_stack_current[2] = finger.finger_ee_current;
	}

	// std::cout << "ee_dq" << std::endl;
	// std::cout << finger.finger_ee_current.transpose() << std::endl;
}


void AR10Kinematics::Ar10KinematicsLoop_finger(finger_params &finger)
{
	ros::Rate r(10); 	

	static_transformStamped_array.clear();
	getSliderPosition(finger);
	computeJointRotation(finger);
	fkmFinger(finger)	;
	publishFramesAsTf(finger, static_transformStamped_array);	

	static_broadcaster.sendTransform(static_transformStamped_array);
  ros::spinOnce();
  r.sleep();
}

bool AR10Kinematics::updateFingerVariables(std::string finger_name)
{
	// std::cout << "Here 16" << std::endl;
	Ar10KinematicsLoop();
	// std::cout << "Here 17" << std::endl;
	if(!finger_name.compare("index_right"))
	{
		// std::cout << "Here 18" << std::endl;
		// std::cout << "ar10_here_0" << std::endl;
		getJacobianCoupled(index_finger);
		return 1;
	}
	else if(!finger_name.compare("thumb_right"))
	{
			// std::cout << "ar10_here_2" << std::endl;
		thumb.finger_jacobian_simple= DQController::jacobianDual(thumb.u, thumb.p, thumb.finger_ee_init, thumb.finger_joint_type, thumb.screwDispalcementArray);
		thumb.finger_jacobian_simple= DQController::jacobianDual_8d(thumb.p.size(), thumb.finger_jacobian_simple);
		return 1;	
	}
	else return 0;
}

bool AR10Kinematics::importHandState(std::string finger_name, Matrix<double,8,1>& pe_init, Matrix<double,8,1>& pe_now, MatrixXd& jacobian, RowVectorXd& q, RowVectorXd& q_init)
{
	if(!finger_name.compare("index_right"))
	{
		pe_init = index_finger.finger_ee_init;
		pe_now = index_finger.finger_ee_current;
		q = index_finger.q;
		q_init = index_finger.q_init;
		jacobian = index_finger.finger_jacobian_coupled;
		return 1;	
	}
	if(!finger_name.compare("thumb_right"))
	{
		pe_init = thumb.finger_ee_init;
		pe_now = thumb.finger_ee_current;
		q = thumb.q;
		q_init = thumb.q_init;
		jacobian = thumb.finger_jacobian_simple;
		return 1;	
	}
	else return 0;
}

void AR10Kinematics::Ar10KinematicsLoop()
{

	Ar10KinematicsLoop_finger(index_finger);
	Ar10KinematicsLoop_finger(thumb);


	// static_transformStamped_array.clear();	
	// getSliderPosition(thumb);
	// computeJointRotation(thumb);
	// fkmFinger(thumb);
	// publishFramesAsTf(thumb, static_transformStamped_array);

	// static_broadcaster.sendTransform(static_transformStamped_array);
 //  ros::spinOnce();
 //  r.sleep(); 
}


void AR10Kinematics::publishFramesAsTf(finger_params &finger, std::vector<geometry_msgs::TransformStamped> &static_transformStamped_array)
{
	
	static_transformStamped_array.push_back(hand_base_static_transformStamped);	


	// static_transformStamped_array.clear();
	geometry_msgs::TransformStamped static_transformStamped;
	Matrix4d mat=Matrix4d::Identity();
	///////////////////////////////////////////////////////////////////////////////////////////////////
	//Make an isolated transform for hand_base wrt circuit_support
	

	///////////////////////////////////////////////////////////////////////////////////////////////////
  static_transformStamped.header.frame_id = "ar10_right_mod/hand_base";  
  for(int i=0; i<(finger.finger_frame_names.size()); i++)
  {
  	// std::cout << "is_thumb: " << finger.is_thumb << std::endl;
  	// std::cout << "child_frame_id[" << i << "]:  " << finger.finger_frame_names[i] << std::endl;
		static_transformStamped.child_frame_id= finger.finger_frame_names[i];
		mat= DQoperations::dq2HTM(finger.dq_frames_stack_current[i]);
		static_transformStamped.transform.translation.x=mat(0,3);
		static_transformStamped.transform.translation.y=mat(1,3);
		static_transformStamped.transform.translation.z=mat(2,3);
	  static_transformStamped.transform.rotation.x = finger.dq_frames_stack_current[i](1);
	  static_transformStamped.transform.rotation.y = finger.dq_frames_stack_current[i](2);
	  static_transformStamped.transform.rotation.z = finger.dq_frames_stack_current[i](3);
	  static_transformStamped.transform.rotation.w = finger.dq_frames_stack_current[i](0);		
	  static_transformStamped_array.push_back(static_transformStamped);
  }
  // std::cout << "ee_mat:" << std::endl;
  // std::cout << mat << std::endl;  

}

void AR10Kinematics::init_AR10IKService()
{
	finger_ik_service=n.advertiseService("/ar10_right/finger_ik_sever", &AR10Kinematics::computeHandIKCallback, this);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ar10_kinematics");
	ros::NodeHandle nh;
	AR10Kinematics*  ar10= new AR10Kinematics();
	ar10->init_AR10Kinematics();
	ros::Rate loop_rate(100);
	ar10->init_fingers();
	ar10->init_AR10IKService();
	while (ros::ok())
	{
		ar10->Ar10KinematicsLoop();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}