#include <dq_robotics/baxter_dq.h>

using namespace Eigen;

ManipulatorModDQ::ManipulatorModDQ(){}
ManipulatorModDQ::~ManipulatorModDQ(){}


bool ManipulatorModDQ::initialize_baxter()
{
	bool initialized=false;
	if(ManipulatorModDQ::getRobotParams_baxter())
		if(ManipulatorModDQ::loadModel_baxter())
			initialized=true;
		else
			ROS_ERROR("Robot model not loaded");
	else
		ROS_INFO("Robot params not loaded");
	
	ROS_INFO("Robot params loaded");
	left_cmd_pub = rh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 10);
	right_cmd_pub = rh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 10);
	
	ManipulatorModDQ::mapJointName2JointCmds();
	cmd_left.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
	cmd_right.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;

	// const sensor_msgs::JointStateConstPtr jointState = ros::topic::waitForMessage<sensor_msgs::JointState>("/robot/joint_states");
	// current_time=jointState->header.stamp;
	std::cout << "joint_size_baxter: " << joint_size_baxter << std::endl;
	q_baxter= RowVectorXd::Zero(joint_size_baxter);
	q_vel_baxter= RowVectorXd::Zero(joint_size_baxter);
	// std::cout << "q:" << std::endl;
	// for (int i=0; i<jointState->name.size(); i++)
	// {
	// 	for(int j=0; j< joint_size_baxter; j++)
	// 	{
	// 		if (!jointState->name[i].compare(joint_names_baxter[j]) )
	// 		{
	// 			q_baxter[j]=jointState->position[i];		
	// 			// std::cout << q[j] << std::endl;		
	// 			q_vel_baxter[j]=jointState->velocity[i];				
	// 		}
	// 	}	
	// }
	// q_baxter= RowVectorXd::Zero(joint_size_baxter);
	// q_vel_baxter= RowVectorXd::Zero(joint_size_baxter);
	joint_state_sub= rh.subscribe("/robot/joint_states", 1, &ManipulatorModDQ::joint_state_callback, this, ros::TransportHints().tcpNoDelay(true)); 	
	return initialized;
}


void ManipulatorModDQ::joint_state_callback(const sensor_msgs::JointState& jointStateConsPtr)
{
	// std::cout << "did it come here for call back after spinOnce!" << std::endl;
	current_time=jointStateConsPtr.header.stamp;
	q_baxter= RowVectorXd::Zero(joint_size_baxter);
	q_vel_baxter= RowVectorXd::Zero(joint_size_baxter);
	// std::cout << "q:" << std::endl;
	for (int i=0; i<jointStateConsPtr.name.size(); i++)
	{
		for(int j=0; j< joint_size_baxter; j++)
		{
			if (!jointStateConsPtr.name[i].compare(joint_names_baxter[j]) )
			{
				q_baxter[j]=jointStateConsPtr.position[i];		
				// std::cout << q[j] << std::endl;		
				q_vel_baxter[j]=jointStateConsPtr.velocity[i];				
			}
		}
	}
}

void ManipulatorModDQ::mapJointName2JointCmds()
{
	cmd_right.names.clear();
	cmd_left.names.clear();

	cmd_right.names.push_back("right_s0");
	cmd_right.names.push_back("right_s1");
	cmd_right.names.push_back("right_e0");
	cmd_right.names.push_back("right_e1");
	cmd_right.names.push_back("right_w0");
	cmd_right.names.push_back("right_w1");
	cmd_right.names.push_back("right_w2");

	cmd_left.names.push_back("left_s0");
	cmd_left.names.push_back("left_s1");
	cmd_left.names.push_back("left_e0");
	cmd_left.names.push_back("left_e1");
	cmd_left.names.push_back("left_w0");
	cmd_left.names.push_back("left_w1");
	cmd_left.names.push_back("left_w2");

	cmd_right.command.resize(cmd_right.names.size());
	cmd_left.command.resize(cmd_left.names.size()); 
}

bool ManipulatorModDQ::getRobotParams_baxter()
{
	joint_size_baxter = 14;
	
	std::string paramName="tip_name_left";
	
	if(!ros::param::get(paramName, tip_name_left))
	{
		std::cout << "tip_name_left param not found" << std::endl;
		return 0;
	}
	paramName="tip_name_right";
	if(!ros::param::get(paramName, tip_name_right))
	{
		std::cout << "tip_name_right param not found" << std::endl;
		return 0;
	}

	paramName="root_name";
	if(!ros::param::get(paramName, root_name))
	{
		std::cout << "root_name param not found" << std::endl;
		return 0;
	}

	paramName="joint_type" ;
	joint_type_baxter.resize(joint_size_baxter);
	if(!ros::param::get(paramName, joint_type_baxter))
	{
		std::cout << "joint_type param not found" << std::endl;
		return 0;
	}

	paramName="fraction_jointLimit";
	if(!ros::param::get(paramName, fraction_jointLimit))
	{
		std::cout << "fraction_jointLimit param not found" << std::endl;
		return 0;
	}


	RowVector3d u_i, p_i;
	p_baxter.clear();
	u_baxter.clear();

	std::string pName="p";
	std::string uName="u";
	std::vector<double> value;	
	value.clear();
	value.resize(3);

	for(int i=0;i<joint_size_baxter;i++)
	{
		paramName=pName + std::to_string(i);
		value.clear();
		value.resize(3);
		if(ros::param::get(paramName, value))
		{
			p_i << value[0], value[1], value[2] ;
			p_baxter.push_back(p_i);
		}	
		else
		{
			std::cout << "p_%d param not found " << i << std::endl;
			return 0;
		}

		paramName=uName + std::to_string(i);
		value.clear();
		value.resize(3);
		if(ros::param::get(paramName, value))
		{
			u_i << value[0], value[1], value[2] ;
			u_baxter.push_back(u_i);
		}	
		else
		{
			std::cout << "u_%d param not found " << i << std::endl;
			return 0;
		}
	}


	paramName="joint_names";
	joint_names_baxter.resize(joint_size_baxter);
	if(!ros::param::get(paramName, joint_names_baxter))
	{
		std::cout << "joint_names param not found" << std::endl;
		return 0;
	}	

	paramName="pe_init_left";
	value.clear();
	value.resize(8);
	if(ros::param::get(paramName, value))
	{
			pe_init_left <<value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7]; 
	}
	else
	{
		std::cout << "pe_init_left param not found" << std::endl;
		return 0;
	}
	
	paramName="pe_init_right";
	value.clear();
	value.resize(8);
	if(ros::param::get(paramName, value))
	{
			pe_init_right <<value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7]; 
	}
	else
	{
		std::cout << "pe_init_right param not found" << std::endl;
		return 0;
	}

	if (u_baxter.size()!=joint_size_baxter || p_baxter.size()!=joint_size_baxter)
	{
		std::cerr << "Size mismatch error for u and p." << std::endl;
		std::cerr << "u_size"<< u.size()<< std::endl;
		std::cerr << "p_size"<< p.size()<< std::endl;
		std::cerr << "joint_size_baxter"<< joint_size_baxter<< std::endl;
		return 0;
	}
	ROS_INFO("Manipulators parameter loaded");
	return 1;
}

bool ManipulatorModDQ::loadModel_baxter()
{
    urdf::Model robot_model;
	std::vector<std::string> joint_names_new; joint_names_new.clear();
	std::string urdf_xml, full_urdf_xml;

	velocity_limit.clear();
	velocity_limit_ordered_baxter.clear();
	joint_low_limit.clear();
	joint_high_limit.clear();
	joint_low_limit_ordered_baxter.clear();
	joint_high_limit_ordered_baxter.clear();
	joint_mid_point_baxter.clear();
	max_safe_baxter.resize(joint_size_baxter);
	min_safe_baxter.resize(joint_size_baxter);
	joint_mid_point_baxter.resize(joint_size_baxter);
    
    // fraction_jointLimit=0.1;

    rh.param("urdf_xml",urdf_xml,std::string("robot_description"));
    rh.searchParam(urdf_xml,full_urdf_xml);
	
    if (!rh.getParam(full_urdf_xml, xml)) {
        std::cout << "Could not load the xml from parameter server:" << urdf_xml << std::endl;
        return false;
    }
    if (!robot_model.initString(xml)) {
        ROS_WARN("Could not initialize robot model");
        return -1;
    }

    if (!ManipulatorModDQ::readJoints_baxter(tip_name_left, joint_names_new, robot_model)) {
        ROS_WARN("Could not read information about the joints");
        return false;
    }
    if (!ManipulatorModDQ::readJoints_baxter(tip_name_right, joint_names_new, robot_model)) {
        ROS_WARN("Could not read information about the joints");
        return false;
    }

	for (int i=0; i<joint_names_baxter.size(); i++)
	{
		// std::cout<< "joint_name[" << i << "]:"<< joint_names[i] << std::endl;
		for(int j=0; j< joint_names_new.size(); j++)
		{
			if(joint_names_baxter[i].compare(joint_names_new[j]) == 0)
			{
				velocity_limit_ordered_baxter.push_back(velocity_limit[j]);
				joint_low_limit_ordered_baxter.push_back(joint_low_limit[j]);
				joint_high_limit_ordered_baxter.push_back(joint_high_limit[j]);
				std::cout << "joint limit for " << joint_names_baxter[i] << ": upper: " <<  joint_high_limit_ordered_baxter[i] << ": lower: " <<  joint_low_limit_ordered_baxter[i] << std::endl;
				std::cout << "velocity limit for " << joint_names_baxter[i] << ": " << velocity_limit_ordered_baxter[i] << std::endl;
			}
		}	
	}
	for(int i=0; i <joint_size_baxter; i++)
	{
		max_safe_baxter[i]=joint_high_limit_ordered_baxter[i]- (joint_high_limit_ordered_baxter[i]- joint_low_limit_ordered_baxter[i])*(fraction_jointLimit);
		min_safe_baxter[i]=joint_low_limit_ordered_baxter[i] + (joint_high_limit_ordered_baxter[i]- joint_low_limit_ordered_baxter[i])*(fraction_jointLimit);
		joint_mid_point_baxter[i]=(max_safe_baxter[i]+min_safe_baxter[i])/2;
		std::cout << "joint_" << i << " ::max_safe: " << max_safe_baxter[i] << "::min_safe: " << min_safe_baxter[i] << "::joint_mid_point: " << joint_mid_point_baxter[i] << std::endl;
	}    
    return true;
}


bool ManipulatorModDQ::readJoints_baxter(std::string tip_name, std::vector<std::string>  &joint_names_new, urdf::Model &robot_model) 
{
	boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
	boost::shared_ptr<const urdf::Joint> joint;
	int num_joints=0;
	int joint_index=0;

	while (link && link->name != root_name) 
	{
		joint = robot_model.getJoint(link->parent_joint->name);
		if (!joint) 
		{
			std::cout << "Could not find joint: " << link->parent_joint->name  << std::endl;
			return false;
		}
		if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) 
		{
			num_joints++;
		}
		link = robot_model.getLink(link->getParent()->name);
	}
	link = robot_model.getLink(tip_name);

	while (link && (joint_index < num_joints)) 
	{
		joint = robot_model.getJoint(link->parent_joint->name);
		if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) 
		{
			// ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
			joint_names_new.push_back(joint->name);
			float lower, upper;
			int hasLimits;
			if ( joint->type != urdf::Joint::CONTINUOUS ) 
			{
				velocity_limit.push_back(joint->limits->velocity);
				lower = joint->limits->lower;
				upper = joint->limits->upper;
				joint_low_limit.push_back(lower);
				joint_high_limit.push_back(upper);
				hasLimits = 1;
			} 
			else 
			{
				lower = -M_PI;
				upper = M_PI;
				hasLimits = 0;
			}
			joint_index++;
		}
		link = robot_model.getLink(link->getParent()->name);
	}

	return true;
}

void ManipulatorModDQ::getCurrentJointState_baxter()
{
	// std::cout << "spinning for joint_state_callback" << std::endl;
	// ros::WallTime waitForJointState_time_start = ros::WallTime::now();
	// ros::Rate r(100);
	// r.sleep();
	ros::spinOnce();
	// r.sleep();
	// ros::Rate r(100);
	// std::cout << "Here 13" << std::endl;

	// const sensor_msgs::JointStateConstPtr jointState = ros::topic::waitForMessage<sensor_msgs::JointState>("/robot/joint_states");
	// current_time=jointState->header.stamp;
	// ros::WallTime waitForJointState_time_end = ros::WallTime::now();
	// ros::WallDuration waitForJointState_time_dur = waitForJointState_time_end  - waitForJointState_time_start;
	// std::cout << "waitForJointState duration_server: " << waitForJointState_time_dur.toSec() << std::endl; 
	// std::cout << "joint_size_baxter: " << joint_size_baxter << std::endl;
	// q_baxter= RowVectorXd::Zero(joint_size_baxter);
	// q_vel_baxter= RowVectorXd::Zero(joint_size_baxter);
	// // std::cout << "q:" << std::endl;
	// for (int i=0; i<jointState->name.size(); i++)
	// {
	// 	for(int j=0; j< joint_size_baxter; j++)
	// 	{
	// 		if (!jointState->name[i].compare(joint_names_baxter[j]) )
	// 		{
	// 			q_baxter[j]=jointState->position[i];		
	// 			// std::cout << q[j] << std::endl;		
	// 			q_vel_baxter[j]=jointState->velocity[i];				
	// 		}
	// 	}
	// }
}

void ManipulatorModDQ::currentRobotState(ros::Time &current_time_, RowVectorXd &q_, RowVectorXd &q_vel_)
{
	current_time_= this->current_time;
	q_=this->q_baxter;
	q_vel_=this->q_vel_baxter;
}

void ManipulatorModDQ::robotParams(std::vector<RowVector3d> &u_, std::vector<RowVector3d> &p_, Matrix<double,8,1> &pe_init_left_, Matrix<double,8,1> &pe_init_right_, std::vector<double> &joint_high_limit_, std::vector<double> &joint_low_limit_, std::vector<double> &velocity_limit_, std::vector<double> &max_safe_, std::vector<double> &min_safe_, std::vector<std::string> &joint_names_, std::vector<int> &joint_type_)
{	
	u_=this->u_baxter;
	p_=this->p_baxter;
	pe_init_left_= this->pe_init_left;
	pe_init_right_= this->pe_init_right;
	joint_high_limit_=this->joint_high_limit_ordered_baxter;
	joint_low_limit_=this->joint_low_limit_ordered_baxter;
	velocity_limit_=this->velocity_limit_ordered_baxter;
	max_safe_=this->max_safe_baxter;
	min_safe_=this->min_safe_baxter;
	joint_names_=this->joint_names_baxter;
	joint_type_=this->joint_type_baxter;
}

void ManipulatorModDQ::sendJointCommandBaxter(std::vector<double> jointCmds_, int mode, double sleepTimeout_, bool velocity_control)
{
	sleepTimeout=sleepTimeout_;
	// ROS_INFO("Hi sendCmd 1");
	cmd_right.command.clear();
	cmd_left.command.clear();

	for(int i = 0; i < 7; i++)
	{
		// ROS_INFO("Hi sendCmd 2, i=%d", i);
		cmd_right.command.push_back(jointCmds_[i]);
		cmd_left.command.push_back(jointCmds_[7+i]);
	}
	// std::cout << "jointCmds: " << std::endl;
	// std::cout << "jointCmds_for_mode_" << mode << ": "<< std::endl;
	// for(int i = 0; i < 14; i++)
	// {
	// 	std::cout << jointCmds_[i] << ", ";
	// 	// std::cout << "cmd_left.command[" << i << "]: " << cmd_left.command[i] << std::endl;
	// }
	std::cout << std::endl;	    	
	if (left_cmd_pub.getNumSubscribers() && right_cmd_pub.getNumSubscribers())	
	{
		if(!velocity_control)
		{
			cmd_left.mode = baxter_core_msgs::JointCommand::RAW_POSITION_MODE;
			cmd_right.mode = baxter_core_msgs::JointCommand::RAW_POSITION_MODE;
		}
		else
		{
			cmd_left.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
			cmd_right.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;	
		}
		if (mode==1 || mode==3 || mode==4|| mode==5)
		    right_cmd_pub.publish(cmd_right);
		if (mode==2 || mode==3 || mode==4|| mode==5)	    
		    left_cmd_pub.publish(cmd_left);
	    // ROS_INFO("Hi sendCmd 3");
	    ros::Duration(sleepTimeout).sleep();		
	    // ROS_INFO("Cmd Sent");
	} 
	else
	{
		ROS_INFO("subscriber not found");
	}  
}


// void ManipulatorModDQ::fkmDual()
// {
// 	q_dual=RowVectorXd::Zero(joint_size*2);
// 	for (int j=0; j< joint_size; j++)
// 	{
// 		if(joint_type[j]==0)
// 			q_dual(2*j)=q(j);
// 		else
// 			q_dual(2*j+1)=q(j);
// 	}
		
// 	fkm_current.clear();
// 	fkm_current.resize(joint_size);

// 	for (int i=0;i<joint_size; i++)
// 	{
// 		double theta_i=q_dual[2*i];
// 		RowVector3d u_i=u[i];
// 		RowVector3d p_i=p[i];
// 		double d_i=q_dual[2*i+1];
// 		Matrix<double,8,1> screwDispalcementArray_i;
// 		screwDispalcementArray_i= DQoperations::screw2DQ(theta_i, u_i, d_i, p_i.cross(u_i));
// 		if (i==0)
// 			fkm_current[i]=screwDispalcementArray_i;
// 		else
// 			fkm_current[i]=DQoperations::mulDQ(fkm_current[i-1],screwDispalcementArray_i);
// 	}

// 	pe_now=fkm_current[joint_size-1];
//     pe_now=DQoperations::mulDQ(pe_now, pe_init);
// }

// void ManipulatorModDQ::jacobianDual()
// {
// 	jacobian=MatrixXd::Zero(6,joint_size);
	

// 	Matrix<double,8,1> screwDispalcementArray_i, screw_axis_i, pose_i;
	
// 	screwDispalcementArray_i= fkm_current[joint_size-1];/*The numbering in C++ starts at 0*/
	
// 	screwDispalcementArray_i=DQoperations::mulDQ(DQoperations::mulDQ(screwDispalcementArray_i, pe_init), DQoperations::combinedConjDQ(screwDispalcementArray_i));
	
// 	RowVector3d pose_ee_now;

// 	pose_ee_now << screwDispalcementArray_i(5), screwDispalcementArray_i(6), screwDispalcementArray_i(7); 

// 	if(joint_type[0]==0)
// 	{
// 		jacobian.col(0)<< (p[0].cross(u[0])).transpose(), u[0].transpose();/*writing Jacobian seperately for first joint for faster operation*/
// 	}
// 	else
// 	{
// 		jacobian.col(0)<< u[0].transpose(), 0, 0, 0;
		
// 	}

// 	for(int i=1; i<joint_size; i++)
// 	{

// 		screwDispalcementArray_i=fkm_current[i-1];
	
// 		screw_axis_i<< 0, u[i].transpose(), 0, (p[i].cross(u[i])).transpose();
	
// 		screw_axis_i=DQoperations::mulDQ(DQoperations::mulDQ(screwDispalcementArray_i, screw_axis_i), DQoperations::classicConjDQ(screwDispalcementArray_i));	
	
// 		RowVector3d u_i, p_i;
// 		u_i << screw_axis_i(1), screw_axis_i(2), screw_axis_i(3);
// 		pose_i << 1, 0, 0, 0, 0, p[i].transpose();
// 		pose_i= DQoperations::mulDQ(DQoperations::mulDQ(screwDispalcementArray_i, pose_i), DQoperations::combinedConjDQ(screwDispalcementArray_i));
// 		p_i << pose_i(5), pose_i(6), pose_i(7);
// 		if(joint_type[i]==0)
// 		{
// 			jacobian.col(i) << (p_i.cross(u_i)).transpose(), u_i.transpose(); 
// 		}
// 		else
// 			jacobian.col(i) << u_i.transpose(), 0, 0, 0;
// 	}
// 	ManipulatorModDQ::jacobianDual_8d();
// }

// void ManipulatorModDQ::jacobianDual_8d()
// {
// 	jacobian_8d=MatrixXd::Zero(8,joint_size);
//     jacobian_8d.row(1)=jacobian.row(0);
// 	jacobian_8d.row(2)=jacobian.row(1);
// 	jacobian_8d.row(3)=jacobian.row(2);
// 	jacobian_8d.row(5)=jacobian.row(3);
// 	jacobian_8d.row(6)=jacobian.row(4);
// 	jacobian_8d.row(7)=jacobian.row(5);
// }


// void ManipulatorModDQ::getDQderivative()
// {
// 	RowVector4d rot_q_from_dq, dual_part_dq, trans_q_from_dq, dq_dot_dual, rot_q_dot, v, w;
// 	RowVectorXd velocity;

// 	velocity=jacobian*q_vel.transpose();

// 	v << 0, velocity(0), velocity(1), velocity(2);
// 	w << 0, velocity(3), velocity(4), velocity(5);

// 	rot_q_from_dq << pe_now(0), pe_now(1), pe_now(2), pe_now(3);
// 	dual_part_dq << pe_now(4), pe_now(5), pe_now(6), pe_now(7);

// 	trans_q_from_dq = 2*DQoperations::multQuat(dual_part_dq, DQoperations::conjQuat(rot_q_from_dq));

// 	rot_q_dot=0.5*(DQoperations::multQuat(w, rot_q_from_dq));

// 	dq_dot_dual=0.5*(DQoperations::multQuat(v, rot_q_from_dq) + DQoperations::multQuat(trans_q_from_dq, rot_q_dot));

// 	pe_now_dot_dq << rot_q_dot(0), rot_q_dot(1), rot_q_dot(2), rot_q_dot(3), dq_dot_dual(0), dq_dot_dual(1), dq_dot_dual(2), dq_dot_dual(3);
// }

// void ManipulatorModDQ::initialize_leftArm()
// {
// 	p_left.clear();
// 	u_left.clear();
// 	joint_type_left.clear();
// 	joint_names_left.clear();
// 	velocity_limit_ordered_left.clear();
// 	joint_size_left=7;
// 	joint_low_limit_ordered_left.clear();
// 	joint_high_limit_ordered_left.clear();
// 	max_safe_left.clear();
// 	min_safe_left.clear();
// 	joint_mid_point_left.clear();

// 	for (int i=0; i<joint_size_left; i++ )
// 	{
// 		p_left.push_back(p_baxter[7+i]);
// 		u_left.push_back(u_baxter[7+i]);
// 		joint_type_left.push_back(joint_type_baxter[7+i]);		
// 		joint_names_left.push_back(joint_names_baxter[7+i]);		
// 		velocity_limit_ordered_left.push_back(velocity_limit_ordered_baxter[7+i]);
// 		joint_low_limit_ordered_left.push_back(joint_low_limit_ordered_baxter[7+i]);
// 		joint_high_limit_ordered_left.push_back(joint_high_limit_ordered_baxter[7+i]);
// 		max_safe_left.push_back(max_safe_baxter[7+i]);
// 		min_safe_left.push_back(min_safe_baxter[7+i]);
// 		joint_mid_point_left.push_back(joint_mid_point_baxter[7+i]);
// 	}
// }

// void ManipulatorModDQ::initialize_rightArm()
// {
// 	p_right.clear();
// 	u_right.clear();
// 	joint_type_right.clear();
// 	joint_names_right.clear();
// 	velocity_limit_ordered_right.clear();
// 	joint_size_right=7;
// 	joint_low_limit_ordered_right.clear();
// 	joint_high_limit_ordered_right.clear();
// 	max_safe_right.clear();
// 	min_safe_right.clear();
// 	joint_mid_point_right.clear();

// 	for (int i=0; i<joint_size_right; i++ )
// 	{
// 		p_right.push_back(p_baxter[i]);
// 		u_right.push_back(u_baxter[i]);
// 		joint_type_right.push_back(joint_type_baxter[i]);		
// 		joint_names_right.push_back(joint_names_baxter[i]);		
// 		velocity_limit_ordered_right.push_back(velocity_limit_ordered_baxter[i]);
// 		joint_low_limit_ordered_right.push_back(joint_low_limit_ordered_baxter[i]);
// 		joint_high_limit_ordered_right.push_back(joint_high_limit_ordered_baxter[i]);
// 		max_safe_right.push_back(max_safe_baxter[i]);
// 		min_safe_right.push_back(min_safe_baxter[i]);
// 		joint_mid_point_right.push_back(joint_mid_point_baxter[i]);
// 	}

// }

// void ManipulatorModDQ::robot_state_right()
// {
// 	u=u_right;
// 	p=p_right;
// 	joint_type=joint_type_right;
// 	joint_names=joint_names_right;
// 	velocity_limit_ordered=velocity_limit_ordered_right;
// 	joint_size=joint_size_right;
// 	joint_low_limit_ordered=joint_low_limit_ordered_right;
// 	joint_high_limit_ordered=joint_high_limit_ordered_right;
// 	max_safe=max_safe_right;
// 	min_safe=min_safe_right;
// 	joint_mid_point=joint_mid_point_right;	
// 	q= q_baxter.head(joint_size_left);
// 	q_vel= q_vel_baxter.head(joint_size_left);
// }

// void ManipulatorModDQ::robot_state_left()
// {
// 	u=u_left;
// 	p=p_left;
// 	joint_type=joint_type_left;
// 	joint_names=joint_names_left;
// 	velocity_limit_ordered=velocity_limit_ordered_left;
// 	joint_size=joint_size_left;
// 	joint_low_limit_ordered=joint_low_limit_ordered_left;
// 	joint_high_limit_ordered=joint_high_limit_ordered_left;
// 	max_safe=max_safe_left;
// 	min_safe=min_safe_left;
// 	joint_mid_point=joint_mid_point_left;	
// 	q= q_baxter.tail(joint_size_left);
// 	q_vel= q_vel_baxter.tail(joint_size_left);
// }