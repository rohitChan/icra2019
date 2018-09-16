#include <dq_robotics/baxter_poseControl_server.h>
#include <dq_robotics/ar10_kinematics.h>
#include <dq_robotics/Grasp.h>
// #include <baxter_trajectory_interface/baxter_poseControl_server.h>
#define PI 3.14159265

double ar10_timeStep, mu, Kp, K_jl, Kp_position, K_jl_position, sleepTimeout, dt, timeStep, norm_error_const, ar10_Kp, ar10_dt;
Matrix<double,8,1> pe_init_baxterRight, pe_now_baxterRight, pe_init_thumbRight, pe_init_indexRight,  pe_now_thumbRight, pe_now_indexRight, pe_desired_thumb, pe_desired_index, tf_rightHand2handBase;
MatrixXd jacobian_baxterRight, jacobian_thumbRight, jacobian_indexRight;
RowVectorXd q_baxterRight, q_thumbRight, q_indexRight, q_init_thumbRight, q_init_indexRight;
std::vector<double> joint_velocity_limit_baxterRight;
BaxterPoseControlServer* baxter_controller;
AR10Kinematics*  ar10;
Matrix<double,8,1> tfRight_baxterBase2handBase;

bool getControllerParams()
{
	std::string paramName="Kp";
	if(!ros::param::get(paramName, Kp))
	{
		std::cout << "controller param Kp not found" << std::endl;
		return 0;
	}

	paramName="ar10_Kp";
	if(!ros::param::get(paramName, ar10_Kp))
	{
		std::cout << "controller param ar10_Kp not found" << std::endl;
		return 0;
	}	
	paramName="Kp_position";
	if(!ros::param::get(paramName, Kp_position))
	{
		std::cout << "controller param Kp_position not found" << std::endl;
		return 0;
	}	

	paramName="ar10_timeStep";
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

	// paramName="use_VM";
	// int value;
	// if(ros::param::get(paramName, value))
	// {
	// 	if(value)
	// 		use_VM=true;
	// }
	// else
	// {
	// 	std::cout << "controller param use_VM not found" << std::endl;
	// 	return 0;
	// }

	// paramName="dualArm";
	// if(ros::param::get(paramName, value))
	// {
	// 	if(value)
	// 		dualArm=true;
	// }
	// else
	// {
	// 	std::cout << "controller param dualArm not found" << std::endl;
	// 	return 0;
	// }


	paramName="sleepTimeout";
	if(!ros::param::get(paramName, sleepTimeout))
	{
		std::cout << "controller param sleepTimeout not found" << std::endl;
		return 0;
	}
	paramName="ar10_dt";
	if(!ros::param::get(paramName, ar10_dt))
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

bool update_baxterAR10()
{

	if(baxter_controller->BaxterPoseControlServer::updateManipulatorVariables("right"))
	{
		// std::cout << "Here 4" << std::endl;
		if(baxter_controller->BaxterPoseControlServer::importManipulatorState("right", pe_init_baxterRight, pe_now_baxterRight, jacobian_baxterRight, q_baxterRight, joint_velocity_limit_baxterRight))
		{		
					// std::cout << "Here 5" << std::endl;
			if(ar10->AR10Kinematics::updateFingerVariables("thumb_right"))
			{
						// std::cout << "Here 6" << std::endl;
				if(ar10->AR10Kinematics::importHandState("thumb_right", pe_init_thumbRight, pe_now_thumbRight, jacobian_thumbRight, q_thumbRight, q_init_thumbRight))
				{
							// std::cout << "Here 7" << std::endl;
					if(ar10->AR10Kinematics::updateFingerVariables("index_right"))
					{
								// std::cout << "Here 8" << std::endl;
						if(!ar10->AR10Kinematics::importHandState("index_right", pe_init_indexRight, pe_now_indexRight, jacobian_indexRight, q_indexRight, q_init_indexRight))
						{
									// std::cout << "Here 9" << std::endl;
							std::cout<< "could not get state for indexRight" << std::endl;
							return false;
						}
						// else return true;
					}
					else 
					{
						std::cout<< "could not update variables for indexRight" << std::endl;			
						return false;							
					}
				}
				else 
				{
					std::cout << "could not update variables for thumb_right" << std::endl;
					return false;
				}
			}
			else
			{
			 std::cout<< "could not get state for thumb_right" << std::endl;
			}
			// std::cout << "jacobian_baxterRight: " << std::endl;
			// std::cout << jacobian_baxterRight << std::endl;
		}
		else 
		{
			std::cout<< "could not get state for baxter" << std::endl;
			return 0;
		}
	}
	else
	{
		std::cout<< "could not update state for baxter" << std::endl;
		return 0;
	}
	// std::cout << "Here 3" << std::endl;
	// std::cout << "Here 3" << std::endl;
	// std::cout << "Here 3" << std::endl;
	std::cout << "htm_pe_now_baxterRight: " << std::endl;
	std::cout << DQoperations::dq2HTM(pe_now_baxterRight) << std::endl;
	std::cout << "q_baxterRight: " << std::endl;
	std::cout << q_baxterRight << std::endl;
	// std::cout << "Here 3" << std::endl;
	// std::cout << "Here 3" << std::endl;
	// std::cout << "Here 3" << std::endl;

	// pe_now_thumbRight = DQoperations::mulDQ(DQoperations::mulDQ(pe_now_baxterRight, tf_rightHand2handBase), pe_now_thumbRight);
	// pe_now_indexRight = DQoperations::mulDQ(DQoperations::mulDQ(pe_now_baxterRight, tf_rightHand2handBase), pe_now_indexRight);

	// std::cout << "jacobian_thumbRight: " << std::endl;
	// std::cout << jacobian_thumbRight << std::endl;	
	
	// MatrixXd jacobian = jacobian_thumbRight;
	// jacobian.block(0, 0, 4, jacobian_thumbRight.cols()) = 	jacobian_thumbRight.block(4, 0, 4, jacobian_thumbRight.cols()) ;
	// jacobian.block(4, 0, 4, jacobian_thumbRight.cols()) = 	jacobian_thumbRight.block(0, 0, 4, jacobian_thumbRight.cols()) ;
	// jacobian.col(0) = DQoperations::transformLine(jacobian.col(0), DQoperations::mulDQ(pe_now_baxterRight, tf_rightHand2handBase));
	// jacobian.col(1) = DQoperations::transformLine(jacobian.col(1), DQoperations::mulDQ(pe_now_baxterRight, tf_rightHand2handBase));
	// jacobian_thumbRight.block(0, 0, 4, jacobian_thumbRight.cols()) = 	jacobian.block(4, 0, 4, jacobian_thumbRight.cols()) ;
	// jacobian_thumbRight.block(4, 0, 4, jacobian_thumbRight.cols()) = 	jacobian.block(0, 0, 4, jacobian_thumbRight.cols()) ;

	// std::cout << "jacobian_thumbRight_aftertf: " << std::endl;
	// std::cout << jacobian_thumbRight << std::endl;	

	// jacobian = jacobian_indexRight;
	// jacobian.block(0, 0, 4, jacobian_indexRight.cols()) = 	jacobian_indexRight.block(4, 0, 4, jacobian_indexRight.cols()) ;
	// jacobian.block(4, 0, 4, jacobian_indexRight.cols()) = 	jacobian_indexRight.block(0, 0, 4, jacobian_indexRight.cols()) ;
	// jacobian.col(0) = DQoperations::transformLine(jacobian.col(0), DQoperations::mulDQ(pe_now_baxterRight, tf_rightHand2handBase));
	// jacobian.col(1) = DQoperations::transformLine(jacobian.col(1), DQoperations::mulDQ(pe_now_baxterRight, tf_rightHand2handBase));
	// jacobian_indexRight.block(0, 0, 4, jacobian_indexRight.cols()) = 	jacobian.block(4, 0, 4, jacobian_indexRight.cols()) ;
	// jacobian_indexRight.block(4, 0, 4, jacobian_indexRight.cols()) = 	jacobian.block(0, 0, 4, jacobian_indexRight.cols()) ;
}



bool grasp_callback(dq_robotics::Grasp::Request& grasp_req, 
														  dq_robotics::Grasp::Response& grasp_res)
{
	// std::cout << "Here 1" << std::endl;
	RowVectorXd screw_error_thumb, screw_error_right, screw_error_index, combined_screw_error, q_dot, q, jointCmds_baxterRight, jointCmds_thumbRight, jointCmds_indexRight;	
	MatrixXd combined_jacobian_8d;
	double normError =1;
  while (normError > norm_error_const)
  {
  		// std::cout << "is it this function!" << std::endl;
		if(!update_baxterAR10())
		{
			std::cout << "The arm and hand could not be updated" << std::endl;
			return 0;
		}
		// std::cout << "or not!" << std::endl;
		DQoperations::doubleToDQ(pe_desired_thumb, grasp_req.grasp_pose_thumb);
		DQoperations::doubleToDQ(pe_desired_index, grasp_req.grasp_pose_index);

		// std::cout << "pe_desired_thumb: " << std::endl;
		// std::cout << DQoperations::dq2HTM(pe_desired_thumb) << std::endl;
		// std::cout << "pe_desired_index: " << std::endl;
		// std::cout << DQoperations::dq2HTM(pe_desired_index) << std::endl;	

		std::cout << "htm_pe_current_index: " << std::endl;
		std::cout << DQoperations::dq2HTM(pe_now_indexRight) << std::endl;
		std::cout << "htm_pe_desired_index: " << std::endl;
		std::cout << DQoperations::dq2HTM(pe_desired_index) << std::endl;		

		std::cout << "htm_pe_current_thumb: " << std::endl;
		std::cout << DQoperations::dq2HTM(pe_now_thumbRight) << std::endl;
		std::cout << "htm_pe_desired_thumb: " << std::endl;		
		std::cout << DQoperations::dq2HTM(pe_desired_thumb) << std::endl;		
		
		screw_error_thumb= DQController::getScrewError_8d(pe_now_thumbRight, pe_desired_thumb);
		screw_error_index= DQController::getScrewError_8d(pe_now_indexRight, pe_desired_index);
		// screw_error_right= DQController::getScrewError_8d(pe_now_baxterRight, pe_desired_index);

		// std::cout << "pe_now_baxterRight: " << std::endl;
		// std::cout << DQoperations::dq2HTM(pe_now_thumbRight) << std::endl;

		// std::cout << "pe_desired_index: " << std::endl;
		// std::cout << DQoperations::dq2HTM(pe_desired_index) << std::endl;
		
		// combined_jacobian_8d= MatrixXd::Zero(16, (jacobian_baxterRight.cols() + jacobian_thumbRight.cols() + jacobian_indexRight.cols()));
		// combined_jacobian_8d.block(0, 0, jacobian_baxterRight.rows(), jacobian_baxterRight.cols()) = jacobian_baxterRight;
		// combined_jacobian_8d.block(jacobian_baxterRight.rows(), 0, jacobian_baxterRight.rows(), jacobian_baxterRight.cols()) = jacobian_baxterRight;
		// combined_jacobian_8d.block(0, jacobian_baxterRight.cols(), jacobian_indexRight.rows(), jacobian_indexRight.cols()) = jacobian_indexRight;
		// combined_jacobian_8d.block(jacobian_baxterRight.rows(), (jacobian_baxterRight.cols() + jacobian_indexRight.cols()), jacobian_thumbRight.rows(), jacobian_thumbRight.cols()) = jacobian_thumbRight;
		// combined_screw_error =RowVectorXd::Zero(screw_error_thumb.cols()+screw_error_index.cols());
		// combined_screw_error << screw_error_index, screw_error_thumb;

		// RowVectorXd errorScrew_positionOnly = RowVectorXd::Zero((screw_error_thumb.cols()+screw_error_index.cols())/2);
		// errorScrew_positionOnly << screw_error_index.head(4), screw_error_thumb.head(4);

		MatrixXd combined_jacobian_positionOnly ;
	combined_jacobian_positionOnly = MatrixXd::Zero(16, (jacobian_thumbRight.cols() + jacobian_indexRight.cols()));
		// combined_jacobian_positionOnly = MatrixXd::Zero(8, (jacobian_baxterRight.cols() + jacobian_thumbRight.cols() + jacobian_indexRight.cols()));

		combined_jacobian_positionOnly.block(0, 0, jacobian_indexRight.rows(), jacobian_indexRight.cols()) = jacobian_indexRight.block(0, 0, jacobian_indexRight.rows(), jacobian_indexRight.cols()); 
		combined_jacobian_positionOnly.block(jacobian_indexRight.rows(), jacobian_indexRight.cols(), jacobian_thumbRight.rows(), jacobian_thumbRight.cols()) = jacobian_thumbRight.block(0, 0, jacobian_thumbRight.rows(), jacobian_thumbRight.cols()); 

		// combined_jacobian_positionOnly.block(jacobian_baxterRight.rows()/2, 0, jacobian_baxterRight.rows()/2, jacobian_baxterRight.cols()) = jacobian_baxterRight.block(0, 0, jacobian_baxterRight.rows()/2, jacobian_baxterRight.cols()); 

		// combined_jacobian_positionOnly.block(0, jacobian_baxterRight.cols(),  jacobian_indexRight.rows()/2, jacobian_indexRight.cols())=jacobian_indexRight.block(0, 0, jacobian_indexRight.rows()/2, jacobian_indexRight.cols());

		// combined_jacobian_positionOnly.block(jacobian_baxterRight.rows()/2, (jacobian_baxterRight.cols()+jacobian_indexRight.cols()),  jacobian_thumbRight.rows()/2, jacobian_thumbRight.cols())=jacobian_thumbRight.block(0, 0, jacobian_thumbRight.rows()/2, jacobian_thumbRight.cols());

		// std::cout << "Jacobian_positionOnly: " << std::endl;
		// std::cout << combined_jacobian_positionOnly << std::endl;

		// RowVectorXd errorScrew_indexOnly = RowVectorXd::Zero((screw_error_index.cols()));
		// RowVectorXd errorScrew_thumbOnly = RowVectorXd::Zero((screw_error_thumb.cols()));
		// errorScrew_indexOnly << screw_error_index;
		// errorScrew_thumbOnly << screw_error_thumb.head(4);
		// errorScrew_indexOnly << screw_error_index.head(4);
		// std::cout << "errorScrew_indexOnly: " << std::endl;
		// std::cout << errorScrew_indexOnly << std::endl;
		// std::cout << "jacobian_thumbRight: " << std::endl;
		// std::cout << jacobian_thumbRight << std::endl;		
		// std::cout << "jacobian_indexRight: " << std::endl;
		// std::cout << jacobian_indexRight << std::endl;				
		
		// std::cout << "combined_jacobian_positionOnly: " << std::endl;
		// std::cout << combined_jacobian_positionOnly << std::endl;						
		// MatrixXd combined_jacobian_indexOnly ;
		
		// combined_jacobian_indexOnly = MatrixXd::Zero(4, (jacobian_baxterRight.cols() + jacobian_indexRight.cols()));

		// combined_jacobian_indexOnly.block(0, 0, jacobian_baxterRight.rows()/2, jacobian_baxterRight.cols()) = jacobian_baxterRight.block(0, 0, jacobian_baxterRight.rows()/2, jacobian_baxterRight.cols()); 

		// combined_jacobian_indexOnly.block(0, jacobian_baxterRight.cols(),  jacobian_indexRight.rows()/2, jacobian_indexRight.cols())=jacobian_indexRight.block(0, 0, jacobian_indexRight.rows()/2, jacobian_indexRight.cols());


		// std::cout << "Jacobian_indexOnly: " << std::endl;
		// std::cout << combined_jacobian_indexOnly << std::endl;

		RowVectorXd errorScrew_positionOnly = RowVectorXd::Zero((screw_error_thumb.cols()+screw_error_index.cols()));
		errorScrew_positionOnly << screw_error_index, screw_error_thumb;
		std::cout << "screw_error_thumb:" << screw_error_thumb << std::endl;
		// std::cout << "screw_error_index:" << screw_error_index << std::endl;
		// std::cout << "errorScrew_positionOnly:" << errorScrew_positionOnly << std::endl;
		// RowVectorXd errorScrew_rightOnly = RowVectorXd::Zero((screw_error_right.cols())/2);
		// errorScrew_rightOnly << screw_error_right.head(4);

		// MatrixXd combined_jacobian_rightOnly ;
		
		// combined_jacobian_rightOnly = MatrixXd::Zero(4, (jacobian_baxterRight.cols()));

		// combined_jacobian_rightOnly= jacobian_baxterRight.block(0, 0, jacobian_baxterRight.rows()/2, jacobian_baxterRight.cols()); 


		// std::cout << "combined_jacobian_rightOnly: " << std::endl;
		// std::cout << combined_jacobian_rightOnly << std::endl;


		// q_dot= DQController::calculateControlVel(Kp, mu, errorScrew_positionOnly, combined_jacobian_positionOnly, (combined_jacobian_positionOnly.cols()));
		// q_dot= DQController::calculateControlVel(Kp, mu, screw_error_index, jacobian_indexRight, (jacobian_indexRight.cols()));
		q_dot= DQController::calculateControlVel(Kp, mu, screw_error_thumb, jacobian_thumbRight, (jacobian_thumbRight.cols()));
		// q_dot= DQController::calculateControlVel(Kp, mu, errorScrew_indexOnly, combined_jacobian_indexOnly, (combined_jacobian_indexOnly.cols()));
		// q_dot= DQController::calculateControlVel(ar10_Kp, mu, errorScrew_indexOnly, jacobian_indexRight.block(0,0,jacobian_indexRight.rows()/2, jacobian_indexRight.cols()), (jacobian_indexRight.cols()));
		// q_dot= DQController::calculateControlVel(ar10_Kp, mu, errorScrew_thumbOnly, jacobian_thumbRight.block(0,0,jacobian_thumbRight.rows()/2, jacobian_thumbRight.cols()), (jacobian_thumbRight.cols()));
		std::cout << "q_dot: " << q_dot << std::endl;
		q.resize(q_dot.size());
		// q << q_baxterRight, q_indexRight, q_thumbRight;
		// q << q_indexRight, q_thumbRight;
		// q << q_baxterRight, q_indexRight;
		// q << q_indexRight;
		q << q_thumbRight;
		q = q + q_dot*ar10_timeStep; // HERE YOU CAN USE (time_now-time_last) in place of timeStep
	
		std::cout << "q: " << q << std::endl;
		// jointCmds_indexRight = q ;
		// jointCmds_thumbRight = q ;
		// jointCmds_baxterRight = q.head(7);  
		// jointCmds_indexRight = q.segment(7, 2) ;
		// jointCmds_thumbRight = q.segment(9, 2) ;
		// jointCmds_thumbRight = q.tail(2) ;
		// jointCmds_indexRight = q.head(2) ;
		jointCmds_thumbRight = q;
		// baxter_controller->sendBaxterJointCmds("right", DQoperations::dqEigenToDQdouble(jointCmds_baxterRight));
		if(ar10->AR10Kinematics::sendJointCmds_AR10(DQoperations::dqEigenToDQdouble(jointCmds_thumbRight), "thumb"))
		{
			std::cout << "Grasp cmds for ar10 thumb sent" << std::endl;
			
		}
		else 
		{
			ROS_ERROR("AR10 joint servive not working for thumb.");
			return 0;
		}
		// if(ar10->AR10Kinematics::sendJointCmds_AR10(DQoperations::dqEigenToDQdouble(jointCmds_indexRight), "index"))
		// {
		// 	std::cout << "Grasp cmds for ar10 index sent" << std::endl;
		// }
		// else 
		// {
		// 	ROS_ERROR("AR10 joint servive not working for index.");
		// 	return 0;
		// }	
			// if(ar10->AR10Kinematics::sendJointCmds_AR10(DQoperations::dqEigenToDQdouble(jointCmds_thumbRight), "thumb"))
			// {
			// 	std::cout << "Grasp cmds for ar10 sent" << std::endl;
			// }
			// else 
			// {
			// 	ROS_ERROR("AR10 joint servive not working.");
			// 	return 0;
			// }			
		// normError = errorScrew_indexOnly.norm();
		// normError = errorScrew_thumbOnly.norm();
		// normError = errorScrew_positionOnly.norm();
		normError = screw_error_thumb.norm();
		std::cout << "normError: " << normError << std::endl;
		ros::Duration(ar10_dt).sleep();
	}
	grasp_res.normError = normError;

	return true;
	// DQoperations::dqEigenToDQdouble(q, jointCmds);

}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "baxter_ar10_kinematics");
	ros::NodeHandle n;

	Matrix4d htm_rightHand2handBase = Matrix4d::Identity();
	htm_rightHand2handBase << -0.0002037,  1.0000000,  0.0000000, 0,
													  -1.0000000, -0.0002037,  0.0000000, 0, 
													   0.0000000,  0.0000000,  1.0000000, -0.0021,
													   0,					 0,					 0, 				 1;
	tf_rightHand2handBase = DQoperations::htm2DQ(htm_rightHand2handBase);

	ros::ServiceServer service = n.advertiseService("baxterAR10/grasp_service", grasp_callback);

	if (!getControllerParams())
		std::cout << "Controller params not found for COMBO controller." << std::endl;


	baxter_controller = new BaxterPoseControlServer();
	if (!baxter_controller->BaxterPoseControlServer::initializeController())
	{
		ROS_ERROR("The robot can not be initialized.");
		return 0;
	}

	ar10 = new AR10Kinematics();
	ar10->AR10Kinematics::init_AR10Kinematics();
	ar10->AR10Kinematics::init_fingers();
	ros::spin();

	return 0;
}