#include <dq_robotics/baxter_poseControl_server_2.h>

BaxterPoseControlServer_2::BaxterPoseControlServer_2()
{}

bool BaxterPoseControlServer_2::initialize_handArmController()
{
	if(initializeController())
		ROS_INFO("Baxter arms and controller initialized");
	BaxterPoseControlServer_2::init_AR10Kinematics();
	BaxterPoseControlServer_2::init_fingers();
}

void BaxterPoseControlServer_2::update_handArm()
{
		BaxterPoseControlServer_2::update_manipulator();
		BaxterPoseControlServer_2::update_right();
		BaxterPoseControlServer_2::update_left();
		BaxterPoseControlServer_2::Ar10KinematicsLoop();
}

void BaxterPoseControlServer_2::jacobian_right_armHand()
{
	index_jacobian_right=MatrixXd::Zero(8,2);	
	finger_params	index_ik, thumb_ik;
	index_ik = index_finger; 
	thumb_ik = thumb;

	getJacobianCoupled(index_ik);
	index_jacobian_right = index_ik.finger_jacobian_coupled;
	std::cout << "index_jacobian_right: " << std::endl;
	std::cout << index_jacobian_right << std::endl;	


	thumb_ik.finger_jacobian_simple= DQController::jacobianDual(thumb_ik.u, thumb_ik.p, thumb_ik.finger_ee_init, thumb_ik.finger_joint_type, thumb_ik.screwDispalcementArray);	
	thumb_ik.finger_jacobian_simple= DQController::jacobianDual_8d(thumb_ik.p.size(), thumb_ik.finger_jacobian_simple);	
	thumb_jacobian_right = thumb_ik.finger_jacobian_simple;
	std::cout << "thumb_jacobian_right: " << std::endl;
	std::cout << thumb_jacobian_right << std::endl;	



	arm_jacobian_right= DQController::jacobianDual(u_right, p_right, pose_now_right, joint_type_right, fkm_matrix_right);	
	arm_jacobian_right= DQController::jacobianDual_8d(joint_size_right, arm_jacobian_right);
	std::cout << "arm_jacobian_right: " << std::endl;
	std::cout << arm_jacobian_right << std::endl;	
}