#include <dq_robotics/baxter_poseControl_server.h>
#include <dq_robotics/ar10_kinematics.h>

class BaxterPoseControlServer_2 
{
private:
	MatrixXd arm_jacobian_right, arm_jacobian_left, index_jacobian_right, thumb_jacobian_right, armThumb_jacobian_right, armIndex_jacobian_right, index_jacobian_left, thumb_jacobian_left, armThumb_jacobian_left, armIndex_jacobian_left;  
public:
// intialize arm and hand 
	bool initialize_handArmController();
	void update_handArm();
	void jacobian_right_armHand();
	BaxterPoseControlServer_2();
	~BaxterPoseControlServer_2();
// get right_arm_jacobian
// get right_hand_index_jacobian
// get right_hand_thumb_jacobian
// make task jacobian
// get desired poses 
// send controller cmds to respective controllers
};