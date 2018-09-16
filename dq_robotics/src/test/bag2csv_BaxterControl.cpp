#include <dq_robotics/baxter_poseControl_server.h>
#include <dq_robotics/ar10_kinematics.h>
#include <dq_robotics/Grasp.h>
#include <dq_robotics/BaxterControl.h>
#include <dq_robotics/ControllerPerformance.h>
  
std::string path;

float64[] desiredRelPose # this can be relative desire pose
float64[] desiredAbsPose # this can be relative desire pose
float64[] currentRelPose # this can be relative desire pose
float64[] currentAbsPose # this can be relative desire pose
float64[] joint_state_left # this can be relative desire pose
float64[] joint_state_right # this can be relative desire pose
float64[] del_q # this can be relative desire pose
float64 norm_error_abs
float64 norm_error_rel
float64 timeStamp
int32 controlMode

void chatterCallback(const dq_robotics::ControllerPerformance::ConstPtr& msg)
{
    if(result_file.is_open())
  {
    std::cout << "file is open" << std::endl;
  }

  result_file << "time" << "," << "desiredRelPose_0"  << "," << "desiredRelPose_1"  << "," << "desiredRelPose_2" << "," << "desiredRelPose_3" <<  "," << "desiredRelPose_4" <<   "," << "desiredRelPose_5" <<   "," << "desiredRelPose_6" <<   "," << "desiredRelPose_7" <<  "," << "currentRelPose_0"  << "," << "currentRelPose_1"  << "," << "currentRelPose_2" << "," << "currentRelPose_3" <<  "," << "currentRelPose_4" <<   "," << "currentRelPose_5" <<   "," << "currentRelPose_6" <<   "," << "currentRelPose_7" <<  "," << "desiredAbsPose_0"  << "," << "desiredAbsPose_1"  << "," << "desiredAbsPose_2" << "," << "desiredAbsPose_3" <<  "," << "desiredAbsPose_4" <<   "," << "desiredAbsPose_5" <<   "," << "desiredAbsPose_6" <<   "," << "desiredAbsPose_7" <<  "," << "norm_error_rel" << "," << "norm_error_abs" << ", \n";
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");


  ros::NodeHandle n;

  std::string path = ros::package::getPath("dq_robotics");
  path.append("/results/");
  path.append("1.csv");
  std::ofstream result_file(path, std::ios::app);
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);


  ros::spin();

  return 0;
}