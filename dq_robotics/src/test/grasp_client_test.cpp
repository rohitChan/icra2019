#include <dq_robotics/baxter_poseControl_server.h>
#include <dq_robotics/ar10_kinematics.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <dq_robotics/Grasp.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "grasp_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<dq_robotics::Grasp>("baxterAR10/grasp_service");

  dq_robotics::Grasp::Request grasp_req;
  const geometry_msgs::PoseStampedConstPtr pen_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("pen_pose");
  Matrix<double,8,1> pen_pose_dq, graspPoint_thumb, graspPoint_index, pre_grasp_pose;
  Matrix4d htm_pen_pose = DQoperations::htmFromGeometryMsgPose(pen_pose->pose);

	Matrix4d htm=Matrix4d::Identity();
	htm(0, 1) = -0.15;
	pre_grasp_pose = DQoperations::htm2DQ(htm_pen_pose*htm);
  grasp_req.pre_grasp_distance = -0.15;

  htm=Matrix4d::Identity();
  htm(0,3) = (0.05/2);
  // graspPoint_index << 0.678793,   -0.19809,  -0.678793,    0.19809, 0.00859172,  0.0318088,  0.0178029,  0.0633727;
  graspPoint_index << 0.518857,  -0.716394,   0.462388,   0.061353,   0.515372,   0.137927,  -0.351655, -0.0976784;


  // graspPoint_thumb << 0.827847,    0.428926,    0.258896,    0.252319, -0.00695001,  -0.0392131,   0.0601057,   0.0277899; 
  graspPoint_thumb << 0.960477,   0.235529,   0.122626, -0.0835014,  0.0014986,   -0.02179,  0.0518687,  0.0319472; 

  // grasp_req.grasp_pose_index = DQoperations::DQToDouble(DQoperations::htm2DQ(htm_pen_pose*htm));
  grasp_req.grasp_pose_index = DQoperations::DQToDouble(graspPoint_index);

  // Matrix4d htm_test=Matrix4d::Identity();
  // htm_test(0,3)= 0.709;
  // htm_test(1,3)= -0.862;
  // htm_test(2,3)= 0.310;
  // grasp_req.grasp_pose_index = DQoperations::DQToDouble(DQoperations::htm2DQ(htm_test));


  htm=Matrix4d::Identity();
  htm(0,3) = (-0.05/2);
  // grasp_req.grasp_pose_thumb = DQoperations::DQToDouble(DQoperations::htm2DQ(htm_pen_pose*htm));
  grasp_req.grasp_pose_thumb = DQoperations::DQToDouble(graspPoint_thumb);

  dq_robotics::Grasp grasp;
  grasp.request = grasp_req;
  ROS_INFO("here 1");  
  if (client.call(grasp))
  {
    ROS_INFO("normError: %f", grasp.response.normError);
  }
  else
  {
    ROS_ERROR("Failed to call service grasp_service");
    return 1;
  }
  return 0;

} 

