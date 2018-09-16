#include <dq_robotics/baxter_poseControl_server.h>
#include <dq_robotics/ar10_kinematics.h>
#include <dq_robotics/Grasp.h>
#include <dq_robotics/BaxterControl.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "grasp_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<dq_robotics::BaxterControl>("baxter/coopTaskSpaceControlCallback");

  dq_robotics::BaxterControl::Request baxterControl_req;

  Matrix<double,8,1> desiredAbsPose, desiredRelPose;
	Matrix4d htm_abs, htm_rel;

    ros::Duration(5).sleep();
	// htm_abs <<   -0.5000,   -0.5000,    0.7071,    0.5,
 //                0.7071,   -0.7071,    0.0000,   -0.0000,
 //                0.5000,    0.5000,    0.7071,    0.2,
 //                     0,         0 ,        0 ,   1.0000;
 //  htm_rel << 0, 1, 0, 0,
 //             1, 0, 0, 0,
 //             0, 0, -1, 0.5,
 //             0, 0, 0, 1;   
 //  desiredAbsPose = DQoperations::htm2DQ(htm_abs);
 //  desiredRelPose = DQoperations::htm2DQ(htm_rel);
 //  baxterControl_req.desiredAbsPose = DQoperations::DQToDouble(desiredAbsPose);
 //  baxterControl_req.desiredRelPose = DQoperations::DQToDouble(desiredRelPose);
 //  baxterControl_req.tracking = false;
 //  dq_robotics::BaxterControl client_msg;
 //  client_msg.request = baxterControl_req;
 //  ROS_INFO("going to grasp now");  
 //  if (client.call(client_msg))
 //  {
 //    ROS_INFO("normError: %f", client_msg.response.normError);
 //  }
 //  else
 //  {
 //    ROS_ERROR("Failed to call service grasp_service");
 //    return 1;
 //  }

 //  ROS_INFO("rotating absolute now.");
 //  double total_time = 0, total_rotation = 0, total_translation =0, intial_rot_trans = 0;
 //  double final_rotation =1;
 //  Matrix<double,8,1> desired_pose;
 //  double rotation_rate = 0.00, d = 0;
 //  double time_start =ros::Time::now().toSec();
 //  std::cout << "time_start: " << time_start;
 //  ros::Duration(0.5).sleep();
 //  time_start =ros::Time::now().toSec();
 //  std::cout << "time_start: " << time_start;

 //  while (total_rotation < final_rotation)
 //  {
 //    double time_now = ros::Time::now().toSec();
 //    total_time = time_now - time_start;
 //    std::cout << "total_time: " << total_time << std::endl;
 //    total_rotation = intial_rot_trans + rotation_rate*total_time; 
 //    RowVector3d l, p, m;
 //    l << 0, 0, 1;
 //    p << 0, 0, 0;
 //    m = p.cross(l);
 //    Matrix<double,8,1> change_dq = DQoperations::screw2DQ(total_rotation, l, d, m);
 //    desired_pose = DQoperations::mulDQ(desiredAbsPose, change_dq);
 //    std::cout << "htm_abs_desired: " << std::endl;  
 //    std::cout << DQoperations::dq2HTM(desired_pose) << std::endl;  
 //    baxterControl_req.desiredAbsPose = DQoperations::DQToDouble(desired_pose);
 //    baxterControl_req.desiredRelPose = DQoperations::DQToDouble(desiredRelPose);
 //    baxterControl_req.tracking = true;
 //    client_msg.request = baxterControl_req;
 //    if (client.call(client_msg))
 //    {
 //      ROS_INFO("normError: %f", client_msg.response.normError);
 //    }
 //    else
 //    {
 //      ROS_ERROR("Failed to call service grasp_service");
 //      return 1;
 //    }
 //  }
  // htm_abs <<   -0.5000,   -0.5000,    0.7071,    0.7,
  //               0.7071,   -0.7071,    0.0000,   -0.0000,
  //               0.5000,    0.5000,    0.7071,    0.2,
  //                    0,         0 ,        0 ,   1.0000;
  htm_abs << 1, 0, 0, 0.770645,
             0, 1, 0, 0,
             0, 0, 1, 0.373905,
             0, 0, 0, 1;

 // htm_abs << -0.991292, 0.0351464, -0.126905,  0.7393,
             // 0.129208,  0.445522, -0.885898,   0,
             //  0.0254027, -0.894581, -0.446184,  0.199363,
             //  0,         0,         0,         1;



  htm_rel << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;   
  desiredAbsPose = DQoperations::htm2DQ(htm_abs);
  desiredRelPose = DQoperations::htm2DQ(htm_rel);
  baxterControl_req.desiredAbsPose = DQoperations::DQToDouble(desiredAbsPose);
  baxterControl_req.desiredRelPose = DQoperations::DQToDouble(desiredRelPose);
  baxterControl_req.starting_abs_pose = DQoperations::DQToDouble(desiredAbsPose);
  baxterControl_req.velocity_control = false;
  baxterControl_req.newTask =true;
  baxterControl_req.virtualSticks =true;
  dq_robotics::BaxterControl client_msg;
  client_msg.request = baxterControl_req;
  ROS_INFO("going to grasp now");  
  if (client.call(client_msg))
  {
    ROS_INFO("normError: %f", client_msg.response.normError);
  }
  else
  {
    ROS_ERROR("Failed to call service grasp_service");
    return 1;
  }

  ROS_INFO("rotating absolute now.");
  double total_time = 0, total_rotation = 0, total_translation =0, intial_rot_trans = 0;
  double final_rotation =0.05;
  Matrix<double,8,1> desired_pose;
  double rotation_rate = 0.005, d = 0;
  double time_start =ros::Time::now().toSec();
  std::cout << "time_start: " << time_start;
  ros::Duration(0.5).sleep();
  time_start =ros::Time::now().toSec();
  std::cout << "time_start: " << time_start;
  baxterControl_req.newTask =false;
  // while (total_rotation < final_rotation)
  // {
  //   double time_now = ros::Time::now().toSec();
  //   total_time = time_now - time_start;
  //   std::cout << "total_time: " << total_time << std::endl;
  //   total_rotation = intial_rot_trans + rotation_rate*total_time; 
  //   RowVector3d l, p, m;
  //   l << 1, 0, 0 ;
  //   p << 0, 0, 0;
  //   m = p.cross(l);
  //   Matrix<double,8,1> change_dq = DQoperations::screw2DQ(total_rotation, l, d, m);
  //   desired_pose = desiredRelPose;
  //   // desired_pose = DQoperations::mulDQ(desiredRelPose, change_dq);
  //   std::cout << "desiredRelPose: " << std::endl;  
  //   std::cout << DQoperations::dq2HTM(desired_pose) << std::endl;  
  //   htm_abs(0,3)= htm_abs(0,3) +  0.5*rotation_rate*total_time;
  //   htm_abs(2,3)= htm_abs(2,3) +  0.5*rotation_rate*total_time;
  //   desiredAbsPose = DQoperations::htm2DQ(htm_abs);
  //   baxterControl_req.desiredAbsPose = DQoperations::DQToDouble(desiredAbsPose);
  //   baxterControl_req.desiredRelPose = DQoperations::DQToDouble(desired_pose);
  //   // baxterControl_req.velocity_control = true;
  //   client_msg.request = baxterControl_req;
  //   if (client.call(client_msg))
  //   {
  //     ROS_INFO("normError: %f", client_msg.response.normError);
  //   }
  //   else
  //   {
  //     ROS_ERROR("Failed to call service grasp_service");
  //     return 1;
  //   }
  // }
  // ros::Duration(0.5).sleep();
  // time_start =ros::Time::now().toSec();
  // std::cout << "time_start: " << time_start;
  // intial_rot_trans = 0;
  // total_rotation = 0;
  // final_rotation =0.05;
  // total_time = 0;
  // total_time = 0, total_rotation = 0, total_translation =0, intial_rot_trans = 0;
  // rotation_rate = 0.005, d = 0;
  // while (total_rotation < final_rotation)
  // {
  //   double time_now = ros::Time::now().toSec();
  //   total_time = time_now - time_start;
  //   std::cout << "total_time: " << total_time << std::endl;
  //   total_rotation = intial_rot_trans + rotation_rate*total_time; 
  //   RowVector3d l, p, m;
  //   l << 1, 0, 0 ;
  //   p << 0, 0, 0;
  //   m = p.cross(l);
  //   Matrix<double,8,1> change_dq = DQoperations::screw2DQ(total_rotation, l, d, m);
  //   desired_pose = desiredRelPose;
  //   // desired_pose = DQoperations::mulDQ(desiredRelPose, change_dq);
  //   std::cout << "desiredRelPose: " << std::endl;  
  //   std::cout << DQoperations::dq2HTM(desired_pose) << std::endl;  
  //   htm_abs(0,3)= htm_abs(0,3) -  rotation_rate*total_time;
  //   htm_abs(2,3)= htm_abs(2,3) +  rotation_rate*total_time;
  //   // htm_abs(2,3)= htm_abs(2,3) +  rotation_rate*total_time;
  //   desiredAbsPose = DQoperations::htm2DQ(htm_abs);
  //   baxterControl_req.desiredAbsPose = DQoperations::DQToDouble(desiredAbsPose);
  //   baxterControl_req.desiredRelPose = DQoperations::DQToDouble(desired_pose);
  //   // baxterControl_req.velocity_control = true;
  //   client_msg.request = baxterControl_req;
  //   if (client.call(client_msg))
  //   {
  //     ROS_INFO("normError: %f", client_msg.response.normError);
  //   }
  //   else
  //   {
  //     ROS_ERROR("Failed to call service grasp_service");
  //     return 1;
  //   }
  // }  

  ros::Duration(0.5).sleep();
  time_start =ros::Time::now().toSec();
  std::cout << "time_start: " << time_start;
  intial_rot_trans = 0;
  total_rotation = 0;
  final_rotation = 1;
  total_time = 0;
  total_time = 0, total_rotation = 0, total_translation =0, intial_rot_trans = 0;
  rotation_rate = 0.01, d = 0;
  ros::WallTime start = ros::WallTime::now();

  while (total_rotation < final_rotation)
  {
    start = ros::WallTime::now();
    double time_now = ros::Time::now().toSec();
    total_time = time_now - time_start;
    std::cout << "total_time: " << total_time << std::endl;
    total_rotation = intial_rot_trans + 2*rotation_rate*total_time; 
    RowVector3d l, p, m;
    l << -1, 0, 0;
    p << 0, 0, 0;
    m = p.cross(l);
    Matrix<double,8,1> change_dq = DQoperations::screw2DQ(total_rotation, l, d, m);
    desired_pose = desiredRelPose;
    desiredAbsPose = DQoperations::mulDQ(desiredAbsPose, change_dq);
    // desired_pose = DQoperations::mulDQ(desiredAbsPose, change_dq);
    // Matrix4d change_htm = DQoperations::dq2HTM(desired_pose);
    // change_htm(1,3) = change_htm(1,3) - 0.001*rotation_rate*total_time;
    // desired_pose = DQoperations::htm2DQ(change_htm); 
    // std::cout << "desiredRelPose: " << std::endl;  
    // std::cout << DQoperations::dq2HTM(desired_pose) << std::endl;  
    // htm_abs(0,3)= htm_abs(0,3) -  rotation_rate*total_time;
    // htm_abs(2,3)= htm_abs(2,3) +  0.01*rotation_rate*total_time;
    // htm_abs(0,3)= htm_abs(0,3) +  0.01*rotation_rate*total_time;
    // desiredAbsPose = DQoperations::htm2DQ(htm_abs);
    baxterControl_req.desiredAbsPose = DQoperations::DQToDouble(desiredAbsPose);
    baxterControl_req.desiredRelPose = DQoperations::DQToDouble(desiredRelPose);
    // baxterControl_req.velocity_control = true;
    client_msg.request = baxterControl_req;
    if (client.call(client_msg))
    {
      ROS_INFO("normError: %f", client_msg.response.normError);
    }
    else
    {
      ROS_ERROR("Failed to call service grasp_service");
      return 1;
    }
    // ros::WallTime end = ros::WallTime::now();
    // ros::WallDuration dur = end - start;
    // std::cout << "master loop duration: " << dur.toSec() << std::endl;
  }    


  //   time_start =ros::Time::now().toSec();
  // std::cout << "time_start: " << time_start;
  // intial_rot_trans = 0;
  // total_rotation = 0;
  // final_rotation = 0.10;
  // total_time = 0;
  // total_time = 0, total_rotation = 0, total_translation =0, intial_rot_trans = 0;
  // rotation_rate = 0.01, d = 0;
  // while (total_rotation < final_rotation)
  // {
  //   double time_now = ros::Time::now().toSec();
  //   total_time = time_now - time_start;
  //   std::cout << "total_time: " << total_time << std::endl;
  //   total_rotation = intial_rot_trans + 0.01*rotation_rate*total_time; 
  //   RowVector3d l, p, m;
  //   l << 0, 1, 0;
  //   p << 0, 0, 0;
  //   m = p.cross(l);
  //   Matrix<double,8,1> change_dq = DQoperations::screw2DQ(total_rotation, l, d, m);
  //   desired_pose = desiredRelPose;
  //   // desiredAbsPose = DQoperations::mulDQ(desiredAbsPose, change_dq);
  //   // desired_pose = DQoperations::mulDQ(desiredRelPose, change_dq);
  //   // Matrix4d change_htm = DQoperations::dq2HTM(desired_pose);
  //   // change_htm(1,3) = change_htm(1,3) - 0.001*rotation_rate*total_time;
  //   // desired_pose = DQoperations::htm2DQ(change_htm); 
  //   // std::cout << "desiredRelPose: " << std::endl;  
  //   // std::cout << DQoperations::dq2HTM(desired_pose) << std::endl;  
  //   // htm_abs(0,3)= htm_abs(0,3) -  rotation_rate*total_time;
  //   htm_abs(2,3)= htm_abs(2,3) - 0.01*rotation_rate*total_time;
  //   htm_abs(0,3)= htm_abs(0,3) -  0.01*rotation_rate*total_time;
  //   desiredAbsPose = DQoperations::htm2DQ(htm_abs);
  //   baxterControl_req.desiredAbsPose = DQoperations::DQToDouble(desiredAbsPose);
  //   baxterControl_req.desiredRelPose = DQoperations::DQToDouble(desired_pose);
  //   // baxterControl_req.velocity_control = true;
  //   client_msg.request = baxterControl_req;
  //   if (client.call(client_msg))
  //   {
  //     ROS_INFO("normError: %f", client_msg.response.normError);
  //   }
  //   else
  //   {
  //     ROS_ERROR("Failed to call service grasp_service");
  //     return 1;
  //   }
  // }    
  return 0;
} 

//////////////////////relative rotation along -x frame and -ve traslation relative along y axis  
  // while (total_rotation < final_rotation)
  // {
  //   double time_now = ros::Time::now().toSec();
  //   total_time = time_now - time_start;
  //   std::cout << "total_time: " << total_time << std::endl;
  //   total_rotation = intial_rot_trans + 2*rotation_rate*total_time; 
  //   RowVector3d l, p, m;
  //   l << -1, 0, 0;
  //   p << 0, 0, 0;
  //   m = p.cross(l);
  //   Matrix<double,8,1> change_dq = DQoperations::screw2DQ(total_rotation, l, d, m);
  //   // desired_pose = desiredRelPose;
  //   desired_pose = DQoperations::mulDQ(desiredRelPose, change_dq);
  //   Matrix4d change_htm = DQoperations::dq2HTM(desired_pose);
  //   change_htm(1,3) = change_htm(1,3) - 0.001*rotation_rate*total_time;
  //   desired_pose = DQoperations::htm2DQ(change_htm); 
  //   std::cout << "desiredRelPose: " << std::endl;  
  //   std::cout << DQoperations::dq2HTM(desired_pose) << std::endl;  
  //   // htm_abs(0,3)= htm_abs(0,3) -  rotation_rate*total_time;
  //   // htm_abs(2,3)= htm_abs(2,3) +  rotation_rate*total_time;
  //   // htm_abs(0,3)= htm_abs(0,3) +  0.01*rotation_rate*total_time;
  //   // desiredAbsPose = DQoperations::htm2DQ(htm_abs);
  //   baxterControl_req.desiredAbsPose = DQoperations::DQToDouble(desiredAbsPose);
  //   baxterControl_req.desiredRelPose = DQoperations::DQToDouble(desired_pose);
  //   // baxterControl_req.velocity_control = true;
  //   client_msg.request = baxterControl_req;
  //   if (client.call(client_msg))
  //   {
  //     ROS_INFO("normError: %f", client_msg.response.normError);
  //   }
  //   else
  //   {
  //     ROS_ERROR("Failed to call service grasp_service");
  //     return 1;
  //   }
  // }    