  
#include <iostream>
#include <math.h>
#include <stdexcept> //for range_error
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <complex>
#include <cmath>
#include <vector>
#include <cstddef>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <ros/package.h>
#include <dq_robotics/dq_controller.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <ar10_kinematics/IkHand.h>

using namespace Eigen;


// float64[] desiredPose  
// int32 finger # finger=0 for thumb, finger4=index, and so on
// bool getJacobian


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ar10_kinematics_client");

  ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<ar10_kinematics::IkHand>("/ar10_right/finger_ik_sever");

	ros::WallDuration sleep_time(2);
  sensor_msgs::JointState right_hand_cmds;
	right_hand_cmds.position.resize(2);
	right_hand_cmds.name.resize(2);
	right_hand_cmds.name[0]= "servo0";
	right_hand_cmds.name[1]= "servo1";
  ros::Publisher right_ar10Hand_pub = n.advertise<sensor_msgs::JointState>("/ar10_right/ar10/right/joint_states", 1000);
  right_hand_cmds.position[0] = 1.2;
  right_hand_cmds.position[0] = 0.85;
  right_hand_cmds.position[1] = 0.92;
	ROS_INFO("moving thumb out of the way"); 
	do
	{
		sleep_time.sleep();
		right_ar10Hand_pub.publish(right_hand_cmds);
		sleep_time.sleep();
	}
	while (!right_ar10Hand_pub.getNumSubscribers());
	sleep_time.sleep();

	ROS_INFO("opening index finger"); 
	right_hand_cmds.position.resize(2);
	right_hand_cmds.name.resize(2);
	right_hand_cmds.name[0]= "servo8";
	right_hand_cmds.name[1]= "servo9";
  // ros::Publisher right_ar10Hand_pub = n.advertise<sensor_msgs::JointState>("/ar10_right/ar10/right/joint_states", 1000);
  right_hand_cmds.position[0] = 0.87;
  right_hand_cmds.position[1] = 0.87;
	do
	{
		sleep_time.sleep();
		right_ar10Hand_pub.publish(right_hand_cmds);
		sleep_time.sleep();
	}
	while (!right_ar10Hand_pub.getNumSubscribers());
	sleep_time.sleep();

	ar10_kinematics::IkHand srv;


  srv.request.getJacobian=false;
  srv.request.finger=5; // finger=5 for thumb, finger4=index, and so on
  srv.request.desiredPose.clear();
  // Result_1 /// K=0.1
  // std::vector<double>  desired_pose= {  0.555484,  -0.437536,  -0.555484,   0.437536, -0.0167888,  0.0221666, 0.00355661,  0.0479966};
  // Result_2 /// K=0.1
    // std::vector<double>  desired_pose={ 0.706236, -0.0350761,  -0.706236,  0.0350761,  0.0229575,  0.0377621,  0.0245886,  0.0706021};
    // std::vector<double>  desired_pose={ 0.695946,  -0.125139,  -0.695946,   0.125139, 0.00773141,  0.0393977,  0.0135504,  0.0717592};
    // std::vector<double>  desired_pose={ 0.680947, -0.190554, -0.680947,  0.190554, 0.0135467, 0.0246946, 0.0224075, 0.0563586};



	/////////////////////// FOR THUMB ////////////////////////////////////////////////////
  // std::vector<double>  desired_pose={0.774791,   0.448202,  -0.134365,  -0.425159,  0.0216271, 0.00461104,  0.0419339,  0.0310207}; 
  std::vector<double>  desired_pose={0.960477,   0.235529,   0.122626, -0.0835014,  0.0014986,   -0.02179,  0.0518687,  0.0319472}; 
	/////////////////////// FOR RELATIVE INDEXTIP AND THUMBTIP ////////////////////////////////////////////////////  
  // std::vector<double>  desired_pose={0.61437,  0.0830506,   0.748433,  -0.235586,  0.0131437, 0.00283279, -0.0162992, -0.0165058}; 
  // std::vector<double>  desired_pose={0.614229,  0.0857312,   0.746268,  -0.241779,   0.012737,  0.0030581, -0.0162134, -0.0166017}; 
  // std::vector<double>  desired_pose={0.732594,  -0.201061,   0.526397,  -0.381821, 0.00459042,  0.0240458, -0.0261999, -0.0399749}; 
  for (int i=0; i<desired_pose.size(); i++)
  {
      srv.request.desiredPose.push_back(desired_pose[i]);    
  }


  if (client.call(srv))
  {
      std::cout << "q: " <<  std::endl;
      for(int i=0; i<srv.response.jointPosition.size(); i++)
        std::cout << srv.response.jointPosition[i] <<  std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service finger_ik_sever");
    return 1;
  }
	// ros::WallDuration sleep_time(2);
  // sensor_msgs::JointState right_hand_cmds;



	// right_hand_cmds.position.resize(2);
	// right_hand_cmds.name.resize(2);
	// right_hand_cmds.name[0]= "servo8";
	// right_hand_cmds.name[1]= "servo9";
 //  // ros::Publisher right_ar10Hand_pub = n.advertise<sensor_msgs::JointState>("/ar10_right/ar10/right/joint_states", 1000);
 //  right_hand_cmds.position[0] = srv.response.jointPosition[0];
 //  right_hand_cmds.position[1] = srv.response.jointPosition[1];
	// do
	// {
	// 	sleep_time.sleep();
	// 	right_ar10Hand_pub.publish(right_hand_cmds);
	// 	sleep_time.sleep();
	// }
	// while (!right_ar10Hand_pub.getNumSubscribers());
	// ROS_INFO("commanding right hand index finger for desired position"); 
  return 0;
}