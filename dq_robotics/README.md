# IFAC WC 2020
Add the package dq_robots in your catkin workspace and catkin_make your workspace. The package requires KDL and Eigen libraries and works with ROS INDIGO. The package was designed to work with Baxter robot in simulation as well as with the real robot.

Steps to repeat the experiments :

1. For real robot: Connect to the robot, and execute the commands from step 2 in the Baxter SDK environment. 

For simulation: 

roslaunch baxter_gazebo baxter_world.launch

2. Launch the robot parameters and controller parameters, rviz and dynamic reconfigure for gain adjustments.

roslaunch dq_robotics baxterposeControlServer.launch

3. Launch the controller node.

rosrun dq_robotics resolved_acc_control

4. Change the controller types from the drop-down menu at the bottom of the window of dynamic reconfigure. For decoupled controller select dq_controller, and for coupled select kdl_controller_quatVec.

5. Adjust the gains and then click on reset_robot. Then click on start_controller. The right arm will move to the starting position.

6. Click on traj_control and uncheck the reset_robot to start trajectory tracking.

7. For data collection for performance computation of the controllers in terms of accuracy and time, subscribe to /dq_robotics/trajCntrlResults topic.

8. The recorded data for /dq_robotics/trajCntrlResults topic can be converted to csv file using following command via rosbag or directly:

rosrun dq_robotics result_analyzer_ResolcedAccControl

The csv file is store in dq_robotics/src/resolvedAccControl/result/ with the name specified in the cpp file src/resolvedAccControl/result_analyzer_ResolcedAccControl.cpp.





