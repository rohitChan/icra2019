cmake_minimum_required(VERSION 2.8.3)
project(dq_robotics C CXX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  baxter_core_msgs
  roslib
  fcl_capsule
  kdl_parser
  message_generation
  baxter_interface
  tf
  joint_limits_interface
  urdf
  baxter_core_msgs
  visualization_msgs
  geometry_msgs
)

list(APPEND CMAKE_MODULE_PATH ${controlit_cmake_DIR}/../../../../src/controlit/controlit_cmake/cmake)

catkin_python_setup()

 add_service_files(
   FILES
   # IkHand.srv
   BaxterControl.srv
   Grasp.srv
#   Service1.srv
#   Service2.srv
 )


add_message_files(
  FILES
  DualArmPlotData.msg
  ControllerPerformance.msg
  ResolveAccControlPrfmnc.msg
  ResolveAccControlPrfmnc_processed.msg
)
 
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  config/acc_control_params.cfg
  #...
)

# add_service_files(
#   FILES
#   GetIKSolution.srv
#   SetJointGoals.srv
#   GetJointTrajectory.srv
#   BaxterControl.srv
# )




catkin_package(
  CATKIN_DEPENDS
  message_runtime
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  baxter_core_msgs
  roslib
  kdl_parser
  baxter_interface
  tf
  tf_conversions
  joint_limits_interface
  urdf
  orocos_kinematics_dynamics
  INCLUDE_DIRS include
  LIBRARIES dq_operations dq_controller baxter_pose_Control_server_lib ar10_kinematics_lib
  DEPENDS orocos_kdl
) 


find_package(urdf REQUIRED) 
find_package(Eigen3 REQUIRED) 
# find_package(orocos_kdl REQUIRED)
# find_package(Rbdl REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${GSTREAMER_INCLUDE_DIRS}
                    ${fcl_capsule_INCLUDE_DIRS})


add_library(graph include/dq_robotics/graph.cpp)
add_dependencies(graph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(dq_operations include/dq_robotics/DQoperations.cpp)
add_dependencies(dq_operations ${control_msgs_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(dq_controller include/dq_robotics/dq_controller.cpp)
add_dependencies(dq_controller ${control_msgs_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(jacDotSolver include/dq_robotics/jacDotSolver.cpp)
add_dependencies(jacDotSolver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_library(chainjnttojacsolver include/dq_robotics/chainjnttojacsolver.cpp)
# add_dependencies(chainjnttojacsolver ${control_msgs_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_library(chainjnttojacdotsolver include/dq_robotics/chainjnttojacdotsolver.cpp)
# add_dependencies(chainjnttojacdotsolver ${control_msgs_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(baxter_dq include/dq_robotics/baxter_dq.cpp)
add_dependencies(baxter_dq ${control_msgs_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(baxter_dq ${catkin_LIBRARIES})

add_library(baxter_pose_Control_server_lib src/baxter_poseControl_server.cpp)
add_dependencies(baxter_pose_Control_server_lib ${control_msgs_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(baxter_pose_Control_server_lib ${catkin_LIBRARIES}  dq_operations baxter_dq dq_controller)

add_executable(baxterPoseControl_server src/baxter_poseControl_server.cpp)
target_link_libraries(baxterPoseControl_server ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller)

# add_executable(manipulationTaskController src/manipulation_task_controller.cpp)
# target_link_libraries(manipulationTaskController ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller)

# add_library(ar10_kinematics_lib src/ar10_kinematics.cpp)
# add_dependencies(ar10_kinematics_lib ${control_msgs_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ar10_kinematics_lib ${catkin_LIBRARIES}  dq_operations baxter_dq dq_controller)

# add_executable(ar10Kinematics src/ar10_kinematics.cpp)
# # add_dependencies(ar10Kinematics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ar10Kinematics ${catkin_LIBRARIES} dq_operations dq_controller)

# add_executable(baxter_ar10_kinematics src/baxter_ar10_kinematics.cpp)
# add_dependencies(baxter_ar10_kinematics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(baxter_ar10_kinematics ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller baxter_pose_Control_server_lib ar10_kinematics_lib)


# add_executable(baxter_AbsRelative_Control src/baxter_AbsRelative_Control.cpp)
# add_dependencies(baxter_AbsRelative_Control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(baxter_AbsRelative_Control ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller baxter_pose_Control_server_lib ar10_kinematics_lib)

# add_executable(baxterTaskSpaceClient src/test/baxterTaskSpaceClient.cpp)
# add_dependencies(baxterTaskSpaceClient ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(baxterTaskSpaceClient ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller baxter_pose_Control_server_lib ar10_kinematics_lib)

# add_executable(ar10Client src/ar10_client.cpp)
# add_dependencies(ar10Client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ar10Client ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller baxter_pose_Control_server_lib ar10_kinematics_lib)

# add_executable(fake_ar10_jointStates src/test/fake_ar10_jointStates.cpp)
# add_dependencies(fake_ar10_jointStates ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(fake_ar10_jointStates ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller baxter_pose_Control_server_lib ar10_kinematics_lib)

# add_executable(rviz_shape_publisher src/test/rviz_shape_publisher.cpp)
# target_link_libraries(rviz_shape_publisher ${catkin_LIBRARIES})

# add_executable(test_graph src/test/test_graph.cpp)
# target_link_libraries(test_graph ${catkin_LIBRARIES})

# add_executable(grasp_client_test src/test/grasp_client_test.cpp)
# add_dependencies(grasp_client_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(grasp_client_test ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller baxter_pose_Control_server_lib ar10_kinematics_lib)

# add_executable(kdl_controller src/kdl_controller.cpp)
# add_dependencies(kdl_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(kdl_controller ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller baxter_pose_Control_server_lib ar10_kinematics_lib)

# add_executable(kdl_controller_new src/kdl_controller_new.cpp)
# add_dependencies(kdl_controller_new ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(kdl_controller_new ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller baxter_pose_Control_server_lib ar10_kinematics_lib jacDotSolver)

add_executable(resolved_acc_control src/resolvedAccControl/resolved_acc_control.cpp)
add_dependencies(resolved_acc_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(resolved_acc_control ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller baxter_pose_Control_server_lib jacDotSolver)

add_executable(new_traj_resolved_acc_control src/resolvedAccControl/new_traj_resolved_acc_control.cpp)
add_dependencies(new_traj_resolved_acc_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(new_traj_resolved_acc_control ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller baxter_pose_Control_server_lib jacDotSolver)

add_executable(disable_gravity_baxSDK src/resolvedAccControl/disable_gravity_baxSDK.cpp)
add_dependencies(disable_gravity_baxSDK ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(disable_gravity_baxSDK ${catkin_LIBRARIES})

add_executable(result_analyzer_ResolcedAccControl src/resolvedAccControl/result_analyzer_ResolcedAccControl.cpp)
add_dependencies(result_analyzer_ResolcedAccControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(result_analyzer_ResolcedAccControl ${catkin_LIBRARIES})

# add_executable(test_rbdl_ros src/test/test_rbdl_ros.cpp)
# target_link_libraries(test_rbdl_ros ${catkin_LIBRARIES} dq_operations)



# add_dependencies(manipulationTaskController ${control_msgs_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(baxterPoseControl_server_2 src/baxter_poseControl_server_2.cpp)
# target_link_libraries(baxterPoseControl_server_2 ${catkin_LIBRARIES} dq_operations baxter_dq dq_controller)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/dualQuaternion_robotics.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/dualQuaternion_robotics_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dualQuaternion_robotics.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
