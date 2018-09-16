/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2016 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */
#include <iostream>
#include <rbdl/rbdl.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <dq_robotics/baxter_poseControl_server.h>
// using namespace Eigen;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
int main (int argc, char* argv[]) {
  rbdl_check_api_version (RBDL_API_VERSION);
  Model* model = NULL;
  unsigned int body_a_id, body_b_id, body_c_id;
  Body body_a, body_b, body_c;
  Joint joint_a, joint_b, joint_c;
  model = new Model();
  model->gravity = Math::Vector3d (0., -9.81, 0.);
  body_a = Body (1., Math::Vector3d (0.5, 0., 0.0), Math::Vector3d (1., 1., 1.));
    joint_a = Joint(
    JointTypeRevolute,
    Math::Vector3d (0., 0., 1.)
  );
  
  body_a_id = model->AddBody(0, Xtrans(Math::Vector3d(0., 0., 0.)), joint_a, body_a);
  
  body_b = Body (1., Math::Vector3d (0., 0.5, 0.), Math::Vector3d (1., 1., 1.));
    joint_b = Joint (
    JointTypeRevolute,
    Math::Vector3d (0., 0., 1.)
  );
  
  body_b_id = model->AddBody(body_a_id, Xtrans(Math::Vector3d(1., 0., 0.)), joint_b, body_b);
  
  body_c = Body (0., Math::Vector3d (0.5, 0., 0.), Math::Vector3d (1., 1., 1.));
    joint_c = Joint (
    JointTypeRevolute,
    Math::Vector3d (0., 0., 1.)
  );
  
  body_c_id = model->AddBody(body_b_id, Xtrans(Math::Vector3d(0., 1., 0.)), joint_c, body_c);
  VectorNd Q = VectorNd::Zero (model->dof_count);
  VectorNd QDot = VectorNd::Zero (model->dof_count);
  VectorNd Tau = VectorNd::Zero (model->dof_count);
  VectorNd QDDot = VectorNd::Zero (model->dof_count);
  ForwardDynamics (*model, Q, QDot, Tau, QDDot);
  std::cout << QDDot.transpose() << std::endl;
  delete model;
  return 0;
}