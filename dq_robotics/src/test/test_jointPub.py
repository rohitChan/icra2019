#!/usr/bin/env python

import numpy as np
import PyKDL

import rospy

import baxter_interface
import sensor_msgs.msg

from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF

def main():
    pub = rospy.Publisher('baxter/joint_states', sensor_msgs.msg.JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)   

    limb_interface_right = baxter_interface.Limb("right")
    limb_interface_left = baxter_interface.Limb("left")
        
    joint_names_right = limb_interface_right.joint_names()
    joint_names_left = limb_interface_left.joint_names()
    # rate = rospy.Rate(10000)
    while not rospy.is_shutdown():
        limb_interface_right = baxter_interface.Limb("right")
        limb_interface_left = baxter_interface.Limb("left")
        joint_names_right = limb_interface_right.joint_names()
        joint_names_left = limb_interface_left.joint_names()        
        joint_angles_right = limb_interface_right.joint_angles()
        # print joint_angles_right
        joint_angles_left = limb_interface_left.joint_angles()          
        # print joint_angles_left
        # print "hello"
        joint_angles_left_values = [
            joint_angles_left[joint_name] for joint_name in joint_names_left]
        # print "hello222"    
        joint_angles_right_values = [
            joint_angles_right[joint_name] for joint_name in joint_names_right]    
        
        baxter_joints_msg = sensor_msgs.msg.JointState()
        baxter_joints_msg.header.stamp = rospy.Time.now()
        baxter_joints_msg.name = joint_names_left
        baxter_joints_msg.name.extend(joint_names_right)
        baxter_joints_msg.position = joint_angles_left_values
        baxter_joints_msg.position.extend(joint_angles_right_values)
        # print("joint angles:\n{}".format(baxter_joints_msg))
        pub.publish(baxter_joints_msg)
        # rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass