#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Example: file playback
"""

import argparse
import sys
import rospy
import baxter_interface
import numpy
from sensor_msgs.msg import JointState
from baxter_interface import CHECK_VERSION
from baxter_trajectory_interface.srv import *


def setJointPositions(joint_name_right, joint_name_left, joint_desired_pose_right, joint_desired_pose_left):
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')

    rate = rospy.Rate(1000)

    position_right={
        joint_name_right[0]: joint_desired_pose_right[0],
        joint_name_right[1]: joint_desired_pose_right[1],
        joint_name_right[2]: joint_desired_pose_right[2],
        joint_name_right[3]: joint_desired_pose_right[3],
        joint_name_right[4]: joint_desired_pose_right[4],
        joint_name_right[5]: joint_desired_pose_right[5],
        joint_name_right[6]: joint_desired_pose_right[6]
    }

    position_left={
        joint_name_left[0]: joint_desired_pose_left[0],
        joint_name_left[1]: joint_desired_pose_left[1],
        joint_name_left[2]: joint_desired_pose_left[2],
        joint_name_left[3]: joint_desired_pose_left[3],
        joint_name_left[4]: joint_desired_pose_left[4],
        joint_name_left[5]: joint_desired_pose_left[5],
        joint_name_left[6]: joint_desired_pose_left[6]
    }
   
    left.move_to_joint_positions(position_left)
    right.move_to_joint_positions(position_right)    
    left.set_joint_positions(position_left)
    rate.sleep()
    right.set_joint_positions(position_right)
    rate.sleep()
    return;

def getCurrentJointState(joint_name_right, joint_name_left):
    current_joint_states=rospy.wait_for_message("/robot/joint_states", JointState)    
    joint_states_right=[0, 0, 0, 0, 0, 0, 0]
    joint_states_left=[0, 0, 0, 0, 0, 0, 0]
    for i in range(len(joint_name_right)):
        for j in range(len(current_joint_states.name)):
            if current_joint_states.name[j]==joint_name_right[i]:
                joint_states_right[i]=current_joint_states.position[j]
    for i in range(len(joint_name_left)):
        for j in range(len(current_joint_states.name)):
            if current_joint_states.name[j]==joint_name_left[i]:
                joint_states_left[i]=current_joint_states.position[j]
    print joint_states_right
    print joint_states_left
    return (joint_states_right, joint_states_left);
    
def printSetPositionResult(joint_desired_state_right, joint_states_current_right, joint_desired_state_left, joint_states_current_left, joint_name_right, joint_name_left):
    error=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    error_joint_right=[0, 0, 0, 0, 0, 0, 0]
    error_joint_left=[0, 0, 0, 0, 0, 0, 0]
    for x in range(len(joint_states_current_right)):
        error_joint_right[x]=(joint_states_current_right[x]-joint_desired_state_right[x])
        if (abs(error_joint_right[x])>0.05):
            print "Target for",  joint_name_right[x],"not achieved"
    for x in range(len(joint_states_current_left)):
        error_joint_left[x]=(joint_states_current_left[x]-joint_desired_state_left[x])
        if (abs(error_joint_left[x])>0.05):
            print "Target for",  joint_name_left[x],"not achieved"
    error_joint_right= error_joint_right[::-1]
    error_joint_left=error_joint_left[::-1]               
    error=numpy.concatenate((error_joint_right,error_joint_left),axis=0)
    print error
    return error

def set_joint_positions_server():
    s = rospy.Service('set_joint_position_gazebo', SetJointGoals, setClientJointPosition)
    print "Ready to set joint positions..."
    rospy.spin()    

def setClientJointPosition(req):
    joint_name_right=["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]
    #left_e1 is special. joint limit [-0.05] is the reason. keeping it zero for now 
    joint_name_left=["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
    joint_desired_state_right=req.desiredJointGoals.position[0:7]
    joint_desired_state_right=joint_desired_state_right[::-1]
    joint_desired_state_right=[ -x for x in joint_desired_state_right]
    joint_desired_state_left=req.desiredJointGoals.position[7:14]
    # joint_desired_state_left=[ -x for x in mylist]
    setJointPositions(joint_name_right, joint_name_left, joint_desired_state_right, joint_desired_state_left)
    joint_states_current_right, joint_states_current_left=getCurrentJointState(joint_name_right, joint_name_left)
    error=printSetPositionResult(joint_desired_state_right, joint_states_current_right, joint_desired_state_left, joint_states_current_left, joint_name_right, joint_name_left)
    response=SetJointPositionResponse()
    response.error=error
    return response

def main():
    print("Initializing node... ")
    rospy.init_node("set_joint_positions")
#################Resposible for setting joint positions for Baxter ############################################    
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()

    print("Enabling robot... ")
    rs.enable()
    #right_e1 is special.   joint limit [-0.05] is the reason. keeping it zero for now 

    #left_e1 is special. joint limit [-0.05] is the reason. keeping it zero for now 
    # joint_name_left=["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
    # joint_name_right=["right_e0", "right_e1", "right_s0", "right_s1", "right_w0", "right_w1", "right_w2"]

    ################################################################################################################### 
    # joint_name_left=["left_e0", "left_e1", "left_s0", "left_s1", "left_w0", "left_w1", "left_w2"]

    # joint_desired_state_right=[0.7410703227630888, 1.5362719506658014, 0.01001108907777759, -0.7006024655803085, 0.9735489859081357, 1.447864944629445, -0.4520477771263609]
    # joint_desired_state_left=[-0.02477285952504804, 1.5454976873355646, -0.34985937977411385, -0.862626895167157, -1.3115418478918244, 1.8487306917056667, -0.892602931497076]    
    ###################################################################################################################
    joint_name_left = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    joint_desired_state_left= [0.041363752507206364, 1.9348352917089535, -0.2726798000286923, -0.5658538514940457, -1.1000522708348006, 1.7430249026235884, -0.1936378991920895]
    joint_name_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    joint_desired_state_right = [0.6068619915741271, 1.911764079997937, -0.13185736132604742, -0.4816721820621366, 0.790763174233601, 1.304727973182544, -1.0247054998116605]

    
    joint_name_left = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    joint_desired_state_left= [0.4345000581685434, 1.4078108680818384, -1.117505003974524, -0.8383205005793786, -0.35128160042575973, 1.0530778108833365, 0.20401944478876002]
    joint_name_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    joint_desired_state_right = [-0.33824276372873374, 1.5082866096883332, 1.035053536625683, -0.9276748814737039, 0.25080585881926515, 1.0895098545956152, -0.07209709703061444]
    setJointPositions(joint_name_right, joint_name_left, joint_desired_state_right, joint_desired_state_left)
######################################################################################################################
#################Resposible for getting joint positions for Baxter from /robot/joint_states topic ############################################  

    joint_states_current_right, joint_states_current_left=getCurrentJointState(joint_name_right, joint_name_left)    
    printSetPositionResult(joint_desired_state_right, joint_states_current_right, joint_desired_state_left, joint_states_current_left,joint_name_right, joint_name_left)

    # set_joint_positions_server()

if __name__ == '__main__':
    main()
    

# name: ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
# position: [0.4345000581685434, 1.4078108680818384, -1.117505003974524, -0.8383205005793786, -0.35128160042575973, 1.0530778108833365, 0.20401944478876002, -0.33824276372873374, 1.5082866096883332, 1.035053536625683, -0.9276748814737039, 0.25080585881926515, 1.0895098545956152, -0.07209709703061444]
