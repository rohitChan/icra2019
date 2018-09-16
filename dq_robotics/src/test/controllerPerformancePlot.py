#!/usr/bin/env python
import rospy
from dq_robotics.msg import ControllerPerformance
import numpy as np
import time
import matplotlib
matplotlib.use('GTKAgg')
from matplotlib import pyplot as plt

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)
def xyzRollPitchYaw_from_dq(x)    
    
def callback(data):
    global time, time_array, abs_des_x, abs_des_y, abs_des_z, abs_des_roll, abs_des_pitch, abs_des_yaw, rel_des_x, rel_des_y, rel_des_z, rel_des_roll, rel_des_pitch, rel_des_yaw, abs_curr_x, abs_curr_y, abs_curr_z, abs_curr_roll, abs_curr_pitch, abs_curr_yaw, rel_curr_x, rel_curr_y, rel_curr_z, rel_curr_roll, rel_curr_pitch, rel_curr_yaw, abs_des_array_x, abs_des_array_y, abs_des_array_z, abs_des_array_roll, abs_des_array_pitch, abs_des_array_yaw, rel_des_array_x, rel_des_array_y, rel_des_array_z, rel_des_array_roll, rel_des_array_pitch, rel_des_array_yaw, abs_curr_array_x, abs_curr_array_y, abs_curr_array_z, abs_curr_array_roll, abs_curr_array_pitch, abs_curr_array_yaw, rel_curr_array_x, rel_curr_array_y, rel_curr_array_z, rel_curr_array_roll, rel_curr_array_pitch, rel_curr_array_yaw, normError_abs, normError_rel, normError_array_abs, normError_array_rel  
    time = data.timeStamp
    time_array.append(time)

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same  
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    global time, time_array, abs_des_x, abs_des_y, abs_des_z, abs_des_roll, abs_des_pitch, abs_des_yaw, rel_des_x, rel_des_y, rel_des_z, rel_des_roll, rel_des_pitch, rel_des_yaw, abs_curr_x, abs_curr_y, abs_curr_z, abs_curr_roll, abs_curr_pitch, abs_curr_yaw, rel_curr_x, rel_curr_y, rel_curr_z, rel_curr_roll, rel_curr_pitch, rel_curr_yaw, abs_des_array_x, abs_des_array_y, abs_des_array_z, abs_des_array_roll, abs_des_array_pitch, abs_des_array_yaw, rel_des_array_x, rel_des_array_y, rel_des_array_z, rel_des_array_roll, rel_des_array_pitch, rel_des_array_yaw, abs_curr_array_x, abs_curr_array_y, abs_curr_array_z, abs_curr_array_roll, abs_curr_array_pitch, abs_curr_array_yaw, rel_curr_array_x, rel_curr_array_y, rel_curr_array_z, rel_curr_array_roll, rel_curr_array_pitch, rel_curr_array_yaw, normError_abs, normError_rel, normError_array_abs, normError_array_rel 
    rospy.Subscriber("/dq_robotics/results", ControllerPerformance, callback)
    time_array = []
    abs_des_array_x = []
    abs_des_array_y = []
    abs_des_array_z = []
    abs_des_array_roll = []
    abs_des_array_pitch = []
    abs_des_array_yaw = []
    abs_curr_array_x = []
    abs_curr_array_y = []
    abs_curr_array_z = []
    abs_curr_array_roll = []
    abs_curr_array_pitch = []
    abs_curr_array_yaw = []
    rel_des_array_x = []
    rel_des_array_y = []
    rel_des_array_z = []
    rel_des_array_roll = []
    rel_des_array_pitch = []
    rel_des_array_yaw = []
    rel_curr_array_x = []
    rel_curr_array_y = []
    rel_curr_array_z = []
    rel_curr_array_roll = []
    rel_curr_array_pitch = []
    rel_curr_array_yaw = []
    normError_array_abs = []    
    normError_array_rel = []    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()