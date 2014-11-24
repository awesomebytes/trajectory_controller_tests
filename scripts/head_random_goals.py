#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 24/11/14

@author: Sam Pfeiffer


"""
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random

JOINT_NAMES = ["head_1_joint", "head_2_joint"]

CONTROLLER_CMD_TOPIC = "/head_controller/command"

def get_n_randoms(n, min, max):
    randoms = []
    max_value = max - min
    for i in range(n):
        randoms.append((random.random() * max_value) + min) # this scales the value between min and max
    return randoms

if __name__ == '__main__':
    rospy.init_node('random_head_goals')
    ctl_pub = rospy.Publisher(CONTROLLER_CMD_TOPIC, JointTrajectory)
    rospy.sleep(0.1)
    
    while not rospy.is_shutdown():
        jt = JointTrajectory()
        jt.joint_names = JOINT_NAMES
        jtp = JointTrajectoryPoint()
        jtp.positions = get_n_randoms(len(JOINT_NAMES), -1.0, 1.0)
        jtp.velocities = [0.0] * len(JOINT_NAMES)
        jtp.accelerations = [0.0] * len(JOINT_NAMES)
        jtp.time_from_start = rospy.Duration(random.random() + 0.5) # 0.5s to 1.5s
        jt.points.append(jtp)
        ctl_pub.publish(jt)
        rospy.loginfo("Sent: " + str(jt))
        rospy.sleep(jtp.time_from_start)

