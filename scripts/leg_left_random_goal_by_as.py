#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 24/11/14

@author: Sam Pfeiffer


"""
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
import random
from actionlib import SimpleActionClient

JOINT_NAMES = ['leg_left_1_joint', 'leg_left_2_joint', 'leg_left_3_joint', 'leg_left_4_joint', 'leg_left_5_joint', 'leg_left_6_joint']

CONTROLLER_AS = "/left_leg_controller/follow_joint_trajectory"

def get_n_randoms(n, min, max):
    randoms = []
    max_value = max - min
    for i in range(n):
        randoms.append((random.random() * max_value) + min) # this scales the value between min and max
    return randoms

if __name__ == '__main__':
    rospy.init_node('random_left_leg_goals')
    ctl_as = SimpleActionClient(CONTROLLER_AS, FollowJointTrajectoryAction)
    rospy.loginfo("Connecting to " + CONTROLLER_AS)
    ctl_as.wait_for_server()
    rospy.loginfo("Connected.")
    
    while not rospy.is_shutdown():
        fjtg = FollowJointTrajectoryGoal()
        jt = JointTrajectory()
        #jt.joint_names = ['leg_left_1_joint', 'leg_left_2_joint', 'leg_left_3_joint', 'leg_left_4_joint', 'leg_left_5_joint', 'leg_left_6_joint']
        jt.joint_names = JOINT_NAMES
        jtp = JointTrajectoryPoint()
        jtp.positions = get_n_randoms(len(JOINT_NAMES), -4.0, 4.0)
        jtp.velocities = [0.0] * len(JOINT_NAMES)
        #jtp.accelerations = [0.0] * len(JOINT_NAMES)
        #jtp.effort = [0.0] * len(JOINT_NAMES)
        jtp.time_from_start = rospy.Duration(random.random() + 0.5) # 0.5s to 1.5s
        jt.points.append(jtp)
        fjtg.trajectory = jt
        ctl_as.send_goal_and_wait(fjtg)
        rospy.loginfo("Sent: " + str(jt))
        rospy.sleep(jtp.time_from_start)

