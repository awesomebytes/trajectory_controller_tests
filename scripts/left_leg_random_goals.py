#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 24/11/14

@author: Sam Pfeiffer


"""
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random

JOINT_NAMES = ['left_leg_1_joint', 'left_leg_2_joint', 'left_leg_3_joint',
                'left_leg_4_joint', 'left_leg_5_joint', 'left_leg_6_joint']


CONTROLLER_CMD_TOPIC = "/left_leg_controller/command"

def get_n_randoms(n, min, max):
    randoms = []
    max_value = max - min
    for i in range(n):
        randoms.append((random.random() * max_value) + min) # this scales the value between min and max
    return randoms

if __name__ == '__main__':
    rospy.init_node('random_left_leg_goals')
    ctl_pub = rospy.Publisher(CONTROLLER_CMD_TOPIC, JointTrajectory)
    rospy.sleep(0.1)
    
    while not rospy.is_shutdown():
        jt = JointTrajectory()
        jt.joint_names = JOINT_NAMES
        jtp = JointTrajectoryPoint()
        jtp.positions = get_n_randoms(len(JOINT_NAMES), -1.0, 4.0)
        jtp.velocities = [0.0] * len(JOINT_NAMES)
        jtp.accelerations = [0.0] * len(JOINT_NAMES)
        #jtp.effort = [0.0] * len(JOINT_NAMES)
        jtp.time_from_start = rospy.Duration(random.random() + 0.5) # 0.5s to 1.5s
        jt.points.append(jtp)
        ctl_pub.publish(jt)
        rospy.loginfo("Sent: " + str(jt))
        rospy.sleep(jtp.time_from_start)


# rostopic pub /left_leg_controller/command trajectory_msgs/JointTrajectory "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# joint_names: ['leg_left_1_joint', 'leg_left_2_joint', 'leg_left_3_joint', 'leg_left_4_joint', 'leg_left_5_joint', 'leg_left_6_joint']
# points:
# - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#   velocities: [0]
#   accelerations: [0]
#   effort: [0]                           
#   time_from_start: {secs: 3, nsecs: 0}" 
# GIVES: 
#Size mismatch in trajectory point position, velocity or acceleration data.

# rostopic pub /left_leg_controller/command trajectory_msgs/JointTrajectory "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# joint_names: ['leg_left_1_joint', 'leg_left_2_joint', 'leg_left_3_joint', 'leg_left_4_joint', 'leg_left_5_joint', 'leg_left_6_joint']
# points:
# - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#   velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#   accelerations: [0]
#   effort: [0]
#   time_from_start: {secs: 3, nsecs: 0}"
#   GIVES:
#   Size mismatch in trajectory point position, velocity or acceleration data.

# rostopic pub /left_leg_controller/command trajectory_msgs/JointTrajectory "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# joint_names: ['leg_left_1_joint', 'leg_left_2_joint', 'leg_left_3_joint', 'leg_left_4_joint', 'leg_left_5_joint', 'leg_left_6_joint']
# points:
# - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#   velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#   accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#   effort: [0]
#   time_from_start: {secs: 3, nsecs: 0}" 
#   GIVES: NO ERROR, BUT NOTHING HAPPENS

