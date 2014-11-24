#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 24/11/14

@author: Sam Pfeiffer


"""
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers, ListControllersRequest, ListControllersResponse
from controller_manager_msgs.msg import ControllerState
import random
import threading

CONTROLLER_MNGR_SRV = "/controller_manager/list_controllers"
JOINT_NAMES = ["head_1_joint", "head_2_joint"]

CONTROLLER_CMD_TOPIC = "/head_controller/command"

def get_n_randoms(n, min, max):
    randoms = []
    max_value = max - min
    for i in range(n):
        randoms.append((random.random() * max_value) + min) # this scales the value between min and max
    return randoms

def spam_controller(controller_name, joint_names):
    rospy.loginfo("Going to spam controller:" + controller_name + 
                  "\n   with joints: " + str(joint_names))
    ctl_pub = rospy.Publisher("/" + cs.name + "/command", JointTrajectory)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        jt = JointTrajectory()
        jt.joint_names = joint_names
        jtp = JointTrajectoryPoint()
        jtp.positions = get_n_randoms(len(joint_names), -4.0, 4.0) # TODO: get min and max values of joints
        jtp.velocities = [0.0] * len(joint_names)
        jtp.accelerations = [0.0] * len(joint_names)
        jtp.time_from_start = rospy.Duration(random.random() + 0.5) # 0.5s to 1.5s
        jt.points.append(jtp)
        ctl_pub.publish(jt)
        rospy.loginfo("Sent: " + str(jt))
        rospy.sleep(jtp.time_from_start)

if __name__ == '__main__':
    rospy.init_node('cntl_crazier_')
    ctl_mngr_srv = rospy.ServiceProxy(CONTROLLER_MNGR_SRV, ListControllers)
    rospy.loginfo("Connecting to " + CONTROLLER_MNGR_SRV)
    ctl_mngr_srv.wait_for_service()
    rospy.loginfo("Connected.")
    
    req = ListControllersRequest()
    resp = ctl_mngr_srv.call(req)
    #ListControllersResponse()
    ctl_mngr_srv.close()
    threads = []
    for cs in resp.controller:
        #cs = ControllerState()
        if len(cs.resources) > 0: # If the controller controls any joint
            threads.append( threading.Thread(target=spam_controller, args=(cs.name,cs.resources)) )
            threads[-1].setDaemon(True)
            threads[-1].start()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
#     for thread in threads:
#         thread.join()

