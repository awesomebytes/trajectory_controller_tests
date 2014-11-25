#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 24/11/14

@author: Sam Pfeiffer


"""
# System imports
import random
import threading
import copy

# ROS imports
import rospy
# messages
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# services
from controller_manager_msgs.srv import ListControllers, ListControllersRequest#, ListControllersResponse


# local imports
# Borrowed from Adolfo Rodriguez Tsouroukdissian
# at https://github.com/ros-controls/ros_controllers/blob/indigo-devel/rqt_joint_trajectory_controller/src/rqt_joint_trajectory_controller/joint_limits_urdf.py
# as the urdf_dom_py interface gives errors
from joint_limits_urdf import get_joint_limits

CONTROLLER_MNGR_SRV = "/controller_manager/list_controllers"

def get_valid_random_values(joint_names, joints_dict):
    joint_vals = []
    for joint in joint_names:
        if joints_dict[joint]['has_position_limits']:
            min_p = joints_dict[joint]['min_position']
            max_p = joints_dict[joint]['max_position']
            max_value = max_p - min_p
            joint_vals.append((random.random() * max_value) + min_p)
        else: # If there is no joint limit set in the URDF
            rospy.logwarn("No joint limit set, making up values from -4.0 to 4.0")
            joint_vals.append((random.random() * 8.0) + -4.0) # just give some value between -4.0 and 4.0
    return joint_vals
        

def spam_controller(controller_name, joint_names):
    cmd_topic = "/" + controller_name + "/command"
    rospy.loginfo("Going to spam controller: " + controller_name + 
                  "\n in command topic: " + cmd_topic + 
                  "\n with joints: " + str(joint_names) )
    ctl_pub = rospy.Publisher(cmd_topic, JointTrajectory)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        jt = JointTrajectory()
        jt.joint_names = joint_names
        jtp = JointTrajectoryPoint()
        jtp.positions = get_valid_random_values(joint_names, joints_dict)
        jtp.velocities = [0.0] * len(joint_names)
        jtp.accelerations = [0.0] * len(joint_names)
        jtp.time_from_start = rospy.Duration(random.random() + 0.5) # 0.5s to 1.5s
        jt.points.append(jtp)
        ctl_pub.publish(jt)
        rospy.logdebug("Sent to: " + cmd_topic + "\n" + str(jt))
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
    joints_dict = get_joint_limits()
    #print "this is joints_dict: " + str(joints_dict)
    threads = []
    for cs in resp.controller: # For every controller, create a random spammer
        #cs = ControllerState()
        if len(cs.resources) > 0: # If the controller controls any joint only
            threads.append( threading.Thread(target=spam_controller,
                                              args=(copy.deepcopy(cs.name),
                                                    copy.deepcopy(cs.resources)
                                                    )
                                             ) 
                           )
            threads[-1].setDaemon(True)
            threads[-1].start()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
