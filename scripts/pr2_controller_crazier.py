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
import subprocess

# ROS imports
import rospy
import rostopic
# messages
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.msg import ControllerState
# services
from pr2_mechanism_msgs.srv import ListControllers, ListControllersRequest#, ListControllersResponse
from control_msgs.srv import QueryTrajectoryState, QueryTrajectoryStateRequest, QueryTrajectoryStateResponse


# local imports
# Borrowed from Adolfo Rodriguez Tsouroukdissian
# at https://github.com/ros-controls/ros_controllers/blob/indigo-devel/rqt_joint_trajectory_controller/src/rqt_joint_trajectory_controller/joint_limits_urdf.py
# as the urdf_dom_py interface gives errors
from joint_limits_urdf import get_joint_limits

CONTROLLER_MNGR_SRV = "/pr2_controller_manager/list_controllers"

def get_valid_random_values(joint_names, joints_dict):
    joint_vals = []
    for joint in joint_names:
        if joints_dict[joint]['has_position_limits']:
            min_p = joints_dict[joint]['min_position']
            max_p = joints_dict[joint]['max_position']
            max_value = max_p - min_p
            joint_vals.append((random.random() * max_value) + min_p)
        else: # If there is no joint limit set in the URDF
            rospy.logwarn("No joint limit set for '" + joint + "', making up values from -4.0 to 4.0")
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

def get_joints_of_controller(controller_name):
    query_state_srv = "/" + controller_name + "/query_state"
    rospy.loginfo("Getting joints from: " + query_state_srv)
    query_srv = rospy.ServiceProxy(query_state_srv, QueryTrajectoryState)
    query_srv.wait_for_service()
    req = QueryTrajectoryStateRequest()
    req.time = rospy.Time.now() + rospy.Duration(999.9) # must be very in the future for some reason
    resp = query_srv.call(req)
    #resp = QueryTrajectoryStateResponse()
    joint_names = resp.name
    return joint_names
    

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
    for cs in resp.controllers: # For every controller, create a random spammer
        # cs is a string with the name of the controller, now check if has command interface
        #published_topics = rostopic.rospy.get_published_topics() #for some reason is not the same than
        # doing the bash call
        ps = subprocess.Popen(["rostopic", "list" ], stdout=subprocess.PIPE)
        controller_related_topics = subprocess.check_output(('grep', cs), stdin=ps.stdout) #, stdout=subprocess.PIPE)
        #print "controller_related_topics to " + cs + ": " + str(controller_related_topics)
        for topic_name in controller_related_topics.split():
            #print "topic_name: " + topic_name
            if "command" in topic_name:
                # We need to check if there is a query state service to ask him
                # what joints does this controller control
                ps2 = subprocess.Popen(["rosservice", "list"], stdout=subprocess.PIPE)
                with_query_controllers = subprocess.check_output(('grep', 'query_state'), stdin=ps2.stdout)
                #print "services with query_state: " + str(with_query_controllers)
                joint_names = []
                for srv_name in with_query_controllers.split():
                    #print "service: " + srv_name
                    if cs in srv_name:
                        print "Controller '" + cs + "' has topic /command interface and service /query_state interface!"
                        joint_names = get_joints_of_controller(cs)
                if not joint_names:
                    continue
                
                threads.append( threading.Thread(target=spam_controller,
                                                  args=(copy.deepcopy(cs),
                                                        copy.deepcopy(joint_names)
                                                        )
                                                 ) 
                               )
                threads[-1].setDaemon(True)
                threads[-1].start()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
