#!/usr/bin/python

import roslib; roslib.load_manifest('hubo_rviz')
import rospy
import time
import xml.dom.minidom
from sensor_msgs.msg import JointState
from hubo_msgs.msg import *
from math import pi

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class JointStatePublisher:

    def __init__(self):
        description = get_param('robot_description')
        print "Loading description file: " + description
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        self.free_joints = {}
        self.latest_state = None
        self.last_time = time.clock()
        # Create all the joints based off of the URDF and assign them joint limits
        # based on their properties
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                    continue
                name = child.getAttribute('name')
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                self.free_joints[name] = joint

        #Setup the HuboState subscriber
        self.hubo_sub = rospy.Subscriber("Hubo/HuboState", JointCommandState, self.hubo_cb)
        #Setup the joint state publisher
        self.hubo_pub = rospy.Publisher('joint_states', JointState)

    def hubo_cb(self, msg):
        new_state = {}
        for joint in msg.joints:
            new_state[joint.name] = joint.position
        self.latest_state = new_state
        self.last_time = time.clock()

    def loop(self):
        hz = get_param("rate", 10) # 10hz
        r = rospy.Rate(hz) 
        
        # Publish Joint States
        while not rospy.is_shutdown():
            #Check to warn if the data from the Hubo is too old
            time_difference = time.clock() - self.last_time
            threshold = 0.05
            if (time_difference > threshold):
                print "*** WARNING - Hubo messages are old ***"
            #Convert the latest state of the robot if it is available
            if (self.latest_state != None):
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                #Convert the HuboState to a series of joint states
                for joint_name in self.latest_state.keys():
                    #Determine if the joint is FREE or DEPENDENT
                    if (joint_name in self.free_joints.keys()):
                        msg.name.append(joint_name)
                        msg.position.append(self.latest_state[joint_name])
                    else:
                        print "*** SHIT! - joint not found in the URDF, bro ***"
                #Check for joints in the URDF that are not in the HuboState
                for joint_name in self.free_joints:
                    if (joint_name not in self.latest_state.keys()):
                        print "*** WHOA! - joint in the URDF not in the message, bro ***"
                        msg.name.append(joint_name)
                        msg.position.append(self.free_joints[joint_name]['zero'])
                self.hubo_pub.publish(msg)
            else:
                print "No valid message received from the Hubo yet"
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                for joint_name in self.free_joints:
                    msg.name.append(joint_name)
                    msg.position.append(self.free_joints[joint_name]['zero'])
                self.hubo_pub.publish(msg)
            #Spin
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('joint_state_publisher_feedback')
    jsp = JointStatePublisher()
    jsp.loop()

