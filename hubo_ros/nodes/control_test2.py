#!/usr/bin/python

import roslib; roslib.load_manifest('ros_hubo_ach')
import rospy
import time
from hubo_ros_msgs.msg import *
from math import pi

class Tester:

    def __init__(self):
        #Setup the joint state publisher
        self.hubo_pub = rospy.Publisher('Hubo/HuboCommand', HuboCommand)
        #self.latest_state = None
        self.raw_joint_names = ["HPY", "not in urdf1", "HNR", "HNP", "LSP", "LSR", "LSY", "LEP", "LWY", "not in urdf2", "LWP", "RSP", "RSR", "RSY", "REP", "RWY", "not in urdf3", "RWP", "not in ach1", "LHY", "LHR", "LHP", "LKP", "LAP", "LAR", "not in ach1", "RHY", "RHR", "RHP", "RKP", "RAP", "RAR", "not in urdf4", "not in urdf5", "not in urdf6", "not in urdf7", "not in urdf8", "not in urdf9", "not in urdf10", "not in urdf11", "not in urdf12", "not in urdf13", "unknown1", "unknown2", "unknown3", "unknown4", "unknown5", "unknown6", "unknown7", "unknown8"]
        #Setup the HuboState subscriber
        #self.hubo_sub = rospy.Subscriber("Hubo/HuboState", HuboState, self.hubo_cb)
        #hz = get_param("rate", 16)
        #r = rospy.Rate(hz) 
        #while not rospy.is_shutdown():
        #    if (self.latest_state != None):
        #        test_command = self.convertToCommand(self.latest_state)
        #        #Publish the message
        #        self.hubo_pub.publish(test_command)
        #    r.sleep()
        self.cycleTester()

    def cycleTester(self):
        for joint_name in self.raw_joint_names:
            new_command = HuboCommand()
            joint_commands = []
            value = float(raw_input("Enter a value for the " + joint_name + " joint: "))
            new_joint = HuboJointCommand()
            new_joint.name = joint_name
            new_joint.position = value
            new_joint.velocity = 0.0
            joint_commands.append(new_joint)
            new_command.joints = joint_commands
            new_command.num_joints = len(joint_commands)
            self.hubo_pub.publish(new_command)
        print "Test done"

    def waistTester(self, value):
        new_command = HuboCommand()
        joint_commands = []
        #Set the waist joint manually
        new_joint = HuboJointCommand()
        new_joint.name = "HPY"
        new_joint.position = value
        new_joint.velocity = 0.0
        joint_commands.append(new_joint)
        new_command.joints = joint_commands
        new_command.num_joints = len(joint_commands)
        return new_command

    def convertToCommand(self, latest_state):
        new_command = HuboCommand()
        joint_commands = []
        for jname in self.raw_joint_names:
            new_command = HuboJointCommand()
            new_command.name = jname
            new_command.position = self.latest_state[jname]
            new_command.velocity = 0.0
            joint_commands.append(new_command)
        new_command.joints = joint_commands
        new_command.num_joints = len(joint_commands)
        return new_command

    def hubo_cb(self, msg):
        new_state = {}
        for joint in msg.joints:
            new_state[joint.name] = joint.position
        self.latest_state = new_state        

if __name__ == '__main__':
    rospy.init_node('python_joint_tester')
    Tester()

