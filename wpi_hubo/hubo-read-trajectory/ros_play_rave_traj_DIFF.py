# Bener Suay 2012 December
# benersuay@wpi.edu
#
# This script reads a native openrave trajectory xml file and executes it with the real Hubo via ROS topics
# 
# Written during the first MIT visit on December 18th, 2012
# 
# All calls to print are there for debugging purposes, you can remove them if they're bothering you.

#import easy to use xml parser called minidom:
from xml.dom.minidom import parseString

import roslib;
roslib.load_manifest('hubo_ros')
import rospy
import math
import random
import time

import numpy

from copy import deepcopy
import sys

from hubo_ros.msg import *
from math import pi

if __name__=='__main__':

<<<<<<< HEAD
    rospy.init_node('python_joint_tester')
=======
    rospy.init_node('hubo_command_node')
>>>>>>> 5c44f45910fcac8262968c811cd0fd8ed51d514a
    
    try:
        input_file_name = sys.argv[1]
    except IndexError:
        print "No filename was given."
        
    # Is this a native openrave format file or a converted plaintext trajectory file?
    if (input_file_name[len(input_file_name)-3:len(input_file_name)] == "txt"):
        nativeFile = True
    else:
        nativeFile = False

    hubo_pub = rospy.Publisher('Hubo/HuboCommand', HuboCommand)
    
    #open the xml file for reading:
    file = open(input_file_name,'r')

    jointNames = ['RHY', 'RHR', 'RHP', 'RKP', 'RAP', 'RAR', 'LHY', 'LHR', 'LHP', 'LKP', 'LAP', 'LAR', 'RSP', 'RSR', 'RSY', 'REP', 'RWY', 'RWR', 'RWP', 'LSP', 'LSR', 'LSY', 'LEP', 'LWY', 'LWR', 'LWP', 'NKY', 'NK1', 'NK2', 'HPY', 'RF1', 'RF2', 'RF3', 'RF4', 'RF5', 'LF1', 'LF2', 'LF3', 'LF4', 'LF5']

    # Calder's
    #jointNames = ["HPY", "not in urdf1", "HNR", "HNP", "LSP", "LSR", "LSY", "LEP", "LWY", "not in urdf2", "LWP", "RSP", "RSR", "RSY", "REP", "RWY", "not in urdf3", "RWP", "not in ach1", "LHY", "LHR", "LHP", "LKP", "LAP", "LAR", "not in ach1", "RHY", "RHR", "RHP", "RKP", "RAP", "RAR", "not in urdf4", "not in urdf5", "not in urdf6", "not in urdf7", "not in urdf8", "not in urdf9", "not in urdf10", "not in urdf11", "not in urdf12", "not in urdf13", "unknown1", "unknown2", "unknown3", "unknown4", "unknown5", "unknown6", "unknown7", "unknown8"]

    # Not used
    #jointIndices = [1, 3, 5, 7, 9, 11, 2, 4, 6, 8, 10, 12, 13, 15, 17, 19, 21, -1, 23, 14, 16, 18, 20, 22, -1, 24, -1, -1, -1, 0, 41, 29, 32, 35, 38, 56, 44, 47, 50, 53]

    if(nativeFile):
        pass
    else:
        # the following loop reads all the lines one by one
        for line in file:
            
            # Convert the line into a word by word array
            line = line.rstrip() # Strip the \n and trailing spaces.
            str_data = line.split(' ') # Make a word by word array
            
            # Create a temp float data array for the joint values
            temp_float_data = []

            # Convert strings into floats
            for d in range(len(str_data)):
                temp_float_data.append(float(str_data[d]))

            # print for debugging
            print temp_float_data
            
            # This part is where we send things to ROS node
            new_command = HuboCommand()
            joint_commands = []

            # Go through the joints list and send commands
            for j in range(len(jointNames)):
                new_joint = HuboJointCommand()
                new_joint.name = jointNames[j]
                new_joint.position = deepcopy(temp_float_data[j])
                new_joint.velocity = 0.0
                joint_commands.append(new_joint)
                
            # Done with reading all joint values
            new_command.joints = joint_commands
            new_command.num_joints = len(joint_commands)
                
            hubo_pub.publish(new_command)
<<<<<<< HEAD
            print "press enter to continue..."            
            sys.stdin.readline()
=======
            print "Press enter to continue..."
            sys.stdin.readline()
        
>>>>>>> 5c44f45910fcac8262968c811cd0fd8ed51d514a
        file.close()
    
        
    
