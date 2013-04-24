#!/usr/bin/env python
# Bener Suay 2013 April
# WPI DRC Hackathon Code
# benersuay@wpi.edu

# system modules
import commands
import sys

# ros modules
import roslib
roslib.load_manifest('trajectory_msgs')
#roslib.load_manifest('hubo_msgs')
roslib.load_manifest('std_srvs')

import rospy
import numpy

from std_srvs.srv  import *

def handle_hubo_walk(req):
    print "Hubo Walk Service request"
    cmd = "/home/jmainpri/workspace/hubomz/build/zmpdemo /home/jmainpri/workspace/hubomz/myhubo.kinbody.xml -w canned -z 0.05 -y 0.0885 -d 0.01 -s 0.5 -c 8 -p 2 -l 0.08 -X 0.038 -h 0.5 -A"
    returnedString = commands.getoutput(cmd)
    print "Returned String"
    print returnedString
    return EmptyResponse()

def main():
    # Initialize ROS node
    rospy.init_node('hubo_walk_control',anonymous=False)
    myService = rospy.Service('hubo_walk', Empty, handle_hubo_walk)
    print "Ready to send walk command..."
    rospy.spin()

if __name__ == "__main__":
    main()
