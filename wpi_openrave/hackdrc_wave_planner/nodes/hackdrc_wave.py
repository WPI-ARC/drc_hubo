#!/usr/bin/env python
# Jim Mainprice, RAIL
# November 2012
# Worcester Polytechnic Institute
#

import os # for file operations
import hubo_cbirrt_wave
import commands
import sys
import roslib
import rospy
import numpy

roslib.load_manifest('hackdrc_wave_planner')

from std_srvs.srv import *
from hackdrc_wave_planner.srv import *

def handle_hubo_wave(req):
    hubo_cbirrt_wave.run()
    parsetraj = rospy.ServiceProxy('parse_wave_trajectory', HuboWaveTraj )
    try:
        resp1 = parsetraj( Empty )
        print resp1
    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)
        
    return EmptyResponse()

def main():
    # Initialize ROS node
    rospy.init_node('hubo_wave_control',anonymous=False)
    myService = rospy.Service('hubo_wave', Empty, handle_hubo_wave )
    print "Ready to send wave command..."
    rospy.spin()

if __name__ == "__main__":
    main()
