#!/usr/bin/python

#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team
#
#   Output console for diagnostic information from Hubo ROS code.
#
#   Diagnostic messages are printed to the console in color and style
#   according to the Hubo-ROS diagnostics convention.
#

import roslib; roslib.load_manifest('hubo_dashboard')
import rospy
from rosgraph_msgs.msg import *

version = "0.1.A"

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    QUESTION = '\033[90m'
    FAIL = '\033[91m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    ENDC = '\033[0m'

class DiagnosticsConsole:

    def __init__(self):
        self.levels = {"1":"DEBUG", "2":"INFO", "4":"WARNING", "8":"ERROR", "16":"FATAL ERROR"}
        print bcolors.OKGREEN + bcolors.BOLD + "Lightweight Diagnostics Console v" + version + "\n==================================================" + bcolors.ENDC
        self.rsagg_sub = rospy.Subscriber("rosout_agg", Log, self.rosout_cb)
        while not rospy.is_shutdown():
            rospy.spin()

    def rosout_cb(self, msg):
        message_level = msg.level
        debug_msg = "| Level: [" + self.levels[str(message_level)] + "] from Node: [" + msg.name + "]\n| Message: " + msg.msg
        if (message_level == 1):
            print bcolors.QUESTION + debug_msg + "\n---" + bcolors.ENDC
        elif (message_level == 2):
            print bcolors.OKBLUE + debug_msg + "\n---" + bcolors.ENDC
        elif (message_level == 4):
            print bcolors.HEADER + debug_msg + "\n---" + bcolors.ENDC
        elif (message_level == 8):
            print bcolors.WARNING + debug_msg + "\n---" + bcolors.ENDC
        elif (message_level == 16):
            print bcolors.FAIL + bcolors.BOLD + "-------------------------\n" + debug_msg + "\n-------------------------" + bcolors.ENDC

if __name__ == "__main__":
    rospy.init_node("lightweight_diagnostics_console")
    DiagnosticsConsole()
