#!/usr/bin/python

import roslib; roslib.load_manifest('task_controller')
import rospy
import subprocess
import actionlib
import time
from actionlib import *
from task_controller.msg import *

if __name__ == '__main__':
    rospy.init_node("test_action_client")
    actionclient = actionlib.SimpleActionClient("hubo_valve_action", HuboValveAction)
    actionclient.wait_for_server()
    temp_goal = HuboValveGoal()
    temp_goal.Data = "test"
    print "Sending test goal"
    actionclient.send_goal(temp_goal)
    result = actionclient.wait_for_result()
    print "Result:"
    print result
    print "Done testing"
