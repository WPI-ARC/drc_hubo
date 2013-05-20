#!/usr/bin/python

#############################################################################
#                                                                           #
#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team    #
#                                                                           #
#   Actionlib interface to the task controller for Hubo valve-turning task. #
#                                                                           #
#############################################################################

import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros

from hubo_controller import *
from task_controller.msg import *

if __name__ == '__main__':
    rospy.init_node("hubo_valve_task_action_controller")
    controller = HuboController()
    # Make an actionserver out of the controller
    actionserver = smach_ros.ActionServerWrapper("hubo_valve_action", HuboValveAction, wrapped_container=controller.sm, succeeded_outcomes=['DONE'], aborted_outcomes=['FAILED'], preempted_outcomes=['SAFE'], goal_key='sm_input', result_key='sm_output')
    # Run server
    actionserver.run_server()
    # Spin
    rospy.spin()
