#!/usr/bin/python

#############################################################################
#                                                                           #
#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team    #
#                                                                           #
#   Task controller for Hubo valve-turning task. This is the "backbone"     #
#   of the Hubo software, handling the coordination and communication       #
#   between UI, execution, and planning.                                    #
#                                                                           #
#############################################################################

import roslib; roslib.load_manifest('task_controller')
import rospy
from task_controller.msg import *
from task_controller.srv import *
from hubo_msgs.msg import *
from tf import *
import actionlib
from actionlib_msgs.msg import *



class HuboController:

    def __init__(self):
        rospy.loginfo("Starting Hubo task controller...")
        rospy.on_shutdown(self.cleanup)

    def mode_switch(self, target_mode):
        # Performs the operations necessary to switch operating modes or sets the robot into a safe error state
        pass

    def cleanup(self):
        rospy.loginfo("Shutting down Hubo task controller...")
        # Complete any steps necessary for task controller shutdown
        pass
        rospy.loginfo("...Hubo task controller shutdown complete")

    def execute_walk(self, walking_goal):
        rospy.loginfo("Calling the walking system...")
        try:
            # Call the planning system
            walking_response = None
            if (walking_response.success_code == walking_response.SUCCESS):
                rospy.loginfo("Walking operation complete")
                return walking_response
            elif (walking_response.success_code == walking_response.FAILURE):
                rospy.logwarn("Unable to walk to the goal")
                return None
            elif (walking_response.success_code == walking_response.ERROR):
                rospy.logerr("Walking controller failure")
        except:
            rospy.logerr("Failure in calling the walking system")
            return None

    def call_planner(self, wheel_pose):
        rospy.loginfo("Calling the planning system...")
        try:
            # Call the planning system
            planning_response = None
            if (planning_response.success_code == planning_response.SUCCESS):
                rospy.loginfo("Call to the planning system returned a solution")
                return planning_response
            elif (planning_response.success_code == planning_response.FAILURE):
                rospy.logwarn("Call to the planning system did not return a solution")
                return None
        except:
            rospy.logerr("Failure in calling the planning system")
            return None

    def execute_trajectory(self, trajectory):
        rospy.loginfo("Calling the trajectory execution system...")
        try:
            # Call the planning system
            execution_response = None
            if (execution_response.success_code == execution_response.SUCCESS):
                rospy.loginfo("Trajectory execution complete")
                return execution_response
            elif (execution_response.success_code == execution_response.FAILURE):
                rospy.logwarn("Unable to execute trajectory")
                return None
        except:
            rospy.logerr("Failure in calling the trajectory execution system")
            return None
