#!/usr/bin/env python
import roslib; roslib.load_manifest('hubo_planner')

import sys

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
#from hubo_msgs.msg import *
from hubo_planner.srv import *
from geometry_msgs.msg import *
#from ach_srvers.msg import *

def call_to_planner():
    rospy.wait_for_service('hubo_planner/PlanningQuery')
    try:
        planner_srv = rospy.ServiceProxy('hubo_planner/PlanningQuery', PlanValveTurning)
        valve_position = Pose()
        valve_position.position.x = 0.18
        valve_position.position.y = 0.09
        valve_position.position.z = 0.9
        valve_position.orientation.x = 0.5 
        valve_position.orientation.y = 0.5
        valve_position.orientation.z = 0.5 
        valve_position.orientation.w = 0.5
        resp1 = planner_srv( valve_position )
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

def joint_traj_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('ach_server', hubo_msgs.msg.JointTrajectoryAction)

    print "waiting...\n"

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    print "started\n"

    # Creates a goal to send to the action server.
    goal = JointTrajectoryGoal()
    goal.trajectory.header.frame_id = 'test'

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('hubo_test_send_command')
        #result = joint_traj_client()
        print call_to_planner()
        #print "Result:", ', '.join([str(n) for n in result.sequence])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
