#! /usr/bin/env python

import roslib;
roslib.load_manifest('ach_servers')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from hubo_msgs.msg import *
#from ach_srvers.msg import *

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
        rospy.init_node('joint_traj_test_client_py')
        result = joint_traj_client()
        #print "Result:", ', '.join([str(n) for n in result.sequence])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
