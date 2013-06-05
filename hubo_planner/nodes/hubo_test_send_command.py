#!/usr/bin/env python
import roslib; roslib.load_manifest('hubo_planner')

import sys

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from hubo_robot_msgs.msg import *
from hubo_planner.srv import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
#from ach_srvers.msg import *


class HuboTestSendCommand:

    def __init__(self):

        self.traj = None

    def call_to_planner(self):

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

            response = planner_srv( valve_position )
            self.traj = response.trajectories
            return self.traj
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def usage():

        return "%s [x y]"%sys.argv[0]


    def joint_traj_client(self):

        print "create action client..."

        # Creates the SimpleActionClient, passing the type of the action
        # (JointTrajectoryAction) to the constructor.
        client = actionlib.SimpleActionClient('joint_trajectory_action', hubo_robot_msgs.msg.JointTrajectoryAction )

        print "waiting for action server..."

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        print "client started"

        res = None

        try:
            # for all trajectories
            for t in self.traj:
            
                # Creates a goal to send to the action server.
                goal = JointTrajectoryGoal()
                t.header.stamp = rospy.Time.now()
                goal.trajectory = t

                # Sends the goal to the action server.
                client.send_goal( goal )
                client.wait_for_result( rospy.Duration.from_sec(500.0) )
                res = client.get_result()

                # Prints out the result of executing the action
                print "end execution of trajectory"
                #print res
            
            return res

        except rospy.ServiceException, e:
            print "Goal Sending failed : %s"%e

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('hubo_test_send_command')
        command = HuboTestSendCommand()
        traj = command.call_to_planner()
        #print traj[0]
        command.joint_traj_client()
        #rospy.spin()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"

