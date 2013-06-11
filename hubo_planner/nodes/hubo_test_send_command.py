#!/usr/bin/env python
import roslib; roslib.load_manifest('hubo_planner')

import sys

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from std_msgs.msg import *
from hubo_robot_msgs.msg import *
from hubo_planner.srv import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
#from ach_srvers.msg import *


class HuboTestSendCommand:

    def __init__(self):
        self.traj = None
        self.hand_pub = rospy.Publisher("hand_command", Bool)
        self.default_pose = Pose()
        self.default_pose.position.x = 0.18
        self.default_pose.position.y = 0.09
        self.default_pose.position.z = 0.9
        self.default_pose.orientation.x = 0.5 
        self.default_pose.orientation.y = 0.5
        self.default_pose.orientation.z = 0.5 
        self.default_pose.orientation.w = 0.5

    def call_to_planner(self, valve_pose=None):

        rospy.wait_for_service('hubo_planner/PlanningQuery')
        try:
            planner_srv = rospy.ServiceProxy('hubo_planner/PlanningQuery', PlanValveTurning)
            valve_position = None
            if (valve_pose == None):
                valve_position = self.default_pose
            else:
                valve_position = valve_pose
                print "Default pose is: " + str(self.default_pose)
                print "Provided pose is: " + str(valve_position)

            response = planner_srv(valve_position)
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
        client = actionlib.SimpleActionClient('/hubo_fullbody_controller/joint_trajectory_action', hubo_robot_msgs.msg.JointTrajectoryAction )

        print "waiting for action server..."

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        print "client started"

        res = None

        # Execute the start -> wheel trajectory
        self.hand_pub.publish(False)
        rospy.sleep(1.0)
        t0_goal = JointTrajectoryGoal()
        t0 = self.traj[0]
        t0.header.stamp = rospy.Time.now()
        t0_goal.trajectory = t0
        client.send_goal( t0_goal )
        client.wait_for_result()
        res = client.get_result()
        print "end execution of trajectory 0"
        # Execute the turning trajectory
        self.hand_pub.publish(True)
        rospy.sleep(1.0)
        t1_goal = JointTrajectoryGoal()
        t1 = self.traj[1]
        t1.header.stamp = rospy.Time.now()
        t1_goal.trajectory = t1
        client.send_goal( t1_goal )
        client.wait_for_result()
        res = client.get_result()
        print "end execution of trajectory 1"
        # Execute the return trajectory
        self.hand_pub.publish(False)
        rospy.sleep(1.0)
        t2_goal = JointTrajectoryGoal()
        t2 = self.traj[2]
        t2.header.stamp = rospy.Time.now()
        t2_goal.trajectory = t2
        client.send_goal( t2_goal )
        client.wait_for_result()
        res = client.get_result()
        print "end execution of trajectory 2"
        # Execute the turning trajectory
        self.hand_pub.publish(True)
        rospy.sleep(1.0)
        t3_goal = JointTrajectoryGoal()
        t3 = self.traj[3]
        t3.header.stamp = rospy.Time.now()
        t3_goal.trajectory = t3
        client.send_goal( t3_goal )
        client.wait_for_result()
        res = client.get_result()
        print "end execution of trajectory 3"
        # Execute the wheel -> start trajectory
        self.hand_pub.publish(False)
        rospy.sleep(1.0)
        t4_goal = JointTrajectoryGoal()
        t4 = self.traj[4]
        t4.header.stamp = rospy.Time.now()
        t4_goal.trajectory = t4
        client.send_goal( t4_goal )
        client.wait_for_result()
        res = client.get_result()
        print "end execution of trajectory 4"

        '''
        try:
            # for all trajectories
            for t in self.traj:
            
                # Creates a goal to send to the action server.
                goal = JointTrajectoryGoal()
                t.header.stamp = rospy.Time.now()
                goal.trajectory = t

                # Sends the goal to the action server.
                client.send_goal( goal )
                client.wait_for_result()
                res = client.get_result()

                # Prints out the result of executing the action
                print "end execution of trajectory"
                #print res
            
            return res

        except rospy.ServiceException, e:
            print "Goal Sending failed : %s"%e
        '''

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

