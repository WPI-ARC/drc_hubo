#!/usr/bin/env python
import roslib
roslib.load_manifest('hubo_planner')
roslib.load_manifest('gui')
import rospy

#tf and stuff
roslib.load_manifest("tf")
import tf
from tf.transformations import *
from transformation_helper import *

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from hubo_robot_msgs.msg import *
from hubo_planner.srv import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from gui.srv import *

class ValveService:

    def __init__(self):
        self.backend = HuboTestSendCommand()
        self.listener = tf.TransformListener()
        self.server = rospy.Service("update_valve_pose", updateValvePos, self.service_handler)
        rospy.loginfo("Started planner/execution server for Hubo+")
        rospy.spin()

    def service_handler(self, request)
        rospy.loginfo("Received a valve turning request")
        # Get the valve pose and turn it into a useful format
        raw_posestamped = request.valve
        useful_pose = self.convert_pose(raw_posestamped)
        # Call the planner
        rospy.loginfo("Calling the planner...")
        self.backend.call_to_planner(useful_pose)
        rospy.loginfo("...planner completed")
        # Execute the trajectories planned
        rospy.loginfo("Executing the trajectories...")
        self.backend.joint_traj_client()
        rospy.loginfo("...execution completed")
        # Put together the response
        respose = updateValvePos._response_class()
        response.valve = raw_posestamped
        response.success_code = "Turning completed"
        return response

    def convert_pose(self, raw_posestamped):
        [trans,rot] = self.listener.lookupTransform("/body_RAR", raw_posestamped.header.frame_id, rospy.Time(0))
        cur_valve_frame_pose = PoseFromTransform(TransformFromComponents(trans,rot))
        cur_valve_pose = raw_posestamped.pose
        full_valve_pose = ComposePoses(cur_valve_frame_pose, cur_valve_pose)
        return full_valve_pose

def service_callback

if __name__ == '__main__':
    rospy.init_node('hubo_test_send_command')
    ValveService()
