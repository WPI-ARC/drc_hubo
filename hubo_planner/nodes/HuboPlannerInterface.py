#!/usr/bin/python

#############################################################################
#                                                                           #
#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team    #
#                                                                           #
#   Service interface to the Hubo CBiRRT planner class                      #
#                                                                           #
#############################################################################

import roslib; roslib.load_manifest('hubo_planner')
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *

import HuboPlusWheelTurning

class HuboPlannerInterface:

    def __init__(self, path):
        rospy.loginfo("Starting Hubo planner interface...")
        self.path = path
        self.current_config = None
        path_to_wheel = path + "/I DONT KNOW YET"
        path_to_robot = path + "/I DONT KNOW YET"
        self.planner_wrapper = HuboPlusWheelTurning.HuboPlusWheelTurning(path_to_robot, path_to_wheel)
        rospy.loginfo("Loaded Hubo CBiRRT wrapper")
        self.config_cb = rospy.Subscriber("/joint_states", JointState, self.Hubo_CB)
        self.RequestHandler = rospy.Service("hubo_planner/PlanningQuery", PlanValveTurning, self.RequestHandler)
        rospy.loginfo("Service host loaded, Planner interface running")
        
    def RequestHandler(self, request):
        # Set up the planner
        wheel_trans = [request.valve_pose.position.x, request.valve_pose.position.y, request.valve_pose.position.z]
        wheel_rot = [request.valve_pose.orientation.x, request.valve_pose.orientation.y, request.valve_pose.orientation.z, request.valve_pose.orientation.w]
        self.planner_wrapper.SetWheelPose(wheel_trans, wheel_rot)
        while (self.current_config is None):
            rospy.logwarn("Planner is waiting to recieve joint states of the robot!")
        self.planner_wrapper.SetRobotConfig(self.current_config)
        # Call the planning operation
        trajectory_files = self.planner_wrapper.PlanTrajectory()
        if (trajectory_files is None or trajectory_files == []):
            # Make an error response
            error = None
            return error
        else:
            return self.BuildResponse(trajectory_files)

    def BuildResponse(self, trajectory_files):
        response = None
        return response

    def Hubo_CB(self, msg):
        # Assemble a new config
        new_config = {}
        if (len(msg.name) != len(msg.position)):
            rospy.logerr("Malformed JointState!")
        else:
            for joint_index in range(len(msg.name)):
                new_config[msg.name[joint_index]] = msg.position[joint_index]
            self.current_config = new_config


if __name__ == '__main__':
    path = subprocess.check_output("rospack find hubo_planner", shell=True)
    path = path.strip("\n")
    rospy.init_node("hubo_planner_interface")
    # Maybe we will need params some day?
    HuboPlannerInterface(path)
