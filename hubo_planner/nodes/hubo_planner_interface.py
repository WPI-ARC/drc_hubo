#!/usr/bin/python

#############################################################################
#                                                                           #
#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team    #
#                                                                           #
#   Service interface to the Hubo CBiRRT planner class                      #
#                                                                           #
#############################################################################

import subprocess

import roslib; roslib.load_manifest('hubo_planner')
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from trajectory_msgs.msg import *
from hubo_planner.srv import *

import hubo_traj_reader
import hubo_plus_wheel_turning


class HuboPlannerInterface:

    def __init__(self, path):

        self.debug = True
        self.no_planning = False
        rospy.loginfo("Starting Hubo planner interface...")
        self.path = path
        self.current_config = None

        # path_to_robot = path + '/../openHubo/huboplus/rlhuboplus_mit.robot.xml'
        # path_to_robot = path + '/../openHubo/huboplus/rlhuboplus.robot.xml'
        # path_to_wheel = path + '/../../drc_common/models/driving_wheel.robot.xml'
        path_to_robot = roslib.packages.get_pkg_dir("drchubo-v2")+'/robots/drchubo-v2.robot.xml'
        self.planner = hubo_plus_wheel_turning.HuboPlusWheelTurning( path_to_robot, path_to_wheel )
        self.planner.SetViewer(True)
        self.planner.SetStopKeyStrokes(False)

        rospy.loginfo("Loaded Hubo CBiRRT wrapper")
        self.config_cb = rospy.Subscriber("/joint_states", JointState, self.Hubo_CB)
        self.RequestHandler = rospy.Service("hubo_planner/PlanningQuery", PlanValveTurning, self.RequestHandler)
        rospy.loginfo("Service host loaded, Planner interface running")
        
        
    # Sets the wheel location in openrave
    # Calls the planner (CiBRRT)
    # Reads the trajectories from the files
    def RequestHandler(self, request):

        # Set wheel location
        wheel_trans = [request.valve_position.position.x, request.valve_position.position.y, request.valve_position.position.z]
        wheel_rot = [request.valve_position.orientation.x, request.valve_position.orientation.y, request.valve_position.orientation.z, request.valve_position.orientation.w]
        
        print wheel_trans
        print wheel_rot

        if( True #not self.debug
            ):
            self.h = self.planner.SetWheelPosition( wheel_trans, wheel_rot )
        else:
            print wheel_trans
            print wheel_rot

        while (self.current_config is None):
            rospy.logwarn("Planner is waiting to recieve joint states of the robot!")
        self.planner.SetRobotConfiguration(self.current_config)

        # Call to CBiRRT if no planning simply read the current files
        if( self.no_planning ):
            trajectory_files = [ 'movetraj0.txt','movetraj1.txt','movetraj2.txt','movetraj1.txt','movetraj3.txt']
        else:
            trajectory_files = self.planner.Run()

        # Read the trajectories from files
        if ((trajectory_files == None) or (trajectory_files == [])):
            print "planning failed"
            # Make an error response
            return PlanValveTurningResponse(None, ["LABELS"] , "PLAN_FAILED" )
        else:
            return self.BuildResponse(trajectory_files)


    # Reads the trajectory from the files generated by openrave
    # then builds PlanValveTurningResponse  
    def BuildResponse(self, trajectory_files):

        #trajectory_msgs/JointTrajectory[]
        traj_array = []

        for f in trajectory_files:
            print "read file : " + f
            traj_array.append( hubo_traj_reader.read( f ) )

        #print traj_array
        return PlanValveTurningResponse( traj_array, ["LABELS"] , "PLAN_OK" )


    def Hubo_CB(self, msg):
        # Assemble a new config
        new_config = {}
        if (len(msg.name) != len(msg.position)):
            rospy.logerr("Malformed JointState!")
        else:
            for joint_index in range(len(msg.name)):
                new_config[msg.name[joint_index]] = msg.position[joint_index]
            self.current_config = new_config
    
    def Hook(self):
        print "End planner node"
        self.planner.KillOpenrave()
       

if __name__ == '__main__':
    path = subprocess.check_output("rospack find hubo_planner", shell=True)
    path = path.strip("\n")
    rospy.init_node("hubo_planner_interface")
    # Maybe we will need params some day?
    huboplan = HuboPlannerInterface( path )
    rospy.on_shutdown( huboplan.Hook )
    print "Robot ready to plan, waiting for a valve pose update..."
    rospy.spin()
    

