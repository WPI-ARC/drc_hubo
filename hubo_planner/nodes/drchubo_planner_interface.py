#!/usr/bin/python

#############################################################################
#                                                                           #
#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team    #
#                                                                           #
#   Service interface to the Hubo CBiRRT planner class                      #
#                                                                           #
#############################################################################

import subprocess

import roslib;
roslib.load_manifest('hubo_planner')

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from trajectory_msgs.msg import *
from valve_planner_msgs.msg import *
from valve_planner_msgs.srv import *

#import hubo_traj_reader
import drchubo_v2_wheel_turning

class HuboPlannerInterface:

    def __init__(self, path):

        self.debug = True
        self.no_planning = False
        rospy.loginfo("Starting Hubo planner interface...")
        self.path = path
        self.current_config = None

        path_to_robot = rospy.get_param("robot_model")
        path_to_wheel = rospy.get_param("tiny_wheel_model")
        self.read_joint_states = rospy.get_param("read_joint_states")

        print path_to_robot
        print path_to_wheel
        print self.read_joint_states

        self.planner = drchubo_v2_wheel_turning.DrcHuboWheelTurning( path_to_robot, path_to_wheel )

        self.planner.SetViewer(True)
        self.planner.SetStopKeyStrokes(True)

        rospy.loginfo("Loaded Hubo CBiRRT wrapper")

        # We can either read the hubo's joint state from the TF tree or assume it starts from home in OpenRAVE
        if(self.read_joint_states):
            self.config_cb = rospy.Subscriber("/joint_states", JointState, self.GetRealRobotConfig)
        else:
            # Assume drchubo starts from home in openrave
            pass

        self.PlanRequestService = rospy.Service("drchubo_planner/PlanningQuery", PlanTurning, self.PlanRequestHandler)
        self.ExecuteRequestService = rospy.Service("drchubo_planner/ExecutionQuery", ExecuteTurning, self.ExecuteRequestHandler)

        rospy.loginfo("Service host loaded, Planner interface running")

    # Replays the last planned trajectories in openrave
    def ExecuteRequestHandler(self, req):
        print "Execute - Identifier: "
        print req.Identifier

        self.planner.Playback()

        res = ExecuteTurningResponse()
        res.ErrorCode = "Happy birthday to you."
        return res

    # Sets the wheel location in openrave
    # Calls the planner (CiBRRT)
    # Reads the trajectories from the files
    def PlanRequestHandler(self, req):

        self.planner.ResetEnv()

        wheel_trans = [req.Request.ValvePose.pose.position.x, req.Request.ValvePose.pose.position.y, req.Request.ValvePose.pose.position.z]
        wheel_rot = [req.Request.ValvePose.pose.orientation.x, req.Request.ValvePose.pose.orientation.y, req.Request.ValvePose.pose.orientation.z, req.Request.ValvePose.pose.orientation.w]
             
        print "wheel_trans"
        print wheel_trans
        print "wheel_rot"
        print wheel_rot
        print "wheel rad."
        print req.Request.ValveSize
        print "frame id"
        print req.Request.ValvePose.header.frame_id.strip("/")
        print "valve type"
        print req.Request.ValveType
        print "valve size"
        print req.Request.ValveSize
        print "hand(s)"
        print req.Request.Hands
        print "rotation direction"
        print req.Request.Direction
        
        self.planner.StartViewer()
        
        # Use the frame id that comes in from RViz and set the wheel pose
        self.h = self.planner.SetValvePoseFromQuaternionInFrame( req.Request.ValvePose.header.frame_id.strip("/"), wheel_trans, wheel_rot )
        
        self.planner.CreateValve(req.Request.ValveSize, req.Request.ValveType)

        while (self.read_joint_states and (self.current_config is None)):
            rospy.logwarn("Planner is waiting to recieve joint states of the robot!")

        if(self.read_joint_states):
            self.planner.SetRobotConfiguration(self.current_config)

        # Call to CBiRRT if no planning simply read the current files
        if( self.no_planning ):
            trajectory_files = [ 'movetraj0.txt','movetraj1.txt','movetraj2.txt','movetraj3.txt','movetraj4.txt','movetraj5.txt']
        else:
            error_code = self.planner.Plan([],req.Request.ValveSize,req.Request.Hands,req.Request.Direction,req.Request.ValveType)

        error_str = ""

        # Read the trajectories from files
        if(error_code == 0):
            # No error
            error_str = "NoError"
        else:
            error_str = str(error_code)
            
        res = PlanTurningResponse()
        res.Response.header = Header()
        res.Response.ErrorCode = error_str
        res.Response.Labels = "LABELS"
        return res

    # NOT USED
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


    def GetRealRobotConfig(self, msg):
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
    

