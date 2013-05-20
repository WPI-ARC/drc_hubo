import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros

from hubo_planner.srv import *

class PLANTURNING(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'], input_keys=['input'], output_keys=['output'])
        rospy.loginfo("Setting up the service connection to the planner...")
        #rospy.wait_for_service("hubo_planner/PlanningQuery")
        self.planner_host = rospy.ServiceProxy("hubo_planner/PlanningQuery", PlanValveTurning)
        rospy.loginfo("Connected to the planner server")

    def execute(self, userdata):
        rospy.loginfo("Trying to plan valve-turning trajectories...")
        try:
            temp = userdata.input['data']
        except:
            output = {}
            output['error'] = "No args provided"
            output['data'] = None
            userdata.output = output
            rospy.logerr("No arguments provided to planner")
            return 'Failure'
        try:
            # Build planning request
            planning_request = PlanValveTurningRequest()
            planning_request.valve_pose = userdata.input['data']
            # Call the planning system
            planning_response = self.planner_host.call(planning_request)
            if (len(planning_response.trajectories) > 0):
                userdata.output['data'] = zip(planning_response.trajectories, planning_response.labels)
                userdata.output['error'] = "None"
                rospy.loginfo("Call to the planning system returned a solution")
                return 'Success'
            else:
                output = {}
                output['error'] = planning_response.error_code
                output['data'] = None
                userdata.output = output
                rospy.logerr("Call to the planning system did not return a solution")
                return 'Failure'
        except:
            output = {}
            output['error'] = "Failure to call planning system"
            output['data'] = None
            userdata.output = output
            rospy.logfatal("Failure in calling the planning system")
            return 'Fatal'
