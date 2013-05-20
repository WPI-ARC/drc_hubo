import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros

class PLANTURNING(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'])

    def execute(self, userdata):
        rospy.loginfo("Trying to plan valve-turning trajectories...")
        try:
            # Call the planning system
            planning_response = None
            if (planning_response.error_code.val == planning_response.error_code.SUCCESS):
                rospy.loginfo("Call to the planning system returned a solution")
                return 'Success'
            else:
                rospy.logerr("Call to the planning system did not return a solution")
                return 'Failure'
        except:
            rospy.logfatal("Failure in calling the planning system")
            return 'Fatal'
