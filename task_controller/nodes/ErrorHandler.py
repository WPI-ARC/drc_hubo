import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros

class ERRORHANDLER(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['ReFind','RePosition','RePlan','ReExecute','ReFinish','Failed','Fatal'])

    def execute(self, userdata):
        rospy.loginfo("ErrorHandler executing...")
        return 'Fatal'
