import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros

class POSITIONROBOT(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'])

    def execute(self, userdata):
        rospy.loginfo("Trying to position the robot...")
        return 'Failure'
