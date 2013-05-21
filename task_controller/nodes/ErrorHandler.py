import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros

class ERRORHANDLER(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['ReFind','RePosition','RePlan','ReExecute','ReFinish','GoManual','Failed','Fatal'], input_keys=['input'], output_keys=['output'])

    def execute(self, userdata):
        rospy.loginfo("ErrorHandler executing...")
        output = {}
        output['error'] = "Finished"
        output['data'] = None
        userdata.output = output
        return 'Fatal'
