import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros

class STARTTASK(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Start'], input_keys=['input'], output_keys=['output'])

    def execute(self, userdata):
        rospy.loginfo("Trying to start up the task...")
        print "Input data: " + str(userdata.input)
        output = {}
        output['data'] = userdata.input
        output['error'] = "None"
        userdata.output = output
        return 'Start'
