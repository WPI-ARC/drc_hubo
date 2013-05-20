import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros

class FINDVALVE(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'], input_keys=['input'], output_keys=['output'])

    def execute(self, userdata):
        rospy.loginfo("Trying to find the valve...")
        i = 0
        while (i < 10):
            if (self.preempt_requested()):
                rospy.logwarn("FindValve Preempted!")
                return 'Fatal'
            print i
            i += 1
            rospy.sleep(5.0)
        output = {}
        output['error'] = "Finished"
        output['data'] = None
        userdata.output = output
        return 'Failure'
