#!/usr/bin/python

#############################################################################
#                                                                           #
#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team    #
#                                                                           #
#   Task controller for Hubo valve-turning task. This is the "backbone"     #
#   of the Hubo software, handling the coordination and communication       #
#   between UI, execution, and planning.                                    #
#                                                                           #
#############################################################################

import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros

import os
import sys
import subprocess
import threading
import ctypes

import StartTask
import FindValve
import PositionRobot
import PlanTurning
import ExecuteTurning
import FinishTask
import ManualMode
import SafeMode
import ErrorHandler

class HuboController:

    def __init__(self):
        rospy.loginfo("Starting valve task controller...")
        rospy.on_shutdown(self.cleanup)
        # Initialize the state machine
        self.running = False
        self.safed = False
        self.sm = smach.StateMachine(outcomes=['DONE', 'FAILED', 'SAFE'], input_keys=['sm_input'], output_keys=['sm_output'])
        # Populate the state machine from the modules
        with self.sm:
            self.safemode = SafeMode.SAFEMODE()
            # Add start state
            smach.StateMachine.add('StartTask', StartTask.STARTTASK(), transitions={'Start':'FindValve'}, remapping={'input':'sm_input', 'output':'sm_data'})
            # Add finding step
            smach.StateMachine.add('FindValve', FindValve.FINDVALVE(), transitions={'Success':'PositionRobot', 'Failure':'ErrorHandler', 'Fatal':'SafeMode'}, remapping={'input':'sm_data', 'output':'sm_data'})
            # Add positioning step
            smach.StateMachine.add('PositionRobot', PositionRobot.POSITIONROBOT(), transitions={'Success':'PlanTurning', 'Failure':'ErrorHandler', 'Fatal':'SafeMode'}, remapping={'input':'sm_data', 'output':'sm_data'})
            # Add planning step
            smach.StateMachine.add('PlanTurning', PlanTurning.PLANTURNING(), transitions={'Success':'ExecuteTurning', 'Failure':'ErrorHandler', 'Fatal':'SafeMode'}, remapping={'input':'sm_data', 'output':'sm_data'})
            # Add execution step
            smach.StateMachine.add('ExecuteTurning', ExecuteTurning.EXECUTETURNING(), transitions={'Success':'FinishTask', 'Failure':'ErrorHandler', 'Fatal':'SafeMode'}, remapping={'input':'sm_data', 'output':'sm_data'})
            # Add finishing step
            smach.StateMachine.add('FinishTask', FinishTask.FINISHTASK(), transitions={'Success':'DONE', 'Failure':'ErrorHandler', 'Fatal':'SafeMode'}, remapping={'input':'sm_data', 'output':'sm_output'})
            # Add manual mode
            smach.StateMachine.add('ManualMode', ManualMode.MANUALMODE(), transitions={'Success':'ErrorHandler', 'Failure':'ErrorHandler', 'Fatal':'SafeMode'}, remapping={'input':'sm_data', 'output':'sm_data'})
            # Add safe mode
            smach.StateMachine.add('SafeMode', self.safemode, transitions={'Safed':'SAFE'}, remapping={'input':'sm_data', 'output':'sm_output'})
            # Add Error handler
            smach.StateMachine.add('ErrorHandler', ErrorHandler.ERRORHANDLER(), transitions={'ReFind':'FindValve', 'RePosition':'PositionRobot', 'RePlan':'PlanTurning', 'ReExecute':'ExecuteTurning', 'ReFinish':'FinishTask', 'GoManual':'ManualMode', 'Failed':'FAILED', 'Fatal':'SafeMode'}, remapping={'input':'sm_data', 'output':'sm_output'})
        # Set up the introspection server
        self.sis = smach_ros.IntrospectionServer('valve_task_smach_server', self.sm, '/SM_ROOT')
        self.sis.start()

    def Start(self):
        # Start the state machine
        self.running = True
        self.safed = False
        self.sm.userdata.sm_input = "Testing"
        print "Starting..."
        print self.sm.userdata.sm_input
        print "...starting"
        final_outcome = self.sm.execute()
        self.running = False
        self.safemode.execute([])
        self.safed = True
        self.cleanup()

    def cleanup(self):        
        # Complete any steps necessary for task controller shutdown
        if (self.running or not self.safed):
            rospy.logfatal("Forced shutdown of Hubo task controller...")
            self.sm.request_preempt()
            self.safemode.execute(None)
        rospy.loginfo("Shutting down Hubo task controller...")
        self.sis.stop()
        rospy.loginfo("...Hubo task controller shutdown complete")
    

if __name__ == '__main__':
    path = subprocess.check_output("rospack find task_controller", shell=True)
    path = path.strip("\n")
    rospy.init_node("valve_task_controller")
    controller = HuboController()
    controller.Start()
