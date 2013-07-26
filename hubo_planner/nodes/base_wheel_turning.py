#!/usr/bin/env python
# Jim Mainprice of ARC, and,
# Ben Suay of RAIL
# May 2013
# Worcester Polytechnic Institute
#

# http://openrave.org/docs/latest_stable/command_line_tools/
# openrave-robot.py /your/path/to/your.robot.xml --info=joints
# On that page you can find more examples on how to use openrave-robot.py.

from openravepy import *
import rospy
import roslib
roslib.load_manifest("hubo_planner")
import sys
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
    import numpy
import time
from wpi_planning_utilities.rodrigues import *
from wpi_planning_utilities.TransformMatrix import *
from wpi_planning_utilities.str2num import *
from wpi_planning_utilities.TSR import *
from math import *
from copy import *
import os # for file operations
from wpi_planning_utilities.RaveCBiRRT import *

def trans_to_str(T):
    myStr = ""
    for c in range(0,3):
        for r in range(0,3):
            myStr += str(T[r,c])+" "
    
    for r in range(0,3):
        myStr += str(T[r,3])+" "

    #print "Tee string : " 
    #print myStr
    return myStr

class BaseWheelTurning:

    def __init__(self, HuboModelPath, WheelModelPath ):
        
        self.crouch = 0.05

        self.default_trajectory_dir = roslib.packages.get_pkg_dir('hubo_planner')+"/trajectories/"
        
        # Set those variables to show or hide the interface
        # Do it using the member functions
        self.StopAtKeyStrokes = False
        self.ShowUserInterface = False
        self.ViewerStarted = False

        self.T_Wheel = None # Wheel transform in world coords.
        self.r_Wheel = None # Default Wheel radius
        self.HuboModelPath = HuboModelPath
        self.WheelModelPath = WheelModelPath

        # Start Environment
        self.env = Environment()
        self.env.SetDebugLevel(DebugLevel.Info) # set output level to debug
        self.robotid = self.env.ReadRobotURI(self.HuboModelPath)
        self.crankid = self.env.ReadRobotURI(self.WheelModelPath)
        self.env.Add(self.robotid)
        self.env.Add(self.crankid)

        # Hands values
        self.rhandopenvals = None
        self.rhandclosevals = None
        self.rhanddofs = None
        self.lhandopenvals = None
        self.lhandclosevals = None
        self.lhanddofs = None

        self.myValveHandle = None
        self.infocylinder = None

        self.probs_cbirrt = None
        self.probs_crankmover = None

        self.T0_tsy_home = self.robotid.GetLinks()[12].GetTransform()
        self.T0_lar_home = self.robotid.GetManipulators()[2].GetEndEffectorTransform()
        self.T0_rar_home = self.robotid.GetManipulators()[3].GetEndEffectorTransform()
        self.Ttsy_lar_home = array(dot(linalg.inv(self.T0_tsy_home),self.T0_lar_home))
        self.Ttsy_rar_home = array(dot(linalg.inv(self.T0_tsy_home),self.T0_rar_home))

        self.wheelDistFromTSY = 0.4

        self.drawingHandles = []

        # CBiRRT variables
        # polyscale: changes the scale of the support polygon
        # polytrans: shifts the support polygon around
        self.footlinknames = ' Body_RAR Body_LAR polyscale 0.5 0.5 0 ' #polytrans -0.015 0 0.0 '

        # Center of Gravity Target
        self.cogtarg = [0, 0, 0]

    def KillOpenrave(self):
        self.env.Destroy()
        RaveDestroy()

    def SetViewer(self,arg=True):
        print "SetViewer"
        self.ShowUserInterface = arg

    def SetStopKeyStrokes(self,arg=True):
        print "SetStopKeyStrokes"
        self.StopAtKeyStrokes = arg

    def SetWheelPosition(self,trans,rot):
        print "SetWheelPosition"
        self.T_Wheel = MakeTransform(rotationMatrixFromQuat(rot),matrix(trans))
        self.crankid.SetTransform(array(self.T_Wheel))

    def ResetEnv(self):
        
        # Remove the valve (cylinder or box)
        if(self.env.GetKinBody("valve") is not None):
            self.env.RemoveKinBody(self.myValveHandle)

        if(self.env.GetKinBody("4by4") is not None):
            self.env.RemoveKinBody(self.my4by4)
            
        if(self.env.GetKinBody("1by1") is not None):
            self.env.RemoveKinBody(self.my1by1)
        
        if(self.infocylinder != None):
            self.infocylinder = None

        # Remove the CBiRRT problems
        if(self.probs_cbirrt != None):
            self.env.Remove(self.probs_cbirrt)
            self.probs_cbirrt = None

        if(self.probs_crankmover != None):
            self.env.Remove(self.probs_crankmover)
            self.probs_crankmover = None

        # Reset the robot's joints
        self.robotid.SetDOFValues(zeros(len(self.robotid.GetJoints())),range(len(self.robotid.GetJoints())))

        # Reset the driving wheel's joint
        self.crankid.SetDOFValues([0],[0])
        self.crankid.GetController().Reset(0)
        

    def SetValvePoseFromQuaternionInFrame(self,frame,trans,rot):
        print "SetWheelPoseFromQuaternion"

        self.tiltDiff = acos(dot(linalg.inv(self.crankid.GetManipulators()[0].GetEndEffectorTransform()),self.crankid.GetLinks()[0].GetTransform())[1,1])

        self.T0_RefLink = None

        for l in self.robotid.GetLinks():
            if(l.GetName() == frame):
                self.T0_RefLink = l.GetTransform()
        
        if(self.T0_RefLink == None):
            rospy.logerr("In base_wheel_turning, SetWheelPoseFromQuaternion: Couldn't find the reference link name.")
        else:
            print "rotation matrix from quat - using openrave function"
            self.TRefLink_Wheel = MakeTransform(matrixFromQuat([rot[3],rot[0],rot[1],rot[2]])[0:3,0:3],matrix(trans))
            
            self.T0_WheelRViz = dot(self.T0_RefLink,self.TRefLink_Wheel)   

            self.TWheelRViz_WheelRave = MakeTransform(dot(rodrigues([0,pi/2-self.tiltDiff,0]),rodrigues([0,0,pi/2])),transpose(matrix([0,0,0])))

            self.T0_WheelRave = dot(self.T0_WheelRViz,self.TWheelRViz_WheelRave)

            # Set wheel location
            # TODO We shall plan after crouchin. But until then just think that the valve 
            # is self.crouch higher. This should be the height of the real wheel after the 
            # robot crouches.
            self.T0_WheelRave[2,3] += self.crouch

            self.crankid.SetTransform(array(self.T0_WheelRave))

    def CreateValve(self,valveRadius,valveType):

        self.r_Wheel = valveRadius

        self.myValveHandle = RaveCreateKinBody(self.env,'')

        if(valveType == "RL"): # valve type: lever with right end at the origin of rotation
            self.myValveHandle.InitFromBoxes(numpy.array([[self.r_Wheel*0.5,0,0,self.r_Wheel*0.5,0.01,0.005]]),True)
        if(valveType == "LL"): # valve type: lever with left end at the origin of rotation
            self.myValveHandle.InitFromBoxes(numpy.array([[-self.r_Wheel*0.5,0,0,self.r_Wheel*0.5,0.01,0.005]]),True)
        elif(valveType == "W"): # valve type: wheel
            # Create a cylinder
            self.infocylinder = KinBody.Link.GeometryInfo()
            self.infocylinder._type = KinBody.Link.GeomType.Cylinder
            self.infocylinder._vGeomData = [self.r_Wheel,0.01] # radius and height/thickness
            self.infocylinder._bVisible = True
            self.infocylinder._fTransparency = 0.0
            self.infocylinder._vDiffuseColor = [0,1,1]           
            self.myValveHandle.InitFromGeometries([self.infocylinder]) # we could add more cylinders in the list

        self.myValveHandle.SetName('valve')
        self.env.Add(self.myValveHandle,True)
        
        self.myValveHandle.SetTransform(self.crankid.GetManipulators()[0].GetTransform())
        # self.Add4by4()
        self.Add1by1()

    def Add1by1(self):
        print "adding a wall"
        self.my1by1 = RaveCreateKinBody(self.env,'')
        self.my1by1.SetName('1by1')
        behindValveClearance = 0.1
        self.my1by1.InitFromBoxes(numpy.array([[0,0,behindValveClearance,0.1525,0.1525,0.001]]),True) # False for not visible
        self.my1by1.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0,0,1,0.5)))
        self.my1by1.SetTransform(self.crankid.GetManipulators()[0].GetEndEffectorTransform())
        self.env.Add(self.my1by1,True)

    def Add4by4(self):

        print "adding a wall"
        self.my4by4 = RaveCreateKinBody(self.env,'')
        self.my4by4.SetName('4by4')
        behindValveClearance = 0.1
        self.my4by4.InitFromBoxes(numpy.array([[0,0,behindValveClearance,0.61,0.61,0.001]]),True) # False for not visible
        self.my4by4.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0,0,1,0.5)))
        self.my4by4.SetTransform(self.crankid.GetManipulators()[0].GetEndEffectorTransform())
        self.env.Add(self.my4by4,True)

    def SetWheelPoseFromTransform(self,T0_Wheel):
        print "SetWheelPoseFromTransform"
        self.T_Wheel = T0_Wheel
        self.crankid.SetTransform(array(self.T_Wheel))        

    def RemoveFiles(self):

        max_traj_num = 10

        # Try to delete all existing trajectory files
        try:
            print "Removing qhullout.txt"
            os.remove("qhullout.txt")
        except OSError, e:
            print e    

        for i in range(max_traj_num):
            try:
                print "Removing movetraj"+str(i)+".txt"
                os.remove(self.default_trajectory_dir+"movetraj"+str(i)+".txt")
            except OSError, e:
                print e

        for i in range(max_traj_num):
            try:
                print "Removing movetraj"+str(i)+".traj"
                os.remove(self.default_trajectory_dir+"movetraj"+str(i)+".traj")
            except OSError, e:
                print e

        for i in range(max_traj_num):
            try:
                print "Removing movetraj"+str(i)+"_retimed.txt"
                os.remove(self.default_trajectory_dir+"movetraj"+str(i)+"_retimed.txt")
            except OSError, e:
                print e


        for i in range(max_traj_num):
            try:
                print "Removing movetraj"+str(i)+"_openhands.traj"
                os.remove(self.default_trajectory_dir+"movetraj"+str(i)+"_openhands.traj")
            except OSError, e:
                print e
                
        for i in range(max_traj_num):
            try:
                print "Removing movetraj"+str(i)+"_closehands.traj"
                os.remove(self.default_trajectory_dir+"movetraj"+str(i)+"_closehands.traj")
            except OSError, e:
                print e

    def StartViewer(self):
        # Start the Viewer and draws the world frame
        if self.ShowUserInterface and not self.ViewerStarted :
            cam_rot = dot(xyz_rotation([3*pi/2,0,0]),xyz_rotation([0,-pi/2,0]))
            cam_rot = dot(cam_rot,xyz_rotation([-pi/10,0,0])) # inclination of the camera
            T_cam = MakeTransform(cam_rot,transpose(matrix([2.0, 0.00, 01.4])))
            self.env.SetViewer('qtcoin')
            self.env.GetViewer().SetCamera(array(T_cam))
            self.env.GetViewer().EnvironmentSync()
            self.ViewerStarted = True
            # handles.append( misc.DrawAxes(self.env,T_cam,1) )
            # handles.append( misc.DrawAxes(self.env,MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0,0]))),1) )

    def StartViewerAndSetValvePos(self, handles, valveType="w"):

        # Start the Viewer and draws the world frame
        if self.ShowUserInterface and not self.ViewerStarted :
            cam_rot = dot(xyz_rotation([3*pi/2,0,0]),xyz_rotation([0,-pi/2,0]))
            cam_rot = dot(cam_rot,xyz_rotation([-pi/10,0,0])) # inclination of the camera
            T_cam = MakeTransform(cam_rot,transpose(matrix([2.0, 0.00, 01.4])))
            self.env.SetViewer('qtcoin')
            self.env.GetViewer().SetCamera(array(T_cam))
            self.env.GetViewer().EnvironmentSync()
            self.ViewerStarted = True
            #handles.append( misc.DrawAxes(self.env,T_cam,1) )
            handles.append( misc.DrawAxes(self.env,MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0,0]))),1) )
            

        # Move the wheel infront of the robot
        # These are the default transform and radius values
        #
        # Get the 
        
        self.wheelHeightFromTSY = 0.15
        self.TSYHeight = self.robotid.GetLinks()[12].GetTransform()[2,3]
        self.wheelHeight = self.TSYHeight + self.wheelHeightFromTSY - self.crouch

        wheelY = 0.0

        self.worldPitch = 0.0

        # Find the difference of angle between the wheel's end effector and the link (23 degrees)
        # We should only do this if we're using the logitech wheel.
        self.tiltDiff = acos(dot(linalg.inv(self.crankid.GetManipulators()[0].GetEndEffectorTransform()),self.crankid.GetLinks()[0].GetTransform())[1,1])
        if self.T_Wheel is None:
            # pi/2-tiltDiff+worldPitch makes sure that the wheel's pitch angle is 0 when it's straight up in the world
            # TODO: This is super hacky. Make it right. As in, define everythig in TSY frame, and then convert it to world frame.
            self.T_Wheel = MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2-self.tiltDiff+self.worldPitch,0,0])),transpose(matrix([self.wheelDistFromTSY, wheelY, self.wheelHeight])))
            self.crankid.SetTransform(array(self.T_Wheel))

        if(self.r_Wheel == None):
            self.r_Wheel = 0.2


        self.myValveHandle = RaveCreateKinBody(self.env,'')

        if(valveType == "RL"): # valve type: right-lever
            self.myValveHandle.InitFromBoxes(numpy.array([[-self.r_Wheel*0.5,0,0,self.r_Wheel*0.5,0.01,0.005]]),True)
        if(valveType == "LL"): # valve type: left-lever
            self.myValveHandle.InitFromBoxes(numpy.array([[self.r_Wheel*0.5,0,0,self.r_Wheel*0.5,0.01,0.005]]),True)
        elif(valveType == "W"): # valve type: wheel
            # Create a cylinder
            self.infocylinder = KinBody.Link.GeometryInfo()
            self.infocylinder._type = KinBody.Link.GeomType.Cylinder
            self.infocylinder._vGeomData = [self.r_Wheel,0.01] # radius and height/thickness
            self.infocylinder._bVisible = True
            self.infocylinder._fTransparency = 0.0
            self.infocylinder._vDiffuseColor = [0,1,1]           
            self.myValveHandle.InitFromGeometries([self.infocylinder]) # we could add more cylinders in the list

        self.myValveHandle.SetName('valve')
        self.env.Add(self.myValveHandle,True)
        

        self.myValveHandle.SetTransform(self.crankid.GetManipulators()[0].GetTransform())
  
        # Draw wheel position
        if self.ShowUserInterface :
            print self.robotid.GetJoints()[11].GetAnchor()
            T_RightFoot = self.robotid.GetLinks()[0].GetTransform()
            T_Torso = self.robotid.GetLinks()[8].GetTransform()
            #handles.append( misc.DrawAxes(self.env,self.T_Wheel,0.5) )  
            #handles.append( misc.DrawAxes(self.env,T_Torso,0.5) ) 
            #handles.append( misc.DrawAxes(self.env,T_RightFoot,0.5) ) 
            print T_Torso
            print T_RightFoot
        

    def Playback(self,retimed=False):

        if( self.StopAtKeyStrokes ):
            print "Press Enter to play the trajectories..."
            sys.stdin.readline()

        retimed_str = ''
        if( retimed ):
            retimed_str = '_retimed'

        close_hands = False
        if( close_hands ):
            # Playback 0:(home-init) -> 1:(init-start) -> 2:(start-goal) -> 3:(goal-start) -> 4:(start-init) -> 5:(init-home)
            self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)

        probs = self.env.GetLoadedProblems()

        try:
            print 'traj '+self.default_trajectory_dir+'movetraj0'+retimed_str+'.txt'
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj0'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 40 # error code 4: playback error, 0: at 0th trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        if( close_hands ):
            self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
        
        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj1'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 41 # error code 4: playback error, 1: at 1st trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        if( close_hands ):
            self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)
            time.sleep(1)

        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj2'+retimed_str+'.txt');
            answer= self.probs_crankmover.SendCommand('traj '+self.default_trajectory_dir+'movetraj2'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 42 # error code 4: playback error, 2: at 2nd trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        if( close_hands ):
            self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
            time.sleep(1)

        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj3'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 43 # error code 4: playback error, 3: at 3rd trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj4'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 44 # error code 4: playback error, 4: at 4th trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        if( close_hands ):
            self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)
            time.sleep(1)

        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj5'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 45 # error code 4: playback error, 5: at 5th trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        
        if( self.StopAtKeyStrokes ):
            print "Press Enter to exit..."
            sys.stdin.readline()

        # file_names = [ 'movetraj0.txt','movetraj1.txt','movetraj2.txt','movetraj3.txt','movetraj4.txt','movetraj5.txt' ]
        # return file_names
            
        return 0 # If you are here, there is no error, return 0

    def AddWall(self,p = [0,0,0]):
        print "adding a wall"
        body = RaveCreateKinBody(self.env,'')
        body.SetName('wall')
        behindValveClearance = 0.08
        body.InitFromBoxes(numpy.array([[self.wheelDistFromTSY+behindValveClearance,0,0.61,0.001,0.61,1.22]]),True) # False for not visible
        self.env.Add(body,True)

    def InitFromTaskWallEnv(self):
        self.env.Load(roslib.packages.get_pkg_dir("wpi_drc_sim")+'/../models/drc_task_wall.env.xml')
