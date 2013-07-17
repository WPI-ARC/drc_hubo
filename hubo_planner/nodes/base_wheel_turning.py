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

        self.T0_tsy_home = self.robotid.GetLinks()[12].GetTransform()
        self.T0_lar_home = self.robotid.GetManipulators()[2].GetEndEffectorTransform()
        self.T0_rar_home = self.robotid.GetManipulators()[3].GetEndEffectorTransform()
        self.Ttsy_lar_home = array(dot(linalg.inv(self.T0_tsy_home),self.T0_lar_home))
        self.Ttsy_rar_home = array(dot(linalg.inv(self.T0_tsy_home),self.T0_rar_home))

        self.wheelDistFromTSY = 0.4

    def KillOpenrave(self):
        self.env.Destroy()
        RaveDestroy()

    def SetViewer(self,arg=True):
        print "SetViewer"
        self.ShowUserInterface = arg

    def SetStopKeyStrokes(self,arg=True):
        print "SetStopKeyStrokes"
        self.StopAtKeyStrokes = arg

    def SetWheelPosition(self,trans,rot,radius):
        print "SetWheelPosition"
        self.T_Wheel = MakeTransform(rotationMatrixFromQuat(rot),matrix(trans))
        self.crankid.SetTransform(array(self.T_Wheel))

    def SetWheelPoseFromTransform(self,T0_Wheel):
        print "SetWheelPosition"
        self.T_Wheel = T0_Wheel
        self.crankid.SetTransform(array(self.T_Wheel))        

    def RemoveFiles(self):

        # Try to delete all existing trajectory files
        try:
            print "Removing qhullout.txt"
            os.remove("qhullout.txt")
        except OSError, e:
            print e    

        for i in range(10):
            try:
                print "Removing movetraj"+str(i)+".txt"
                os.remove("movetraj"+str(i)+".txt")
            except OSError, e:
                print e

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

        if(valveType == "l"): # valve type: lever
            self.myValveHandle.InitFromBoxes(numpy.array([[-self.r_Wheel*0.5,0,0,self.r_Wheel*0.5,0.01,0.005]]),True)
        elif(valveType == "w"): # valve type: wheel
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
            print "Press Enter to exit..."
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
            answer= self.probs_cbirrt.SendCommand('traj movetraj0'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        if( close_hands ):
            self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
        
        try:
            answer= self.probs_cbirrt.SendCommand('traj movetraj1'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
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
            answer= self.probs_cbirrt.SendCommand('traj movetraj2'+retimed_str+'.txt');
            answer= self.probs_crankmover.SendCommand('traj movetraj2'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
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
            answer= self.probs_cbirrt.SendCommand('traj movetraj3'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        try:
            answer= self.probs_cbirrt.SendCommand('traj movetraj4'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
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
            answer= self.probs_cbirrt.SendCommand('traj movetraj5'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        
        if( self.StopAtKeyStrokes ):
            print "Press Enter to exit..."
            sys.stdin.readline()

        file_names = [ 'movetraj0.txt','movetraj1.txt','movetraj2.txt','movetraj3.txt','movetraj4.txt','movetraj5.txt' ]
        return file_names

    def AddWall(self,p = [0,0,0]):
        print "adding a wall"
        body = RaveCreateKinBody(self.env,'')
        body.SetName('wall')
        behindValveClearance = 0.08
        body.InitFromBoxes(numpy.array([[self.wheelDistFromTSY+behindValveClearance,0,0.61,0.001,0.61,1.22]]),True) # False for not visible
        self.env.Add(body,True)

    def InitFromTaskWallEnv(self):
        self.env.Load(roslib.packages.get_pkg_dir("wpi_drc_sim")+'/../models/drc_task_wall.env.xml')

