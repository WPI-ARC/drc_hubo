#!/usr/bin/env python
# Jim Mainprice, RAIL
# May 2013
# Worcester Polytechnic Institute
#

# http://openrave.org/docs/latest_stable/command_line_tools/
# openrave-robot.py /your/path/to/your.robot.xml --info=joints
# On that page you can find more examples on how to use openrave-robot.py.

from openravepy import *
import sys
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
    import numpy
import time
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *
from math import *
from copy import *
import os # for file operations
from RaveCBiRRT import *

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
        
        # Set those variables to show or hide the interface
        # Do it using the member functions
        self.StopAtKeyStrokes = False
        self.ShowUserInterface = False
        self.ViewerStarted = False

        self.T_Wheel = None
        self.HuboModelPath = HuboModelPath
        self.WheelModelPath = WheelModelPath

        # Start Environment
        self.env = Environment()
        RaveSetDebugLevel(DebugLevel.Info) # set output level to debug
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

    def RemoveFiles(self):

        # Try to delete all existing trajectory files
        try:
            print "Removing qhullout.txt"
            os.remove("qhullout.txt")
        except OSError, e:
            print e    

        for i in range(4):
            try:
                print "Removing movetraj"+str(i)+".txt"
                os.remove("movetraj"+str(i)+".txt")
            except OSError, e:
                print e

    def Playback(self):
        # Playback 0:(init-start) -> 1:(start-goal) -> 2:(goal-start) -> 3:(start-init)

        self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
        self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)

        probs = self.env.GetLoadedProblems()

        try:
            answer= probs[0].SendCommand('traj movetraj0.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
        self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)
        time.sleep(1)

        try:
            answer= probs[0].SendCommand('traj movetraj1.txt');
            answer= probs[1].SendCommand('traj movetraj1.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
        self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
        time.sleep(1)

        try:
            answer= probs[0].SendCommand('traj movetraj2.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        try:
            answer= probs[0].SendCommand('traj movetraj3.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
        
        self.robotid.GetController().Reset(0)

        if( self.StopAtKeyStrokes ):
            print "Press Enter to exit..."
            sys.stdin.readline()

        file_names = [ 'movetraj0.txt','movetraj1.txt','movetraj2.txt','movetraj1.txt','movetraj3.txt']
        return file_names

