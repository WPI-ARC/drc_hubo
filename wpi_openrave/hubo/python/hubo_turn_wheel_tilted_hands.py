#!/usr/bin/env python
# Ben Suay, RAIL
# May 2013
# Worcester Polytechnic Institute
#

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
import os # for file operations

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

def run():
    normalsmoothingitrs = 150;
    fastsmoothingitrs = 20;
    env = Environment()
    RaveSetDebugLevel(DebugLevel.Info) # set output level to debug
    robotid = env.ReadRobotURI('../../../openHubo/huboplus/rlhuboplus_mit.robot.xml')
    crankid = env.ReadRobotURI('../../../../drc_common/models/driving_wheel.robot.xml')
    env.Add(robotid)
    env.Add(crankid)
    env.SetViewer('qtcoin')
    
    # Move the wheel infront of the robot
    crankid.SetTransform(array(MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2,0,0])),transpose(matrix([0.18, 0.09, 0.9])))))
    
    probs_cbirrt = RaveCreateModule(env,'CBiRRT')
    probs_crankmover = RaveCreateModule(env,'CBiRRT')


    manips = robotid.GetManipulators()
    crankmanip = crankid.GetManipulators()
    
    try:
        env.AddModule(probs_cbirrt,'rlhuboplus') # this string should match to kinematic body
        env.AddModule(probs_crankmover,'crank')
    except openrave_exception, e:
        print e

    print "Getting Loaded Problems"
    probs = env.GetLoadedProblems()

    activedofs = [0]
    for m in manips:
        activedofs.extend(m.GetArmIndices())
    activedofs.sort()
    print activedofs

    robotid.SetDOFValues([-0.95,-0.95,1,1],[19,20,41,56]) # elbows and thumbs
    robotid.SetActiveDOFs(activedofs)
    initconfig = robotid.GetDOFValues()
    
    
    print "Press Enter to exit..."
    sys.stdin.readline()
    env.Destroy()
    RaveDestroy()

if __name__ == "__main__":
    run()
    
