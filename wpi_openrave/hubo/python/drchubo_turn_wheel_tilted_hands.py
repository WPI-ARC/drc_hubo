#!/usr/bin/env python
# Ben Suay, RAIL
# July 2013
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
from math import *
from copy import *
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

        # This is a list of handles of the objects that are
    # drawn on the screen in OpenRAVE Qt-Viewer.
    # Keep appending to the end, and pop() if you want to delete.
    handles = [] 

    normalsmoothingitrs = 150;
    fastsmoothingitrs = 20;
    env = Environment()
    RaveSetDebugLevel(DebugLevel.Info) # set output level to debug
    robotid = env.ReadRobotURI('../../../openHubo/drchubo/robots/drchubo.robot.xml')
    crankid = env.ReadRobotURI('../../../../drc_common/models/driving_wheel.robot.xml')
    env.Add(robotid)
    env.Add(crankid)
    env.SetViewer('qtcoin')

    shiftUp = 0.95

        # Move the wheel infront of the robot
    crankid.SetTransform(array(MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2,0,0])),transpose(matrix([0.5, 0.0, shiftUp])))))
    
    probs_cbirrt = RaveCreateModule(env,'CBiRRT')
    probs_crankmover = RaveCreateModule(env,'CBiRRT')

    manips = robotid.GetManipulators()
    crankmanip = crankid.GetManipulators()
    
    try:
        env.AddModule(probs_cbirrt,robotid.GetName())
        env.AddModule(probs_crankmover,crankid.GetName())
    except openrave_exception, e:
        print e

    print "Getting Loaded Problems"
    probs = env.GetLoadedProblems()

    # Keep Active Joint Indices
    activedofs = [0]
    for m in manips:
        activedofs.extend(m.GetArmIndices())

    # Sort Active Joint Indices
    activedofs.sort()

    # elbows: Left Elbow Pitch: 4; Right Elbow Pitch: 22
    robotid.SetDOFValues([-0.95,-0.95],[4,22]) 
    robotid.SetActiveDOFs(activedofs)

    # Current configuration of the robot is its initial configuration
    initconfig = robotid.GetActiveDOFValues()

    # List of Robot Links
    links = robotid.GetLinks()
    
    # List of Wheel (Crank Links)
    cranklinks = crankid.GetLinks()
    
    # End Effector Transforms
    Tee = []
    for i in range(len(manips)):
        # Returns End Effector Transform in World Coordinates
        Tlink = manips[i].GetEndEffectorTransform()
        Tee.append(Tlink)

    # Wheel Joint Index  
    crankjointind = 0

        # Get Transformation Matrix for the Wheel
    # Note that crank's links are not rotated
    # If you want use the wheel's end effector's transformation
    # matrix (which is 23 degrees tilted) then see
    # CTee matrix below.
    #
    # crank has two links: 
    # 0) pole - the blue cylinder in the model, and, 
    # 1) crank - the driving wheel itself.
    jointtm = cranklinks[0].GetTransform()
    # handles.append(misc.DrawAxes(env,matrix(jointtm),1))
    

    # We can also get the transformation matrix
    # with the following command as a string
    jointtm_str = probs[0].SendCommand('GetJointTransform name crank jointind '+str(crankjointind))
    # And then we can convert the string to a 1x12 array
    jointtm_str = jointtm_str.replace(" ",",")
    jointtm_num = eval('['+jointtm_str+']')

    # In this script we will use jointtm.
    # jointtm_str and jointtm_num are given as example.
    
    # Crank Transform End Effector in World Coordinates
    # This is the transformation matrix of the end effector 
    # named "dummy" in the xml file.
    # Note that dummy is tilted 23 degress around its X-Axis
    CTee = crankmanip[0].GetEndEffectorTransform()

    

    tilt_angle_deg = acos(dot(linalg.inv(CTee),jointtm)[1,1])*180/pi
    tilt_angle_rad = acos(dot(linalg.inv(CTee),jointtm)[1,1])
    

    # Center of Gravity Target
    cogtarg = [-0.05, 0.085, 0]
    
    # Right Hand Joints 
    # Open - Closed Values
    rhanddofs = range(26,29)
    rhandclosevals = [0.95, 0.95, -0.95]
    rhandopenvals = [0, 0, 0]

    # Left Hand Joints
    lhanddofs = range(8,11)
    lhandclosevals = [0.95, 0.95, -0.95]
    lhandopenvals = [0, 0, 0]

    # polyscale: changes the scale of the support polygon
    # polytrans: shifts the support polygon around
    footlinknames = ' Body_RAR Body_LAR polytrans 0 0 1.0 '

    # What is this?
    handrot = rodrigues([0,-pi/2,0])
    
    # Translation Offset from the wheel center for the hands
    transoffset = [0, 0.15, 0];

    # Figure out where to put the left hand on the wheel
    temp = dot(CTee, MakeTransform(rodrigues([0,-pi/2,0]),transpose(matrix([0,0,0]))))
    temp = dot(temp, MakeTransform(rodrigues([-pi,0,0]),transpose(matrix([0,0,0]))))
    
    # Left Hand Pose in World Coordinates
    T0_LH1 = dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0,0.145]))))

    # Uncomment if you want to see where T0_LH1 is 
    handles.append(misc.DrawAxes(env,matrix(T0_LH1),1))

    # Figure out where to put the right hand on the wheel
    temp = dot(CTee, MakeTransform(rodrigues([0,-pi/2,0]),transpose(matrix([0,0,0]))))
    
    # Right Hand Pose in World Coordinates
    T0_RH1 = dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0,0.145]))))

    # Uncomment if you want to see where T0_RH1 is 
    handles.append(misc.DrawAxes(env,matrix(T0_RH1),1))

        # Define Task Space Region strings
    # Left Hand
    TSRString1 = SerializeTSR(0,'NULL',T0_LH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

    # Right Hand
    TSRString2 = SerializeTSR(1,'NULL',T0_RH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    
    # Left Foot
    TSRString3 = SerializeTSR(2,'NULL',Tee[2],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

    # Right Foot
    TSRString4 = SerializeTSR(3,'NULL',Tee[3],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

    # Head
    # Grasp transform in Head coordinates
    Tw0_eH = eye(4) 
    # How much freedom do we want to give to the Head
    # [x,x,y,y,z,z,R,R,P,P,Y,Y]
    Bw0H = matrix([-0.05,0.05,-0.1,0.1,-100,100,-pi,pi,-pi,pi,-pi,pi])
    TSRString5 = SerializeTSR(4,'NULL',Tee[4],Tw0_eH,Bw0H)

    # We defined Task Space Regions. Now let's concatenate them.
    TSRChainStringGrasping = SerializeTSRChain(0,1,1,1,TSRString1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString2,'NULL',[])

    print "Press Enter to plan initconfig --> startik"
    sys.stdin.readline()

    print "Press Enter to exit..."
    sys.stdin.readline()
    env.Destroy()
    RaveDestroy()

if __name__ == "__main__":
    run()
