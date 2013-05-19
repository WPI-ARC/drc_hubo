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
from math import *
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
    # This is a list of handles of the objects that are
    # drawn on the screen in OpenRAVE Qt-Viewer.
    # Keep appending to the end, and pop() if you want to delete.
    handles = [] 

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

    # Keep Active Joint Indices
    # Note that 0 is the driving wheel
    activedofs = [0]
    for m in manips:
        activedofs.extend(m.GetArmIndices())

    # Sort Active Joint Indices
    activedofs.sort()

    # Set Elbows and Thumbs Joint Values
    robotid.SetDOFValues([-0.95,-0.95,1,1],[19,20,41,56]) 
    robotid.SetActiveDOFs(activedofs)

    # Current configuration of the robot is its initial configuration
    initconfig = robotid.GetDOFValues()

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
    handles.append(misc.DrawAxes(env,matrix(jointtm),1))
    

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
    rhanddofs = range(27,42)
    rhandclosevals = [0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0, 0, 1.2]
    rhandopenvals = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.08]

    # Left Hand Joints
    lhanddofs = range(42,57)
    lhandclosevals = [0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0, 0, 1.2]
    lhandopenvals =  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.08]

    # polyscale: changes the scale of the support polygon
    # polytrans: shifts the support polygon around
    footlinknames = ' Body_RAR Body_LAR polyscale 0.7 0.5 0 polytrans -0.015 0 0 '

    # What is this?
    handrot = rodrigues([0,-pi/2,0])
    
    # Translation Offset from the wheel center for the hands
    transoffset = [0, 0.15, 0];
    
    # Figure out where to put the left hand on the wheel
    temp = dot(CTee, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))
    temp = dot(temp, MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0]))))
    
    # Left Hand Pose in World Coordinates
    T0_LH1 = dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.15,0]))))

    # Uncomment if you want to see where T0_LH1 is 
    # handles.append(misc.DrawAxes(env,matrix(T0_LH1),1))

    # Figure out where to put the right hand on the wheel
    temp = dot(CTee, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))
    temp = dot(temp, MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0]))))
    # Right Hand Pose in World Coordinates
    T0_RH1 = dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,-0.15,0]))))
    
    # Uncomment if you want to see where T0_RH1 is 
    # handles.append(misc.DrawAxes(env,matrix(T0_RH1),1))
    
    # Define Task Space Region strings
    # Left Hand
    TSRString1 = SerializeTSR(0,'NULL',T0_LH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

    # Right Hand
    TSRString2 = SerializeTSR(1,'NULL',T0_RH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    
    # Left Foot
    TSRString3 = SerializeTSR(2,'NULL',Tee[2],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

    # Head
    # Grasp transform in Head coordinates
    Tw0_eH = eye(4) 
    # How much freedom do we want to give to the Head
    # [x,x,y,y,z,z,R,R,P,P,Y,Y]
    Bw0H = matrix([0,0,-0.1,0.1,-0.1,0.01,0,0,0,0,0,0])
    TSRString4 = SerializeTSR(4,'NULL',Tee[4],Tw0_eH,Bw0H)

    # We defined Task Space Regions. Now let's concatenate them.
    TSRChainStringGrasping = SerializeTSRChain(0,1,0,1,TSRString1,'NULL',[])+' '+SerializeTSRChain(0,1,0,1,TSRString2,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString4,'NULL',[])
    
    sys.stdin.readline()
        
    # Get a trajectory from initial configuration to grasp configuration
    try:
        answer = probs[0].SendCommand('RunCBiRRT psample 0.2 supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' '+TSRChainStringGrasping)
        print "RunCBiRRT answer: ",str(answer)
    except openrave_exception, e:
        print "Cannot send command RunCBiRRT: "
        print e

    try:
        os.rename("cmovetraj.txt","movetraj0.txt")
        print "Executing trajectory 0"
        try:
            answer= probs[0].SendCommand('traj movetraj0.txt');
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
        robotid.WaitForController(0)
    except OSError, e:
        # No file cmovetraj
        print e
    
    # Get the current configuration of the robot
    # and assign it to startik (start of the wheel
    # rotation path).
    startik = robotid.GetDOFValues()
    
    # Left Hand's index is less than the right hand.
    # Hence it is evaluated first by the CBiRRT Module.
    # That's why We need to define the right hand's 
    # transform relative to the wheel (ask Dmitry Berenson
    # about this for more information).
    print jointtm[0:3,3]
    temp1 = MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0])))
    temp2 = MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0])))
    # Rotate the wheel's transform to a suitable pose
    # for the Left Hand
    # T0_w0L stands for: 
    # left hand's transform on wheel in world coordinates
    T0_w0L = dot(dot(CTee,temp1),temp2)

    # This is what's happening: 
    #
    # Tw0L_0 = linalg.inv(T0_w0L)
    # Tw0L_LH1 = Tw0L_0*T0_LH1
    #
    # Left hand's transform in wheel's coordinates
    Tw0L_LH1 = dot(linalg.inv(T0_w0L),T0_LH1)

    # Transform of the left hand's end effector in wheel's coords.
    # Required by CBiRRT
    Tw0_eL = Tw0L_LH1

    # How much freedom do we want to give to the left hand
    Bw0L = matrix([0,0,0,0,0,0,0,pi,0,0,0,0])

    # Now calculate Right Hand's transforms:
    T0_crankcrank = CTee
    T0_w0R = MakeTransform(rodrigues([tilt_angle_rad,0,0]),transpose(matrix([0,0,0])))

    # End effector transform in wheel coordinates
    Tw0_eR = dot(linalg.inv(T0_crankcrank),T0_RH1)
    
    # How much freedom? (note: in frame of crank)
    Bw0R = matrix([0,0,0,0,0,0,0,0,0,0,0,0])
    
    # Define Task Space Regions
    # Left Hand
    TSRString1 = SerializeTSR(0,'NULL',T0_w0L,Tw0_eL,Bw0L)
    TSRstring2 = SerializeTSR(1,'crank crank',T0_w0R,Tw0_eR,Bw0R)
    TSRstring3 = SerializeTSR(2,'NULL',Tee[2],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    TSRChainStringFootOnly = SerializeTSRChain(0,0,1,1,TSRstring3,'NULL',[])
    

    print "Press Enter to exit..."
    sys.stdin.readline()
    env.Destroy()
    RaveDestroy()

if __name__ == "__main__":
    run()
    
