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
    # cogtarg = [-0.05, 0.085, 0]
    cogtarg = [0, 0, 0]
    
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
    footlinknames = ' Body_RAR Body_LAR ' #polytrans -0.015 0 0.0 '

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
    TSRChainStringGrasping = SerializeTSRChain(0,1,0,1,TSRString1,'NULL',[])+' '+SerializeTSRChain(0,1,0,1,TSRString2,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString4,'NULL',[])#+' '+SerializeTSRChain(0,1,0,1,TSRString5,'NULL',[])

    print "Press Enter to go to startik"
    sys.stdin.readline()

    startik = probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 0 '+trans_to_str(T0_LH1)+' maniptm 1 '+trans_to_str(T0_RH1))
    
    # Rotate the wheel's transform to a suitable pose
    # for the Left Hand
    # T0_w0L stands for: 
    # left hand's transform on wheel in world coordinates
    T0_w0L = MakeTransform(rodrigues([0,tilt_angle_rad,0]),transpose(matrix(jointtm[0:3,3])))
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

    # Right Hand's transforms:
    T0_crankcrank = crankid.GetManipulators()[0].GetTransform()
    T0_w0R = MakeTransform(rodrigues([tilt_angle_rad,0,0]),transpose(matrix([0,0,0])))
    # End effector transform in wheel coordinates
    Tw0_eR = dot(linalg.inv(T0_crankcrank),T0_RH1)

    # handles.append(misc.DrawAxes(env,matrix(Tw0_eR),1))

    # How much freedom? (note: in frame of crank)
    Bw0R = matrix([0,0,0,0,0,0,0,0,0,0,0,0])

    # Head's transforms:
    T0_w0H =  Tee[4]
    Tw0_eH = eye(4);
    Bw0H = matrix([-0.05,0.05,-0.1,0.1,-100,100,-pi,pi,-pi,pi,-pi,pi])
    
    
    # Define Task Space Regions
    # Left Hand
    TSRString1 = SerializeTSR(0,'NULL',T0_w0L,Tw0_eL,Bw0L)
    # Right Hand
    TSRString2 = SerializeTSR(1,'crank crank',T0_w0R,Tw0_eR,Bw0R)
    # Left Foot
    TSRString3 = SerializeTSR(2,'NULL',Tee[2],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    # Right Foot
    TSRString4 = SerializeTSR(3,'NULL',Tee[3],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    # Head
    TSRString5 = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H)

    TSRChainStringFeetandHead = SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

    TSRChainString = SerializeTSRChain(0,0,1,1,TSRString1,'crank',matrix([crankjointind]))+' '+SerializeTSRChain(0,0,1,1,TSRString2,'NULL',matrix([]))+' '+TSRChainStringFeetandHead
    
    # Calculate hand transforms after rotating the wheel (they will help us find the goalik):
    # How much do we want to rotate the wheel?
    crank_rot = pi/6.5
    
    # Which joint do we want the CBiRRT to mimic the TSR for?
    TSRChainMimicDOF = 1
    
    # Create the transform for the wheel that we would like to reach to
    Tcrank_rot = MakeTransform(rodrigues([crank_rot,0,0]),transpose(matrix([0,0,0])))
    
    # What is this?
    temp = MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0])))
    
    # Rotate the left hand's transform on the wheel in world transform "crank_rot" radians around it's Z-Axis
    T0_cranknew = dot(T0_w0L,Tcrank_rot)
    
    # Where will the left hand go after turning the wheel?
    # This is what's happening:
    #
    # Tcranknew_LH2 = dot(Tw0L_0,T0_LH1) --> Left hand in wheel's coordinate
    # T0_LH2 = dot(T0_cranknew,Tcranknew_LH2) --> Left hand rotated around wheel's origin
    T0_LH2 = dot(T0_cranknew,dot(linalg.inv(T0_w0L),T0_LH1))

    # Uncomment to see T0_LH2
    handles.append(misc.DrawAxes(env,matrix(T0_LH2),1))
    
    # Where will the right hand go after turning the wheel?
    T0_RH2 = dot(T0_crankcrank,dot(temp,dot(linalg.inv(T0_crankcrank),T0_RH1)))

    # Uncomment to see T0_RH2
    handles.append(misc.DrawAxes(env,matrix(T0_RH2),1))

    print "Press Enter to find a goalIK"
    sys.stdin.readline()

    crankid.SetDOFValues([crank_rot],[crankjointind])

    arg1 = str(cogtarg).strip("[]").replace(', ',' ')
    arg2 = trans_to_str(T0_LH2)
    arg3 = trans_to_str(T0_RH2)
    

    goalik = probs[0].SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames+' movecog '+arg1+' nummanips 2 maniptm 0 '+arg2+' maniptm 1 '+arg3)
    
    robotid.SetActiveDOFValues(str2num(goalik))
    crankid.SetDOFValues([crank_rot],[crankjointind])

    crankid.SetDOFValues([0],[crankjointind])
    robotid.SetActiveDOFValues(initconfig)

    robotid.SetDOFValues(rhandopenvals,rhanddofs)
    robotid.SetDOFValues(lhandopenvals,lhanddofs)

    print "Press Enter to plan initconfig --> startik"
    sys.stdin.readline()

    # Get a trajectory from initial configuration to grasp configuration
    with robotid:
        try:
            answer = probs[0].SendCommand('RunCBiRRT psample 0.2 supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' '+TSRChainStringGrasping)
            print "RunCBiRRT answer: ",str(answer)
        except openrave_exception, e:
            print "Cannot send command RunCBiRRT: "
            print e

    try:
        os.rename("cmovetraj.txt","movetraj0.txt")
    except OSError, e:
        # No file cmovetraj
        print e

    # The following is the same as commented out try-except section
    traj = RaveCreateTrajectory(env,'').deserialize(open('movetraj0.txt','r').read())   
    robotid.GetController().SetPath(traj) 
    robotid.WaitForController(0)
    robotid.GetController().Reset(0) 

    robotid.SetDOFValues(rhandclosevals,rhanddofs)
    robotid.SetDOFValues(lhandclosevals,lhanddofs)

    print "Press Enter to plan startik --> goalik "
    sys.stdin.readline()

    # Get a trajectory from goalik to grasp configuration
    goaljoints = deepcopy(goalik)
    for i in range(TSRChainMimicDOF):
        goaljoints += ' 0'

    goaljoints = str2num(goaljoints)

    try:
        answer = probs[0].SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(fastsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainString)
        print "RunCBiRRT answer: ",str(answer)
    except openrave_exception, e:
        print "Cannot send command RunCBiRRT: "
        print e

    try:
        os.rename("cmovetraj.txt","movetraj1.txt")
    except OSError, e:
        # No file cmovetraj
        print e

    try:
        answer= probs[0].SendCommand('traj movetraj1.txt');
        answer= probs[1].SendCommand('traj movetraj1.txt');
        robotid.WaitForController(0)
        # debug
        print "traj call answer: ",str(answer)
    except openrave_exception, e:
        print e
    
    robotid.GetController().Reset(0)
    robotid.SetDOFValues(rhandopenvals,rhanddofs)
    robotid.SetDOFValues(lhandopenvals,lhanddofs)

    time.sleep(2)

    print "Press Enter to plan goalik --> startik "
    sys.stdin.readline()

    goaljoints = str2num(startik)

    try:
        answer = probs[0].SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetandHead)
        print "RunCBiRRT answer: ",str(answer)
    except openrave_exception, e:
        print "Cannot send command RunCBiRRT: "
        print e

    try:
        os.rename("cmovetraj.txt","movetraj2.txt")
    except OSError, e:
        # No file cmovetraj
        print e

    try:
        answer= probs[0].SendCommand('traj movetraj2.txt');
        robotid.WaitForController(0)
        # debug
        print "traj call answer: ",str(answer)
    except openrave_exception, e:
        print e
    
    robotid.GetController().Reset(0)
    
    print "Press Enter to plan startik --> initconfig "
    sys.stdin.readline()

    goaljoints = initconfig

    print goaljoints
    try:
        answer = probs[0].SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetandHead)
        print "RunCBiRRT answer: ",str(answer)
    except openrave_exception, e:
        print "Cannot send command RunCBiRRT: "
        print e

    try:
        os.rename("cmovetraj.txt","movetraj3.txt")
    except OSError, e:
        # No file cmovetraj
        print e

    try:
        answer= probs[0].SendCommand('traj movetraj3.txt');
        robotid.WaitForController(0)
        # debug
        print "traj call answer: ",str(answer)
    except openrave_exception, e:
        print e
    
    robotid.GetController().Reset(0)
    
    # Playback 0:(init-start) -> 1:(start-goal) -> 2:(goal-start) -> 3:(start-init)

    robotid.SetDOFValues(rhandopenvals,rhanddofs)
    robotid.SetDOFValues(lhandopenvals,lhanddofs)
    try:
        answer= probs[0].SendCommand('traj movetraj0.txt');
        robotid.WaitForController(0)
        # debug
        print "traj call answer: ",str(answer)
    except openrave_exception, e:
        print e
    
    robotid.GetController().Reset(0)
    time.sleep(1)

    robotid.SetDOFValues(rhandclosevals,rhanddofs)
    robotid.SetDOFValues(lhandclosevals,lhanddofs)
    time.sleep(1)

    try:
        answer= probs[0].SendCommand('traj movetraj1.txt');
        answer= probs[1].SendCommand('traj movetraj1.txt');
        robotid.WaitForController(0)
        # debug
        print "traj call answer: ",str(answer)
    except openrave_exception, e:
        print e
    
    robotid.GetController().Reset(0)
    time.sleep(1)

    robotid.SetDOFValues(rhandopenvals,rhanddofs)
    robotid.SetDOFValues(lhandopenvals,lhanddofs)
    time.sleep(1)

    try:
        answer= probs[0].SendCommand('traj movetraj2.txt');
        robotid.WaitForController(0)
        # debug
        print "traj call answer: ",str(answer)
    except openrave_exception, e:
        print e
    
    robotid.GetController().Reset(0)
    time.sleep(1)

    try:
        answer= probs[0].SendCommand('traj movetraj3.txt');
        robotid.WaitForController(0)
        # debug
        print "traj call answer: ",str(answer)
    except openrave_exception, e:
        print e
    
    robotid.GetController().Reset(0)


    print "Press Enter to exit..."
    sys.stdin.readline()
    env.Destroy()
    RaveDestroy()

if __name__ == "__main__":
    run()
