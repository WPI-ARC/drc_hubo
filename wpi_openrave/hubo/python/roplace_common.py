# Bener Suay, June 2013
#
# benersuay@wpi.edu
#

## OPENRAVE ##
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

from openravepy.misc import OpenRAVEGlobalArguments

## ROBOT PLACEMENET ##
# from Reachability import *

## MATH ##
from random import *

## SYSTEM - FILE OPS ##
import sys
import os
from datetime import datetime
import time
import commands

## Constraint Based Manipulation ##
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *

from copy import deepcopy

from math import *

# if ||qA-qB|| > threshold then consider this diff as a configuration jump
# This number would change from manipulator to manipulator
configurationJumpThreshold = 100.0

# Activates some prints and keyboard inputs
debug = False

def pattern_exists(s1, s2, myPatterns, myRmaps):
    # try to grow the search patterns out of s1 and s2
    # if successful return the path elements
    pass
    
def pair_exists(s, myPatterns, myRmaps):
    # find the sister of s
    # see if there is the transform we want
    # between s and its sister.
    
    pass

def rank_reachability_spheres(s):
    # this function returns a list of sphere indices
    # based on their ranking of potential success
    # as a candidate initial reachability sphere
    #
    # For now let's start with the 1st order
    # neighbors of the sphere, but we may need
    # to do something more complicated, and expand
    # to 2nd or 3rd order neighbors later on...
    return s.neighbors
    

def find_nearest_reachability_sphere(x, y, z, sList):
    print "Test point:"
    print x
    print y
    print z

    minDist = 1000.0 # some big number
    minIdx = None # to be assigned when found

    for sIdx, s in enumerate(sList): # remember rmap is a list of reachability spheres
        euclideanDistance = pow(pow(s.T[0][0,3]-x,2)+pow(s.T[0][1,3]-y,2)+pow(s.T[0][2,3]-z,2),0.5)
        if(euclideanDistance < minDist):
            found_x = s.T[0][0,3]
            found_y = s.T[0][1,3]
            found_z = s.T[0][2,3]
            minDist = euclideanDistance
            minIdx = sIdx
            
    print "Closest reachability sphere is at:"
    print found_x
    print found_y
    print found_z

    print "Euclidean distance in between:"
    print minDist

    return [minIdx, minDist]

def execute(myRobot, myObject, myTraj):
    # close hands
    myRobot.GetController().SetPath(myTraj)
    myObject.GetController().SetPath(myTraj)
    myRobot.WaitForController(0)
    myObject.WaitForController(0)
    myRobot.GetController().Reset(0)
    myObject.GetController().Reset(0)
    # open hands

def go_to_startik(myRobot, startik):
    # print "Went to startik"
    myRobot.SetActiveDOFValues(str2num(startik))
    time.sleep(0.05)
    if(debug):
        sys.stdin.readline()

def get_lin_goalik(myRobot,candidates, c):
    # This function calculates the goalik by moving
    # the robot to the last solution in
    myIK = myRobot.GetActiveDOFValues()
    return Serialize1DMatrix(matrix(myIK))

def get_rot_goalik(myRobot, T0_LH2, T0_RH2):
    # This function calculates the goalik using the rotAngle
    # and the initial hand transforms instead of using the last
    # reachability sphere's transform. This is required because
    # of the discretization that the reachbility map is imposing
    # on trajectories.

    # T0_LH1
    # myRmaps[0].go_to(candidates[0][c][0].sIdx,candidates[0][c][0].tIdx)

    # T0_RH1
    # myRmaps[1].go_to(candidates[1][c][0].sIdx,candidates[1][c][0].tIdx)
    
    myManip = myRobot.SetActiveManipulator('leftArmManip')
    qL = myManip.FindIKSolution(array(T0_LH2), IkFilterOptions.CheckEnvCollisions) # get collision-free solution
    if(qL != None):
        LJ = myManip.GetArmJoints()
        myRobot.SetDOFValues(qL,LJ)

        myManip = myRobot.SetActiveManipulator('rightArmManip')
        qR = myManip.FindIKSolution(array(T0_RH2), IkFilterOptions.CheckEnvCollisions) # get collision-free solution
        if(qR != None):
            RJ = myManip.GetArmJoints()
            myRobot.SetDOFValues(qR,RJ)

            myIK = myRobot.GetActiveDOFValues()
            return Serialize1DMatrix(matrix(myIK))
        
    return None

# This function returns the TSRChain String assuming that
# the robot is currently in goal configuration
def get_tsr_chain_string(myRobot, TSRLeft, TSRRight, myObject, mimicObjectKinBody, mimicObjectLink, mimicObjectJoint=None):

    T0_LH = TSRLeft[0] # left hand's transform on wheel in world coordinates
    TObj_LH = TSRLeft[1] # Left hand's transform in object's coordinates
    Bw0L = TSRLeft[2] # How much freedom do we want to give to the left hand

    T0_RH = TSRRight[0] # Right hand's transform in world coordinates
    TObj_RH = TSRRight[1] # Right hand's transform in object's coordinates
    Bw0R = TSRRight[2] # How much freedom do we want to give to the right hand

    # Define Task Space Regions
    # Left Hand
    TSRString0 = SerializeTSR(0,'NULL',T0_LH,TObj_LH,Bw0L)

    # Right Hand
    TSRString1 = SerializeTSR(1, mimicObjectKinBody+' '+mimicObjectLink, T0_RH, TObj_RH, Bw0R)
    # Left Foot
    TSRString2 = SerializeTSR(2,'NULL',myRobot.GetManipulators()[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    # Head
    TSRString3 = SerializeTSR(3,'NULL',myRobot.GetManipulators()[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

    if(mimicObjectJoint != None):
        # For rotational trajectories
        TSRChainStringTurning = SerializeTSRChain(0,0,1,1,TSRString0,mimicObjectLink,matrix([mimicObjectJoint]))+' '+SerializeTSRChain(0,0,1,1,TSRString1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString2,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])
    else:
        # For linear trajectories
        TSRChainStringTurning = SerializeTSRChain(0,0,1,1,TSRString0,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString2,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])
    
    return TSRChainStringTurning

def plan(myEnv, myRobot, myObject, startikStr, goalikStr, footlinknames, TSRChainString, trajName, returnTraj=False):

    myRobotProblem = RaveCreateModule(myEnv,'CBiRRT')
    myObjectProblem = RaveCreateModule(myEnv,'CBiRRT')
    
    try:
        myEnv.AddModule(myRobotProblem,myRobot.GetName()) # this string should match to kinematic body
        myEnv.AddModule(myObjectProblem,myObject.GetName())
    except openrave_exception, e:
        print e

    TSRChainMimicDOF = 1
    
    fastsmoothingitrs = 150

    # Get a trajectory from goalik to grasp configuration
    jointgoalsStr = deepcopy(goalikStr)
    for i in range(TSRChainMimicDOF):
        jointgoalsStr += ' 0'

    jointgoalsNum = str2num(jointgoalsStr)

    # print "STARTIK"
    # myRobot.SetActiveDOFValues(str2num(startikStr))
    # time.sleep(0.1)
    # if(debug):
    #     sys.stdin.readline()
    
    # print "GOALIK"
    # myRobot.SetActiveDOFValues(str2num(goalikStr))
    # time.sleep(0.1)
    # #myObject.SetDOFValues([pi/4],[0])
    # if(debug):
    #     sys.stdin.readline()
    
    # print "BACK TO STARTIK"
    # myRobot.SetActiveDOFValues(str2num(startikStr))
    # time.sleep(0.1)
    # if(debug):
    #     print "PRESS ENTER TO PLAN"
    #     sys.stdin.readline()

    try:
        answer = myRobotProblem.SendCommand('RunCBiRRT timelimit 5 supportlinks 2 '+footlinknames+' smoothingitrs '+str(fastsmoothingitrs)+' jointgoals '+str(len(jointgoalsNum))+' '+Serialize1DMatrix(matrix(jointgoalsNum))+' '+TSRChainString)
        print "RunCBiRRT answer: ",str(answer)
    except openrave_exception, e:
        print "Cannot send command RunCBiRRT: "
        print e

    # cleanup the cbirrt problem object
    myEnv.Remove(myRobotProblem)
    myEnv.Remove(myObjectProblem)
    del myRobotProblem
    del myObjectProblem
    time.sleep(0.1)

    
    if(str(answer) == '1'):
        if(returnTraj):
            try:
                os.rename("cmovetraj.txt", trajName)
                traj = RaveCreateTrajectory(myEnv,'').deserialize(open(trajName,'r').read()) 
                return traj
            except OSError, e:
                # No file cmovetraj: [Errno 2] No such file or directory
                print e
                return None
        else:
            return True
    else:
        return None



def check_support(T0_COM,myRobot):

    # TODO: Check support with cbirrt

    T0_COMXY = deepcopy(T0_COM)
    T0_COMXY[2,3] = 0.0

    # How strict do we want to be in checking balance?
    # This radius is the distance threshold between the 
    # projection of the COM on the support polygon 
    # and the center of mass of the feet of the robot
    radius = 0.1
    
    # center of feet in world coordinates [XYZ only]
    T0_LF = myRobot.GetManipulators()[2].GetEndEffectorTransform()
    T0_RF = myRobot.GetManipulators()[3].GetEndEffectorTransform()
    COF = []
    distSumSq = 0
    for axis in range(3):
        COF.append((T0_LF[axis,3]+T0_RF[axis,3])*0.5)
        # euclidean distance between COF and COM
        distSumSq += pow((COF[axis]-T0_COMXY[axis,3]),2)

    dist = pow(distSumSq,0.5)
    # print "COF"
    # print COF
    # print "COM (PROJECTED ON THE GROUND PLANE)"
    # print T0_COMXY
    # print "dist:"
    # print dist
    if (dist <= radius):
        # print "in balance"
        return True
    else:
        # print "not in balance"
        return False
    

def get_robot_com(myRobot):
    
    # print "let's calculate the COM here..."
    #
    # This is how the center of mass is calculated
    #
    # https://en.wikipedia.org/wiki/Center_of_mass
    # R*M = SUM(r_i*m_i)
    #
    # R = SUM(r_i*m_i)/M
    M = 0
    rm = 0
    for l in myRobot.GetLinks():
        
        # mass of this link
        m = l.GetMass() 

        # keep the total mass of the robot
        M += m

        # l.GetTransform() does not return the center of mass of the link
        r = dot(l.GetTransform(),array(MakeTransform(rodrigues([0,0,0]),transpose(matrix(l.GetCOMOffset())))))
        
        # multiplication by element
        rm += multiply(r,m) 

    # print "total mass:"
    # print M

    # division by element
    R = divide(rm,M)

    # print "T0_COM for "+myRobot.GetName()
    # print R.round(4)

    # Draw the center of mass if you want
    # h.append(misc.DrawAxes(myEnv,array(R.round(4)),0.1))
    
    return R

# input: T0_FACING, where do we want the robot's feet to face?
def put_feet_on_the_ground(myRobot, T0_FACING, myEnv, footlinknames=' Body_RAR Body_LAR '):

    # TODO: We don't need to do this at every call
    # Let's find a place to make this call only once
    # in the beginning
    lowerLimits, upperLimits = myRobot.GetDOFLimits()

    # Sometimes CBiRRT plugin segfaults if we make
    # a lot of calls. Let's get a new
    # instace and clean it up at each function call
    myProblem = RaveCreateModule(myEnv,'CBiRRT')

    try:
        myEnv.AddModule(myProblem, myRobot.GetName()) # this string should match to <Robot name="" > in robot.xml
    except openrave_exception, e:
        print e     

    # stupid python to c++ datatype hack around
    # This will prevent the error of "can't find an ik solution
    # because joint X is out of range -0.0"
    howManyJoints = len(myRobot.GetJoints())
    for i in range(howManyJoints):
        j = round(myRobot.GetDOFValues([i]),2)
        if( (nonzero(lowerLimits[i])[0].size == 0) and (nonzero(j)[0].size == 0) ):
            # print "setting joint ",str(i)," to positive zero"
            myRobot.SetDOFValues([0.000001],[i])
        elif( (nonzero(upperLimits[i])[0].size == 0) and (nonzero(j)[0].size == 0) ):
            # print "setting joint ",str(i)," to negative zero"
            myRobot.SetDOFValues([-0.000001],[i])


    # Calculate the center of mass
    T0_COM = get_robot_com(myRobot)
    T0_LF = myRobot.GetManipulators()[2].GetEndEffectorTransform()
    T0_RF = myRobot.GetManipulators()[3].GetEndEffectorTransform()
    T0_TORSO = myRobot.GetLinks()[6].GetTransform()
    T0_TORSOXY = array(MakeTransform(T0_FACING[0:3,0:3],transpose(matrix([T0_TORSO[0,3],T0_TORSO[1,3],0]))))

    for x in range(21):
        # x is on the X-Axis of roboground and facing wherever we want it to
        TCOM_LF = dot(linalg.inv(T0_COM),T0_LF)
        TCOM_RF = dot(linalg.inv(T0_COM),T0_RF)

        #Let's shift roboground to where the center of mass is on the ground
        # Note TCOMXY is where the COM is but it's rotation is the same with the world
        # TODO: Instead of calculating T0_COMXY's rotation from T0_FACING,
        # get the torso frame and rotate it back to being flat
        # around x and y but don't touch the orientation around z.
        T0_COMXY = array(MakeTransform(T0_FACING[0:3,0:3],transpose(matrix([T0_COM[0,3],T0_COM[1,3],0]))))
        #TCOMXY_LFTARGET = array(MakeTransform(rodrigues([0,0,0]),transpose(matrix([((-0.1)+x*0.01),TCOM_LF[1,3],0.0]))))
        TCOMXY_LFTARGET = array(MakeTransform(rodrigues([0,0,0]),transpose(matrix([((-0.1)+x*0.01),0.09,0.0]))))
        #TCOMXY_RFTARGET = array(MakeTransform(rodrigues([0,0,0]),transpose(matrix([((-0.1)+x*0.01),TCOM_RF[1,3],0.0]))))
        TCOMXY_RFTARGET = array(MakeTransform(rodrigues([0,0,0]),transpose(matrix([((-0.1)+x*0.01),-0.09,0.0]))))
        
        # Now we know where to face and where the feet should be
        T0_LFTARGET = dot(T0_COMXY,TCOMXY_LFTARGET)
        # T0_LFTARGET = dot(T0_TORSOXY,array(MakeTransform(rodrigues([0,0,0]),transpose(matrix([((-0.1)+x*0.01),0.1,0.0])))))
        T0_RFTARGET = dot(T0_COMXY,TCOMXY_RFTARGET)
        # T0_RFTARGET = dot(T0_TORSOXY,array(MakeTransform(rodrigues([0,0,0]),transpose(matrix([((-0.1)+x*0.01),-0.1,0.0])))))
    
    # old code - delete if the code above works
    # 
    # for x in range(21):
    #     # Center of Gravity Target
    #     # T0_TORSO = myRobot.GetManipulators()[5].GetEndEffectorTransform()

    #     # Where to put the left foot?
    #     # Rotation matrix is eye(3)
    #     # T0_lf = array(MakeTransform(T0_FACING[0:3,0:3],transpose(matrix([T0_LF[0,3]-(0.1+x*0.01),T0_COM[1,3],0]))))
    #     T0_lf = array(MakeTransform(T0_FACING[0:3,0:3],transpose(matrix([T0_LF[0,3],T0_COM[1,3]-(0.1+x*0.01),0]))))

    #     # Same for the right foot
    #     # T0_rf = array(MakeTransform(T0_FACING[0:3,0:3],transpose(matrix([T0_RF[0,3]-(0.1+x*0.01),T0_COM[1,3],0]))))
    #     T0_rf = array(MakeTransform(T0_FACING[0:3,0:3],transpose(matrix([T0_RF[0,3],T0_COM[1,3]-(0.1+x*0.01),0]))))

        # print "this is where the feet should go."
        myHandle1 = misc.DrawAxes(myEnv,array(T0_LFTARGET),0.1)
        myHandle2 = misc.DrawAxes(myEnv,array(T0_RFTARGET),0.1)
        # sys.stdin.readline()

        # cogtarg = [-0.05+T0_TORSO[0,3], 0.085+T0_TORSO[1,3], 0]
        # cogtargStr = str(cogtarg).strip("[]").replace(', ',' ')

        # goalik = myProblem.SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames+' movecog '+cogtargStr+' nummanips 3 maniptm 2 '+trans_to_str(T0_LF)+' maniptm 3 '+trans_to_str(T0_RF)+' maniptm 5 '+trans_to_str(T0_TORSO))

        # goalik = myProblem.SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames+' nummanips 3 maniptm 2 '+trans_to_str(T0_LF)+' maniptm 3 '+trans_to_str(T0_RF)+' maniptm 5 '+trans_to_str(T0_TORSO))

        
        goalik = myProblem.SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames+' nummanips 2 maniptm 2 '+trans_to_str(T0_LFTARGET)+' maniptm 3 '+trans_to_str(T0_RFTARGET))

        # print "goalik"
        # print str2num(goalik)
        # print len(str2num(goalik))
        
        if (goalik != '' and 
            (not myEnv.CheckCollision(myRobot)) and 
            (not myRobot.CheckSelfCollision()) 
            ):

            # cleanup the cbirrt problem object
            myEnv.Remove(myProblem)
            del myProblem

            return goalik
        else:
            goalik = ''

    # cleanup the cbirrt problem object
    myEnv.Remove(myProblem)
    del myProblem

    return goalik

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

# robots: List of all robots in the environment
# numRobots: (int) Starting from the 0th robot, how many of the robots do we care about?
# numManips: (list) For each robot, starting from the 0th manipulator, how many of the manipulators do we care about?
# h: (list) handles for objects that we draw on the screen
def play(T0_starts, T0_FACING, relBaseConstraint,candidates,numRobots,numManips,c,myRmaps,robots,h,myEnv,howLong,doGeneralIk,footlinknames=''):
    # constraints to check
    relBaseConstOK = True
    collisionConstOK = True
    noConfigJump = True
    
    # Get the length of the candidate path
    # First index stands for the robot index, and the second index stands for the candidate path index. We're calling find_random_candidates() function with an argument of 1. Thus there will be only one candidate returned. Let's find it's length.
    pathLength = min(len(candidates[0][c]), len(candidates[1][c]))

    # print "numRobots: ",str(numRobots)
    # print "numManips: ",str(numManips)
    [goAhead, activeDOFStartConfig] = start(T0_starts,T0_FACING, candidates,numRobots,numManips,c,myRmaps,robots,h,myEnv,doGeneralIk,footlinknames)

    if(goAhead):
        # Get their relative Transformation matrix
        T0_base = []
        for myRobotIndex in range(numRobots):
            for myManipulatorIndex in range(numManips[myRobotIndex]):
                T0_base.append(robots[myRobotIndex].GetManipulators()[myManipulatorIndex].GetBase().GetTransform())
        Trobot0_robot1 = dot(linalg.inv(T0_base[0]),T0_base[1])

        # i) Does the candidate satisfy the robot base transform constraint?
        if(type(relBaseConstraint) == type([])):
            # if the input argument type is a list then we have bounds
            # Check translation constraints
            if((abs(Trobot0_robot1[0:3,3].transpose()) > relBaseConstraint[0:3]).any()):
                relBaseConstOK = False
                return [False, '']

            # Check rotation constraints
            if(not allclose(Trobot0_robot1[0:3,0:3],rodrigues(relBaseConstraint[3:6]))):
                relBaseConstOK = False
                return [False, '']

        elif(type(relBaseConstraint) == type(array(()) or relBaseConstraint) == type(matrix(()))):
            # if input argument type is a numpy array then we have an exact transformation
            #print "relBaseConstraint?"
            #print allclose(Trobot0_robot1,relBaseConstraint)
            if(not allclose(Trobot0_robot1,relBaseConstraint)):
                relBaseConstOK = False
                return [False, '']

        # If the solution meets the base constraint:
        if(relBaseConstOK):
            # print "Base Constraints OK."
            pathConfigs = [[],[]] # A 2D List
            # ii) Check if the solution collision-free throughout the path?
            # For each path element, go step by step and check
            prevConfig = [[],[]]
            currentConfig = [[],[]]
            # print "----"
            for pElementIndex in range(pathLength):
                # Move the manipulator to this path element
                for myRobotIndex in range(numRobots):
                    for myManipulatorIndex in range(numManips[myRobotIndex]):
                        currentSphereIndex = candidates[myManipulatorIndex][c][pElementIndex].sIdx
                        currentTransformIndex = candidates[myManipulatorIndex][c][pElementIndex].tIdx
                        myRmaps[myManipulatorIndex].go_to(currentSphereIndex,currentTransformIndex)
                        pathConfigs[myManipulatorIndex].append(robots[myRobotIndex].GetDOFValues(robots[myRobotIndex].GetManipulators()[myManipulatorIndex].GetArmIndices()))

                        # DEBUG SECTION FOR SENSING CONFIGURATION JUMP
                        currentConfig[myManipulatorIndex] = robots[myRobotIndex].GetDOFValues(robots[myRobotIndex].GetManipulators()[myManipulatorIndex].GetArmIndices())

                        # print "For robot ",str(myRobotIndex)," ||qA-qB||:"
                        # print "path element ",str(pElementIndex)
                        if(prevConfig[myManipulatorIndex] != []):
                            # print "previous config: "
                            # print prevConfig[myRobotIndex]
                            # print "current config: "
                            # print currentConfig[myRobotIndex]
                            qdiff = absolute(subtract(currentConfig[myManipulatorIndex],prevConfig[myManipulatorIndex]))
                            configDistSq = 0
                            # for each joint do:
                            for j in range(len(qdiff)):
                                configDistSq += pow(qdiff[j],2)
                            # find ||qA-qB||
                            euclideanConfigDistance = pow(configDistSq,0.5)
                            # print "euclidean configuration distance: "
                            # print euclideanConfigDistance

                            # if(euclideanConfigDistance > configurationJumpThreshold):
                            #     noConfigJump = False
                            #     return False
                        else:
                            #print "skipping path element:"
                            pass
                        # END OF DEBUG SECTION FOR SENSING CONFIGURATION JUMP

                        prevConfig[myManipulatorIndex] = deepcopy(currentConfig[myManipulatorIndex])
                        # Check collision with self and with the environment
                        if(myEnv.CheckCollision(robots[myRobotIndex]) or robots[myRobotIndex].CheckSelfCollision()):
                            collisionConstOK = False
                            return [False, '']
                    
                    # After setting manipulator configurations for this
                    # robot, check if the center of mass is close enough
                    # to the center of robot's feet
                    #
                    # TODO: This part is very messy and ugly.
                    # I need to clean it up and make nice function calls.
                    if(doGeneralIk):
                        robots[myRobotIndex].SetActiveDOFValues(zeros(robots[myRobotIndex].GetActiveDOF()).tolist())
                        # Bend the knees to avoid singularity issues
                        robots[myRobotIndex].SetDOFValues([-0.3,0.6,-0.3],[32,33,34])
                        robots[myRobotIndex].SetDOFValues([-0.3,0.6,-0.3],[26,27,28])
                        # currentIk = robots[myRobotIndex].GetActiveDOFValues()
                        if(footlinknames==''):
                            myIK = put_feet_on_the_ground(robots[myRobotIndex], T0_FACING, myEnv)
                        else:
                            myIK = put_feet_on_the_ground(robots[myRobotIndex], T0_FACING, myEnv, footlinknames)
                            
                        if(myIK != ''):
                            robots[myRobotIndex].SetActiveDOFValues(str2num(myIK))
                            # print "checking support in play..."
                            if(not check_support(array(get_robot_com(robots[myRobotIndex])),robots[myRobotIndex])):
                                #robots[myRobotIndex].SetActiveDOFValues(currentIk)
                                robots[myRobotIndex].SetActiveDOFValues(zeros(robots[myRobotIndex].GetActiveDOF()).tolist())
                                # Bend the knees to avoid singularity issues
                                robots[myRobotIndex].SetDOFValues([-0.3,0.6,-0.3],[32,33,34])
                                robots[myRobotIndex].SetDOFValues([-0.3,0.6,-0.3],[26,27,28])
                                return [False, '']
                            else:
                                # print "in balance - path element: ",str(pElementIndex)
                                #sys.stdin.readline()
                                # robots[myRobotIndex].SetActiveDOFValues(currentIk)
                                robots[myRobotIndex].SetActiveDOFValues(zeros(robots[myRobotIndex].GetActiveDOF()).tolist())
                                # Bend the knees to avoid singularity issues
                                robots[myRobotIndex].SetDOFValues([-0.3,0.6,-0.3],[32,33,34])
                                robots[myRobotIndex].SetDOFValues([-0.3,0.6,-0.3],[26,27,28])
                        else:
                            # robots[myRobotIndex].SetActiveDOFValues(currentIk)
                            robots[myRobotIndex].SetActiveDOFValues(zeros(robots[myRobotIndex].GetActiveDOF()).tolist())
                            # Bend the knees to avoid singularity issues
                            robots[myRobotIndex].SetDOFValues([-0.3,0.6,-0.3],[32,33,34])
                            robots[myRobotIndex].SetDOFValues([-0.3,0.6,-0.3],[26,27,28])
                            return [False, '']

                # If you didn't break yet, wait before the next path element for visualization
                time.sleep(howLong)

            for manipIdx, manipConfs in enumerate(pathConfigs):
                # print "for manip ",str(manipIdx)
                for confIdx, conf in enumerate(manipConfs):
                    if(confIdx > 0):
                        qdiff = absolute(subtract(conf,manipConfs[confIdx-1]))
                        
                        confDistSq = 0
                        for j in range(len(qdiff)):
                            configDistSq += pow(qdiff[j],2)
                        eConfDist = pow(configDistSq,0.5)
                        # print "euclidean configuration distance:"
                        # print eConfDist
                        if(eConfDist > configurationJumpThreshold):
                            noConfigJump = False
                            return [False, '']
            
            # If you made it here, 
            # it means no configuration jump, no collision, and
            # the center of mass was close enough to the center
            # of robot's feet for all reachability spheres.
            return [True, activeDOFStartConfig]

            # #
            # # one last time check if the COM is in the support polygon
            # print "checking balance constraint..."
            # # print "Press enter to see the result..."
            # # sys.stdin.readline()
            # myCOM = array(get_robot_com(robots[myRobotIndex]))
            # myCOM[2,3] = 0.0
            # COMHandle = misc.DrawAxes(myEnv,myCOM,0.3)
            # print "if you made it here it means no config jump"
            # print "no collision and the com mas in support poly."
            # sys.stdin.readline()
            # if(check_support(myCOM,robots[myRobotIndex])):
            #     return [True, activeDOFConfig]
            # else:
            #     print "COM is out of the support polygon"
            #     return [False, '']
    else:
        # start() failed
        return [False, '']
    

def start(T0_starts, T0_FACING, candidates,numRobots,numManips,c,myRmaps,robots,h,myEnv,doGeneralIk,footlinknames=''):
    # Define this variable so python doesn't complain
    myIK = ''
    
    # constraints to check
    masterBaseConstOK = True
    # Move each robot / manipulators
    for myRobotIndex in range(numRobots):
        for myManipulatorIndex in range(numManips[myRobotIndex]):
            # Find where to move the base
            startSphereIndex = candidates[myManipulatorIndex][c][0].sIdx
            startTransformIndex = candidates[myManipulatorIndex][c][0].tIdx
            # Set the manipulator to its configuration (we will check for collision)
            # print "for manipulator: ",str(myManipulatorIndex)
            # print "go_to: start_sphere_index: "
            # print startSphereIndex
            # print "go_to: start_transform_index: "
            # print startTransformIndex
            myRmaps[myManipulatorIndex].go_to(startSphereIndex,startTransformIndex)
        
            Tbase_start = myRmaps[myManipulatorIndex].map[startSphereIndex].T[startTransformIndex]
            
            # for myT in myRmaps[myManipulatorIndex].map[startSphereIndex].T:
            #    print myT
            
            T0_newManipPose = dot(T0_starts[myManipulatorIndex],linalg.inv(Tbase_start))

            # Finally move the robot base 
            robots[myRobotIndex].SetTransform(T0_newManipPose)

            # print "manip index: ",str(myManipulatorIndex)
            # sys.stdin.readline()
            
            if(myManipulatorIndex == 0):
                h.append(misc.DrawAxes(myEnv,T0_newManipPose,0.4))

            
            # Check master base constraint
            # if(myRobotIndex == 0):
            #     if(type(masterBaseConstraint) == type([])):
            #         if((abs(T0_newManipPose[0:3,3].transpose()) > masterBaseConstraint[0:3]).any()):
            #             masterBaseConstOK = False
            #             break    
            #         #if(not allclose(T0_newManipPose[0:3,0:3],rodrigues(masterBaseConstraint[3:6]))):
            #         #    masterBaseConstOK = False
            #         # HACK-AROUND
            #         # For now just check if the robot base is on XY plane
            #         if((not allclose(T0_newManipPose[0:3,2].transpose(),[0,0,1])) or (not allclose(T0_newManipPose[2,0:3],[0,0,1]))):
            #             masterBaseConstOK = False
            #             break
            #     elif(type(masterBaseConstraint) == type(array(()))):
            #         pass
        
        # Now check if the feet are on the ground
        #
        # TODO: We should do this in a robot-agnostic way.
        # masterBaseConstraint works for mobile manipulators
        # or for industrial arms, but for humanoids the base
        # and the feet can be different, so for balance check
        # we "may" need an extra step. How do we set this flag?
        # Maybe we should have a "isHumanoid" bool? And try to
        # put the feet on the ground if(myReachabilityMap.isHumanoid)
        #
        # currentIk = robots[myRobotIndex].GetActiveDOFValues()
        if(doGeneralIk):
            robots[myRobotIndex].SetActiveDOFValues(zeros(robots[myRobotIndex].GetActiveDOF()).tolist())
            # Bend the knees to avoid singularity issues
            robots[myRobotIndex].SetDOFValues([-0.3,0.6,-0.3],[32,33,34])
            robots[myRobotIndex].SetDOFValues([-0.3,0.6,-0.3],[26,27,28])
            # print "trying to put the feet on the ground..."
            if(footlinknames==''):
                myIK = put_feet_on_the_ground(robots[myRobotIndex], T0_FACING, myEnv)
            else:
                myIK = put_feet_on_the_ground(robots[myRobotIndex], T0_FACING, myEnv, footlinknames)

            if(myIK != ''):
                # robots[myRobotIndex].SetDOFValues(str2num(myIK), range(len(robots[myRobotIndex].GetJoints())))
                robots[myRobotIndex].SetActiveDOFValues(str2num(myIK))
            else:
                masterBaseConstOK = False
                
            # robots[myRobotIndex].SetDOFValues(currentIk, range(len(robots[myRobotIndex].GetJoints())))
            # robots[myRobotIndex].SetActiveDOFValues(currentIk)
            robots[myRobotIndex].SetActiveDOFValues(zeros(robots[myRobotIndex].GetActiveDOF()).tolist())
            # Bend the knees to avoid singularity issues
            robots[myRobotIndex].SetDOFValues([-0.3,0.6,-0.3],[32,33,34])
            robots[myRobotIndex].SetDOFValues([-0.3,0.6,-0.3],[26,27,28])
            
                
            # sys.stdin.readline()

        # Check collision with self and with the environment
        if(myEnv.CheckCollision(robots[myRobotIndex]) or robots[myRobotIndex].CheckSelfCollision()):
            collisionConstOK = False
        else:
            collisionConstOK = True

    
    return [(masterBaseConstOK and collisionConstOK), myIK]
