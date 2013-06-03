# Bener Suay, May 2013
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
from Reachability import *

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

# This changes the height and the pitch angle of the wheel
version = 1

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

    print "total mass:"
    print M

    # division by element
    R = divide(rm,M)

    print "T0_COM for "+myRobot.GetName()
    print R.round(4)

    # Draw the center of mass if you want
    # h.append(misc.DrawAxes(env,array(R.round(4)),0.1))
    
    return R

# input: T0_FACING, where do we want the robot's feet to face?
def put_feet_on_the_ground(myRobot,T0_FACING):
    

    # stupid python to c++ datatype hack around
    # This will prevent the error of "can't find an ik solution
    # because joint X is out of range -0.0"
    howManyJoints = len(myRobot.GetJoints())
    for i in range(howManyJoints):
        j = round(myRobot.GetDOFValues([i]),2)
        if( (nonzero(lowerLimits[i])[0].size == 0) and (nonzero(j)[0].size == 0) ):
            print "setting joint ",str(i)," to positive zero"
            myRobot.SetDOFValues([0.000001],[i])
        elif( (nonzero(upperLimits[i])[0].size == 0) and (nonzero(j)[0].size == 0) ):
            print "setting joint ",str(i)," to negative zero"
            myRobot.SetDOFValues([-0.000001],[i])

    # Calculate the center of mass
    T0_COM = get_robot_com(myRobot)
    T0_LF = manips[2].GetEndEffectorTransform()
    T0_RF = manips[3].GetEndEffectorTransform()

    for y in range(21):
        # Center of Gravity Target
        # T0_TORSO = manips[5].GetEndEffectorTransform()

        # Where to put the left foot?
        # Rotation matrix is eye(3)
        T0_lf = array(MakeTransform(T0_FACING[0:3,0:3],transpose(matrix([T0_LF[0,3],T0_COM[1,3]-(0.1+y*0.01),0]))))

        # Same for the right foot
        T0_rf = array(MakeTransform(T0_FACING[0:3,0:3],transpose(matrix([T0_RF[0,3],T0_COM[1,3]-(0.1+y*0.01),0]))))

        print "this is where the feet should go."
        myHandle1 = misc.DrawAxes(env,array(T0_lf),0.1)
        myHandle2 = misc.DrawAxes(env,array(T0_rf),0.1)
        # sys.stdin.readline()

        # cogtarg = [-0.05+T0_TORSO[0,3], 0.085+T0_TORSO[1,3], 0]
        # cogtargStr = str(cogtarg).strip("[]").replace(', ',' ')

        # goalik = probs[0].SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames+' movecog '+cogtargStr+' nummanips 3 maniptm 2 '+trans_to_str(T0_LF)+' maniptm 3 '+trans_to_str(T0_RF)+' maniptm 5 '+trans_to_str(T0_TORSO))

        # goalik = probs[0].SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames+' nummanips 3 maniptm 2 '+trans_to_str(T0_LF)+' maniptm 3 '+trans_to_str(T0_RF)+' maniptm 5 '+trans_to_str(T0_TORSO))

        goalik = probs[0].SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames+' nummanips 2 maniptm 2 '+trans_to_str(T0_lf)+' maniptm 3 '+trans_to_str(T0_rf))
        
        if goalik != '':
            return goalik

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
def play(relBaseConstraint,candidates,numRobots,numManips,c,myRmaps,robots,h,env,howLong):
    # constraints to check
    relBaseConstOK = True
    collisionConstOK = True
    noConfigJump = True

    # print "numRobots: ",str(numRobots)
    # print "numManips: ",str(numManips)

    if(start(candidates,numRobots,numManips,c,myRmaps,robots,h,env)):
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
                return False

            # Check rotation constraints
            if(not allclose(Trobot0_robot1[0:3,0:3],rodrigues(relBaseConstraint[3:6]))):
                relBaseConstOK = False
                return False

        elif(type(relBaseConstraint) == type(array(()) or relBaseConstraint) == type(matrix(()))):
            # if input argument type is a numpy array then we have an exact transformation
            #print "relBaseConstraint?"
            #print allclose(Trobot0_robot1,relBaseConstraint)
            if(not allclose(Trobot0_robot1,relBaseConstraint)):
                relBaseConstOK = False
                return False

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
                        if(env.CheckCollision(robots[myRobotIndex]) or robots[myRobotIndex].CheckSelfCollision()):
                            collisionConstOK = False
                            return False

                # If you didn't break yet, wait before the next path element for visualization
                time.sleep(howLong)

            for manipIdx, manipConfs in enumerate(pathConfigs):
                print "for manip ",str(manipIdx)
                for confIdx, conf in enumerate(manipConfs):
                    if(confIdx > 0):
                        qdiff = absolute(subtract(conf,manipConfs[confIdx-1]))
                        
                        confDistSq = 0
                        for j in range(len(qdiff)):
                            configDistSq += pow(qdiff[j],2)
                        eConfDist = pow(configDistSq,0.5)
                        print "euclidean configuration distance:"
                        print eConfDist
                        if(eConfDist > configurationJumpThreshold):
                            noConfigJump = False
                            return False

            # If you made it here, 
            # it means no configuration jump, and no collision
            return True

    else:
        # start() failed
        return False
    

def start(candidates,numRobots,numManips,c,myRmaps,robots,h,env):
    # constraints to check
    masterBaseConstOK = True
    # Move each robot / manipulators
    for myRobotIndex in range(numRobots):
        for myManipulatorIndex in range(numManips[myRobotIndex]):
            # Find where to move the base
            startSphereIndex = candidates[myManipulatorIndex][c][0].sIdx
            startTransformIndex = candidates[myManipulatorIndex][c][0].tIdx
            # Set the manipulator to its configuration (we will check for collision)
            print "for manipulator: ",str(myManipulatorIndex)
            print "go_to: start_sphere_index: "
            print startSphereIndex
            print "go_to: start_transform_index: "
            print startTransformIndex
            myRmaps[myManipulatorIndex].go_to(startSphereIndex,startTransformIndex)
        
            Tbase_start = myRmaps[myManipulatorIndex].map[startSphereIndex].T[startTransformIndex]
            
            for myT in myRmaps[myManipulatorIndex].map[startSphereIndex].T:
                print myT
            
            T0_newManipPose = dot(T0_starts[myManipulatorIndex],linalg.inv(Tbase_start))
            # Finally move the robot base 
            robots[myRobotIndex].SetTransform(T0_newManipPose)
            
            if(myManipulatorIndex == 0):
                h.append(misc.DrawAxes(env,T0_newManipPose,0.4))

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
                
        # Check collision with self and with the environment
        if(env.CheckCollision(robots[myRobotIndex]) or robots[myRobotIndex].CheckSelfCollision()):
            collisionConstOK = False
            return False
    return True

env = Environment()
env.SetViewer('qtcoin')

T0_p = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))

# 1. Create a trajectory for the tool center point to follow
# Left Hand
traj0 = []
Tstart0 = array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0]))))
traj0.append(Tstart0)

# traj0.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.05])))))

# traj0.append(array(MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([0.0,0.0,0.1])))))

# Tgoal0 = array(MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([0.0,0.0,0.15]))))
# Tgoal0 = array(MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([0.0,0.0,0.05]))))

Tgoal0 = array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.05]))))

traj0.append(Tgoal0)

# Right Hand
traj1 = []

Tstart1 = array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0]))))

traj1.append(Tstart1)

# traj1.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,-0.05])))))

# traj1.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,-0.1])))))

# Tgoal1 = array(MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([0.0,0.0,-0.15]))))

# Tgoal1 = array(MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([0.0,0.0,-0.05]))))

Tgoal1 = array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,-0.05]))))

traj1.append(Tgoal1)

h = []
h.append(misc.DrawAxes(env,array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))),1.0))

myPatterns = []
# 2. Create a search pattern from the trajectory for the first manipulator
myPattern = SearchPattern(traj0)
myPattern.T0_p = T0_p
myPattern.setColor(array((0,0,1,0.5))) 
myPattern.show(env)
# sys.stdin.readline()
myPattern.hide("all")
myPatterns.append(myPattern)

# 3. Create a search pattern from the trajectory for the second manipulator
myPattern = SearchPattern(traj1)
myPattern.T0_p = T0_p
myPattern.setColor(array((1,0,0,0.5))) 
myPattern.show(env)
# sys.stdin.readline()
myPattern.hide("all")
myPatterns.append(myPattern)

for p in myPatterns:
    p.hide("spheres")
    #sys.stdin.readline()

for p in myPatterns:
    p.hide("all")
    # sys.stdin.readline()

# 3. Add drchubo
robots = []
robots.append(env.ReadRobotURI('../../../openHubo/huboplus/huboplus.robot.xml'))
env.Add(robots[0])
lowerLimits, upperLimits = robots[0].GetDOFLimits()

# print lowerLimits
# print upperLimits
# sys.stdin.readline()
           

# Let's keep the rotation of the robot around it's Z-Axis in a variable...
rotz=[]
rotz.append(pi/3);

# Define the transform and apply the transform
shift_robot0 = MakeTransform(matrix(rodrigues([0,0,rotz[0]])),transpose(matrix([-2.5,0.0,0.95])))
robots[0].SetTransform(array(shift_robot0))

# Add the wheel
wheel = env.ReadRobotURI('../../../../drc_common/models/driving_wheel.robot.xml')

if(version == 0):
    wheelHeight = 0.2
    wheelPitch = 0
elif(version == 1):
    gR = 0.6 # ground color
    gG = 0.6 # ground color
    gB = 0.6 # ground color

    ge = 5.0 # ground extension
    wheelHeight = 0.8 #random.random()
    wheelPitch = 1.7 # pi*random.random()
    h.append(env.drawtrimesh(points=array(((0,0,0),(ge,0,0),(0,ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))
    h.append(env.drawtrimesh(points=array(((ge,0,0),(0,ge,0),(ge,ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))

    h.append(env.drawtrimesh(points=array(((0,0,0),(-ge,0,0),(0,-ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))
    h.append(env.drawtrimesh(points=array(((-ge,0,0),(0,-ge,0),(-ge,-ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))


    h.append(env.drawtrimesh(points=array(((0,0,0),(-ge,0,0),(0,ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))
    h.append(env.drawtrimesh(points=array(((-ge,0,0),(0,ge,0),(-ge,ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))

    
    h.append(env.drawtrimesh(points=array(((0,0,0),(ge,0,0),(0,-ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))
    h.append(env.drawtrimesh(points=array(((ge,0,0),(0,-ge,0),(ge,-ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))

    Tee = []
    manips = robots[0].GetManipulators()
    for i in range(len(manips)):
        # Returns End Effector Transform in World Coordinates
        Tlink = manips[i].GetEndEffectorTransform()
        Tee.append(Tlink)

    footlinknames = ' Body_RAR Body_LAR '

    probs_cbirrt = RaveCreateModule(env,'CBiRRT')

    try:
        env.AddModule(probs_cbirrt,robots[0].GetName()) # this string should match to <Robot name="" > in robot.xml
    except openrave_exception, e:
        print e

    print "Getting Loaded Problems"
    probs = env.GetLoadedProblems()

    
wheelOffset = matrix([0,0,wheelHeight])
wheelRotation = matrix(rodrigues([wheelPitch,0,0]))
env.Add(wheel)

T0_wheelBase = MakeTransform(wheelRotation,transpose(wheelOffset))
wheel.SetTransform(array(T0_wheelBase))

# h.append(misc.DrawAxes(env,T0_wheelBase,0.3))

# 4. Load the reachability map for drchubo left arm
# Robot 1
myRmaps = []
rm = ReachabilityMap("./rlhuboplus_leftArm_ik_solver",robots[0],robots[0].GetManipulators()[0])
print "Loading reachability map for left arm..."
rm.load("rlhuboplus_left")
# print "map size before crop: ",str(len(rm.map))
# rm.crop([-1.0,1.0,-1.0,0.2,0.0,1.0])
# print "map size after crop: ",str(len(rm.map))
# rm.show(env) # slows down the process a lot
# Append the reachability map, and keep it in a list
# sys.stdin.readline()
# rm.hide()
myRmaps.append(rm)

print "Left arm Reachability Map loaded.."
# sys.stdin.readline()

# Do the same for Robot 2
rm2 = ReachabilityMap("./rlhuboplus_rightArm_ik_solver",robots[0],robots[0].GetManipulators()[1])
print "Loading reachability map for right arm..."
rm2.load("rlhuboplus_right")
rm2.print_rm3D()
sys.stdin.readline()
rm2.print_all_transforms()

print "Reachability map loaded for right arm."

# rm2.crop([-1.0,1.0,0.0,1.0,0.0,1.0])
# rm2.show(env)
# sys.stdin.readline()
# rm2.hide()
myRmaps.append(rm2)


# sys.stdin.readline()

# 4. Where do we want the end effectors to start from in world coordinates?
T0_starts = []

# Crank Transform End Effector in World Coordinates
# This is the transformation matrix of the end effector 
# named "dummy" in the xml file.
# Note that dummy is tilted 23 degress around its X-Axis
#
# wheelEndEffector is tilted 23 degrees.
# wheelBase is not tilted.
T0_wheelEndEffector = wheel.GetManipulators()[0].GetEndEffectorTransform()

TwheelEndEffector_start0 = MakeTransform(matrix(rodrigues([-pi/2, 0, 0])),transpose(matrix([0.0, 0.0, 0.0])))

TwheelEndEffector_start0 = dot(TwheelEndEffector_start0, MakeTransform(matrix(rodrigues([0, 0, -pi/2])),transpose(matrix([0.0, 0.0, 0.0]))))

TwheelEndEffector_start0 = dot(TwheelEndEffector_start0,MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, 0.15, 0.0]))))

T0_start0 = dot(T0_wheelEndEffector, TwheelEndEffector_start0)
h.append(misc.DrawAxes(env, T0_start0, 0.4))

TwheelEndEffector_start1 = MakeTransform(matrix(rodrigues([-pi/2, 0, 0])),transpose(matrix([0.0, 0.0, 0.0])))

TwheelEndEffector_start1 = dot(TwheelEndEffector_start1, MakeTransform(matrix(rodrigues([0, 0, -pi/2])),transpose(matrix([0.0, 0.0, 0.0]))))

TwheelEndEffector_start1 = dot(TwheelEndEffector_start1,MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, -0.15, 0.0]))))

T0_start1 = dot(T0_wheelEndEffector, TwheelEndEffector_start1)

h.append(misc.DrawAxes(env, T0_start1, 0.4))

T0_starts.append(array(T0_start0))
T0_starts.append(array(T0_start1))

# Define robot base constraint(s)
# a) Bounds <type 'list'>
#    xyz in meters, rpy in radians
#    [1.0, 1.0, 1.0, 0.0, 0.0, 0.0] would mean, we allow robot bases
#    to be 1 meter apart in each direction but we want them to have the 
#    same rotation

# Relative Base Constraints between the two robots
# First three elements are boundaries (abs) of XYZ (in meters) of robot1 w.r.t robot0 base coords.
# Last three elements are boundaries for RPY of robot1 w.r.t robot0 base coords. RPY will be used to calculate the rotation matrix of robot1 in robot0 coords.
relBaseConstraint = [0.5, 0.5, 0.0, 0.5, 0.0, 0.0, 0.0] # This is type (a)

#relBaseConstraint = MakeTransform() # This is type (b)
relBaseConstraint = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))

# Base Constraints of the first (master) robot in the world coordinate frame
#
# Same as relative base constraints. First 3 elements are XYZ bounds and the last three elements determine the desired rotation matrix of the base of the master robot in world coords.
masterBaseConstraint = [1.0, 1.0, 0.0, 0.0, 0.0, 0.0]

# b) Exact transform <type 'numpy.ndarray'>
# relBaseConstraint = dot(something, some_other_thing)

# Try to find a valid candidate that satisfies
# all the constraints we have (base location, collision, and configuration-jump)


# 5. Search for the pattern in the reachability model and get the results
numRobots = len(robots)

# Keep the number of manipulators of the robots 
# that are going to be involved in search() 
# The length of this list should be the same with 
# the number of robots involved.
numManips = [2]

totalTime = array(())
timeToFindAPath = 0.0
findAPathStarts = time.time()
iters = 0

# Relative transform of initial grasp transforms
Tstart0_start1 = dot(linalg.inv(T0_start0),T0_start1)

# if ||qA-qB|| > threshold then consider this diff as a configuration jump
# This number would change from manipulator to manipulator
configurationJumpThreshold = 100.0

success = False
end = False

print "Ready to search... ",str(datetime.now())
sys.stdin.readline()

while((not success) and (not end)):
    iters += 1
    myStatus = ""  

    candidates = None
    
    # find a random candidate in both maps
    findStarts = time.time()
    #candidates = find_random_candidates(myPatterns,myRmaps,1)
    while(candidates == None):
        candidates = search(myRmaps,[relBaseConstraint],myPatterns,[Tstart0_start1],env)

    findEnds = time.time()
    thisDiff = findEnds-findStarts
    totalTime = append(totalTime,thisDiff)
    print "Found  (a) result(s) in ",str(thisDiff)," secs."
        
    # find how many candidates search() function found.
    # candidates[0] is the list of candidate paths for the 0th robot
    howMany = len(candidates[0])
    valids = []
    for c in range(howMany):
        print "trying ",str(c)," of ",str(howMany)," candidates."
        # Get the length of the candidate path
        # First index stands for the robot index, and the second index stands for the candidate path index. We're calling find_random_candidates() function with an argument of 1. Thus there will be only one candidate returned. Let's find it's length.
        pathLength = len(candidates[0][c]) 

        allGood = play(relBaseConstraint,candidates,numRobots,numManips,c,myRmaps,robots,h,env,0.0)
        h.pop() # delete the robot base axis we added last

        # We went through all our constraints. Is the candidate valid?
        if(allGood):
            valids.append(c)
            success = True
            findAPathEnds = time.time()
            print "Success! All constraints met."
            #print Trobot0_robot1
            print "Total time spent to find candidate path(s): "
            print str(totalTime.cumsum()[-1])," sec."
            print "Total time spent to validate a path: "
            print str(findAPathEnds-findAPathStarts)," sec."
            print "# of iterations: "
            print str(iters)
        else:
            print "Constraint(s) not met ."
            #print "Collision OK?: "
            #print collisionConstOK
            #print "Base Constraint OK?:"
            #print relBaseConstOK

    # Went through all candidates      
    print "Found ",str(len(valids))," valid solutions in ",str(howMany)," candidates."
    
    if (len(valids) > 0) :
        # Have we found at least 1 valid path?
        success = True

        print "Press Enter to iterate through valid solutions."
        sys.stdin.readline()
        nxt = 0
        ask = True
        playAll = False
        while(True):
            print "Playing valid solution #: ",str(nxt)
            play(relBaseConstraint,candidates,numRobots,numManips,valids[nxt],myRmaps,robots,h,env,0.3)
            h.pop() # delete the robot base axis we added last

            currentIk = robots[0].GetDOFValues()
            print currentIk

            # Try to put your feet on the ground
            print "trying to put the feet on the ground..."
            whereToFace = MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0])))
            myIK = put_feet_on_the_ground(robots[0],whereToFace)

            print "myIK"
            print myIK
            
            if(myIK != ''):
                print "Press enter to see the result..."
                sys.stdin.readline()
                robots[0].SetDOFValues(str2num(myIK), range(len(robots[0].GetJoints())))
                myCOM = array(get_robot_com(robots[0]))
                myCOM[2,3] = 0.0
                COMHandle = misc.DrawAxes(env,myCOM,0.3)
                print "Press enter to proceed with the next solution..."
                sys.stdin.readline()

            robots[0].SetDOFValues(currentIk, range(len(robots[0].GetJoints())))

            if(ask):
                print "Next [n]"
                print "Previous [p]"
                print "Replay [r]"
                print "Play All [a]"
                print "Exit [e]"
                answer = sys.stdin.readline()
                answer = answer.strip('\n')
                if( answer == 'n'):
                    nxt += 1
                if(nxt == len(valids)):
                    nxt -= 1
                elif(answer == 'p'):
                    nxt -= 1
                    if(nxt == -1):
                        nxt += 1
                elif(answer == 'r'):
                    pass
                elif(answer == 'a'):
                    playAll = True
                    ask = False
                elif(answer.strip('\n') == 'e'):
                    end = True # End the outer while success for loop
                    break
            else:
                if(playAll and (nxt < len(valids)-1)):
                    nxt += 1
                else:
                    playAll = False
                    ask = True

    # if we found a successful path at this point, we will exit
    
# 6. Add an object in the environment to a random location

# 7. Use the object location and define where Tstart_left and Tstart_right is on the object

# 8. Find the relative transform between Tstart_left and Tstart_right.
#    We assume that this relative transform will remain constant throughout the path.

env.Destroy()
RaveDestroy()

