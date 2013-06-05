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

# robot-placement common functions
from roplace_common import *

# This changes the height and the pitch angle of the wheel
version = 1

env = Environment()
env.SetViewer('qtcoin')

T0_p = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))

# 1. Create a trajectory for the tool center point to follow
# Left Hand
traj0 = []
Tstart0 = array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0]))))
traj0.append(Tstart0)

# traj0.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.05])))))

#traj0.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.1])))))

# traj0.append(array(MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([0.0,0.0,0.1])))))

# Tgoal0 = array(MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([0.0,0.0,0.15]))))
# Tgoal0 = array(MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([0.0,0.0,0.05]))))


Tgoal0 = array(MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([0.0,0.0,0.05]))))

traj0.append(Tgoal0)

# Right Hand
traj1 = []

Tstart1 = array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0]))))

traj1.append(Tstart1)

# traj1.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,-0.05])))))

# traj1.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,-0.05])))))

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
    wheelHeight = 1.1 #random.random()
    wheelPitch = -0.5 # pi*random.random()
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
rm.load("rlhuboplus_left_m12")
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
rm2.load("rlhuboplus_right_m12")
# rm2.print_rm3D()
# sys.stdin.readline()
# rm2.print_all_transforms()

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

TwheelEndEffector_start0 = dot(TwheelEndEffector_start0,MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, 0.1, 0.0]))))

T0_start0 = dot(T0_wheelEndEffector, TwheelEndEffector_start0)
h.append(misc.DrawAxes(env, T0_start0, 0.4))

TwheelEndEffector_start1 = MakeTransform(matrix(rodrigues([-pi/2, 0, 0])),transpose(matrix([0.0, 0.0, 0.0])))

TwheelEndEffector_start1 = dot(TwheelEndEffector_start1, MakeTransform(matrix(rodrigues([0, 0, -pi/2])),transpose(matrix([0.0, 0.0, 0.0]))))

TwheelEndEffector_start1 = dot(TwheelEndEffector_start1,MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, -0.1, 0.0]))))

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

success = False
end = False

print "Ready to search... ",str(datetime.now())
# sys.stdin.readline()

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
    collisionFreeSolutions = []
    valids = []
    for c in range(howMany):
        print "trying ",str(c)," of ",str(howMany)," candidates."

        

        allGood = play(T0_starts, relBaseConstraint,candidates,numRobots,numManips,c,myRmaps,robots,h,env,0.0)
        h.pop() # delete the robot base axis we added last

        # We went through all our constraints. Is the candidate valid?
        if(allGood):
            collisionFreeSolutions.append(c)
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
    print "Found ",str(len(collisionFreeSolutions))," valid solutions in ",str(howMany)," candidates."
    
    if (len(collisionFreeSolutions) > 0) :
        # Have we found at least 1 valid path?
        success = True

        print "Press Enter to iterate through valid solutions."
        sys.stdin.readline()
        nxt = 0
        ask = True
        playAll = False
        while(True):
            print "Playing valid solution #: ",str(nxt)
            play(T0_starts, relBaseConstraint,candidates,numRobots,numManips,collisionFreeSolutions[nxt],myRmaps,robots,h,env,0.3)
            h.pop() # delete the robot base axis we added last

            currentIk = robots[0].GetDOFValues()
            print currentIk

            # Try to put your feet on the ground
            print "trying to put the feet on the ground..."
            whereToFace = MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0])))
            myIK = put_feet_on_the_ground(probs[0], robots[0], whereToFace, lowerLimits, upperLimits, env)

            print "myIK"
            print myIK
            
            if(myIK != ''):
                print "checking balance constraint..."
                # print "Press enter to see the result..."
                # sys.stdin.readline()
                robots[0].SetDOFValues(str2num(myIK), range(len(robots[0].GetJoints())))
                myCOM = array(get_robot_com(robots[0]))
                myCOM[2,3] = 0.0
                COMHandle = misc.DrawAxes(env,myCOM,0.3)
                
                if(check_support(myCOM,robots[0])):
                    print "This solution is valid"
                    print "left hand path [s:t]: "
                    for pe in candidates[0][collisionFreeSolutions[nxt]]:
                        print str(pe.sIdx),' : ',str(pe.tIdx)
                    print "right hand path [s:t]: "
                    for pe in candidates[1][collisionFreeSolutions[nxt]]:
                        print str(pe.sIdx),' : ',str(pe.tIdx)
                    print "Press enter to proceed with the next solution..."
                    sys.stdin.readline()
                else:
                    print "COM is out of the support polygon"
                
                

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
                if(nxt == len(collisionFreeSolutions)):
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
                if(playAll and (nxt < len(collisionFreeSolutions)-1)):
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

