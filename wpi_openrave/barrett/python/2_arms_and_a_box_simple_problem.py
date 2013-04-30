# Bener Suay, April 2013
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

env = Environment()
env.SetViewer('qtcoin')

T0_p = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))

# 1. Create a trajectory for the tool center point to follow
Tstart = array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0]))))
Tgoal = array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,-0.2,0.0]))))

traj = []
traj.append(Tstart)
traj.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,-0.02,0.0])))))
traj.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,-0.07,0.0])))))
traj.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,-0.12,0.0])))))
traj.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,-0.16,0.0])))))
traj.append(Tgoal)

# Just to try
# Move, then rotate around the axis with that shift being the radius
#traj.append(dot(array(MakeTransform(matrix(rodrigues([0,0,pi/2])),transpose(matrix([0.0,0.0,0.0])))),array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.07+offset[0],0.0+offset[1],0.0]))))))

h = []
h.append(misc.DrawAxes(env,array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))),1.0))

myPatterns = []
# 2. Create a search pattern from the trajectory for the first manipulator
myPattern = SearchPattern(traj)
myPattern.T0_p = T0_p
myPattern.setColor(array((0,1,1,0.5))) 
myPattern.show(env)
# sys.stdin.readline()

myPatterns.append(myPattern)

# 3. Create a search pattern from the trajectory for the second manipulator
myPattern = SearchPattern(traj)
myPattern.T0_p = T0_p
myPattern.setColor(array((1,1,0,0.5))) 
myPattern.show(env)
sys.stdin.readline()

myPatterns.append(myPattern)

for p in myPatterns:
    p.hide("spheres")
    # sys.stdin.readline()

for p in myPatterns:
    p.hide("all")
    # sys.stdin.readline()

# 3. Add both robots
robots = []

# the following for loop will add N robots from the same xml file
# and rename them to prevent failure
for i in range(2):
    robots.append(env.ReadRobotURI('robots/barrettwam.robot.xml'))
    env.Add(robots[len(robots)-1]) # add the last robot in the environment so that we can rename it.

    for body in env.GetBodies():
        rname = body.GetName()
        if(rname == 'BarrettWAM'):
            newname = 'robot'+str(i)
            body.SetName(newname)
        print body

rotz=[]
rotz.append(pi/3);
rotz.append(0.0);

shift_robot0 = MakeTransform(matrix(rodrigues([0,0,rotz[0]])),transpose(matrix([0.5,0.0,0.0])))
shift_robot1 = MakeTransform(matrix(rodrigues([0,0,rotz[1]])),transpose(matrix([-1.5,0.0,0.0])))

robots[0].SetTransform(array(shift_robot0))
robots[1].SetTransform(array(shift_robot1))

# Add a box to the environment
mybox = RaveCreateKinBody(env,'')
mybox.SetName('box')



boxD1 = 0.025
boxD2 = 0.025
boxH = 0.3

boxX = -0.5
boxY = 0.0
boxZ = boxH # T0_box is at the tip of the box

print mybox.InitFromBoxes(numpy.array([[boxX,boxY,boxZ,boxD1,boxD2,boxH]]),True)
env.Add(mybox,True)
T0_box = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([boxX,boxY,boxZ])))
h.append(misc.DrawAxes(env,T0_box,0.3))

# sys.stdin.readline()

# 4. Generate reachability models for robots
# Robot 1
myRmaps = []
rm = ReachabilityMap("./barrettwam_ik_solver",robots[0],robots[0].GetManipulators()[0])
print "Loading reachability map for Robot0..."
rm.load("barrettwam_arm")
rm.name = "barrettwam_arm_0"
#rm.show(env) # slows down the process a lot

# Append the reachability map, and keep it in a list
myRmaps.append(rm)

print "Robot0 Reachability Map loaded. Press Enter to continue with Robot1..."
sys.stdin.readline()

# Do the same for Robot 2
rm2 = ReachabilityMap("./barrettwam_ik_solver",robots[1],robots[1].GetManipulators()[0])
print "Loading reachability map for Robot1..."
rm2.load("barrettwam_arm")
rm2.name = "barrettwam_arm_1"
rm2.r = 1
rm2.g = 0
rm2.b = 0
#print "Reachability map loaded for Robot1. Press Enter to show the map."
#sys.stdin.readline()
#rm2.show(env)
myRmaps.append(rm2)

print "Press Enter to find path candidates..."
sys.stdin.readline()

# 4. Where do we want the end effectors to start from in world coordinates?
T0_starts = []

Tbox_start0 = MakeTransform(matrix(rodrigues([-pi/2,0,0])),transpose(matrix([0.0,0.0,boxZ*0.5])))
Tbox_start0 = dot(Tbox_start0, MakeTransform(matrix(rodrigues([0,pi,0])),transpose(matrix([0.0,0.0,0.0]))))
T0_start0 = dot(T0_box,Tbox_start0)
h.append(misc.DrawAxes(env,T0_start0,0.4))

Tbox_start1 = MakeTransform(matrix(rodrigues([-pi/2,0,0])),transpose(matrix([0.0,0.0,-boxZ*0.5])))
Tbox_start1 = dot(Tbox_start1, MakeTransform(matrix(rodrigues([0,pi,0])),transpose(matrix([0.0,0.0,0.0]))))
T0_start1 = dot(T0_box,Tbox_start1)
h.append(misc.DrawAxes(env,T0_start1,0.4))

T0_starts.append(array(T0_start0))
T0_starts.append(array(T0_start1))



# Define robot base constraint(s)
# a) Bounds <type 'list'>
#    xyz in meters, rpy in radians
#    [1.0, 1.0, 1.0, 0.0, 0.0, 0.0] would mean, we allow robot bases
#    to be 1 meter apart in each direction but we want them to have the 
#    same rotation
baseConstraint = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]

# b) Exact transform <type 'numpy.ndarray'>
# baseConstraint = dot(something, some_other_thing)

# Try to find a valid candidate that satisfies
# all the constraints we have (base location, collision, and configuration-jump)


# 5. Search for the pattern in the reachability model and get the results
success = False
numRobots = len(robots)
totalTime = array(())
timeToFindAPath = 0.0
findAPathStarts = time.time()
while(not success):
    myStatus = ""
    baseConstOK = True
    collisionConstOK = True

    # find a random candidate in both maps
    findStarts = time.time()
    candidates = find_random_candidates(myPatterns,myRmaps,1)
    # Get the length of the candidate path
    # First index stands for the robot index, and the second index stands for the candidate path index. We're calling find_random_candidates() function with an argument of 1. Thus there will be only one candidate returned. Let's find it's length.
    pathLength = len(candidates[0][0]) 
    findEnds = time.time()
    thisDiff = findEnds-findStarts
    totalTime = append(totalTime,thisDiff)
    print "Found  a random result in ",str(thisDiff)," secs."

    # Move each robot
    for myRobotIndex in range(numRobots):
        # Find where to move the base
        startSphereIndex = candidates[myRobotIndex][0][0].sIdx
        startTransformIndex = candidates[myRobotIndex][0][0].tIdx
        Tbase_start = myRmaps[myRobotIndex].map[startSphereIndex].T[startTransformIndex]
        T0_newManipPose = dot(T0_starts[myRobotIndex],linalg.inv(Tbase_start))
        # Finally move the robot base 
        robots[myRobotIndex].SetTransform(T0_newManipPose)

    # Get their relative Transformation matrix
    T0_base = []
    for myManipulatorIndex in range(len(robots)):
        T0_base.append(robots[myManipulatorIndex].GetManipulators()[0].GetBase().GetTransform())    
    Trobot0_robot1 = dot(linalg.inv(T0_base[0]),T0_base[1])

    # i) Does the candidate satisfy the robot base transform constraint?
    if(type(baseConstraint) == type([])):
        # then we have bounds
        for i in range(3):
            # Check if all constraints are met
            if(Trobot0_robot1[i,3] > baseConstraint[i]):
                baseConstOK = False
    elif(type(baseConstraint) == type(array(()))):
        # then we have an exact transformation
        pass   

    # If the solution meets the base constraint:
    if(baseConstOK):
        # ii) Check if the solution collision-free throughout the path?
        # For each path element, go step by step and check
        for pElementIndex in range(pathLength):
            # Move the robot to this path element
            for myRobotIndex in range(numRobots):
                currentSphereIndex = candidates[myRobotIndex][0][pElementIndex].sIdx
                currentTransformIndex = candidates[myRobotIndex][0][pElementIndex].tIdx
                myRmaps[myRobotIndex].go_to(currentSphereIndex,currentTransformIndex)
                # Check collision with self and with the environment
                if(env.CheckCollision(robots[myRobotIndex]) or robots[myRobotIndex].CheckSelfCollision()):
                    collisionConstOK = False
                    break
            if(not collisionConstOK):
                break
            # If you didn't break yet, wait before the next path element for visualization
            #time.sleep(0.05)
    
            # iii) And finally, is there a jump between two consecutive configurations? 
            # Should we check this nicely or should we just use a delta constant?

    # We went through all our constraints. Is the candidate valid?
    if(baseConstOK and collisionConstOK):
        findAPathEnds = time.time()
        print "Success! Constraint(s) met."
        print "Total time spent to find the candidate(s): "
        print str(totalTime.cumsum()[-1])," sec."
        print "Total time spent to find a candidate and validate the path: "
        print str(findAPathEnds-findAPathStarts)," sec."
        success = True
    else:
        print "Constraint failure."
        print "Collision OK?: "
        print collisionConstOK
        print "Base Constraint OK?:"
        print baseConstOK

# 6. Add an object in the environment to a random location

# 7. Use the object location and define where Tstart_left and Tstart_right is on the object

# 8. Find the relative transform between Tstart_left and Tstart_right.
#    We assume that this relative transform will remain constant throughout the path.

env.Destroy()
RaveDestroy()

