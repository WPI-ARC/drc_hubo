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

# 5. Search for the pattern in the reachability model and get the results
success = False

# Define robot base constraint(s)
# a) Bounds <type 'list'>
#    xyz in meters, rpy in radians
#    [1.0, 1.0, 1.0, 0.0, 0.0, 0.0] would mean, we allow robot bases
#    to be 1 meter apart in each direction but we want them to have the 
#    same rotation
baseConstraint = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]

# b) Exact transform <type 'numpy.ndarray'>
# baseConstraint = dot(something, some_other_thing)

# Try to find a valid candidate that satisfies
# all the constraints we have (base location, collision, and configuration-jump)
while(not success):
    # find a random candidate in both maps
    candidates = find_random_candidates(myPatterns,myRmaps,1)

    # Move each robot
    for myManipulatorIndex in range(len(robots)):
        startSphereIndex = candidates[myManipulatorIndex][0][0].sIdx
        startTransformIndex = candidates[myManipulatorIndex][0][0].tIdx
        Tbase_start = myRmaps[myManipulatorIndex].map[startSphereIndex].T[startTransformIndex]
        T0_newManipPose = dot(T0_starts[myManipulatorIndex],linalg.inv(Tbase_start))
        robots[myManipulatorIndex].SetTransform(T0_newManipPose)

    # Get their relative Transformation matrix
    T0_base = []
    for myManipulatorIndex in range(len(robots)):
        T0_base.append(robots[myManipulatorIndex].GetManipulators()[0].GetBase().GetTransform())    
    Trobot0_robot1 = dot(linalg.inv(T0_base[0]),T0_base[1])

    # i) Does the candidate satisfy the robot base transform constraint?
    ok = True
    if(type(baseConstraint) == type([])):
        # then we have bounds
        for i in range(3):
            # Check if all constraints are met
            if(Trobot0_robot1[i,3] > baseConstraint[i]):
                ok = False

    elif(type(baseConstraint) == type(array(()))):
        # then we have an exact transformation
        pass   
    # ii) Is the solution candidate collision-free?
    # iii) And finally, is there a jump between two consecutive configurations? 
    # We went through all our constraints. Is the candidate valid?
    if(ok):
        success = True

for myCandidatePathIndex in range(min(len(candidates[0]),len(candidates[1]))):
    for myManipulatorIndex in range(len(robots)):
        startSphereIndex = candidates[myManipulatorIndex][myCandidatePathIndex][0].sIdx
        startTransformIndex = candidates[myManipulatorIndex][myCandidatePathIndex][0].tIdx
        Tbase_start = myRmaps[myManipulatorIndex].map[startSphereIndex].T[startTransformIndex]
        print startSphereIndex
        print startTransformIndex
        print Tbase_start
        T0_newManipPose = dot(T0_starts[myManipulatorIndex],linalg.inv(Tbase_start))
        robots[myManipulatorIndex].SetTransform(T0_newManipPose)
        myRmaps[myManipulatorIndex].go_to(startSphereIndex,startTransformIndex)

    print "Press enter to run the trajectory..."
    sys.stdin.readline()
    #time.sleep(1)

    for myPathElement in range(1,len(candidates[myManipulatorIndex][myCandidatePathIndex])):
        for myManipulatorIndex in range(len(robots)):
            currentSphereIndex = candidates[myManipulatorIndex][myCandidatePathIndex][myPathElement].sIdx
            currentTransformIndex = candidates[myManipulatorIndex][myCandidatePathIndex][myPathElement].tIdx
            myRmaps[myManipulatorIndex].go_to(currentSphereIndex,currentTransformIndex)
        #"Press enter..."
        #sys.stdin.readline()
        time.sleep(1)

    "Press enter..."
    sys.stdin.readline()
    time.sleep(1)


#constraints = []

# Transform of the end-effector-1 in end-effector-0's coordinate frame throughout
# the manipulation trajectory
#
# Tee0_ee1
#constraints.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.2,0.0]))))

# Trobot0_robot1
# constraints.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.2,0.0]))))

# 6. Add an object in the environment to a random location

# 7. Use the object location and define where Tstart_left and Tstart_right is on the object

# 8. Find the relative transform between Tstart_left and Tstart_right.
#    We assume that this relative transform will remain constant throughout the path.

env.Destroy()
RaveDestroy()

