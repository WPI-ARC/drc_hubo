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

from datetime import datetime

env = Environment()
env.SetViewer('qtcoin')

robot = env.ReadRobotURI('../../../openHubo/drchubo/drchubo-urdf/robots/drchubo.robot.xml')
env.Add(robot)

leftRm = ReachabilityMap("./drchubo_leftArm_ik_solver_f5",robot,robot.GetManipulators()[0])

# rightRm = ReachabilityMap("./drchubo_rightArm_ik_solver_f23",robot,robot.GetManipulators()[1])

print "Press Enter to load drchubo's reachability maps."
sys.stdin.readline()

# Note to self: Add attribute "name" in params class
print "Loading..."
leftRm.load("drchubo_left_old")
# rightRm.load("drchubo_right_old")

print "Loaded the reachability maps for drchubo."

# Implement a rm.info() method
# Until then...
print "number of reachability spheres (left arm):"
print len(leftRm.map)

print "Press Enter to sort the left map..."
sys.stdin.readline()

print "Extracting reachability spheres... ",str(datetime.now())
myArr = []
for sIdx, s in enumerate(leftRm.map):
    myArr.append(s.reachability)

print "Sorting started (qsort): ",str(datetime.now())
mySortedArr = sort(myArr,axis=-1,kind='quicksort')
print "Sorting ended: ",str(datetime.now())

print "Sorting started (msort): ",str(datetime.now())
mySortedArr = sort(myArr,axis=-1,kind='mergesort')
print "Sorting ended: ",str(datetime.now())

print "Sorting started (hsort): ",str(datetime.now())
mySortedArr = sort(myArr,axis=-1,kind='heapsort')
print "Sorting ended: ",str(datetime.now())



print "Getting indices: ",str(datetime.now())
mySortedArrReverseLookup = argsort(myArr,axis=-1,kind='quicksort')
print "Got indices: ",str(datetime.now())

print mySortedArr
print mySortedArrReverseLookup

print myArr[mySortedArrReverseLookup[0]]
print mySortedArr[0]


print "Done. Press Enter to exit..."
sys.stdin.readline()

# Clean the object(s)
del leftRm
# del rightRm

# Clean all
env.Destroy()
RaveDestroy()
