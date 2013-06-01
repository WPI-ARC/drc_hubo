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

env = Environment()
#env.SetViewer('qtcoin')

robot = env.ReadRobotURI('../../../openHubo/drchubo/robots/drchubo2.robot.xml')
env.Add(robot)

leftRm = ReachabilityMap("./drchubo_leftArm_ik_solver_f3",robot,robot.GetManipulators()[0])

# leftRm.xmax=0.3
leftRm.xmin=0.0

# leftRm.ymax=0.3
# leftRm.ymin=0.0

# leftRm.zmax=0.3
# leftRm.zmin=0.0

leftRm.r = 0
leftRm.g = 0
leftRm.b = 1

# in this arm the index 0 1 2 3 4 5 6
#                           |
#                           V
# leftArm joint indices 1,2,3,4,5,6,7
leftRm.free_joint_index = 2 
leftRm.free_joint_val = 0.0

leftRm.generate(env)

print "Done. Press Enter to save the reachability map."
sys.stdin.readline()

# Save the reachability map
print "Saving the reachability map"
leftRm.name = "drchubo_left"
leftRm.save()

rightRm = ReachabilityMap("./drchubo_rightArm_ik_solver_f21",robot,robot.GetManipulators()[1])

# rightRm.xmax=0.3
rightRm.xmin=0.0

# rightRm.ymax=-0.0
# rightRm.ymin=-0.3

# rightRm.zmax=0.3
# rightRm.zmin=0.0

rightRm.r = 1
rightRm.g = 0
rightRm.b = 0

# in this arm the index 0  1  2  3  4  5  6
#                             |
#                             V
# leftArm joint indices 19,20,21,22,23,24,25
rightRm.free_joint_index = 2
rightRm.free_joint_val = 0.0

rightRm.generate(env)

print "Done. Press Enter to save the reachability map."
sys.stdin.readline()

# Save the reachability map
print "Saving the reachability map"
rightRm.name = "drchubo_right"
rightRm.save()

# Clean the object, just in case
del leftRm
del rightRm

print "Done. Press Enter to exit..."
sys.stdin.readline()

env.Destroy()
RaveDestroy()

