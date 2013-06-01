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
env.SetViewer('qtcoin')

robot = env.ReadRobotURI('../../../openHubo/drchubo/robots/drchubo2.robot.xml')
env.Add(robot)

leftRm = ReachabilityMap("./drchubo_leftArm_ik_solver_f3",robot,robot.GetManipulators()[0])

rightRm = ReachabilityMap("./drchubo_rightArm_ik_solver_f21",robot,robot.GetManipulators()[1])

print "Press Enter to load drchubo's reachability maps."
sys.stdin.readline()

# Note to self: Add attribute "name" in params class
print "Loading..."
leftRm.load("drchubo_left")
rightRm.load("drchubo_right")

print "Loaded the reachability maps for drchubo."

# Implement a rm.info() method
# Until then...
print "number of reachability spheres (left arm):"
print len(leftRm.map)

print "Press Enter to show the left map..."
sys.stdin.readline()
leftRm.show(env)

print "Press Enter to show the right map..."
sys.stdin.readline()
rightRm.show(env)

print "Done. Press Enter to exit..."
sys.stdin.readline()

# Clean the object
del leftRm
del rightRm

# Clean all
env.Destroy()
RaveDestroy()
