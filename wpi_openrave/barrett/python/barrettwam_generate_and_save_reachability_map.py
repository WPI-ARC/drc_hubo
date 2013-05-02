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

robot = env.ReadRobotURI('robots/barrettwam.robot.xml')
env.Add(robot)

rm = ReachabilityMap("./barrettwam_ik_solver",robot,robot.GetManipulators()[0])

rm.r = 0
rm.g = 0
rm.b = 1

rm.free_joint_val = 0.0
rm.free_joint_index = 2

rm.generate(env)

print "Done. Press Enter to save the reachability map."
sys.stdin.readline()

# Save the reachability map
print "Saving the reachability map"
rm.name = "barrettwam_arm"
rm.save()

# Clean the object, just in case
del rm

print "Done. Press Enter to exit..."
sys.stdin.readline()

env.Destroy()
RaveDestroy()

