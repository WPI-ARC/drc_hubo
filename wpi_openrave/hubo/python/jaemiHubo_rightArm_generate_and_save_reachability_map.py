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

robot = env.ReadRobotURI('../../../openHubo/jaemi/humanoids2013.jaemiHubo.planning.robot.xml')
env.Add(robot)

rightRm = ReachabilityMap(robot,'rightArmManip')

rightRm.xmax=0.45
rightRm.xmin=0.0

rightRm.ymax=0.15
rightRm.ymin=-0.55

rightRm.zmax=0.4
rightRm.zmin=-0.4

rightRm.r = 1
rightRm.g = 0
rightRm.b = 0

rightRm.generate(env)

# print "Done. Press Enter to save the reachability map."
# sys.stdin.readline()

# Save the reachability map
print "Saving the reachability map"
rightRm.name = "jaemi_right_n8_m12_awesome"
rightRm.save()

# Clean the object, just in case
del rightRm

print "Done. Press Enter to exit..."
sys.stdin.readline()

env.Destroy()
RaveDestroy()

