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

robot = env.ReadRobotURI('../../../openHubo/drchubo/drchubo-urdf/robots/drchubo.robot.xml')
env.Add(robot)

leftRm = ReachabilityMap("./drchubo_leftArm_ik_solver_f5",robot,robot.GetManipulators()[0])

leftRm.xmax=0.6
leftRm.xmin=0.4

leftRm.ymax=0.6
leftRm.ymin=0.4

leftRm.zmax=0.6
leftRm.zmin=0.4

leftRm.r = 0
leftRm.g = 0
leftRm.b = 1

leftRm.free_joint_val = 0.0
leftRm.free_joint_index = 4

leftRm.generate(env)

# print "Done. Press Enter to save the reachability map."
# sys.stdin.readline()

rightRm = ReachabilityMap("./drchubo_rightArm_ik_solver_f23",robot,robot.GetManipulators()[1])

rightRm.r = 1
rightRm.g = 0
rightRm.b = 0

rightRm.free_joint_val = 0.0
rightRm.free_joint_index = 22

rightRm.generate(env)

# print "Done. Press Enter to save the reachability map."
# sys.stdin.readline()

# Clean the object, just in case
del rightRm

print "Done. Press Enter to exit..."
sys.stdin.readline()

env.Destroy()
RaveDestroy()

