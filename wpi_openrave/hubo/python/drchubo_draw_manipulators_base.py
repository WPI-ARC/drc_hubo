#!/usr/bin/env python
#
# Bener Suay, March 2013
#
# benersuay@wpi.edu
#

## OPENRAVE ##
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

from openravepy.misc import OpenRAVEGlobalArguments
from random import *

## SYSTEM - FILE OPS ##
import sys
import os
from datetime import datetime
import time
import commands

env = Environment()
RaveSetDebugLevel(1)
robot = env.ReadRobotURI('../../../openHubo/drchubo/drchubo-urdf/robots/drchubo.robot.xml')
env.Add(robot)
manips  = robot.GetManipulators()

env.SetViewer('qtcoin')

print "Press Enter to draw axes for manip0..."
sys.stdin.readline()
print manips[0].GetBase().GetTransform()
h1=misc.DrawAxes(env,manips[0].GetBase().GetTransform(),1)


print "Press Enter to draw axes for manip1"
sys.stdin.readline()
del h1
print manips[1].GetBase().GetTransform()
h2=misc.DrawAxes(env,manips[1].GetBase().GetTransform(),1)


print "Done. Press Enter to exit..."
sys.stdin.readline()
del h2
env.Destroy()
RaveDestroy()
