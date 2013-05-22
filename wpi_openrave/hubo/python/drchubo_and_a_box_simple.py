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

h = []
h.append(misc.DrawAxes(env,array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))),1.0))

# Add drchubo
robots = []
robots.append(env.ReadRobotURI('../../../openHubo/drchubo/drchubo-urdf/robots/drchubo.robot.xml'))
env.Add(robots[0])

# Add a box to the environment
mybox = RaveCreateKinBody(env,'')
mybox.SetName('box')

boxD1 = 0.025
boxD2 = 0.025
boxH = 0.3

boxX = 0.4
boxY = 0.0
boxZ = 0.2 # T0_box is at the tip of the box

print mybox.InitFromBoxes(numpy.array([[boxX,boxY,boxZ,boxD1,boxD2,boxH]]),True)
env.Add(mybox,True)

T0_box = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([boxX,boxY,boxZ])))
h.append(misc.DrawAxes(env,T0_box,0.3))

# Where do we want the end effectors to start from in world coordinates?
T0_starts = []

Tbox_start0 = MakeTransform(matrix(rodrigues([pi/2,0,0])),transpose(matrix([0.0,0.0,-0.5*boxZ])))
T0_start0 = dot(T0_box,Tbox_start0)
h.append(misc.DrawAxes(env,T0_start0,0.4))

Tbox_start1 = MakeTransform(matrix(rodrigues([-pi/2,0,0])),transpose(matrix([0.0,0.0,0.5*boxZ])))
T0_start1 = dot(T0_box,Tbox_start1)
h.append(misc.DrawAxes(env,T0_start1,0.4))

robots[0].SetDOFValues([-1.331775888269517, 0.844968947358352, -1.570796326794897, -1.843331583826181, 0.0, -0.572433690327068, 2.902572215064414],robots[0].GetManipulators()[0].GetArmIndices())

robots[0].SetDOFValues([-1.823509732949837, -0.842047526923209, 1.570796326794897, -1.836964697066926, 0.0, -0.57587915665118, 2.888879247434853],robots[0].GetManipulators()[1].GetArmIndices())

print "Tbox_start0"
print Tbox_start0

print "Tbox_start1"
print Tbox_start1

print "Press Enter to exit..."
sys.stdin.readline()

env.Destroy()
RaveDestroy()

