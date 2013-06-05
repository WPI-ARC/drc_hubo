#!/usr/bin/env python
#
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

from copy import deepcopy

from Reachability import *

import scipy.misc

env = Environment()
RaveSetDebugLevel(1)


rlhuboplus = env.ReadRobotURI('../../../openHubo/huboplus/huboplus2.robot.xml')
env.Add(rlhuboplus)



rlhuboplusLeftRm = ReachabilityMap("./rlhuboplus_leftArm_ik_solver",rlhuboplus,rlhuboplus.GetManipulators()[0])
rlhuboplusRightRm = ReachabilityMap("./rlhuboplus_rightArm_ik_solver",rlhuboplus,rlhuboplus.GetManipulators()[1])

print "loading"
rlhuboplusLeftRm.load("rlhuboplus_left_m12")
rlhuboplusRightRm.load("rlhuboplus_right_m12")

env.SetViewer('qtcoin')

h = misc.DrawAxes(env,array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))),1.0)

rlhuboplusLeftRm.map[1025].show(env,array((0,0,1,0.5)))
rlhuboplusLeftRm.map[1026].show(env,array((0,0,1,0.5)))
rlhuboplusLeftRm.map[1055].show(env,array((0,0,1,0.5)))
rlhuboplusLeftRm.map[1056].show(env,array((0,0,1,0.5)))
rlhuboplusLeftRm.map[1227].show(env,array((0,0,1,0.5)))
rlhuboplusLeftRm.map[1228].show(env,array((0,0,1,0.5)))
rlhuboplusLeftRm.hide()

rlhuboplusRightRm.map[1071].show(env,array((1,0,0,0.5)))
rlhuboplusRightRm.map[1070].show(env,array((1,0,0,0.5)))
rlhuboplusRightRm.map[1101].show(env,array((1,0,0,0.5)))
rlhuboplusRightRm.map[1100].show(env,array((1,0,0,0.5)))
rlhuboplusRightRm.map[1257].show(env,array((1,0,0,0.5)))
rlhuboplusRightRm.map[1256].show(env,array((1,0,0,0.5)))

print "Press enter to take a screenshot."
sys.stdin.readline()

viewer = env.GetViewer()
viewer.SendCommand('SetFiguresInCamera 1') 
scipy.misc.imsave(str(datetime.now())+'.jpg', v.GetCameraImage(1024,768,v.GetCameraTransform(),[1024,1024,512,384]))

# Note: if we don't delete v we get the following exception
#
# QObject::startTimer: QTimer can only be used with threads started with QThread
# Segmentation fault (core dumped)
del v

print "Done! Press Enter to exit..."
sys.stdin.readline()

env.Destroy()
RaveDestroy()
