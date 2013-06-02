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

env = Environment()
RaveSetDebugLevel(1)
drchubo = env.ReadRobotURI('../../../openHubo/drchubo/robots/drchubo2.robot.xml')
env.Add(drchubo)
drchubo_manips  = drchubo.GetManipulators()
drchubo_rightFoot = drchubo_manips[3]

T0_RF = drchubo_rightFoot.GetEndEffectorTransform() # right foot end effector transform
print T0_RF
shift_drchubo_to_its_right_foot = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.5,-1*T0_RF[1,3],-1*T0_RF[2,3]])))
drchubo.SetTransform(array(shift_drchubo_to_its_right_foot))

rlhuboplus = env.ReadRobotURI('../../../openHubo/huboplus/rlhuboplus.robot.xml')
shift_rlhuboplus_back = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-0.5,0.0,0.0])))
rlhuboplus.SetTransform(array(shift_rlhuboplus_back))
env.Add(rlhuboplus)

drchuboLeftRm = ReachabilityMap("./drchubo_leftArm_ik_solver_f3",drchubo,drchubo.GetManipulators()[0])
drchuboRightRm = ReachabilityMap("./drchubo_rightArm_ik_solver_f21",drchubo,drchubo.GetManipulators()[1])
rlhuboplusLeftRm = ReachabilityMap("./rlhuboplus_leftArm_ik_solver",rlhuboplus,rlhuboplus.GetManipulators()[0])
rlhuboplusRightRm = ReachabilityMap("./rlhuboplus_rightArm_ik_solver",rlhuboplus,rlhuboplus.GetManipulators()[1])

print "loading"
drchuboLeftRm.load("drchubo_left")
drchuboRightRm.load("drchubo_right")
rlhuboplusLeftRm.load("rlhuboplus_left")
rlhuboplusRightRm.load("rlhuboplus_right")

env.SetViewer('qtcoin')

drchuboLeftRm.show(env)
drchuboRightRm.show(env)
rlhuboplusLeftRm.show(env)
rlhuboplusRightRm.show(env)

print "Done! Press Enter to exit..."
sys.stdin.readline()
