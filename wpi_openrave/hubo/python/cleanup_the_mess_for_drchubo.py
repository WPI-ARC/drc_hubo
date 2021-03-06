#!/usr/bin/env python
#
# Bener Suay, June 2013
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

drchuboLeftRm = ReachabilityMap("./drchubo_rightArm_ik_solver_f3",drchubo,drchubo.GetManipulators()[0])
drchuboRightRm = ReachabilityMap("./drchubo_rightArm_ik_solver_f21",drchubo,drchubo.GetManipulators()[1])

drchuboLeftRm.load("drchubo_left_m12_messy")
drchuboRightRm.load("drchubo_right_m12_messy")

matricesToPop = []

matricesToPop.append(array([[ 1.,0.,0.],[ 0.,1.,0.],[ 0.,0.,1.]]))

matricesToPop.append(array([[ 1.,0.,0.],[ 0.,0.70710678,-0.70710678],[ 0.,0.70710678,0.70710678]]))

matricesToPop.append(array([[1.00000000e+00,0.00000000e+00,0.00000000e+00],[0.00000000e+00,1.11022302e-16,-1.00000000e+00],[0.00000000e+00,1.00000000e+00,1.11022302e-16]]))

matricesToPop.append(array([[ 1.,0.,0.],[ 0.,-0.70710678,-0.70710678],[ 0.,0.70710678,-0.70710678]]))

matricesToPop.append(array([[1.00000000e+00,0.00000000e+00,0.00000000e+00],[0.00000000e+00,-1.00000000e+00,-1.22460635e-16],[0.00000000e+00,1.22460635e-16,-1.00000000e+00]]))

matricesToPop.append(array([[1.,0.,0.],[ 0.,-0.70710678,0.70710678],[ 0.,-0.70710678,-0.70710678]]))

matricesToPop.append(array([[1.00000000e+00,0.00000000e+00,0.00000000e+00],[0.00000000e+00,-2.22044605e-16,1.00000000e+00],[0.00000000e+00,-1.00000000e+00,-2.22044605e-16]]))

matricesToPop.append(array([[1.,0.,0.],[0.,0.70710678,0.70710678],[0.,-0.70710678,0.70710678]]))

matricesToPop.append(array([[7.07106781e-01,0.00000000e+00,7.07106781e-01],[7.07106781e-01,1.11022302e-16,-7.07106781e-01],[-7.85046229e-17,1.00000000e+00,7.85046229e-17]]))

matricesToPop.append(array([[1.11022302e-16,0.00000000e+00,1.00000000e+00],[  1.00000000e+00,1.11022302e-16,-1.11022302e-16],[ -1.11022302e-16,1.00000000e+00,1.23259516e-32]]))

matricesToPop.append(array([[ -7.07106781e-01,0.00000000e+00,7.07106781e-01],[  7.07106781e-01,1.11022302e-16,7.07106781e-01],[ -7.85046229e-17,1.00000000e+00,-7.85046229e-17]]))

matricesToPop.append(array([[ -7.07106781e-01,0.00000000e+00,-7.07106781e-01],[ -7.07106781e-01,1.11022302e-16,7.07106781e-01],[  7.85046229e-17,1.00000000e+00,-7.85046229e-17]]))

matricesToPop.append(array([[ -2.22044605e-16,0.00000000e+00,-1.00000000e+00],[ -1.00000000e+00,1.11022302e-16,2.22044605e-16],[  1.11022302e-16,1.00000000e+00,-2.46519033e-32]]))

matricesToPop.append(array([[  7.07106781e-01,0.00000000e+00,-7.07106781e-01],[ -7.07106781e-01,1.11022302e-16,-7.07106781e-01],[  7.85046229e-17,1.00000000e+00,7.85046229e-17]]))


#### LEFT MAP ###

print "Processing the left map..."

i = 0
c = 0
moveToNextSphere = False
whatToPop = []
for pIdx, p in enumerate(matricesToPop):
    for sIdx, s in enumerate(drchuboLeftRm.map):
        c = 0
        for tIdx, t in enumerate(s.T):
            if (allclose(t[0:3,0:3],p)):
                c+=1
                if(c == 2):
                    whatToPop.append([sIdx, tIdx, pIdx])
                    moveToNextSphere = True
                    break

print "whatToPop"
for w in whatToPop:
    print w

for sIdx, s in enumerate(drchuboLeftRm.map):
    print "\n \n"
    for tIdx, t in enumerate(s.T):
        print "s: ",str(sIdx)," , t: ",str(tIdx)
        print t

myCleanLeftRmMap = []
totalNumOfTransformsBefore = 0
totalNumOfTransformsAfter = 0

for sIdx, s in enumerate(drchuboLeftRm.map):
    myCleanS = deepcopy(s)
    myCleanS.T = []
    myCleanS.configs = []
    isThisOneOnTheList = False
    for tIdx, t in enumerate(s.T):
        totalNumOfTransformsBefore += 1
        for w in whatToPop:
            if(w[0] == sIdx and w[1] == tIdx):
                isThisOneOnTheList = True
                break
        if(isThisOneOnTheList):
            isThisOneOnTheList = False
        else:
            totalNumOfTransformsAfter += 1
            myCleanS.T.append(t)
            myCleanS.configs.append(deepcopy(s.configs[tIdx]))

    myCleanLeftRmMap.append(myCleanS)

print "report:"
print totalNumOfTransformsBefore
print totalNumOfTransformsAfter
print len(whatToPop)

drchuboLeftRm.map = deepcopy(myCleanLeftRmMap)
drchuboLeftRm.name = "drchubo_left_m12"
drchuboLeftRm.save()

#### RIGHT MAP ###

print "Processing the right map..."

i = 0
c = 0
moveToNextSphere = False
whatToPop = []
for pIdx, p in enumerate(matricesToPop):
    for sIdx, s in enumerate(drchuboRightRm.map):
        c = 0
        for tIdx, t in enumerate(s.T):
            if (allclose(t[0:3,0:3],p)):
                c+=1
                if(c == 2):
                    whatToPop.append([sIdx, tIdx, pIdx])
                    moveToNextSphere = True
                    break

print "whatToPop"
for w in whatToPop:
    print w

for sIdx, s in enumerate(drchuboRightRm.map):
    print "\n \n"
    for tIdx, t in enumerate(s.T):
        print "s: ",str(sIdx)," , t: ",str(tIdx)
        print t

myCleanRightRmMap = []
totalNumOfTransformsBefore = 0
totalNumOfTransformsAfter = 0

for sIdx, s in enumerate(drchuboRightRm.map):
    myCleanS = deepcopy(s)
    myCleanS.T = []
    myCleanS.configs = []
    isThisOneOnTheList = False
    for tIdx, t in enumerate(s.T):
        totalNumOfTransformsBefore += 1
        for w in whatToPop:
            if(w[0] == sIdx and w[1] == tIdx):
                isThisOneOnTheList = True
                break
        if(isThisOneOnTheList):
            isThisOneOnTheList = False
        else:
            totalNumOfTransformsAfter += 1
            myCleanS.T.append(t)
            myCleanS.configs.append(deepcopy(s.configs[tIdx]))

    myCleanRightRmMap.append(myCleanS)

print "report:"
print totalNumOfTransformsBefore
print totalNumOfTransformsAfter
print len(whatToPop)

drchuboRightRm.map = deepcopy(myCleanRightRmMap)
drchuboRightRm.name = "drchubo_right_m12"
drchuboRightRm.save()

print "Done!..."
sys.stdin.readline()

env.Destroy()
RaveDestroy()
