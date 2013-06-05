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


def trans_to_str(T):
    myStr = ""
    for c in range(0,3):
        for r in range(0,3):
            myStr += str(T[r,c])+" "
    
    for r in range(0,3):
        myStr += str(T[r,3])+" "

    return myStr

env = Environment()
env.SetViewer('qtcoin')

h = []
h.append(misc.DrawAxes(env,array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))),1.0))

# Add drchubo
robots = []

# limitless drchubo (old)
robots.append(env.ReadRobotURI('../../../openHubo/drchubo/old/drchubo-urdf/robots/drchubo2.robot.xml'))

# up-to-date drchubo with joint limits defined
robots.append(env.ReadRobotURI('../../../openHubo/drchubo/robots/drchubo2.robot.xml'))

# rlhuboplus
robots.append(env.ReadRobotURI('../../../openHubo/huboplus/rlhuboplus.robot.xml'))


# Which robot?
R = 2

env.Add(robots[R])

robots[R].SetTransform(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.5,0.0,0.0])))))

manips = robots[R].GetManipulators()

lowerLimits, upperLimits = robots[R].GetDOFLimits()

print lowerLimits
print upperLimits

for i in range(35):
    j = round(robots[R].GetDOFValues([i]),2)
    if( (nonzero(lowerLimits[i])[0].size == 0) and (nonzero(j)[0].size == 0) ):
        print "setting joint ",str(i)," to positive zero"
        robots[R].SetDOFValues([0.000001],[i])
    elif( (nonzero(upperLimits[i])[0].size == 0) and (nonzero(j)[0].size == 0) ):
        print "setting joint ",str(i)," to negative zero"
        robots[R].SetDOFValues([-0.000001],[i])

probs_cbirrt = RaveCreateModule(env,'CBiRRT')

try:
    env.AddModule(probs_cbirrt,robots[R].GetName()) # this string should match to <Robot name="" > in robot.xml
except openrave_exception, e:
    print e
    
print "Getting Loaded Problems"
probs = env.GetLoadedProblems()

footlinknames = ' Body_RAR Body_LAR '

if( R == 1 or R == 0 ):
    ## DRCHUBO
    # Center of Gravity Target
    T0_TORSO = manips[5].GetEndEffectorTransform()

    cogtarg = [1.0, 2.0, 0]
    cogtarg = [-0.05+T0_TORSO[0,3], 0.085+T0_TORSO[1,3], 0]

    T0_LF = manips[2].GetEndEffectorTransform()
    T0_LF[0,3]+= 0.1
    T0_LF[2,3]+= 0.3
    
    T0_RF = manips[3].GetEndEffectorTransform()
    T0_RF[0,3]+= 0.1
    T0_RF[2,3]+= 0.3
elif( R == 2 ):
    ## RLHUBOPLUS
    ## DRCHUBO
    T0_LF = manips[2].GetEndEffectorTransform()
    T0_LF[0,3]+= 0.1
    T0_LF[2,3]+= 0.1
    
    T0_RF = manips[3].GetEndEffectorTransform()

    cogtarg = [-0.05, 0.085, 0]

cogtargStr = str(cogtarg).strip("[]").replace(', ',' ')
    


h.append(misc.DrawAxes(env,T0_LF,0.1))
h.append(misc.DrawAxes(env,T0_RF,0.1))

goalik = probs[0].SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames
+' movecog '+cogtargStr
+' nummanips 2 maniptm 2 '+trans_to_str(T0_LF)
+' maniptm 3 '+trans_to_str(T0_RF)
)

print "goalik"
print goalik

if(goalik != ''):
    robots[R].SetDOFValues(str2num(goalik),range(35))


print "Press Enter to exit..."
sys.stdin.readline()

env.Destroy()
RaveDestroy()

