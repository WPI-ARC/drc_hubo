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


# Which robot?
idx = 0

env.Add(robots[idx])

robots[idx].SetTransform(array(MakeTransform(matrix(rodrigues([0,pi/12,0])),transpose(matrix([0.0,0.0,0.0])))))

manips = robots[idx].GetManipulators()

lowerLimits, upperLimits = robots[idx].GetDOFLimits()

print lowerLimits
print upperLimits

for i in range(35):
    j = round(robots[idx].GetDOFValues([i]),2)
    if( (nonzero(lowerLimits[i])[0].size == 0) and (nonzero(j)[0].size == 0) ):
        print "setting joint ",str(i)," to positive zero"
        robots[idx].SetDOFValues([0.000001],[i])
    elif( (nonzero(upperLimits[i])[0].size == 0) and (nonzero(j)[0].size == 0) ):
        print "setting joint ",str(i)," to negative zero"
        robots[idx].SetDOFValues([-0.000001],[i])

probs_cbirrt = RaveCreateModule(env,'CBiRRT')

try:
    env.AddModule(probs_cbirrt,robots[idx].GetName()) # this string should match to <Robot name="" > in robot.xml
except openrave_exception, e:
    print e
    
print "Getting Loaded Problems"
probs = env.GetLoadedProblems()

footlinknames = ' Body_RAR Body_LAR '

if( idx == 1 or idx == 0 ):
    ## DRCHUBO
    # Center of Gravity Target
    T0_TORSO = manips[5].GetEndEffectorTransform()

    cogtarg = [1.0, 2.0, 0]
    cogtarg = [-0.05+T0_TORSO[0,3], 0.085+T0_TORSO[1,3], 0]

    T0_LF = manips[2].GetEndEffectorTransform()
    T0_LF[0,3]+= 0.3
    T0_LF[2,3]+= 0.1
    
    T0_RF = manips[3].GetEndEffectorTransform()
    T0_RF[0,3]+= 0.3
    T0_RF[2,3]+= 0.1

cogtargStr = str(cogtarg).strip("[]").replace(', ',' ')
    


h.append(misc.DrawAxes(env,T0_LF,0.1))
h.append(misc.DrawAxes(env,T0_RF,0.1))

goalik = probs[0].SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames
#+' movecog '+cogtargStr
+' nummanips 2 maniptm 2 '+trans_to_str(T0_LF)
+' maniptm 3 '+trans_to_str(T0_RF)
)

print "goalik"
print goalik

if(goalik != ''):
    robots[idx].SetDOFValues(str2num(goalik),range(35))

print "inBalance"
print probs[0].SendCommand('CheckSupport supportlinks 2 '+footlinknames)

print "let's calculate the COM here..."

# https://en.wikipedia.org/wiki/Center_of_mass
# R*M = SUM(r_i*m_i)
#
# R = SUM(r_i*m_i)/M
M = 0
rm = 0
for l in robots[idx].GetLinks():
    m = l.GetMass()
    M += m
    
    r = dot(l.GetTransform(),array(MakeTransform(rodrigues([0,0,0]),transpose(matrix(l.GetCOMOffset())))))
    rm += multiply(r,m)

print "total mass:"
print M

R = divide(rm,M)

print "COM:"
print R.round(4)

h.append(misc.DrawAxes(env,array(R.round(4)),0.1))

print "Press Enter to exit..."
sys.stdin.readline()

env.Destroy()
RaveDestroy()

