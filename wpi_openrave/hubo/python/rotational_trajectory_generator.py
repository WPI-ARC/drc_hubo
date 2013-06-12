#!/usr/bin/env python

## OPENRAVE ##
from openravepy import *
import sys
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

from openravepy.misc import OpenRAVEGlobalArguments
import os # for file operations
from math import *

def myFmod(a,b):
    a1,a2 = a.as_integer_ratio()
    b1,b2 = b.as_integer_ratio()
    div = float(a1*b2) / float(a2*b1)
    mod = a - b*div
    return mod

def frange(start,stop,inc):
    i=start
    a=[]
    a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
    while i < stop:
        i += inc
        a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
    return a

def sign(num):
    if(num >= 0):
        return 1
    elif(num < 0):
        return -1

#for r in frange(0,0.3,0.05):
currentMx = None
currentMy = None
prevMx = None
prevMy = None
r = 0.2
myDiscretizedXYR=[]

start = 0.0
finish = pi/4
sphereDiam = 0.05
rotInc = pi/4

transforms=[[],[]] # 0: leftArm, 1: rightArm

for i in frange(start,finish,0.01):
    
    x = round(cos(i)*r,6)
    y = round(sin(i)*r,6)    
    
    currentMx = 0.0+round(x/sphereDiam) # adding a 0.0 prevents getting -0.0
    currentMy = 0.0+round(y/sphereDiam)
    
    if((currentMx != prevMx) or (currentMy != prevMy)):
        myDiscretizedXYR.append([currentMx, currentMy, i])
    
    prevMx = currentMx
    prevMy = currentMy

if(start == 0.0 and finish == 2*pi):
    myDiscretizedXYR.pop()

print len(myDiscretizedXYR)

for xyr in myDiscretizedXYR:
    #print xyr
    print [round(xyr[0]*sphereDiam,2), round(xyr[1]*sphereDiam,2), round(xyr[2]/rotInc)]
        

