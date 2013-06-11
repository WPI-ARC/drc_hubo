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
r = 0.1
myDiscretizedXY=[]

for i in frange(0,2*pi,0.01):
    x = round(cos(i)*r,2)
    y = round(sin(i)*r,2)
    
    currentMx = 0.0+round(x/0.05) # adding a 0.0 prevents getting -0.0
    currentMy = 0.0+round(y/0.05)
    
    if((currentMx != prevMx) or (currentMy != prevMy)):
        myDiscretizedXY.append([currentMx, currentMy])
    
    prevMx = currentMx
    prevMy = currentMy

print len(myDiscretizedXY)    
for xy in myDiscretizedXY:
    print xy
        

