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
    if(stop > start):
        while i < stop:
            i += inc
            a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
    elif(start >= stop):
        while i > stop:
            i += inc
            a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
    return a

def get_discrete_xyr(r=0.2, start=0.0, finish=2*pi, inc=0.01):    
    currentMx = None
    currentMy = None
    prevMx = None
    prevMy = None

    myDiscretizedXYR=[]
    
    sphereDiam = 0.05
    rotInc = pi/4
    
    transformXYR=[]
    
    for i in frange(start,finish,inc):
        
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
        transformXYR.append([round(xyr[0]*sphereDiam,2), round(xyr[1]*sphereDiam,2), 0.0+round(xyr[2]/rotInc)])
        # note, adding a 0.0 prevents getting -0.0
        
        
    return [xyr, transformXYR]

if __name__ == '__main__':
    # [0]: this script's name
    # [1]: r
    # [2]: start angle in rads.
    # [3]: finish angle in rads
    if(len(sys.argv)==5):
        ans = get_discrete_xyr(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))[1]
    else:
        ans = get_discrete_xyr()[1]

    for txyr in ans:
        print txyr
            
        

