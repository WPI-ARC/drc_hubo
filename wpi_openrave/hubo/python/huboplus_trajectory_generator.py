from openravepy import *
from numpy import *
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *
import commands
import sys
import pickle
from random import *
from copy import *
from datetime import datetime
from math import *
import RotationalTrajSphereDiscretizer

def frange(start,stop,inc):
    i=start
    a=[]
    a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
    while i < stop:
        i += inc
        a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
    return a

def get_push(minTrajLength,maxTrajLength, delta1, delta2):
    pushTrajsL = []
    for length in frange(minTrajLength,maxTrajLength,delta1):
        currentTraj = []

        for l in frange(0,length,delta2):
            currentTraj.append(array(MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([l, 0.0, 0.0])))))

        pushTrajsL.append(currentTraj)

    pushTrajsR = deepcopy(pushTrajsL)
    return [pushTrajsL, pushTrajsR]

def get_lift(minTrajLength,maxTrajLength,delta1, delta2):
    liftTrajsL = []
    for length in frange(minTrajLength,maxTrajLength,delta1):
        currentTraj = []

        for l in frange(0,length,delta2):
            currentTraj.append(array(MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, 0.0, l])))))

        liftTrajsL.append(currentTraj)

    liftTrajsR = deepcopy(liftTrajsL)
    return [liftTrajsL, liftTrajsR]
    

def get_cw_rotate():
    pass
