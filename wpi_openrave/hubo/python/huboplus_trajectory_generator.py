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

minDistanceBetweenHands = 0.05
maxDistanceBetweenHands = 1.0
delta1 = 0.05
minTrajLength = 0.05
maxTrajLength = 1.0
delta2 = 0.05
minTrajRot = 0.0
maxTrajRot = pi/2
delta3 = pi/4

def frange(start,stop,inc):
    i=start
    a=[]
    a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
    while i < stop:
        i += inc
        a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
    return a

def get_starts(T0_OBJECT):
    # 4. Where do we want the end effectors to start from in world coordinates?
    T0_starts = []

    T0_wheelEndEffector = 

    TwheelEndEffector_start0 = MakeTransform(matrix(rodrigues([-pi/2, 0, 0])),transpose(matrix([0.0, 0.0, 0.0])))

    TwheelEndEffector_start0 = dot(TwheelEndEffector_start0, MakeTransform(matrix(rodrigues([0, 0, -pi/2])),transpose(matrix([0.0, 0.0, 0.0]))))
    
    TwheelEndEffector_start0 = dot(TwheelEndEffector_start0,MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, 0.1, 0.0]))))
    
    T0_start0 = dot(T0_wheelEndEffector, TwheelEndEffector_start0)
    h.append(misc.DrawAxes(env, T0_start0, 0.4))
    
    TwheelEndEffector_start1 = MakeTransform(matrix(rodrigues([-pi/2, 0, 0])),transpose(matrix([0.0, 0.0, 0.0])))

    TwheelEndEffector_start1 = dot(TwheelEndEffector_start1, MakeTransform(matrix(rodrigues([0, 0, -pi/2])),transpose(matrix([0.0, 0.0, 0.0]))))
    
    TwheelEndEffector_start1 = dot(TwheelEndEffector_start1,MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, -0.1, 0.0]))))

    T0_start1 = dot(T0_wheelEndEffector, TwheelEndEffector_start1)

    h.append(misc.DrawAxes(env, T0_start1, 0.4))

    T0_starts.append(array(T0_start0))
    T0_starts.append(array(T0_start1))

def get_push():
    pushTrajL = [array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0]))))]

    for d in frange(minDistanceBetweenHands,maxDistanceBetweenHands,delta1):
        for t in frange(minTrajLength,maxTrajLength,delta1):
            pushTrajL.append(array(MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([t, 0.0, 0.0])))))
    
    pushTrajR = deepcopy(pushTrajL)
    pass

def get_lift():
    pass

def get_cw_rotate():
    pass
