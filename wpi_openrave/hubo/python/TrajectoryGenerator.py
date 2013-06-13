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
import RotationalTrajDiscretizer

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

def get_lift(minTrajLength, maxTrajLength, delta1, delta2):
    liftTrajsL = []
    for length in frange( minTrajLength, maxTrajLength, delta1):
        currentTraj = []

        for l in frange(0, length, delta2):
            currentTraj.append(array(MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, 0.0, l])))))

        liftTrajsL.append(currentTraj)

    liftTrajsR = deepcopy(liftTrajsL)
    return [liftTrajsL, liftTrajsR]
    

def get_rotate_cw( minRotAngle, maxRotAngle, delta1, delta2, r):
    rotTrajsL = [] # rotational trajectories for the left hand
    rotTrajsR = [] # rotational trajectories for the right hand
    for angle in frange( minRotAngle, maxRotAngle, delta1):
        currentTrajL = []
        currentTrajR = []        

        xyrL = RotationalTrajDiscretizer.get_discrete_xyr(r, pi, pi-angle, -0.01)
        rawL = xyrL[0]
        discreteL  = xyrL[1]
        #print "got discrete elements - L"
        for element in discreteL:            
            #print element
            adjustedY = -1.0*(element[0]+r)+0.0
            adjustedZ = element[1]
            adjustedR = (4.0 - element[2])*(pi/4.0)
            currentTrajL.append(array(MakeTransform(matrix(rodrigues([adjustedR, 0, 0])),transpose(matrix([0.0, adjustedY, adjustedZ])))))
            #print str(adjustedY), str(adjustedZ), str(adjustedR)


        xyrR = RotationalTrajDiscretizer.get_discrete_xyr(r, 0.0, -1.0*angle, -0.01)
        rawR = xyrR[0]
        discreteR  = xyrR[1]
        # print "got discrete elements - R"
        for element in discreteR:            
            # print element
            adjustedY = r-element[0]
            adjustedZ = element[1]
            adjustedR = (-1.0*element[2]+0.0)*(pi/4.0)
            # print str(adjustedY), str(adjustedZ), str(adjustedR)
            currentTrajR.append(array(MakeTransform(matrix(rodrigues([adjustedR, 0, 0])),transpose(matrix([0.0, adjustedY, adjustedZ])))))

        rotTrajsL.append(currentTrajL)
        rotTrajsR.append(currentTrajR)

    return [rotTrajsL, rotTrajsR]
            

def get(robotModel, trajType, mi, ma, d1, d2, r=None):
    # TODO change the index and rotation matrices depending on the robot model
    if(robotModel == 'jaemiPlanning' or robotModel == 'huboplus'):
        if(trajType == 'lift'):
            return get_lift(mi, ma, d1, d2)
        elif(trajType == 'push'):
            return get_push(mi, ma, d1, d2)
        elif(trajType == 'rotcw'):
            return get_rotate_cw(mi, ma, d1, d2, r)
