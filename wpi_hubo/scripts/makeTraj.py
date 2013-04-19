#!/usr/bin/env python

import tab
from openravepy import *
from numpy import *
from numpy.linalg import *
import sys
import time
from copy import copy
import openhubo
#TODO: Work with the concept of activeDOF?
def my_range(start, end, step):
    while start <= end:
        yield start
        start += step

for x in my_range(1, 10, 0.5):
    print x
def createTrajectory(robot):
    """ Create a trajectory based on a robot's config spec"""
    traj=RaveCreateTrajectory(robot.GetEnv,'')
    config=robot.GetConfigurationSpecification()
    config.AddDeltaTimeGroup()
    traj.Init(config)
    return traj

def loadTraj(robot,filename):
#dan    traj=RaveCreateTrajectory(robot.GetEnv(),'')
    traj=RaveCreateTrajectory(robot.GetEnv(),'')
    with open(filename,'r') as f:
        data=f.read()

    traj.deserialize(data)
    planningutils.RetimeActiveDOFTrajectory(traj,robot,True)
    return traj

""" Simple test script to run some of the functions above. """
if __name__=='__main__':
    try:
        input_file_name = sys.argv[1]
    except IndexError:
        input_file_name = 'movetraj0.txt'
        
    try:
        file_env = sys.argv[2]
        
    except IndexError:
        file_env = 'huboplus/rlhuboplus.robot.xml'
        #file_env = 'huboplus/huboplus.robot.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)

    timestep=0.01

    #-- Set the robot controller and start the simulation
    with env:
        env.StopSimulation()
        env.Load(file_env)
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)
        #Clear physics for kinematic trajectory playback
        env.SetPhysicsEngine(RaveCreatePhysicsEngine(env,'GenericPhysicsEngine'))
        robot = env.GetRobots()[0]
        #Create a "shortcut" function to translate joint names to indices
        ind = openhubo.makeNameToIndexConverter(robot)

        #initialize the servo controller
        controller=RaveCreateController(env,'achcontroller')
        robot.SetController(controller)
        controller.SendCommand("SetCheckCollisions false")

        #Set an initial pose before the simulation starts
        robot.SetDOFValues([pi/8,-pi/8],[ind('LSR'),ind('RSR')])
        time.sleep(1)

        #Use the new SetDesired command to set a whole pose at once.
        pose=array(zeros(robot.GetDOF()))

        #Manually align the goal pose and the initial pose so the thumbs clear
        pose[ind('RSR')]=-pi/8
        pose[ind('LSR')]=pi/8

        controller.SetDesired(pose)

    #The name-to-index closure makes it easy to index by name 
    # (though a bit more expensive)

    pose0=robot.GetDOFValues()
    pose1=pose0.copy()

    pose1[ind('LAP')]=-20
    pose1[ind('RAP')]=-20

    pose1[ind('LKP')]=40
    pose1[ind('RKP')]=40

    pose1[ind('LHP')]=-20
    pose1[ind('RHP')]=-20
    pose1=pose1*pi/180*2

#dan    traj=RaveCreateTrajectory(env,'')
#    traj=loadTraj(robot,'movetraj0.txt')
#    traj=loadTraj(robot,'movetraj1.txt')
#    traj=loadTraj(robot,'movetraj2.txt')
#    traj=loadTraj(robot,'movetraj3.txt')
    traj=loadTraj(robot,input_file_name)
    #Set up basic parameters
    config=robot.GetConfigurationSpecification()
    config.AddDeltaTimeGroup()

#dan    traj.Init(config)

    t0=0
    t1=2

    waypt0=list(pose0)
    waypt1=list(pose1)

    waypt0.extend(zeros(7))
    waypt1.extend(zeros(7))

    waypt0.append(t0)
    waypt1.append(t1)
    waypt2=copy(waypt0)
    waypt2[-1]=t1;

#dan    traj.Insert(0,waypt0)
#dan    traj.Insert(1,waypt1)
#dan    traj.Insert(2,waypt2)
#dan    traj.Insert(3,waypt1)
#dan    traj.Insert(4,waypt2)

    # set initial waypoint to zero
    wp = traj.GetWaypoint(0)
##    print traj.GetWaypoint(0)
##    print traj.GetWaypoint(1)
    lwp = len(wp)-2
#    for i in range(0,lwp-1):
#        wp[i] = 0.0
#    
#    wp[lwp] = 1.0
    #traj.Insert(0,wp)
 
##    print traj.GetWaypoint(0)
##    print traj.GetWaypoint(1)

    jntN = []
    jntV = []

    for i in range(0,lwp):
        jntN.append(i)
        jntV.append(wp[i])
    print "len = ", lwp
#    robot.SetDOFValues(jntV, jntN)
#    robot.SetDOFValues([1],[1])


    robot.SetDOFValues([0],[15])
    robot.SetDOFValues([0],[16])
    robot.SetDOFValues([-0.95],[19])
    robot.SetDOFValues([-0.95],[20])

    planningutils.RetimeActiveDOFTrajectory(traj,robot,True)

    #Prove that the retiming actually works
    #for k in range(40):
        #data=traj.Sample(float(k)/10)
        #print data[ind('LKP')]
    controller.SendCommand('SetRecord 1 timedata.txt')
    time.sleep(1)

    tmp1 = traj.GetConfigurationSpecification()
    print "Config Spec = ", tmp1

    tmp = traj.Sample(0.1)
    print "tmp = ", tmp

    fp = open('out.traj','w')

    print 'ind = ',tmp[ind('RHY')]

    for i in my_range(0,1.8,0.01):
        tmp = traj.Sample(i)
        fp.write(str(tmp[ind('RHY')]))
        fp.write(' ')
        fp.write(str(tmp[ind('RHR')]))
        fp.write(' ')
        fp.write(str(tmp[ind('RHP')]))
        fp.write(' ')
        fp.write(str(tmp[ind('RKP')]))
        fp.write(' ')
        fp.write(str(tmp[ind('RAP')]))
        fp.write(' ')
        fp.write(str(tmp[ind('RAR')]))
        fp.write(' ')
        fp.write(str(tmp[ind('LHY')]))
        fp.write(' ')
        fp.write(str(tmp[ind('LHR')]))
        fp.write(' ')
        fp.write(str(tmp[ind('LHP')]))
        fp.write(' ')
        fp.write(str(tmp[ind('LKP')]))
        fp.write(' ')
        fp.write(str(tmp[ind('LAP')]))
        fp.write(' ')
        fp.write(str(tmp[ind('LAR')]))
        fp.write(' ')
        fp.write(str(tmp[ind('RSP')]))
        fp.write(' ')
        fp.write(str(tmp[ind('RSR')]))
        fp.write(' ')
        fp.write(str(tmp[ind('RSY')]))
        fp.write(' ')
        fp.write(str(tmp[ind('REP')]))
        fp.write(' ')
        fp.write(str(tmp[ind('RWY')]))
        fp.write(' ')
        #fp.write(str(tmp[ind('RWR')]))
        fp.write(str(0.0))
        fp.write(' ')
        fp.write(str(tmp[ind('RWP')]))
        fp.write(' ')
        fp.write(str(tmp[ind('LSP')]))
        fp.write(' ')
        fp.write(str(tmp[ind('LSR')]))
        fp.write(' ')
        fp.write(str(tmp[ind('LSY')]))
        fp.write(' ')
        fp.write(str(tmp[ind('LEP')]))
        fp.write(' ')
        fp.write(str(tmp[ind('LWY')]))
        fp.write(' ')
        #fp.write(str(tmp[ind('LWR')]))
        fp.write(str(0.0))
        fp.write(' ')
        fp.write(str(tmp[ind('LWP')]))
        fp.write(' ')
        #fp.write(str(tmp[ind('NKY')]))
        fp.write(str(0.0))
        fp.write(' ')
        #fp.write(str(tmp[ind('NK1')]))
        fp.write(str(0.0))
        fp.write(' ')
        #fp.write(str(tmp[ind('NK2')]))
        fp.write(str(0.0))
        fp.write(' ')
        fp.write(str(tmp[ind('HPY')]))
        fp.write(' ')
        #fp.write(str(tmp[ind('RF1')]))
        fp.write(str(0.0))
        fp.write(' ')
        #fp.write(str(tmp[ind('RF2')]))
        fp.write(str(0.0))
        fp.write(' ')
        #fp.write(str(tmp[ind('RF3')]))
        fp.write(str(0.0))
        fp.write(' ')
        #fp.write(str(tmp[ind('RF4')]))
        fp.write(str(0.0))
        fp.write(' ')
        #fp.write(str(tmp[ind('RF5')]))
        fp.write(str(0.0))
        fp.write(' ')
        #fp.write(str(tmp[ind('LF1')]))
        fp.write(str(0.0))
        fp.write(' ')
        #fp.write(str(tmp[ind('LF2')]))
        fp.write(str(0.0))
        fp.write(' ')
        #fp.write(str(tmp[ind('LF3')]))
        fp.write(str(0.0))
        fp.write(' ')
        #fp.write(str(tmp[ind('LF4')]))
        fp.write(str(0.0))
        fp.write(' ')
        #fp.write(str(tmp[ind('LF5')]))
        fp.write(str(0.0))

        fp.write('\n')

    print "done writing file"

    env.Destroy()
    exit()
