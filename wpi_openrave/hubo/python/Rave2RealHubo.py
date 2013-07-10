# Ben Suay, RAIL
# July 2013
# Worcester Polytechnic Institute
#

import openhubo
import openhubo.trajectory as achtraj
import openravepy
from copy import deepcopy
from numpy import *

def traj2ach(env,robot,traj,fname):

    myAchTraj=openravepy.RaveCreateTrajectory(robot.GetEnv(),'')
    config=deepcopy(traj.GetConfigurationSpecification())
    config.AddDeltaTimeGroup()
    myAchTraj.Init(config)

    trajLength = traj.GetNumWaypoints()
    numJoints = len(robot.GetJoints())
    
    freq = 1.0/25.0
    for i in range(trajLength):
        wp = traj.GetWaypoint(i)
        awp = deepcopy(wp)
        print "joint values"
        q = wp[0:numJoints]
        print q
        print "joint velocities"
        qdot = wp[numJoints:(2*numJoints)]
        print qdot
        print "deltatime"
        dt = wp[2*numJoints]
        print dt
        if(dt != 0.0):
            awp[2*numJoints] = 0.5

        myAchTraj.Insert(i,awp)

        # pose = openhubo.Pose(robot,controller)

        # print pose.jointmap
        # sys.stdin.readline()
        
        # pose['RHY'] = q[36]
        # pose['RHR'] = q[37]
        # pose['RHP'] = q[38]
        # pose['RKN'] = q[39]
        # pose['RAP'] = q[40]
        # pose['RAR'] = q[41]
        # pose['LHY'] = q[11]
        # pose['LHR'] = q[12]
        # pose['LHP'] = q[13]
        # pose['LKN'] = q[14]
        # pose['LAP'] = q[15]
        # pose['LAR'] = q[16]
        # pose['RSP'] = q[26]
        # pose['RSR'] = q[27]
        # pose['RSY'] = q[28]
        # pose['REB'] = q[29]
        # pose['RWY'] = q[30]
        # pose['RWR'] = q[32]
        # pose['RWP'] = q[31]
        # pose['LSP'] = q[0]
        # pose['LSR'] = q[1]
        # pose['LSY'] = q[2]
        # pose['LEB'] = q[3]
        # pose['LWY'] = q[4]
        # pose['LWR'] = q[6]
        # pose['LWP'] = q[5]
        # pose['NKY'] = q[17]
        # pose['NK1'] = q[18]
        # pose['NK2'] = q[19]
        # pose['WST'] = q[10]
        # pose['RF1'] = q[33]
        # pose['RF2'] = q[42]
        # pose['RF3'] = q[45]
        # pose['RF4'] = q[48]
        # pose['RF5'] = 0
        # pose['LF1'] = q[7]
        # pose['LF2'] = q[20]
        # pose['LF3'] = q[23]
        # pose['LF4'] = 0
        # pose['LF5'] = 0

    openravepy.planningutils.RetimeTrajectory(myAchTraj,True)
    achTrajLength = myAchTraj.GetNumWaypoints()
    
    fRaveRetimed = open(fname+'_retimed.txt','w')
    fRaveRetimed.write(myAchTraj.serialize(0))
    fRaveRetimed.close()

    f = open(fname+'.traj','w')

    for i in range(achTrajLength):
        q = myAchTraj.GetWaypoint(i)
        myAchQ = [q[36], q[37], q[38], q[39], q[40], q[41], q[11], q[12], q[13], q[14], q[15], q[16], q[26], q[27], q[28], q[29], q[30], q[32], q[31], q[0], q[1], q[2], q[3], q[4], q[6], q[5], q[17], q[18], q[19], q[10], q[33], q[42], q[45], q[48], 0, q[7], q[20], q[23],  0, 0]
        print myAchQ
        f.write(' '.join([str(x) for x in myAchQ])+'\n')

    f.close()
