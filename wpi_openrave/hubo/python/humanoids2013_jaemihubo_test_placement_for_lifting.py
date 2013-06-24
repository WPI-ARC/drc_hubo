# Bener Suay, June 2013
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

# robot-placement common functions
from roplace_common import *
import TrajectoryGenerator
from bens_logger import *

# To save screen-shots
import scipy.misc

import csv

h = []

viewerOn = False

saveImg = False

def go_to_start_config_and_pose(myRobot, myRmaps, sol, T0_starts):
    for myManipulatorIndex in range(2):
         # Find where to move the base
        startSphereIndex = sol[myManipulatorIndex][0].sIdx
        startTransformIndex = sol[myManipulatorIndex][0].tIdx
        myRmaps[myManipulatorIndex].go_to(startSphereIndex,startTransformIndex)
        Tbase_start = myRmaps[myManipulatorIndex].map[startSphereIndex].T[startTransformIndex]
        T0_newManipPose = dot(T0_starts[myManipulatorIndex],linalg.inv(Tbase_start))
        myRobot.SetTransform(array(T0_newManipPose))
    
    startConfigStr = sol[2]
    myRobot.SetActiveDOFValues(str2num(startConfigStr))
    time.sleep(0.05)

def get_pair(TLH_RH, env, robot, myRmaps, sIdx):

    # h.append(misc.DrawAxes(env,array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))),1.0))

    gR = 0.0 # ground color
    gG = 0.0 # ground color
    gB = 0.0 # ground color

    ge = 5.0 # ground extension

    h.append(env.drawtrimesh(points=array(((0,0,0),(ge,0,0),(0,ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))
    h.append(env.drawtrimesh(points=array(((ge,0,0),(0,ge,0),(ge,ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))
    h.append(env.drawtrimesh(points=array(((0,0,0),(-ge,0,0),(0,-ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))
    h.append(env.drawtrimesh(points=array(((-ge,0,0),(0,-ge,0),(-ge,-ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))
    h.append(env.drawtrimesh(points=array(((0,0,0),(-ge,0,0),(0,ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))
    h.append(env.drawtrimesh(points=array(((-ge,0,0),(0,ge,0),(-ge,ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))
    h.append(env.drawtrimesh(points=array(((0,0,0),(ge,0,0),(0,-ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))
    h.append(env.drawtrimesh(points=array(((ge,0,0),(0,-ge,0),(ge,-ge,0))),
                             indices=None,
                             colors=array(((gR,gG,gB),(gR,gG,gB),(gR,gG,gB)))))

    # Define robot base constraint(s)
    # a) Bounds <type 'list'>
    #    xyz in meters, rpy in radians
    #    [1.0, 1.0, 1.0, 0.0, 0.0, 0.0] would mean, we allow robot bases
    #    to be 1 meter apart in each direction but we want them to have the 
    #    same rotation

    # Relative Base Constraints between the two robots
    #relBaseConstraint = MakeTransform() # This is type (b)
    relBaseConstraint = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))

    # Base Constraints of the first (master) robot in the world coordinate frame
    # Same as relative base constraints. First 3 elements are XYZ bounds and the last three elements determine the desired rotation matrix of the base of the master robot in world coords.
    masterBaseConstraint = [1.0, 1.0, 0.0, 0.0, 0.0, 0.0]

    # b) Exact transform <type 'numpy.ndarray'>
    # relBaseConstraint = dot(something, some_other_thing)

    # Try to find a valid candidate that satisfies
    # all the constraints we have (base location, collision, and configuration-jump)

    print "Ready to find sisters... ",str(datetime.now())

    resp = find_sister_pair(myRmaps,[relBaseConstraint],[TLH_RH],env,sIdx)
    
    # resp can be None or [pairs, rm, p]
    if resp == None:
        return resp
    else:
        return [resp[0], resp[1], [relBaseConstraint]]


def get_candidates(leftTraj, rightTraj, env, robot, myRmaps, myProblem, pairs, rm):
    
    T0_p = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))
    
    myPatterns = []
    # 2. Create a search pattern from the trajectory for the first manipulator
    myPattern = SearchPattern(leftTraj)
    myPattern.T0_p = T0_p
    myPatterns.append(myPattern)

    # 3. Create a search pattern from the trajectory for the second manipulator
    myPattern = SearchPattern(rightTraj)
    myPattern.T0_p = T0_p
    myPatterns.append(myPattern)

    
    candidates = None
    print "Ready to look for candidates... ",str(datetime.now())
    candidates = look_for_candidates(pairs, rm, myPatterns)
    return candidates
    
# This changes the height and the pitch angle of the wheel
def run(candidates, leftTraj, rightTraj, env, robot, myRmaps, myProblem, pairs, rm, T0_LH, T0_RH, relBaseConstraint):

    mySamples = []

    # Search for the pattern in the reachability model and get the results
    numRobots = 1

    # Keep the number of manipulators of the robots 
    # that are going to be involved in search() 
    # The length of this list should be the same with 
    # the number of robots involved.
    numManips = [2]

    T0_starts = []
    T0_starts.append(array(T0_LH))
    T0_starts.append(array(T0_RH))

    whereToFace = MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0,0])))

    # find how many candidates search() function found.
    # candidates[0] is the list of candidate paths for the 0th robot
    howMany = len(candidates[0])
    collisionFreeSolutions = [[],[]]
    valids = []
    
    for c in range(howMany):
        print "trying ",str(c)," of ",str(howMany)," candidates."
        [allGood, activeDOFStartConfigStr] = play(T0_starts, whereToFace, relBaseConstraint, candidates, numRobots, numManips, c ,myRmaps,[robot], h, env, 0.0, True, ' leftFootBase rightFootBase ')

        h.pop() # delete the robot base axis we added last
        # We went through all our constraints. Is the candidate valid?
        if(allGood):
            collisionFreeSolutions[0].append(c)
            collisionFreeSolutions[1].append(activeDOFStartConfigStr)
            findAPathEnds = time.time()
            # print "Success! Collision and configuration constraints met."
        else:
            print "Constraint(s) not met ."

    # Went through all candidates
    print "Found ",str(len(collisionFreeSolutions[0]))," collision-free solutions in ",str(howMany)," candidates."

    if (len(collisionFreeSolutions[0]) > 0) :
        print "FOUND COLLISION FREE SOLUTIONS!!! WHOOHOO!!!! WILL PLAY RESULTS!!!"
        # sys.stdin.readline()
        
        # Have we found at least 1 collision free path?
        for colFreeSolIdx, solIdx in enumerate(collisionFreeSolutions[0]):
            mySamples.append([candidates[0][solIdx],candidates[1][solIdx],collisionFreeSolutions[1][colFreeSolIdx]])
        
    return mySamples

if __name__ == '__main__':
    
    # Call this script from the terminal using the following command:
    # 
    # $ python humanoids2013_jaemihubo_test_placement_for_lifting test 
    # $ python humanoids2013_jaemihubo_test_placement_for_lifting validation
    #
    # 'validation' uses validation data and saves the results under data/timestamp-validation directory
    # 'test' uses test data and saves the results under dasta/timestamp-test directory
    purpose = sys.argv[1]

    # Activates some prints and keyboard inputs
    debug = False
    
    # directory where we keep the data files
    myInitTimeStr = str(datetime.now())
    myPathStr = './humanoids2013_data/lifting_'+purpose+'_'+myInitTimeStr
    os.mkdir(myPathStr)

    
    env = Environment()
    env.SetDebugLevel(DebugLevel.Fatal)

    if(viewerOn):
        env.SetViewer('qtcoin')

    # Add huboplus
    robot = env.ReadRobotURI('../../../openHubo/jaemi/humanoids2013.jaemiHubo.planning.robot.xml')
    env.Add(robot)         

    robot.SetActiveDOFs(range(6,36)) # Active joints: [torso yaw - RAR]. Don't include the fingers.
    
    wheel = env.ReadRobotURI('../../../../drc_common/models/driving_wheel_tiny.robot.xml')
    env.Add(wheel)
    
    # Rotate the wheel so it's oriented along world's X axis
    # Also move it towards -y so it stands between two hands
    wheelOffset = matrix([0,0,0])
    wheelRotation = matrix(rodrigues([0,0,pi/2]))
    T0_wheelBase = MakeTransform(wheelRotation,transpose(wheelOffset))
    wheel.SetTransform(array(T0_wheelBase))

    # Angle between the wheel's end effector (wheel) and the base (rotation shaft)
    tilt_angle_rad = acos(dot(linalg.inv(wheel.GetManipulators()[0].GetEndEffectorTransform()),wheel.GetLinks()[0].GetTransform())[1,1])

    temp1 = MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0])))
    temp2 = MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0])))

    lowerLimits, upperLimits = robot.GetDOFLimits()

    # Let's keep the rotation of the robot around it's Z-Axis in a variable...
    rotz=[]
    rotz.append(pi/3);

    # Define the transform and apply the transform
    shift_robot0 = MakeTransform(matrix(rodrigues([0,0,rotz[0]])),transpose(matrix([-2.5,0.0,0.95])))
    robot.SetTransform(array(shift_robot0))

    probs_cbirrt = RaveCreateModule(env,'CBiRRT')

    try:
        env.AddModule(probs_cbirrt,robot.GetName()) # this string should match to <Robot name="" > in robot.xml
    except openrave_exception, e:
        print e

    print "Getting Loaded Problems"
    probs = env.GetLoadedProblems()

    # Load the reachability map for left arm
    myRmaps = []
    rm = ReachabilityMap(robot,'leftArmManip')
    print "Loading reachability map for left arm..."
    rm.load("jaemi_left_n8_m12_awesome")
    myRmaps.append(rm)
    print "Left arm Reachability Map loaded.."

    # Do the same for right arm
    rm2 = ReachabilityMap(robot,'rightArmManip')
    print "Loading reachability map for right arm..."
    rm2.load("jaemi_right_n8_m12_awesome")
    print "Reachability map loaded for right arm."

    initialWheelTransform = wheel.GetTransform()    

    myRmaps.append(rm2)
    minLiftDist= 0.05
    maxLiftDist = 0.5
    delta1 = 0.05
    delta2 = 0.05

    tstamp = str(datetime.now())
    myLogger = BensLogger(arg_note = tstamp, arg_name=myPathStr+'/humanoids2013_jaemiPlanning_lifting_'+purpose+'_results')
    myLogger.open('a')
    myLogger.header(['label',
                     'test_pitch',
                     'test_height',
                     'test_traj_length',
                     'test_dist_left_right',
                     'test_left_start_x',
                     'test_left_start_y',
                     'test_left_start_z',
                     'nearest_left_sphere_distance',
                     'nearest_left_start_sphere_ind',
                     'successful_left_sphere_index',
                     'successful_left_sphere_rank',
                     'successful_left_sphere_x',
                     'successful_left_sphere_y',
                     'successful_left_sphere_z',
                     'right_start_sphere_ind',
                     'right_start_sphere_x',
                     'right_start_sphere_y',
                     'right_start_sphere_z',
                     'resultId',
                     'iterationToResult',
                     'timeToResult',
                     'planning_time'])
    
    if(purpose == "test"):
        myFailLogger = BensLogger(arg_note=tstamp, arg_name=myPathStr+'/humanoids2013_jaemiPlanning_lifting_'+purpose+'_failed_results')
        myFailLogger.open('a')
        myFailLogger.header(['test_pitch',
                             'test_height',
                             'test_traj_length',
                             'test_dist_raw',
                             'test_dist_round',
                             'test_left_start_x',
                             'test_left_start_y',
                             'test_left_start_z',
                             'nearest_left_sphere_distance',
                             'nearest_left_start_sphere_ind'])
    

    resultCount = 0

    robot.SetActiveDOFValues(zeros(robot.GetActiveDOF()).tolist())
    # Bend the knees
    robot.SetDOFValues([-0.3,0.6,-0.3],[32,33,34])
    robot.SetDOFValues([-0.3,0.6,-0.3],[26,27,28])

    with open('humanoids2013_lifting_'+purpose+'_points.csv', 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for r, data in enumerate(reader):
            
            test_pitch = float(data[0])
            test_height = float(data[1])
            test_traj_length = float(data[2])
            # We should find the closest distance to this value that is a multiple of 0.05
            fdist = float(data[3])
            fdistRem = round(fmod(fdist,0.05),2)

            if(fdistRem > 0.25):
                test_dist = round(0.05 + round(fdist-fdistRem,2))
            else:
                test_dist = round(fdist-fdistRem,2)

            test_x = float(data[4])
            test_y = float(data[5])
            test_z = float(data[6])

            dist = test_dist
            height = test_height
            pitch = test_pitch

            TLH_RH = MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, -dist, 0.0])))

            # With the following input variables, trajectory generator should generate 
            # only 1 trajectory of length test_traj_length
            traj = TrajectoryGenerator.get('jaemiPlanning', 'lift', test_traj_length,test_traj_length,1.0,0.05)

            [nearestSphereIdx, minDist] = find_nearest_reachability_sphere(test_x, test_y, test_z, myRmaps[0].map)

            # Note: tryTheseSpheres is a dictionary
            [sortedKeys, tryTheseSpheres] = get_ranked_neighbors(nearestSphereIdx, myRmaps[0])

            # tryTheseSpheres = [nearestSphereIdx]
            # tryTheseSpheres.extend(myRmaps[0].map[nearestSphereIdx].neighbors)

            stopTrying = False # This is for breaking out of trying the neighbors

            iterationCount = 0
            startTime = time.time()

            for sIdx in sortedKeys:
                
                iterationCount += 1
                # Note sIdx is a key (sphere index) and rank is a value between 0 and 3 (depending on its distance from the main sphere of interest) 
                rank = tryTheseSpheres[sIdx]

                resp = get_pair(TLH_RH, env, robot, myRmaps, sIdx)

                if(resp != None):
                    pairs = resp[0] # The same sphere may have more than 1 transforms
                    rm = resp[1]
                    [relBaseConstraint] = resp[2]
                    for t in range(len(traj[0])):
                        # print "t"
                        # print t
                        leftTraj = traj[0][t]
                        rightTraj = traj[1][t]

                        pathElements = get_candidates(leftTraj, rightTraj, env, robot, myRmaps, probs[0], pairs, rm)
                        if(pathElements != None):
                            # when the rotation around it's X axis is zero,
                            # the wheel is facing to the ground. 
                            # We add pi/2 to make it's zero the same with the hands
                            # Also when we say "SetTransform" it sets the transform
                            # of the base of the wheel, not the handle.
                            # There's a difference of 23 degrees between the handle 
                            # and the base, that's why we substract the tilt angle
                            wheelOffset = matrix([-0.5*dist,0.0,height])
                            wheelRotation = matrix(rodrigues([pitch+(pi/2)-tilt_angle_rad,0,0]))
                            T0_wheelBase = MakeTransform(wheelRotation,transpose(wheelOffset))
                            wheel.SetTransform(dot(initialWheelTransform,array(T0_wheelBase)))

                            # Where do we want the end effectors (hands) to start from in world coordinates?
                            T0_OBJ = MakeTransform(matrix(rodrigues([0, pitch, 0])),transpose(matrix([0.0, -dist*0.5, height])))
                            # T0_OBJ = wheel.GetManipulators()[0].GetEndEffectorTransform()

                            TOBJ_LH = MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, dist*0.5, 0.0])))
                            T0_LH = dot(T0_OBJ, TOBJ_LH)
                            # h.append(misc.DrawAxes(env, T0_LH, 0.4))

                            TOBJ_RH = MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, -dist*0.5, 0.0])))
                            T0_RH = dot(T0_OBJ, TOBJ_RH)
                            # h.append(misc.DrawAxes(env, T0_RH, 0.4))

                            samples = run(pathElements, leftTraj, rightTraj, env, robot, myRmaps, probs[0], pairs, rm, T0_LH, T0_RH, relBaseConstraint)
                            if(samples != []):
                                for nIdx, n in enumerate(samples):
                                    go_to_start_config_and_pose(robot,myRmaps,n,[T0_LH, T0_RH])


                                    wheel.SetDOFValues([0],[0])
                                    time.sleep(0.1)

                                    CTee = wheel.GetManipulators()[0].GetEndEffectorTransform()

                                    liftDist = test_traj_length

                                    T0_LH2 = dot(dot(dot(dot(CTee,temp1),temp2),MakeTransform(rodrigues([0,0,liftDist]),transpose(matrix([0,0,0])))),dot(linalg.inv(dot(dot(CTee,temp1),temp2)),robot.GetManipulators()[0].GetTransform()))
                                    
                                    T0_RH2 = dot(dot(dot(dot(CTee,temp1),temp2),MakeTransform(rodrigues([0,0,liftDist]),transpose(matrix([0,0,0])))),dot(linalg.inv(dot(dot(CTee,temp1),temp2)),robot.GetManipulators()[1].GetTransform()))

                                    liftingGoalIK = get_lin_goalik(robot, T0_LH2, T0_RH2)


                                    if(debug):
                                        print "press enter to reset configs"
                                        sys.stdin.readline()

                                    wheel.SetDOFValues([0],[0])
                                    go_to_startik(robot, n[2])

                                    if(liftingGoalIK != None):
                                        TSRL = [dot(dot(CTee,temp1),temp2),
                                                dot(linalg.inv(dot(dot(CTee,temp1),temp2)),robot.GetManipulators()[0].GetTransform()),
                                                matrix([0,0,0,0,-1000,1000,0,0,0,0,0,0])]

                                        TSRR = [MakeTransform(rodrigues([tilt_angle_rad,0,0]),transpose(matrix([0,0,0]))),
                                                dot(linalg.inv(wheel.GetManipulators()[0].GetTransform()),robot.GetManipulators()[1].GetTransform()),
                                                matrix([0,0,0,0,0,0,0,0,0,0,0,0])]

                                        TSRChainStringLifting = get_tsr_chain_string(robot, TSRL, TSRR, wheel, 'crank', 'crank', mimicObjectJoint=None)
                                        resultCount += 1
                                        trajName = myPathStr+'/humanoids2013_liftingTraj_'+str(resultCount)+'_'+str(datetime.now())+'.txt'
                                        startikStr = n[2]
                                        myTraj = None
                                        planningStart = time.time()
                                        myTraj = plan(env, robot, 'crank', startikStr, liftingGoalIK, ' leftFootBase rightFootBase ', TSRChainStringLifting, trajName, False)
                                        planningEnd = time.time()

                                        if(myTraj != None):
                                            print "planning done."

                                            # Take a screenshot for the record
                                            # uncomment this block if you'd like to save pngs of successful solutions
                                            if(viewerOn and saveImg):
                                                viewer = env.GetViewer()
                                                viewer.SendCommand('SetFiguresInCamera 1') 
                                                scipy.misc.imsave(myPathStr+'/'+str(datetime.now())+'_lifting_h-'+str(height)+'_p-'+str(pitch)+'_d-'+str(dist)+'_'+str(nIdx)+'_.jpg', viewer.GetCameraImage(1024,768,viewer.GetCameraTransform(),[1024,1024,512,384]))                           
                                                del viewer
                                            
                                            endTime = time.time()
                                            durationInSecs = endTime - startTime
                                            planningTime = planningEnd - planningStart
                                            myLogger.save([3, # label: 0 for lift, 1 for push, 2 for rotate, 3 for lift test, 4 for push test, 5 for rotate test
                                                           pitch,  
                                                           height, 
                                                           test_traj_length, 
                                                           dist, 
                                                           test_x,
                                                           test_y,
                                                           test_z,
                                                           round(minDist,3),
                                                           nearestSphereIdx,
                                                           n[0][0].sIdx, 
                                                           rank,
                                                           round(myRmaps[0].map[n[0][0].sIdx].T[0][0,3],2), 
                                                           round(myRmaps[0].map[n[0][0].sIdx].T[0][1,3],2), 
                                                           round(myRmaps[0].map[n[0][0].sIdx].T[0][2,3],2), 
                                                           n[1][0].sIdx, 
                                                           round(myRmaps[1].map[n[1][0].sIdx].T[0][0,3],2), 
                                                           round(myRmaps[1].map[n[1][0].sIdx].T[0][1,3],2), 
                                                           round(myRmaps[1].map[n[1][0].sIdx].T[0][2,3],2), 
                                                           resultCount,
                                                           iterationCount,
                                                           durationInSecs,
                                                           planningTime])
            
                                            if(debug):
                                                print "press enter to execute the trajectory"
                                                sys.stdin.readline()
                                           
                                            wheel.SetDOFValues([0],[0])
                                            robot.SetActiveDOFValues(zeros(robot.GetActiveDOF()).tolist())
                                            # Bend the knees
                                            robot.SetDOFValues([-0.3,0.6,-0.3],[32,33,34])
                                            robot.SetDOFValues([-0.3,0.6,-0.3],[26,27,28])
                                            # go_to_startik(robot, startikStr)
                                            # execute(robot, wheel, myTraj)
                                            stopTrying = True
                                            fail = False
                                        else:
                                            print "planning failed."
                                            wheel.SetDOFValues([0],[0])
                                            robot.SetActiveDOFValues(zeros(robot.GetActiveDOF()).tolist())
                                            # Bend the knees
                                            robot.SetDOFValues([-0.3,0.6,-0.3],[32,33,34])
                                            robot.SetDOFValues([-0.3,0.6,-0.3],[26,27,28])
                                            fail = True
                                        if(debug):
                                            print "press enter to see the next solution."
                                            sys.stdin.readline()
                                        del TSRChainStringLifting
                                        del myTraj
                                        
                                    else:
                                        fail = True
                                        startikStr = n[2]
                                        wheel.SetDOFValues([0],[0])
                                        robot.SetActiveDOFValues(zeros(robot.GetActiveDOF()).tolist())
                                        # Bend the knees
                                        robot.SetDOFValues([-0.3,0.6,-0.3],[32,33,34])
                                        robot.SetDOFValues([-0.3,0.6,-0.3],[26,27,28])
                                        # go_to_startik(robot, startikStr)

                            # samples == []
                            else:
                                fail = True

                        # pathElements == None
                        else:
                            fail = True

                # resp == None
                else: 
                    fail = True

                if(stopTrying):
                    break

            if(fail and (purpose == "test") ):
                myFailLogger.save([pitch,
                                   height, 
                                   test_traj_length, 
                                   fdist, # distance before being rounded to closest multiple of 0.05
                                   dist, # distance after being rounded up to closest multiple of 0.05
                                   test_x, 
                                   test_y,
                                   test_z,
                                   round(minDist,3),
                                   nearestSphereIdx])
                
    print resultCount
    myLogger.close()
    
    if(purpose == "test"):
        myFailLogger.close()

    try:
        env.Remove(probs_cbirrt)
        del probs_cbirrt
        del probs
        env.Destroy()
        RaveDestroy()
        del robot
        del wheel
        del myRmaps
        del rm
        del rm2
        del myLogger
        if(purpose == "test"):
            del myFailLogger
    except openrave_exception, e:
        print e




