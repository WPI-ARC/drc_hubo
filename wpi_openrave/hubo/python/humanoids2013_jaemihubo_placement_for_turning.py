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

h = []



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

def get_pairs(TLH_RH, env, robot, myRmaps):

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

    resp = find_sister_pairs(myRmaps,[relBaseConstraint],[TLH_RH],env)
    
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
    myPattern.setColor(array((0,0,1,0.5))) 
    myPattern.show(env)
    # sys.stdin.readline()
    myPattern.hide("all")
    myPatterns.append(myPattern)

    # 3. Create a search pattern from the trajectory for the second manipulator
    myPattern = SearchPattern(rightTraj)
    myPattern.T0_p = T0_p
    myPattern.setColor(array((1,0,0,0.5))) 
    myPattern.show(env)
    # sys.stdin.readline()
    myPattern.hide("all")
    myPatterns.append(myPattern)

    for p in myPatterns:
        p.hide("spheres")
        #sys.stdin.readline()

    for p in myPatterns:
        p.hide("all")
        # sys.stdin.readline()

    
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

            # print "Playing valid solution #: ",str(colFreeSolIdx)
            # play(T0_starts, whereToFace, relBaseConstraint, candidates, numRobots, numManips, solIdx, myRmaps,[robot], h, env, 0.3, False, ' leftFootBase rightFootBase ')
            # h.pop() # delete the robot base axis we added last

            

            # currentIk = robot.GetDOFValues()

            # # Try to put your feet on the ground
            # print "trying to put the feet on the ground..."
            # myIK = put_feet_on_the_ground(robot, whereToFace, env)

            # print "myIK"
            # print myIK

            # if(myIK != ''):
            #     print "checking balance constraint..."
            #     robot.SetDOFValues(str2num(myIK), range(len(robot.GetJoints())))
            #     myCOM = array(get_robot_com(robot))
            #     myCOM[2,3] = 0.0
            #     COMHandle = misc.DrawAxes(env,myCOM,0.3)

            #     if(check_support(myCOM,robot)):
            #         print "FOUND A SOLUTION WHOOHOO!!!!"
            #         sys.stdin.readline()
                    
            #     else:
            #         print "COM is out of the support polygon"

            # robot.SetDOFValues(currentIk, range(len(robot.GetJoints())))
        
    return mySamples

if __name__ == '__main__':
    # Activates some prints and keyboard inputs
    debug = False
    
    # directory where we keep the data files
    myInitTimeStr = str(datetime.now())
    myPathStr = './humanoids2013_data/turning_'+myInitTimeStr
    os.mkdir(myPathStr)

    RaveSetDebugLevel(DebugLevel.Fatal)
    env = Environment()
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
    minRotAngle = pi/8
    maxRotAngle = pi/4
    delta1 = pi/8
    delta2 = None
    myLogger = BensLogger(arg_note=str(datetime.now()),arg_name=myPathStr+'/humanoids2013_jaemiPlanning_turning_samples')
    myLogger.header(['label','pitch','height','traj_length','dist_left_right','left_start_sphere_ind','left_start_sphere_x','left_start_sphere_y','left_start_sphere_z','right_start_sphere_ind','right_start_sphere_x','right_start_sphere_y','right_start_sphere_z','timestamp'])
    resultCount = 0
    for dist in TrajectoryGenerator.frange(0.1,0.5,0.1):        
        TLH_RH = MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, -dist, 0.0])))
        trajs = TrajectoryGenerator.get('jaemiPlanning', 'rotcw', minRotAngle,maxRotAngle,delta1,delta2,dist)
        #print trajs
        resp = get_pairs(TLH_RH, env, robot, myRmaps)
        if(resp != None):
            pairs = resp[0]
            rm = resp[1]
            [relBaseConstraint] = resp[2]
            for t in range(len(trajs[0])):
                leftTraj = trajs[0][t]
                rightTraj = trajs[1][t]
                candidates = get_candidates(leftTraj, rightTraj, env, robot, myRmaps, probs[0], pairs, rm)
                if(candidates != None):
                    for height in TrajectoryGenerator.frange(0.7,1.2,0.1):
                        for pitch in TrajectoryGenerator.frange(-pi/2,pi/2,pi/36):
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
                            
                            samples = run(candidates, leftTraj, rightTraj, env, robot, myRmaps, probs[0], pairs, rm, T0_LH, T0_RH, relBaseConstraint)
                            if(samples != []):
                                for nIdx, n in enumerate(samples):
                                    
                                    go_to_start_config_and_pose(robot,myRmaps,n,[T0_LH, T0_RH])

                                    myComMat = get_robot_com(robot).round(4)
                                    myComMat[2,3] = 0.0
                                    comHandle = misc.DrawAxes(env,array(myComMat.round(4)),0.1)

                                    checkSupportProblem = RaveCreateModule(env,'CBiRRT')

                                    try:
                                        env.AddModule(checkSupportProblem, robot.GetName()) # this string should match to <Robot name="" > in robot.xml
                                    except openrave_exception, e:
                                        print e
                                    
                                    print "CheckSupport: ",str(checkSupportProblem.SendCommand('CheckSupport supportlinks 2 leftFootBase rightFootBase draw'))
                                    print "CheckSelfCollision: ",str(robot.CheckSelfCollision())
                                    print "CheckEnvCollision: ",str(env.CheckCollision(robot))
                                    
                                    # sys.stdin.readline()
                                    

                                    ############# CBIRRT ####################
                                    wheel.SetDOFValues([0],[0])
                                    time.sleep(0.1)

                                    CTee = wheel.GetManipulators()[0].GetEndEffectorTransform()
                                    
                                    rotAng = minRotAngle+(t*delta1)
                                    
                                    T0_LH2 = dot(dot(dot(dot(CTee,temp1),temp2),MakeTransform(rodrigues([rotAng,0,0]),transpose(matrix([0,0,0])))),dot(linalg.inv(dot(dot(CTee,temp1),temp2)),robot.GetManipulators()[0].GetTransform()))
                                    
                                    T0_RH2 = dot(wheel.GetManipulators()[0].GetTransform(),dot(MakeTransform(rodrigues([0,0,rotAng]),transpose(matrix([0,0,0]))),dot(linalg.inv(wheel.GetManipulators()[0].GetTransform()),robot.GetManipulators()[1].GetTransform())))

                                    rotationGoalIK = get_rot_goalik(robot, T0_LH2, T0_RH2)

                                    if(debug):
                                        print "press enter to reset configs"
                                        sys.stdin.readline()

                                    wheel.SetDOFValues([0],[0])
                                    go_to_startik(robot, n[2])

                                    if(rotationGoalIK != None):
                                        TSRL = [dot(dot(CTee,temp1),temp2),
                                                dot(linalg.inv(dot(dot(CTee,temp1),temp2)),robot.GetManipulators()[0].GetTransform()),
                                                matrix([0,0,0,0,0,0,0,pi,0,0,0,0])]

                                        TSRR = [MakeTransform(rodrigues([tilt_angle_rad,0,0]),transpose(matrix([0,0,0]))),
                                                dot(linalg.inv(wheel.GetManipulators()[0].GetTransform()),robot.GetManipulators()[1].GetTransform()),
                                                matrix([0,0,0,0,0,0,0,0,0,0,0,0])]

                                        TSRChainStringTurning = get_tsr_chain_string(robot, TSRL, TSRR, wheel, 'crank', 'crank', 0)
                                        resultCount += 1
                                        trajName = myPathStr+'/humanoids2013_turningTraj_'+str(resultCount)+'_'+str(datetime.now())+'.txt'
                                        startikStr = n[2]
                                        myTrak = None
                                        wheel.SetDOFValues([0],[0])
                                        go_to_startik(robot, startikStr)
                                        # print startikStr
                                        # print robot.GetActiveDOFValues()
                                        time.sleep(0.1)
                                        myTraj = plan(env, robot, wheel, startikStr, rotationGoalIK, ' leftFootBase rightFootBase ', TSRChainStringTurning, trajName)
                                        
                                        if(myTraj != None):
                                            print "planning done."
                                            
                                            # Take a screenshot for the record
                                            viewer = env.GetViewer()
                                            viewer.SendCommand('SetFiguresInCamera 1') 
                                            scipy.misc.imsave(myPathStr+'/'+str(datetime.now())+'_turning_h-'+str(height)+'_p-'+str(pitch)+'_d-'+str(dist)+'_'+str(nIdx)+'_.jpg', viewer.GetCameraImage(1024,768,viewer.GetCameraTransform(),[1024,1024,512,384]))                           
                                            del viewer

                                            myLogger.save([0, # label: 0 for lift, 1 for push, 2 for rotate
                                                           pitch, # feature1: pitch angle of the object
                                                           height, # feature2: height of the object from the ground
                                                           round(minRotAngle+(t*delta1),3), # feature3: length of the trajectory
                                                           dist, # feature4: distance between hands
                                                           n[0][0].sIdx, # feature5: left reachability sphere index
                                                           round(myRmaps[0].map[n[0][0].sIdx].T[0][0,3],2), # feature6: left reachability sphere X (in manip base coords.)
                                                           round(myRmaps[0].map[n[0][0].sIdx].T[0][1,3],2), # feature7: left reachability sphere Y (in manip base coords.)
                                                           round(myRmaps[0].map[n[0][0].sIdx].T[0][2,3],2), # # feature8: left reachability sphere Z
                                                           n[1][0].sIdx, # feature9: right reachability sphere index
                                                           round(myRmaps[1].map[n[1][0].sIdx].T[0][0,3],2), # feature10: right reachability sphere X
                                                           round(myRmaps[1].map[n[1][0].sIdx].T[0][1,3],2), # feature11: right reachability sphere Y
                                                           round(myRmaps[1].map[n[1][0].sIdx].T[0][2,3],2), # feature12: right reachability sphere Z
                                                           resultCount]) # feature13: resultCount

                                            if(debug):
                                                print "press enter to execute the trajectory"
                                                sys.stdin.readline()

                                            wheel.SetDOFValues([0],[0])
                                            go_to_startik(robot, startikStr)
                                            execute(robot, wheel, myTraj)
                                        else:
                                            print "planning failed."

                                        if(debug):
                                            print "press enter to see the next solution."
                                            sys.stdin.readline()
                                        ############## END OF CBIRRT ##############
                                        del TSRChainStringTurning
                                        del myTraj

                                    else:
                                        startikStr = n[2]
                                        wheel.SetDOFValues([0],[0])
                                        go_to_startik(robot, startikStr)

                                    del myComMat
                                    del comHandle
                                    checkSupportProblem.SendCommand('ClearDrawn')
                                    env.Remove(checkSupportProblem)
                                    del checkSupportProblem
                                    
    
    env.Destroy()
    RaveDestroy()

