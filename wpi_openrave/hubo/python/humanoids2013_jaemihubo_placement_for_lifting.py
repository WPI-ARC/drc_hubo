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
import huboplus_trajectory_generator
from bens_logger import *
h = []
def get_pairs(pitch, height, dist, env, robot, myRmaps):

    h.append(misc.DrawAxes(env,array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))),1.0))

    gR = 0.6 # ground color
    gG = 0.6 # ground color
    gB = 0.6 # ground color

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
    Tee = []
    manips = robot.GetManipulators()
    for i in range(len(manips)):
        # Returns End Effector Transform in World Coordinates
        Tlink = manips[i].GetEndEffectorTransform()
        Tee.append(Tlink)

    # Where do we want the end effectors (hands) to start from in world coordinates?
    T0_LIFT = MakeTransform(matrix(rodrigues([0, pitch, 0])),transpose(matrix([0.0, 0.0, height])))
    TLIFT_LH = MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, 0.0, 0.0])))
    T0_LH = dot(T0_LIFT, TLIFT_LH)
    h.append(misc.DrawAxes(env, T0_LH, 0.4))

    TLIFT_RH = MakeTransform(matrix(rodrigues([0, 0, 0])),transpose(matrix([0.0, -dist, 0.0])))
    T0_RH = dot(T0_LIFT, TLIFT_RH)
    h.append(misc.DrawAxes(env, T0_RH, 0.4))

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

    


    # Relative transform of initial grasp transforms
    TLH_RH = dot(linalg.inv(T0_LH),T0_RH)

    print "Ready to find sisters... ",str(datetime.now())

    resp = find_sister_pairs(myRmaps,[relBaseConstraint],[TLH_RH],env)
    
    # resp can be None or [pairs, rm, p]
    if resp == None:
        return resp
    else:
        return [resp[0], resp[1], [relBaseConstraint], [TLH_RH], T0_LH, T0_RH]

# This changes the height and the pitch angle of the wheel
def run(leftTraj, rightTraj, env, robot, myRmaps, myProblem, pairs, rm, T0_LH, T0_RH):

    # Search for the pattern in the reachability model and get the results
    numRobots = 1

    # Keep the number of manipulators of the robots 
    # that are going to be involved in search() 
    # The length of this list should be the same with 
    # the number of robots involved.
    numManips = [2]

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

    T0_starts = []
    T0_starts.append(array(T0_LH))
    T0_starts.append(array(T0_RH))

    whereToFace = MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0,0])))

    mySamples = []
    candidates = None
    print "Ready to look for candidates... ",str(datetime.now())
    candidates = look_for_candidates(pairs, rm, myPatterns)
    if(candidates != None):
        # find how many candidates search() function found.
        # candidates[0] is the list of candidate paths for the 0th robot
        howMany = len(candidates[0])
        collisionFreeSolutions = [[],[]]
        valids = []
        for c in range(howMany):
            print "trying ",str(c)," of ",str(howMany)," candidates."
            [allGood, activeDOFConfigStr] = play(T0_starts, whereToFace, relBaseConstraint, candidates, numRobots, numManips, c ,myRmaps,[robot], h, env, 0.0, True, ' leftFootBase rightFootBase ')

            h.pop() # delete the robot base axis we added last
            # We went through all our constraints. Is the candidate valid?
            if(allGood):
                collisionFreeSolutions[0].append(c)
                collisionFreeSolutions[1].append(activeDOFConfigStr)
                findAPathEnds = time.time()
                print "Success! Collision and configuration constraints met."
            else:
                print "Constraint(s) not met ."

        # Went through all candidates
        print "Found ",str(len(collisionFreeSolutions[0]))," collision-free solutions in ",str(howMany)," candidates."

        if (len(collisionFreeSolutions[0]) > 0) :
            # Have we found at least 1 collision free path?
            for colFreeSolIdx, solIdx in enumerate(collisionFreeSolutions[0]):
                print "Playing valid solution #: ",str(colFreeSolIdx)
                play(T0_starts, whereToFace, relBaseConstraint, candidates, numRobots, numManips, solIdx, myRmaps,[robot], h, env, 0.3, False, ' leftFootBase rightFootBase ')
                h.pop() # delete the robot base axis we added last

                currentIk = robot.GetDOFValues()

                # Try to put your feet on the ground
                print "trying to put the feet on the ground..."
                myIK = put_feet_on_the_ground(myProblem, robot, whereToFace, lowerLimits, upperLimits, env)

                print "myIK"
                print myIK

                if(myIK != ''):
                    print "checking balance constraint..."
                    robot.SetDOFValues(str2num(myIK), range(len(robot.GetJoints())))
                    myCOM = array(get_robot_com(robot))
                    myCOM[2,3] = 0.0
                    COMHandle = misc.DrawAxes(env,myCOM,0.3)

                    if(check_support(myCOM,robot)):    
                        mySamples.append([candidates[0][solIdx],candidates[1][solIdx]])
                    else:
                        print "COM is out of the support polygon"

                robot.SetDOFValues(currentIk, range(len(robot.GetJoints())))
        
    return mySamples

if __name__ == '__main__':
    env = Environment()
    env.SetViewer('qtcoin')

    # Add huboplus
    robot = env.ReadRobotURI('../../../openHubo/jaemi/humanoids2013.jaemiHubo.planning.robot.xml')
    env.Add(robot)         

    robots[0].SetActiveDOFs(range(6,66))

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
    rm = ReachabilityMap("./rlhuboplus_leftArm_ik_solver",robot,robot.GetManipulators()[0])
    print "Loading reachability map for left arm..."
    rm.load("rlhuboplus_left_m12")
    myRmaps.append(rm)
    print "Left arm Reachability Map loaded.."

    # Do the same for right arm
    rm2 = ReachabilityMap("./rlhuboplus_rightArm_ik_solver",robot,robot.GetManipulators()[1])
    print "Loading reachability map for right arm..."
    rm2.load("rlhuboplus_right_m12")
    print "Reachability map loaded for right arm."
    myRmaps.append(rm2)
    minTrajLength = 0.05
    maxTrajLength = 0.25
    delta1 = 0.1
    delta2 = 0.05
    trajs = huboplus_trajectory_generator.get_lift(minTrajLength,maxTrajLength,delta1,delta2)
    myLogger = BensLogger(arg_note=str(datetime.now()),arg_name='huboplus_lift_samples')
    myLogger.header(['label','pitch','height','traj_length','dist_left_right','left_start_sphere_ind','left_start_sphere_x','left_start_sphere_y','left_start_sphere_z','right_start_sphere_ind','right_start_sphere_x','right_start_sphere_y','right_start_sphere_z'])
    for pitch in huboplus_trajectory_generator.frange(-pi/2,pi/2,pi/6):
        for height in huboplus_trajectory_generator.frange(0.75,1.25,0.25):
            for dist in huboplus_trajectory_generator.frange(0.05,0.45,0.2):
                resp = get_pairs(pitch, height, dist, env, robot, myRmaps)
                if(resp != None):
                    pairs = resp[0]
                    rm = resp[1]
                    [relBaseConstraint] = resp[2]
                    [TLH_RH] = resp[3]
                    T0_LH = resp[4]
                    T0_RH = resp[5]
                    for t in range(len(trajs[0])):
                        leftTraj = trajs[0][t]
                        rightTraj = trajs[1][t]
                        samples = run(leftTraj, rightTraj, env, robot, myRmaps, probs[0], pairs, rm, T0_LH, T0_RH)
                        if(samples != []):
                            for n in samples:
                                # label: 0 for lift, 1 for push, 2 for rotate
                                # feature1: pitch angle of the object
                                # feature2: height of the object from the ground
                                # feature3: length of the trajectory
                                # feature4: distance between hands
                                # feature5: left reachability sphere index
                                # feature6: left reachability sphere X
                                # feature7: left reachability sphere Y
                                # feature8: left reachability sphere Z
                                # feature9: right reachability sphere index
                                # feature10: right reachability sphere X
                                # feature11: right reachability sphere Y
                                # feature12: right reachability sphere Z
                                myLogger.save([0, pitch, height, minTrajLength+(t*delta1), dist, n[0][0].sIdx, T0_LH[0,3], T0_LH[1,3], T0_LH[2,3], n[1][0].sIdx], T0_RH[0,3], T0_RH[1,3], T0_RH[2,3])
    
    env.Destroy()
    RaveDestroy()

