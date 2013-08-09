#!/usr/bin/env python
# Ben Suay, RAIL
# July 2013
# Worcester Polytechnic Institute
#

# http://openrave.org/docs/latest_stable/command_line_tools/
# openrave-robot.py /your/path/to/your.robot.xml --info=joints
# On that page you can find more examples on how to use openrave-robot.py.

from openravepy import *
import roslib
roslib.load_manifest("hubo_planner")
import sys
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
    import numpy
import time
from wpi_planning_utilities.rodrigues import *
from wpi_planning_utilities.TransformMatrix import *
from wpi_planning_utilities.str2num import *
from wpi_planning_utilities.TSR import *
from math import *
from copy import *
import os # for file operations
from base_wheel_turning import *
import rave2realhubo

class ConstrainedPath():
    def __init__(self, myName):
        self.name = myName
        self.elements = []
        self.valveType = None
        
    def PlayInOpenRAVE(self):
        for e in self.elements:
            [success, why] = e.PlayInOpenRAVE()
            if(not success):
                return why

        return [True, ""]

    def GetOpenRAVETrajectory(self, robot, filepath):
        myPath = [] # a list of configurations for the whole path
        for e in self.elements:
            # populate the path with the list of configurations
            # of each path element.
            listOfQs = e.GetOpenRAVETrajectory(robot, filepath)
            myPath.extend(listOfQs)

        return myPath

class ConstrainedPathElement():
    def __init__(self, myName):
        self.name = myName
        self.startik = None
        self.goalik = None
        self.TSR = None
        self.smoothing = None
        self.psample = None
        self.mimicdof = None
        self.filename = None
        self.errorCode = None
        # The following attributes are 
        # lists. And they will be used
        # when playing trajectories with
        # cbirrt 'traj' command
        self.cbirrtProblems = None
        self.cbirrtRobots = None
        self.cbirrtTrajectories = None
        
        # The following attributes
        # will auto-generate hand
        # trajectories 
        self.openHandsBefore = False
        self.openHandsAfter = False
        self.closeHandsBefore = False
        self.closeHandsAfter = False
        self.hands = None

    def GetOpenRAVETrajectory(self, robot, filepath):
        myPathElementQs = []
        traj = RaveCreateTrajectory(robot.GetEnv(),'').deserialize(open(filepath+self.filename+'.txt','r').read())
        cs = traj.GetConfigurationSpecification()
        robotJointValsGroup = cs.GetGroupFromName("joint_values "+robot.GetName())
        trajLength = traj.GetNumWaypoints()

        wpFirst = traj.GetWaypoint(0)
        qFirst = wpFirst[robotJointValsGroup.offset:(robotJointValsGroup.offset+robotJointValsGroup.dof)]

        wpLast = traj.GetWaypoint(trajLength-1)
        qLast = wpLast[robotJointValsGroup.offset:(robotJointValsGroup.offset+robotJointValsGroup.dof)]

        for jIdx, joint in enumerate(robot.GetJoints()):
            if ( joint.GetName()[0:3] == 'LF1' ):
                leftHandFinger1Idx = jIdx
            if ( joint.GetName()[0:3] == 'RF1' ):
                rightHandFinger1Idx = jIdx
            if ( joint.GetName()[0:3] == 'RF2' ):
                rightHandFinger2Idx = jIdx

        freq = 25 # Play speed in Hz.
        howManySeconds = 6 # Play time in sec.
        howManyTimes = howManySeconds*freq
                
        # This is where we open/close the hands before the trajectory
        if( self.openHandsBefore ):
            qOpenHandsBefore = deepcopy(qFirst)
            qOpenHandsBefore[leftHandFinger1Idx] = -1.45
            qOpenHandsBefore[rightHandFinger1Idx] = -1.45
            qOpenHandsBefore[rightHandFinger2Idx] = -1.45
            for i in range(howManyTimes):
                myPathElementQs.append(qOpenHandsBefore)

        if( self.closeHandsBefore ):
            qCloseHandsBefore = deepcopy(qFirst)
            qCloseHandsBefore[leftHandFinger1Idx] = 0.1
            qCloseHandsBefore[rightHandFinger1Idx] = 0.1
            qCloseHandsBefore[rightHandFinger2Idx] = 0.1
            for i in range(howManyTimes):
                myPathElementQs.append(qCloseHandsBefore)

        # This is where we get the trajectory
        for i in range(trajLength):
             wp = traj.GetWaypoint(i)
             q = wp[robotJointValsGroup.offset:(robotJointValsGroup.offset+robotJointValsGroup.dof)]
             myPathElementQs.append(q)

             
        # This is where we open/close the hands after the trajectory
        if( self.openHandsAfter ):
            qOpenHandsAfter = deepcopy(qLast)
            qOpenHandsAfter[leftHandFinger1Idx] = -1.45
            qOpenHandsAfter[rightHandFinger1Idx] = -1.45
            qOpenHandsAfter[rightHandFinger2Idx] = -1.45
            for i in range(howManyTimes):
                myPathElementQs.append(qOpenHandsAfter)

        if( self.closeHandsAfter ):
            qCloseHandsAfter = deepcopy(qLast)
            qCloseHandsAfter[leftHandFinger1Idx] = 0.1
            qCloseHandsAfter[rightHandFinger1Idx] = 0.1
            qCloseHandsAfter[rightHandFinger2Idx] = 0.1
            for i in range(howManyTimes):
                myPathElementQs.append(qCloseHandsAfter)

        return myPathElementQs

    def PlayInOpenRAVE(self):
        # play this path element
        # in openrave for
        # confirmation / error check
        
        answers = []

        if(len(self.cbirrtProblems) != len(self.cbirrtTrajectories)):
            return [False, "Error: size of problem - trajectory mismatch."]
        else:
            try:
                for i in range(len(self.cbirrtTrajectories)):
                    answers.append(self.cbirrtProblems[i].SendCommand('traj '+self.cbirrtTrajectories[i]+'.txt'))
                    print "traj call answer: ",str(answers[i])

                for r in self.cbirrtRobots:
                    r.WaitForController(0)

            except openrave_exception, e:
                print e
                return [False, "OS exception in PlayTrajectory."]

            for r in self.cbirrtRobots:
                r.GetController().Reset(0)

        time.sleep(2)
        return [True, ""]

class DrcHuboV2WheelTurning( BaseWheelTurning ):

    def __init__(self,
                 HuboModelPath = roslib.packages.get_pkg_dir("drchubo_v2")+'/robots/drchubo-v2.robot.xml',
                 WheelModelPath = roslib.packages.get_pkg_dir("wpi_drc_sim")+'/../models/driving_wheel_tiny.robot.xml' ):

        BaseWheelTurning.__init__( self, HuboModelPath, WheelModelPath )        
        
        # 0: Not initialized
        # 1: Initialized
        # 2: At init
        # 3: Ready to turn
        # 4: Started turning
        # 5: Finished turning
        # 6: Finished task
        self.state = 0
        
        self.T0_LH1 = None
        self.T0_RH1 = None
        self.initik = None
        self.homeik = None
        
        # Set those variables to show or hide the interface
        # Do it using the member functions
        self.StopAtKeyStrokes = False
        self.ShowUserInterface = False
        self.ViewerStarted = False

        # Right Hand Joints 
        # Open - Closed Values
        self.lhanddofs = [7,20,23]
        self.lhandclosevals = [-0.15, -0.15, -0.15]
        self.lhandopenvals = [-1.490, -1.490, -1.490]

        # Left Hand Joints
        self.rhanddofs = [33,42,45,48]
        self.rhandclosevals = [-0.15, -0.15, -0.15, -0.15]
        self.rhandopenvals = [-1.490, -1.490, -1.490, -1.490]

        self.bothhandscloseval = -0.15
        self.bothhandsopenval = -1.490
        
        self.GenerateJointDict()

        self.optPlay = False
        self.optTaskWall = False
        self.optWall = False
        self.optDemo = False

        self.useIKFast = True
        
    def GenerateJointDict(self):
        self.jointDict = {}
        for jIdx, j in enumerate(self.robotid.GetJoints()):
            self.jointDict[j.GetName()] = jIdx

    def SetProblems(self):
        self.probs_cbirrt = RaveCreateModule(self.env,'CBiRRT')
        self.probs_crankmover = RaveCreateModule(self.env,'CBiRRT')

        try:
            self.env.AddModule(self.probs_cbirrt,self.robotid.GetName())
            self.env.AddModule(self.probs_crankmover,self.crankid.GetName())
        except openrave_exception, e:
            print e

        print "Getting Loaded Problems"
        self.probs = self.env.GetLoadedProblems()

    def SetHandDOFs(self,hand,vals):
        if(hand == "LH"):
            self.robotid.SetDOFValues(multiply(ones(len(self.lhanddofs)),vals),self.lhanddofs)
        elif(hand == "RH"):
            self.robotid.SetDOFValues(multiply(ones(len(self.rhanddofs)),vals),self.rhanddofs)
        elif(hand == "BH"):
            self.robotid.SetDOFValues(multiply(ones(len(self.lhanddofs)),vals),self.lhanddofs)
            self.robotid.SetDOFValues(multiply(ones(len(self.rhanddofs)),vals),self.rhanddofs)

    def OpenHands(self,hand,fname="openHandsHere"):
        # Wait 4 seconds to fully open the hands when exporting for ach
        waitThisMuch = 6.0 # seconds.
        rave2realhubo.openHandsHere(self.robotid,25.0,waitThisMuch,hand,fname)
        print "waiting for "+hand+" to open..."
        # time.sleep(waitThisMuch)
        self.SetHandDOFs(hand,self.bothhandsopenval)
        

    def CloseHands(self,hand,fname="closeHandsHere",fake=False):
        # Wait 4 seconds to fully close the hands when exporting for ach
        waitThisMuch = 6.0 # seconds.
        rave2realhubo.closeHandsHere(self.robotid,25.0,waitThisMuch,hand,fname)
        print "waiting for "+hand+" to close..."
        # time.sleep(waitThisMuch)
        if(fake):
            self.SetHandDOFs(hand,self.bothhandsopenval)
        else:
            self.SetHandDOFs(hand,self.bothhandscloseval)

    def AvoidSingularity(self,robot):
        # This function sets the robot's joints to values
        # that are close to zero as much as possible, while
        # taking the joint limits into consideration
        for jIdx, j in enumerate(robot.GetJoints()):
            lims = j.GetLimits()
            if(lims[1] > 0.0):
                robot.SetDOFValues([0.001],[jIdx])
            else:
                robot.SetDOFValues([-0.001],[jIdx])

        robot.GetController().Reset(0)

    def IKFast(self, manipname, T, allSolutions=True):
        self.robotid.SetActiveManipulator(manipname)
        ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robotid,iktype=IkParameterizationType.Transform6D)
        ikmodel.load()

        # check if ik solution(s) exist
        if(allSolutions):
            return ikmodel.manip.FindIKSolutions(array(T),IkFilterOptions.CheckEnvCollisions)
        else:
            return ikmodel.manip.FindIKSolution(array(T),IkFilterOptions.CheckEnvCollisions)

    # Plans a trajectory using a CBiRRT problem and goaljoints (active dofs)
    def PlanTrajectory(self, q_start, q_target, TSRChainString, smoothingitrs, error_code_str, mimicdof=None,psample=None):
        
        if(type(q_start) == type("")):
            q_start = str2num(q_start)

        # First convert q_target to numbers
        if(type(q_target) == type("")):
            goaljoints = deepcopy(str2num(q_target))
        else:
            goaljoints = deepcopy(q_target)

        self.robotid.SetActiveDOFValues(q_start)
        self.robotid.GetController().Reset(0)
        time.sleep(2)

        # Then add extra dofs for each TSRMimicDOF
        if(mimicdof is not None):
            for i in range(mimicdof):
                goaljoints = append(goaljoints, [0], 0)        
        
        cmdStr = 'RunCBiRRT '

        try:
            if(psample != None):
                cmdStr += 'psample '+str(psample)+' '

            cmdStr += 'supportlinks 2 '+self.footlinknames+' smoothingitrs '+str(smoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainString

            answer = self.probs_cbirrt.SendCommand(cmdStr)

            print "RunCBiRRT answer: ",str(answer)
            if(answer[0] != '1'):
                return [False, error_code_str+" - "+answer[1:]]
        except openrave_exception, e:
            print "Cannot send command RunCBiRRT: "
            print e
            return [False, "CBiRRT Plug-in Exception."]

        return [True, ""]

    def ExportTraj2RealHubo(self,trajfilename):
        traj = RaveCreateTrajectory(self.env,'').deserialize(open(trajfilename+'.txt','r').read())
        cs = traj.GetConfigurationSpecification()
        drchuboJointValsGroup = cs.GetGroupFromName("joint_values "+self.robotid.GetName())
        drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities "+self.robotid.GetName())
        deltatimeGroup = cs.GetGroupFromName("deltatime")
        rave2realhubo.traj2ach(self.env,self.robotid,traj,trajfilename,drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
    
    def RenameTrajectory(self,src,dst):
        try:
            os.rename(src,dst)
        except OSError, e:
            # No file cmovetraj
            print e
            return [False, "OS exception in RenameTrajectory."]

        return [True, ""]
    
    def EndTask(self, hands, valveType):
        # Wherever you are,
        currentik = self.robotid.GetActiveDOFValues()

        # Set the TSRs for current --> initik
        # Left Hand
        TSRStringLH1 = SerializeTSR(0,'NULL',self.robotManips[0].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Hand
        TSRStringRH1 = SerializeTSR(1,'NULL',self.robotManips[1].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Left Foot
        TSRStringLF1 = SerializeTSR(2,'NULL',self.robotManips[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Foot
        TSRStringRF1 = SerializeTSR(3,'NULL',self.robotManips[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Head
        TSRStringH = SerializeTSR(4,'NULL',self.robotManips[4].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # Set the TSRs for initik --> home
        # To do that, we need the end effector transforms at homeIK
        self.AvoidSingularity(self.robotid)
        self.homeik = self.robotid.GetActiveDOFValues()
        
        if(type(self.initik) == type("")):
            self.robotid.SetActiveDOFValues(str2num(self.initik))
        else:
            self.robotid.SetActiveDOFValues(self.initik)

        self.robotid.GetController().Reset(0)
        
        # Left Foot
        TSRStringLF2 = SerializeTSR(2,'NULL',self.robotManips[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))
        # Right Foot
        TSRStringRF2 = SerializeTSR(3,'NULL',self.robotManips[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))

        # We have the strings. Let's chain them together.
        
        TSRChainStringFeetandHead_current2init_bh = SerializeTSRChain(0,0,1,1,TSRStringLF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringH,'NULL',[])

        TSRChainStringFeetandHead_current2init_lh = SerializeTSRChain(0,0,1,1,TSRStringRH1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringLF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringH,'NULL',[])

        TSRChainStringFeetandHead_current2init_rh = SerializeTSRChain(0,0,1,1,TSRStringLH1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringLF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringH,'NULL',[])

        TSRChainStringFeetandHead_init2home = SerializeTSRChain(0,0,1,1,TSRStringLF2,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF2,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringH,'NULL',[])
        
        # We now have the TSRs
        # Set the robot back to currentik
        self.robotid.SetActiveDOFValues(currentik)
                
        # 1. make sure you open your hands
        # 2. Go to a safe position

        # Set the path element
        cpe0 = ConstrainedPathElement("current2init")
        cpe0.startik = currentik
        cpe0.goalik = self.initik
        if( hands == "BH" ):
            cpe0.TSR = TSRChainStringFeetandHead_current2init_bh
        elif( hands == "LH" ):
            cpe0.TSR = TSRChainStringFeetandHead_current2init_lh
        elif( hands == "RH" ):
            cpe0.TSR = TSRChainStringFeetandHead_current2init_rh
        cpe0.smoothing = self.normalsmoothingitrs
        cpe0.errorCode = "17"
        cpe0.psample = 0.2
        cpe0.filename = "movetraj7"
        cpe0.hands = hands
        cpe0.cbirrtProblems = [self.probs_cbirrt]
        cpe0.cbirrtRobots = [self.robotid]
        cpe0.cbirrtTrajectories = [self.default_trajectory_dir+cpe0.filename]

        # 3. open your hands before
        cpe0.openHandsBefore = True
        # 4. close your hands after
        cpe0.closeHandsAfter = True

        # 4. Go back home
        cpe1 = ConstrainedPathElement("init2home")
        cpe1.startik = self.initik
        cpe1.goalik = self.homeik
        cpe1.TSR = TSRChainStringFeetandHead_init2home
        cpe1.smoothing = self.normalsmoothingitrs
        cpe1.errorCode = "18"
        cpe1.psample = 0.2
        cpe1.filename = "movetraj8"
        cpe1.hands = hands
        cpe1.cbirrtProblems = [self.probs_cbirrt]
        cpe1.cbirrtRobots = [self.robotid]
        cpe1.cbirrtTrajectories = [self.default_trajectory_dir+cpe1.filename]
            
        cp = ConstrainedPath("EndTask")
        cp.valveType = valveType

        cp.elements.append(cpe0)
        cp.elements.append(cpe1)
        
        [success, why] = self.PlanPath(cp)
        if(not success):
            return why
        else:
            return 0

    def FindStartIK(self, hands, valveType):
        handles = []

        # Now try to find an IK to get ready to turn the valve
        #
        # Go through different grasp indices until one of them works
        graspIndex = 0

        # Left Hand Transform in World Coordinates
        self.T0_LH1 = self.GetT0_LH1(hands, graspIndex ,valveType)

        # Uncomment if you want to see where T0_LH1 is 
        handles.append(misc.DrawAxes(self.env,matrix(self.T0_LH1),1))

        # Right Hand Pose in World Coordinates
        self.T0_RH1 = self.GetT0_RH1(hands, graspIndex, valveType)

        # Uncomment if you want to see where T0_RH1 is 
        handles.append(misc.DrawAxes(self.env,matrix(self.T0_RH1),1))

        # Define Task Space Region strings
        # Left Hand
        TSRStringLH1 = SerializeTSR(0,'NULL',self.T0_LH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Hand
        TSRStringRH1 = SerializeTSR(1,'NULL',self.T0_RH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Define TSR for this path
        # Left Foot
        TSRStringLF1 = SerializeTSR(2,'NULL', self.robotManips[2].GetEndEffectorTransform(), eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Foot
        TSRStringRF1 = SerializeTSR(3,'NULL', self.robotManips[3].GetEndEffectorTransform(), eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Head
        TSRStringH = SerializeTSR(4,'NULL',self.robotManips[4].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # We defined Task Space Regions. Now let's concatenate them.
        TSRChainStringFeetandHead_init2start_bh = SerializeTSRChain(0,1,0,1,TSRStringLH1,'NULL',[])+' '+SerializeTSRChain(0,1,0,1,TSRStringRH1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringLF1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringRF1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringH,'NULL',[])

        TSRChainStringFeetandHead_init2start_lh = SerializeTSRChain(0,1,0,1,TSRStringLH1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringRH1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringLF1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringRF1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringH,'NULL',[])

        TSRChainStringFeetandHead_init2start_rh = SerializeTSRChain(0,1,1,1,TSRStringLH1,'NULL',[])+' '+SerializeTSRChain(0,1,0,1,TSRStringRH1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringLF1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringRF1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringH,'NULL',[])

        # Open the hand we will use to avoid collision:
        if( hands == "BH" ):
            self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
        if( hands == "LH" ):
            self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
        if( hands == "RH" ):
            self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
                    
        startik = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+self.footlinknames+' movecog '+self.cogTargStr+' nummanips 2 maniptm 0 '+trans_to_str(self.T0_LH1)+' maniptm 1 '+trans_to_str(self.T0_RH1))
        
        #GeneralIK does not go collision check
        if( startik == '' or (self.env.CheckCollision(self.robotid) or self.robotid.CheckSelfCollision()) ):
            print "Error: GeneralIK could not find startik, or startik is in collision."
            
            if( self.useIKFast  ):
                print "Info: using IKFast."
                
                if( hands == "BH" or hands == "LH" ):
                    sol0 = self.IKFast('leftArm', array(self.T0_LH1), False)
                    if( sol0 is not None):
                        self.robotid.SetDOFValues(sol0,self.robotid.GetManipulators()[0].GetArmIndices())
                    else:
                        print "Error: IKFast Could not found startik."
                        return [False, 32, "", "", "", ""] # 3: ikfast error, 2: startik
       
                if( hands == "BH" or hands == "RH" ):
                    sol1 = self.IKFast('rightArm', array(self.T0_RH1), False)
                    if( sol1 is not None):
                        self.robotid.SetDOFValues(sol1,self.robotid.GetManipulators()[1].GetArmIndices())
                    else:
                        print "Error: IKFast Could not found startik."
                        return [False, 32, "", "", "", ""] # 3: ikfast error, 2: startik

                startik = self.robotid.GetActiveDOFValues()
            else:
                return [False, 22, "", "", "", ""] # 2: generalik error, 2: at startik
        else:
            print "Info: GeneralIK found a startik."

        return [True, "", startik, TSRChainStringFeetandHead_init2start_bh, TSRChainStringFeetandHead_init2start_lh, TSRChainStringFeetandHead_init2start_rh]

    def GetReady(self, hands, valveType):
        
        self.AvoidSingularity(self.robotid)

        # Wherever you are, make sure you
        # 1. Go to a safe position
        handles = []

        # Current configuration of the robot is its initial configuration
        currentik = self.robotid.GetActiveDOFValues()

        # Define TSR for this path
        TSRStringLF0 = SerializeTSR(2,'NULL',self.robotManips[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))
        TSRStringRF0 = SerializeTSR(3,'NULL',self.robotManips[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))
        TSRStringH = SerializeTSR(4,'NULL',self.robotManips[4].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRChainStringFeetandHead_current2init = SerializeTSRChain(0,0,1,1,TSRStringLF0,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF0,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRStringH,'NULL',[])

        # Set a "safe pose"
        # elbows: Left Elbow Pitch: 3; Right Elbow Pitch: 29
        self.AvoidSingularity(self.robotid)
        self.robotid.SetDOFValues([-0.65,-0.65],[3,29]) 
        self.BendTheKnees()
        [T0_LFTarget, T0_RFTarget] = self.GetFeetTargets()

        self.initik = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+self.footlinknames+' movecog '+self.cogTargStr+' nummanips 2 maniptm 2 '+trans_to_str(T0_LFTarget)+' maniptm 3 '+trans_to_str(T0_RFTarget))

        if( self.initik == ''):
            print "Error: could not find initik"
            return 21 # 2: generalik error, 1: at initik

        self.robotid.SetActiveDOFValues(str2num(self.initik))

        [success, why, startik, TSRChainStringFeetandHead_init2start_bh, TSRChainStringFeetandHead_init2start_lh, TSRChainStringFeetandHead_init2start_rh] = self.FindStartIK(hands, valveType)
        
        if(not success):
            return why

        # If all is good we have a currentik, an initik and a startik
        # Close both hands to avoid collision at currentik
        self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
        self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)

        cp = ConstrainedPath("GetReady")
        cp.valveType = valveType
        
        # Set the path elements
        # From current configuration to a known init configuration
        cpe0 = ConstrainedPathElement("current2init")
        cpe0.startik = currentik
        cpe0.goalik = self.initik
        cpe0.TSR = TSRChainStringFeetandHead_current2init
        cpe0.smoothing = self.normalsmoothingitrs
        cpe0.errorCode = "10"
        cpe0.psample = 0.2
        cpe0.filename = "movetraj0"
        cpe0.hands = hands
        cpe0.cbirrtProblems = [self.probs_cbirrt]
        cpe0.cbirrtRobots = [self.robotid]
        cpe0.cbirrtTrajectories = [self.default_trajectory_dir+cpe0.filename]

        # 2. Open your hands after going to "ready" config.
        cpe0.openHandsAfter = True

        print "startik"
        print startik

        # From a known init configuration to a known start configuration
        cpe1 = ConstrainedPathElement("init2start")
        cpe1.startik = self.initik
        cpe1.goalik = startik

        if( hands == "BH" ):
            cpe1.TSR = TSRChainStringFeetandHead_init2start_bh
        elif( hands == "LH" ):
            cpe1.TSR = TSRChainStringFeetandHead_init2start_lh
        elif( hands == "RH" ):
            cpe1.TSR = TSRChainStringFeetandHead_init2start_rh

        cpe1.smoothing = self.normalsmoothingitrs
        cpe1.errorCode = "11"
        cpe1.psample = 0.2
        cpe1.filename = "movetraj1"
        cpe1.hands = hands
        cpe1.cbirrtProblems = [self.probs_cbirrt]
        cpe1.cbirrtRobots = [self.robotid]
        cpe1.cbirrtTrajectories = [self.default_trajectory_dir+cpe1.filename]

        cp.elements.append(cpe0)
        cp.elements.append(cpe1)
        
        [success, why] = self.PlanPath(cp)
        if(not success):
            return why
        else:
            return 0


    def PlanPath(self, path):

        if( self.StopAtKeyStrokes ):
            print "Press Enter to plan "+path.name
            sys.stdin.readline()

        for pe in path.elements:

            if( pe.openHandsBefore ):
                # Open the hand we will use to avoid collision:
                if( pe.hands == "BH" ):
                    self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
                    self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
                if( pe.hands == "LH" ):
                    self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
                if( pe.hands == "RH" ):
                    self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)

                self.OpenHands(pe.hands,self.default_trajectory_dir+"openhands_before_"+pe.filename)
            if( pe.closeHandsBefore ):
                self.CloseHands(pe.hands,self.default_trajectory_dir+"closehands_before_"+pe.filename,True)

            [success, why] = self.PlanTrajectory(pe.startik, pe.goalik, pe.TSR, pe.smoothing, pe.errorCode, pe.mimicdof, pe.psample)
            if(not success):
                return [False, why]

            [success, why] = self.RenameTrajectory("cmovetraj.txt",self.default_trajectory_dir+pe.filename+".txt")
            if(not success):
                return [False, why]

            self.ExportTraj2RealHubo(self.default_trajectory_dir+pe.filename)
            
            [success, why] = pe.PlayInOpenRAVE()
            if(not success):
                return [False, why]

            if( pe.openHandsAfter ):
                # Open the hand we will use to avoid collision:
                if( pe.hands == "BH" ):
                    self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
                    self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
                if( pe.hands == "LH" ):
                    self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
                if( pe.hands == "RH" ):
                    self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
            
                self.OpenHands(pe.hands,self.default_trajectory_dir+"openhands_after_"+pe.filename)
            if( pe.closeHandsAfter ):
                # super hacky way to keep the robot from being in self collision when going home
                if(pe.name == "current2init"):
                    self.CloseHands(pe.hands,self.default_trajectory_dir+"closehands_after_"+pe.filename,False)
                else:
                    self.CloseHands(pe.hands,self.default_trajectory_dir+"closehands_after_"+pe.filename,True)

        self.trajectory = path

        return [True, ""]

    def GetT0_LH1(self, hands, whichGrasp, valveType):

        if( hands == "BH" ):
            # Figure out where to put the left hand on the valve
            if(valveType == "W"):
                if(whichGrasp == 0):
                    temp = dot(self.valveTee, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))
                    temp = dot(temp, MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0]))))
                    return dot(temp, MakeTransform(rodrigues([0,0,pi/4]),transpose(matrix([-0.02,self.r_Wheel+0.005,0]))))
                
        if( hands == "LH" ):
            # Figure out where to put the left hand on the wheel
            temp = dot(self.valveTee, MakeTransform(rodrigues([0,0,pi/2]),transpose(matrix([0,0,0]))))
            temp = dot(temp, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))

            # Left Hand Pose in World Coordinates
            if(valveType == "RL"): # if lever (right end at the origin of rotation), hold it from the tip of the handle
                return dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.01,-1*(self.r_Wheel-0.005)]))))
            if(valveType == "LL"): # if lever (left end at the origin of rotation), hold it from the tip of the handle
                return dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.01,(self.r_Wheel-0.005)]))))

            if(valveType == "W"): # if it's a small wheel, hold it from the center but back off a little
                return dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.02,0]))))

        if( hands == "RH" ):
            return self.robotManips[0].GetEndEffectorTransform()

        return None


    def GetT0_RH1(self, hands, whichGrasp, valveType):
        if( hands == "BH" ):
            # Figure out where to put the right hand on the valve
            if(valveType == "W"):
                if(whichGrasp == 0):
                    temp = dot(self.valveTee, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))
                    temp = dot(temp, MakeTransform(rodrigues([0,pi,0]),transpose(matrix([0,0,0]))))
                    temp = dot(temp, MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0]))))
                    return dot(temp, MakeTransform(rodrigues([0,0,pi/4]),transpose(matrix([-0.02,self.r_Wheel+0.005,0]))))      

        if( hands == "LH" ):
            return self.robotManips[1].GetEndEffectorTransform()

        if( hands == "RH" ):
            temp = dot(self.valveTee, MakeTransform(rodrigues([0,0,pi/2]),transpose(matrix([0,0,0]))))
            temp = dot(temp, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))

            # Right Hand Pose in World Coordinates
            if(valveType == "RL"): # if lever (right end at the origin of rotation), hold it from the tip of the handle
                return dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.01,-1*(self.r_Wheel-0.005)]))))
            if(valveType == "LL"): # if lever (left end at the origin of rotation), hold it from the tip of the handle
                return dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.01,self.r_Wheel-0.005]))))
            if(valveType == "W"): # if it's a small wheel, hold it from the center but back off a little
                return dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.02,0]))))

        return None

    def CheckHands(self, radius, valveType, direction):
        # Check if, because of compliance, or some other reason (active balancing / sensor error etc.)
        # the end effectors of the robot matches to {R,L}H1 for the most recent valve pose.
        # 
        # If the end effector's are not where they should be, then plan a trajectory in between
        # and go to start configuration.
        T0_CurrentLH = self.robotManips[0].GetEndEffectorTransform()
        T0_CurrentRH = self.robotManips[1].GetEndEffectorTransform()

        # set the result to false first
        goToStartIK = False

        if(not allclose(self.T0_LH1, T0_CurrentLH) ):
            print "Warning: leftArm end effector has moved."
            goToStartIK = True

        if(not allclose(self.T0_RH1, T0_CurrentRH) ):
            print "Warning: rightArm end effector has moved."
            goToStartIK = True

        return goToStartIK

    def BothHands(self, radius, valveType, direction):

        handles = []
        
        self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
        self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)

        # Current configuration of the robot is its initial configuration
        currentik = self.robotid.GetActiveDOFValues()

        [success, why, startik, TSRChainStringFeetandHead_init2start_bh, TSRChainStringFeetandHead_init2start_lh, TSRChainStringFeetandHead_init2start_rh] = self.FindStartIK("BH", valveType)
        
        if(not success):
            return why

        # Calculate hand transforms after rotating the wheel (they will help us find the goalik):
        # How much do we want to rotate the wheel?
        if(direction == "CCW"):
            multiplier = -1
        elif(direction == "CW"):
            multiplier = 1

        crank_rot = (multiplier)*(pi/8)

        # The coordinate system of the valve model we're using is not aligned with the world.
        # This means when we say "valve.SetTransform(eye(4))" XYZ axes don't match to the world's XYZ axes.
        # If they did, we could simply do "T0_w0L = valve.GetTransform()"
        # However, instead we have to fix the valve's transform to make it match world's transform when zeroed.
        # This is totally for convenience, so it's easier to think of the limits and the TSR.
        T0_w0L = dot(self.valveTroot,MakeTransform(rodrigues([0,-pi/2,0]),transpose(matrix([0,0,0]))))
        T0_w0L = dot(T0_w0L,MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))

        # Left hand's transform in wheel's coordinates
        Tw0L_LH1 = dot(linalg.inv(T0_w0L),self.T0_LH1) # self.T0_LH1 is set in GetReady() method

        # Transform of the left hand's end effector in wheel's coords.
        # Required by CBiRRT
        Tw0_eL = Tw0L_LH1

        # Right Hand's TSR:
        # Note that the right hand is defined in the wheel coordinate frame
        T0_crankHandle = self.crankid.GetManipulators()[0].GetEndEffectorTransform()
        T0_w0R = MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0,0])))

        # End effector transform in wheel coordinates
        Tw0_eR = dot(linalg.inv(T0_crankHandle),self.T0_RH1)

        # Which joint do we want the CBiRRT to mimic the TSR for?
        TSRChainMimicDOF = 1
        TcrankHandle_crankHandleRotated = MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0])))

        # Where will the right hand go after turning the wheel?
        T0_crankHandleRotated = dot(T0_crankHandle,TcrankHandle_crankHandleRotated)
        TcrankHandle_RH1 = dot(linalg.inv(T0_crankHandle),self.T0_RH1)
        T0_RH2 = dot(T0_crankHandleRotated, TcrankHandle_RH1)

        # How much freedom? (note: this end effector is mimicking, everything is defined 
        # in the frame of crank)
        Bw0R = matrix([0,0,0,0,0,0,0,0,0,0,0,0])

        # Head's transforms:
        T0_w0H =  self.robotManips[4].GetEndEffectorTransform()
        Tw0_eH = eye(4)
        Bw0H = matrix([0,0,0,0,0,0,0,0,0,0,0,0])

        # Create the transform for the wheel that we would like to reach to                
        # Rotate the left hand's transform on the wheel in world transform "crank_rot" radians around it's Z-Axis
        T0_cranknew = dot(self.crankid.GetManipulators()[0].GetEndEffectorTransform(), MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0]))))

        handles.append(misc.DrawAxes(self.env,matrix(T0_cranknew),1))

        # Where will the left hand go after turning the wheel?
        T0_LH2 = dot(T0_cranknew,dot(linalg.inv(self.crankid.GetManipulators()[0].GetEndEffectorTransform()),self.T0_LH1))

        # Exit configurations
        backOff = 0.02
        T0_LH3 = dot(T0_LH2, MakeTransform(eye(3),transpose(matrix([0,backOff,0]))))
        T0_RH3 = dot(T0_RH2, MakeTransform(eye(3),transpose(matrix([0,backOff,0]))))

        T0_LH4 = dot(self.T0_LH1, MakeTransform(eye(3),transpose(matrix([0,backOff,0]))))
        T0_RH4 = dot(self.T0_RH1, MakeTransform(eye(3),transpose(matrix([0,backOff,0]))))

        if(direction == "CW"):
            Bw0L = matrix([0,0,0,0,0,0,0,crank_rot,0,0,0,0])
        elif(direction == "CCW"):
            Bw0L = matrix([0,0,0,0,0,0,crank_rot,0,0,0,0,0])

        # Uncomment to see T0_LH1,2,3
        handles.append(misc.DrawAxes(self.env,matrix(self.T0_LH1),1))    
        handles.append(misc.DrawAxes(self.env,matrix(T0_LH2),1))
        handles.append(misc.DrawAxes(self.env,matrix(T0_LH3),1))

        # Uncomment to see T0_RH1,2,3
        handles.append(misc.DrawAxes(self.env,matrix(self.T0_RH1),1))
        handles.append(misc.DrawAxes(self.env,matrix(T0_RH2),1))
        handles.append(misc.DrawAxes(self.env,matrix(T0_RH3),1))
        

        # Define Task Space Regions
        # Left Hand
        TSRStringLH2 = SerializeTSR(0,'NULL',T0_w0L,Tw0_eL,Bw0L)
        # Right Hand
        TSRStringRH2 = SerializeTSR(1,'crank crank',T0_w0R,Tw0_eR,Bw0R)
        # Left Foot
        TSRStringLF = SerializeTSR(2,'NULL',self.robotManips[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Foot
        TSRStringRF = SerializeTSR(3,'NULL',self.robotManips[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Head
        TSRStringH = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H)

        TSRChainStringFeetandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRStringLF,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringH,'NULL',[])

        TSRChainString_start2goal = SerializeTSRChain(0,0,1,1,TSRStringLH2,'crank',matrix([self.valveJointInd]))+' '+SerializeTSRChain(0,0,1,1,TSRStringRH2,'NULL',matrix([]))+' '+TSRChainStringFeetandHead_goal2start

        arg2 = trans_to_str(T0_LH2)
        arg3 = trans_to_str(T0_RH2)
        arg4 = trans_to_str(self.robotManips[2].GetEndEffectorTransform())
        arg5 = trans_to_str(self.robotManips[3].GetEndEffectorTransform())

        self.crankid.SetDOFValues([crank_rot],[self.valveJointInd])
        self.crankid.GetController().Reset(0)

        goalik = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+self.footlinknames+' movecog '+self.cogTargStr+' nummanips 4 maniptm 0 '+arg2+' maniptm 1 '+arg3+' maniptm 2 '+arg4+' maniptm 3 '+arg5)


        if(goalik == '' or (self.env.CheckCollision(self.robotid) or self.robotid.CheckSelfCollision()) ):
            print "Error: GeneralIK could not find goalik, or goalik is in collision."

            if( self.useIKFast ):
                print "Info: using IKFast."
                sol0 = self.IKFast('leftArm', array(T0_LH2), False)
                sol1 = self.IKFast('rightArm', array(T0_RH2), False)
                if( (sol0 is not None) and (sol1 is not None) ):
                    self.robotid.SetDOFValues(sol0, self.robotid.GetManipulators()[0].GetArmIndices())
                    self.robotid.SetDOFValues(sol1, self.robotid.GetManipulators()[1].GetArmIndices())
                else:
                    print "Error: IKFast could not find goalik."
                    return 33 # 3: ikfast error, 3: goalik

                goalik = self.robotid.GetActiveDOFValues()
            else:
                return 23 # 2: generalik error, 3: at goal ik
        else:
            print "Info: GeneralIK found a goalik."
            self.robotid.SetActiveDOFValues(str2num(goalik))
            self.robotid.GetController().Reset(0)            


        arg6 = trans_to_str(T0_LH3)
        arg7 = trans_to_str(T0_RH3)
            
        # Find an exit IK to clear the hands before going back to startik
        exitik1 = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+self.footlinknames+' movecog '+self.cogTargStr+' nummanips 4 maniptm 0 '+arg6+' maniptm 1 '+arg7+' maniptm 2 '+arg4+' maniptm 3 '+arg5)

        if(exitik1 == '' or (self.env.CheckCollision(self.robotid) or self.robotid.CheckSelfCollision()) ):
            print "Error: GeneralIK could not find exitik1, or exitik1 is in collision."

            if( self.useIKFast ):
                print "Info: sing IKFast."
                sol0 = self.IKFast('leftArm', array(T0_LH3), False)
                sol1 = self.IKFast('rightArm', array(T0_RH3), False)
                if( (sol0 is not None) and (sol1 is not None) ):
                    self.robotid.SetDOFValues(sol0, self.robotid.GetManipulators()[0].GetArmIndices())
                    self.robotid.SetDOFValues(sol1, self.robotid.GetManipulators()[1].GetArmIndices())
                else:
                    print "Error: IKFast could not find exitik1."
                    return 34 # 3: ikfast error, 3: exitik1

                exit = self.robotid.GetActiveDOFValues()
            else:
                return 24 # 2: generalik error, 4: at exitik1
        else:
            print "Info: GeneralIK found an exitik."
            self.robotid.SetActiveDOFValues(str2num(exitik1))
            self.robotid.GetController().Reset(0)

        arg8 = trans_to_str(T0_LH4)
        arg9 = trans_to_str(T0_RH4)
            
        # Find an exit IK to clear the hands before going back to startik
        exitik2 = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+self.footlinknames+' movecog '+self.cogTargStr+' nummanips 4 maniptm 0 '+arg8+' maniptm 1 '+arg9+' maniptm 2 '+arg4+' maniptm 3 '+arg5)

        if(exitik2 == '' or (self.env.CheckCollision(self.robotid) or self.robotid.CheckSelfCollision()) ):
            print "Error: GeneralIK could not find exitik2, or exitik2 is in collision."

            if( self.useIKFast ):
                print "Info: sing IKFast."
                sol0 = self.IKFast('leftArm', array(T0_LH4), False)
                sol1 = self.IKFast('rightArm', array(T0_RH4), False)
                if( (sol0 is not None) and (sol1 is not None) ):
                    self.robotid.SetDOFValues(sol0, self.robotid.GetManipulators()[0].GetArmIndices())
                    self.robotid.SetDOFValues(sol1, self.robotid.GetManipulators()[1].GetArmIndices())
                else:
                    print "Error: IKFast could not find exitik2."
                    return 35 # 3: ikfast error, 5: exitik2

                exit = self.robotid.GetActiveDOFValues()
            else:
                return 25 # 2: generalik error, 5: at exitik2
        else:
            print "Info: GeneralIK found an exitik."
            self.robotid.SetActiveDOFValues(str2num(exitik2))
            self.robotid.GetController().Reset(0)

        self.crankid.SetDOFValues([0],[0])
        self.crankid.GetController().Reset(0)

        # At this point we should have a currentik and a goalik
        cp = ConstrainedPath("TurnValveBH")
        cp.valveType = valveType

        # Define current to a known start configuration
        cpe0 = ConstrainedPathElement("current2start")
        cpe0.startik = currentik
        cpe0.goalik = startik
        cpe0.TSR = TSRChainStringFeetandHead_init2start_bh
        cpe0.smoothing = self.normalsmoothingitrs
        cpe0.errorCode = "12"
        cpe0.psample = 0.2
        cpe0.filename = "movetraj2"
        cpe0.hands = "BH"
        cpe0.cbirrtProblems = [self.probs_cbirrt]
        cpe0.cbirrtRobots = [self.robotid]
        cpe0.cbirrtTrajectories = [self.default_trajectory_dir+cpe0.filename]

        # Define start to goal
        cpe1 = ConstrainedPathElement("start2goal")
        cpe1.startik = startik
        cpe1.goalik = goalik
        cpe1.TSR = TSRChainString_start2goal
        cpe1.smoothing = self.fastsmoothingitrs
        cpe1.errorCode = "13"
        cpe1.mimicdof = TSRChainMimicDOF
        cpe1.filename = "movetraj3"
        cpe1.hands = "BH"
        cpe1.cbirrtProblems = [self.probs_cbirrt, self.probs_crankmover]
        cpe1.cbirrtRobots = [self.robotid, self.crankid]
        cpe1.cbirrtTrajectories = [self.default_trajectory_dir+cpe1.filename, self.default_trajectory_dir+cpe1.filename]
        cpe1.closeHandsBefore = True
        cpe1.openHandsAfter = True

        # Define goal to exit1
        cpe2 = ConstrainedPathElement("goal2exit1")
        cpe2.startik = goalik
        cpe2.goalik = exitik1
        cpe2.TSR = TSRChainStringFeetandHead_goal2start
        cpe2.smoothing = self.normalsmoothingitrs
        cpe2.errorCode = "14"
        cpe2.filename = "movetraj4"
        cpe2.hands = "BH"
        cpe2.cbirrtProblems = [self.probs_cbirrt]
        cpe2.cbirrtRobots = [self.robotid]
        cpe2.cbirrtTrajectories = [self.default_trajectory_dir+cpe2.filename]

        # Define exit1 to exit2
        cpe3 = ConstrainedPathElement("goal2exit1")
        cpe3.startik = exitik1
        cpe3.goalik = exitik2
        cpe3.TSR = TSRChainStringFeetandHead_goal2start
        cpe3.smoothing = self.normalsmoothingitrs
        cpe3.errorCode = "15"
        cpe3.filename = "movetraj5"
        cpe3.hands = "BH"
        cpe3.cbirrtProblems = [self.probs_cbirrt]
        cpe3.cbirrtRobots = [self.robotid]
        cpe3.cbirrtTrajectories = [self.default_trajectory_dir+cpe3.filename]

        # Define goal to start
        cpe4 = ConstrainedPathElement("exit2start")
        cpe4.startik = exitik2
        cpe4.goalik = currentik
        cpe4.TSR = TSRChainStringFeetandHead_goal2start
        cpe4.smoothing = self.normalsmoothingitrs
        cpe4.errorCode = "16"
        cpe4.filename = "movetraj6"
        cpe4.hands = "BH"
        cpe4.cbirrtProblems = [self.probs_cbirrt]
        cpe4.cbirrtRobots = [self.robotid]
        cpe4.cbirrtTrajectories = [self.default_trajectory_dir+cpe4.filename]

        # Add both elements to the path
        cp.elements.append(cpe0)
        cp.elements.append(cpe1)
        cp.elements.append(cpe2)
        cp.elements.append(cpe3)
        cp.elements.append(cpe4)

        # Plan for start -> goal -> start
        [success, why] = self.PlanPath(cp)
        if(not success):
            return why

        return 0

    def FindActiveQ(self, manipIndices, manipTransforms):
        # Try to find a GeneralIK solution for the manipulators
        # if it fails, try finding an IKFast solution
        cmdStr = 'DoGeneralIK exec supportlinks 2'+self.footlinknames+' movecog '+self.cogTargStr+' nummanips '+str(len(manipIndices))+' maniptm'
        for i, manipIdx in enumerate(manipIndices):
            cmdStr += ' '+str(manipIdx)+' '+trans_to_str(manipTransforms[i])
        
        # command string is ready. Solve GeneralIK
        generalik = self.probs_cbirrt.SendCommand(cmdStr)

        # check if generalik succeeded without any collision
        if( generalik == '' or (self.env.CheckCollision(self.robotid) or self.robotid.CheckSelfCollision()) ):
            print "Error: generalik failed, or the solution found is in collision"

            if( self.useIKFast ):
                for i, manipIdx in enumerate(manipIndices):
                    print "Info: using IKFast."
                    sol = self.IKFast(self.robotManips[manipIdx], array(manipTransforms[i]), False)
                    if( sol is not None):
                        self.robotid.SetDOFValues(sol, self.robotManips[manipIdx].GetArmIndices())
                    else:
                        print "Error: IKFast could not find a solution."
                        return None
        else:
            return generalik

        # if we are here, it means generalik failed, or there was a collision
        # so, we tried IKFast, and it succeeded, let's return active dof configuration
        return self.robotid.GetActiveDOFValues()
            

    def LeftHand(self, radius, valveType, direction):

        currentik = self.robotid.GetActiveDOFValues()

        [success, why, startik, TSRChainStringFeetandHead_init2start_bh, TSRChainStringFeetandHead_init2start_lh, TSRChainStringFeetandHead_init2start_rh] = self.FindStartIK("LH", valveType)
        
        if(not success):
            return why
        
        # This is a list of handles of the objects that are
        # drawn on the screen in OpenRAVE Qt-Viewer.
        # Keep appending to the end, and pop() if you want to delete.
        handles = [] 

        # Calculate hand transforms after rotating the wheel (they will help us find the goalik):
        # How much do we want to rotate the wheel?
        if(direction == "CCW"):
            multiplier = -1
        elif(direction == "CW"):
            multiplier = 1
            
        crank_rot = (multiplier)*pi/4

        T0_w0L = dot(self.valveTroot,MakeTransform(rodrigues([0,-pi/2,0]),transpose(matrix([0,0,0]))))
        T0_w0L = dot(T0_w0L,MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))

        # Left hand's transform in wheel's coordinates
        Tw0L_LH1 = dot(linalg.inv(T0_w0L),self.T0_LH1)

        # Transform of the left hand's end effector in wheel's coords.
        # Required by CBiRRT
        Tw0_eL = Tw0L_LH1

        # How much freedom do we want to give to the left hand
        if(direction == "CW"):
            Bw0L = matrix([0,0,0,0,0,0,0,crank_rot,0,0,0,0])
        elif(direction == "CCW"):
            Bw0L = matrix([0,0,0,0,0,0,crank_rot,0,0,0,0,0])

        # Right Hand's transforms:
        T0_crankcrank = self.crankid.GetManipulators()[0].GetTransform()

        # Head's transforms:
        T0_w0H =  self.robotManips[4].GetEndEffectorTransform()
        Tw0_eH = eye(4)
        Bw0H = matrix([0,0,0,0,0,0,0,0,0,0,0,0])

        # Define Task Space Regions
        # Left Hand
        TSRString1 = SerializeTSR(0,'NULL',T0_w0L,Tw0_eL,Bw0L)
        # Right Hand
        TSRString2 = SerializeTSR(1,'NULL',self.T0_RH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Left Foot
        TSRString3 = SerializeTSR(2,'NULL',self.robotManips[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Foot

        TSRString4 = SerializeTSR(3,'NULL',self.robotManips[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Head
        TSRString5 = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H)

        TSRChainStringFeetHeadandRightHand_start2init = SerializeTSRChain(0,0,1,1,TSRString2,'NULL',[])+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

        TSRChainStringFeetRightHandandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString2,'NULL',[])+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

        TSRChainStringFeetandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

        TSRChainString = SerializeTSRChain(0,0,1,1,TSRString1,'crank',matrix([self.valveJointInd]))+' '+SerializeTSRChain(0,0,1,1,TSRString2,'NULL',matrix([]))+' '+TSRChainStringFeetandHead_goal2start

        # Which joint do we want the CBiRRT to mimic the TSR for?
        TSRChainMimicDOF = 1

        # Create the transform for the wheel that we would like to reach to
        Tcrank_rot = MakeTransform(rodrigues([crank_rot,0,0]),transpose(matrix([0,0,0])))

        # What is this?
        temp = MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0])))

        # Rotate the left hand's transform on the wheel in world transform "crank_rot" radians around it's Z-Axis
        T0_cranknew = dot(T0_w0L,Tcrank_rot)

        T0_cranknew = dot(self.crankid.GetManipulators()[0].GetEndEffectorTransform(), MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0]))))

        # Where will the left hand go after turning the wheel?
        T0_LH2 = dot(T0_cranknew,dot(linalg.inv(self.crankid.GetManipulators()[0].GetEndEffectorTransform()),self.T0_LH1))

        # Uncomment to see T0_LH2
        handles.append(misc.DrawAxes(self.env,matrix(T0_LH2),1))

        # check if ik solutions exist
        goalik = self.FindActiveQ([0],[array(T0_LH2)])
        if( goalik == None ):
            print "Error: No goalik found!"
            return 33 # 3: 
            
        cp = ConstrainedPath("TurnValveLH")
        cp.valveType = valveType

        # Define current to a known start configuration
        cpe0 = ConstrainedPathElement("current2start")
        cpe0.startik = currentik
        cpe0.goalik = startik
        cpe0.TSR = TSRChainStringFeetandHead_init2start_lh
        cpe0.smoothing = self.normalsmoothingitrs
        cpe0.errorCode = "12"
        cpe0.psample = 0.2
        cpe0.filename = "movetraj2"
        cpe0.hands = "LH"
        cpe0.cbirrtProblems = [self.probs_cbirrt]
        cpe0.cbirrtRobots = [self.robotid]
        cpe0.cbirrtTrajectories = [self.default_trajectory_dir+cpe0.filename]
        
        cpe1 = ConstrainedPathElement("start2goal")
        cpe1.startik = startik
        cpe1.goalik = goalik
        cpe1.TSR = TSRChainString
        cpe1.smoothing = self.fastsmoothingitrs
        cpe1.errorCode = "13"
        cpe1.mimicdof = TSRChainMimicDOF
        cpe1.filename = "movetraj3"
        cpe1.hands = "LH"
        cpe1.cbirrtProblems = [self.probs_cbirrt, self.probs_crankmover]
        cpe1.cbirrtRobots = [self.robotid, self.crankid]
        cpe1.cbirrtTrajectories = [self.default_trajectory_dir+cpe1.filename, self.default_trajectory_dir+cpe1.filename]

        cpe1.closeHandsBefore = True
        cpe1.openHandsAfter = True

        # Define goal to current
        cpe2 = ConstrainedPathElement("goal2current")
        cpe2.startik = goalik
        cpe2.goalik = currentik
        cpe2.TSR = TSRChainStringFeetandHead_init2start_lh
        cpe2.smoothing = self.normalsmoothingitrs
        cpe2.errorCode = "14"
        cpe2.psample = 0.2
        cpe2.filename = "movetraj4"
        cpe2.hands = "LH"
        cpe2.cbirrtProblems = [self.probs_cbirrt]
        cpe2.cbirrtRobots = [self.robotid]
        cpe2.cbirrtTrajectories = [self.default_trajectory_dir+cpe2.filename]

        cp.elements.append(cpe0)
        cp.elements.append(cpe1)

        # If the valve is round, then go back to currentik after goal
        if( valveType == "W" ):
            cp.elements.append(cpe2)
        
        [success, why] = self.PlanPath(cp)
        if(not success):
            return why
        else:
            return 0 # no error

    
    def RightHand(self, radius, valveType, direction):

        currentik = self.robotid.GetActiveDOFValues()

        [success, why, startik, TSRChainStringFeetandHead_init2start_bh, TSRChainStringFeetandHead_init2start_lh, TSRChainStringFeetandHead_init2start_rh] = self.FindStartIK("RH", valveType)
        
        if(not success):
            return why

        # This is a list of handles of the objects that are
        # drawn on the screen in OpenRAVE Qt-Viewer.
        # Keep appending to the end, and pop() if you want to delete.
        handles = [] 

        if(direction == "CCW"):
            multiplier = -1
        elif(direction == "CW"):
            multiplier = 1

        crank_rot = (multiplier)*pi/4

        T0_w0R = dot(self.valveTroot,MakeTransform(rodrigues([0,-pi/2,0]),transpose(matrix([0,0,0]))))
        T0_w0R = dot(T0_w0R,MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))

        Tw0R_RH1 = dot(linalg.inv(T0_w0R),self.T0_RH1)

        Tw0_eR = Tw0R_RH1
        
        if(direction == "CW"):
            Bw0R = matrix([0,0,0,0,0,0,0,crank_rot,0,0,0,0])
        elif(direction == "CCW"):
            Bw0R = matrix([0,0,0,0,0,0,crank_rot,0,0,0,0,0])

        # Right Hand's transforms:
        T0_crankcrank = self.crankid.GetManipulators()[0].GetTransform()
            
        # Head's transforms:
        T0_w0H =  self.robotManips[4].GetEndEffectorTransform()
        Tw0_eH = eye(4);
        Bw0H = matrix([0,0,0,0,0,0,0,0,0,0,0,0])

        # Define Task Space Regions
        # Left Hand
        TSRString1 = SerializeTSR(0,'NULL',self.T0_LH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Hand
        TSRString2 = SerializeTSR(1,'NULL',T0_w0R,Tw0_eR,Bw0R)
        # Left Foot
        TSRString3 = SerializeTSR(2,'NULL',self.robotManips[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Foot
        TSRString4 = SerializeTSR(3,'NULL',self.robotManips[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Head
        TSRString5 = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H)

        TSRChainStringFeetHeadandLeftHand_start2init = SerializeTSRChain(0,0,1,1,TSRString1,'NULL',[])+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

        TSRChainStringFeetandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])
        
        TSRChainStringFeetLeftHandandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString1,'NULL',[])+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

        TSRChainString = SerializeTSRChain(0,0,1,1,TSRString1,'NULL',matrix([]))+' '+SerializeTSRChain(0,0,1,1,TSRString2,'crank',matrix([self.valveJointInd]))+' '+TSRChainStringFeetandHead_goal2start

        # Which joint do we want the CBiRRT to mimic the TSR for?
        TSRChainMimicDOF = 1

        # Create the transform for the wheel that we would like to reach to
        Tcrank_rot = MakeTransform(rodrigues([crank_rot,0,0]),transpose(matrix([0,0,0])))

        # What is this?
        temp = MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0])))

        # Rotate the left hand's transform on the wheel in world transform "crank_rot" radians around it's Z-Axis
        T0_cranknew = dot(T0_w0R,Tcrank_rot)


        T0_cranknew = dot(self.crankid.GetManipulators()[0].GetEndEffectorTransform(), MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0]))))

        T0_RH2 = dot(T0_cranknew,dot(linalg.inv(self.crankid.GetManipulators()[0].GetEndEffectorTransform()),self.T0_RH1))
        
        # Uncomment to see T0_RH2
        handles.append(misc.DrawAxes(self.env,matrix(T0_RH2),1))

        # check if ik solutions exist
        goalik = self.FindActiveQ([1],[array(T0_RH2)])
        if( goalik == None ):
            print "Error: No goalik found!"
            return 33

        cp = ConstrainedPath("TurnValveRH")
        cp.valveType = valveType

        # Define current to a known start configuration
        cpe0 = ConstrainedPathElement("current2start")
        cpe0.startik = currentik
        cpe0.goalik = startik
        cpe0.TSR = TSRChainStringFeetandHead_init2start_rh
        cpe0.smoothing = self.normalsmoothingitrs
        cpe0.errorCode = "12"
        cpe0.psample = 0.2
        cpe0.filename = "movetraj2"
        cpe0.hands = "RH"
        cpe0.cbirrtProblems = [self.probs_cbirrt]
        cpe0.cbirrtRobots = [self.robotid]
        cpe0.cbirrtTrajectories = [self.default_trajectory_dir+cpe0.filename]
        
        cpe1 = ConstrainedPathElement("start2goal")
        cpe1.startik = startik
        cpe1.goalik = goalik
        cpe1.TSR = TSRChainString
        cpe1.smoothing = self.fastsmoothingitrs
        cpe1.mimicdof = TSRChainMimicDOF
        cpe1.filename = "movetraj3"
        cpe1.hands = "RH"
        cpe1.errorCode = "13"
        cpe1.cbirrtProblems = [self.probs_cbirrt, self.probs_crankmover]
        cpe1.cbirrtRobots = [self.robotid, self.crankid]
        cpe1.cbirrtTrajectories = [self.default_trajectory_dir+cpe1.filename, self.default_trajectory_dir+cpe1.filename]
        cpe1.closeHandsBefore = True
        cpe1.openHandsAfter = True

        # Define goal to current
        cpe2 = ConstrainedPathElement("goal2current")
        cpe2.startik = goalik
        cpe2.goalik = currentik
        cpe2.TSR = TSRChainStringFeetandHead_init2start_rh
        cpe2.smoothing = self.normalsmoothingitrs
        cpe2.errorCode = "14"
        cpe2.psample = 0.2
        cpe2.filename = "movetraj4"
        cpe2.hands = "RH"
        cpe2.cbirrtProblems = [self.probs_cbirrt]
        cpe2.cbirrtRobots = [self.robotid]
        cpe2.cbirrtTrajectories = [self.default_trajectory_dir+cpe2.filename]

        cp.elements.append(cpe0)
        cp.elements.append(cpe1)
        # If the valve is round, then go back to currentik after goal
        if( valveType == "W" ):
            cp.elements.append(cpe2)

        [success, why] = self.PlanPath(cp)
        if(not success):
            return why
        else:
            return 0

    def SetRobotConfiguration(self,q):
        # q is a dictionary
        #
        # This method matches the joint indices
        # between ROS and OpenRAVE.
        # For planning purposes, we skip head and finger
        # joints.
        for jName, jValue in q.iteritems():
            if( jName[0:2] != 'RF' and jName[0:2] != 'LF' and jName[0:2] != 'NK' ):
                rosValue = jValue
                openraveIdx  = self.jointDict[jName]
                self.robotid.SetDOFValues([rosValue],[openraveIdx])
                self.robotid.GetController().Reset(0)
            else:
                print "Info: set robot config is skipping :"+jName

    def GetFeetTargets(self):
        T0_TSY = self.robotid.GetLinks()[12].GetTransform()
        
        # Left Foot Target in World Coords.
        T0_LF = deepcopy(T0_TSY)
        T0_LF[0,3] = T0_TSY[0,3]+self.Ttsy_lar_home[0,3]
        T0_LF[1,3] = T0_TSY[1,3]+self.Ttsy_lar_home[1,3]
        T0_LF[2,3] = T0_TSY[2,3]+self.Ttsy_lar_home[2,3]+self.crouch

        # Right Foot Target in World Coords.
        T0_RF = deepcopy(T0_TSY)
        T0_RF[0,3] = T0_TSY[0,3]+self.Ttsy_rar_home[0,3]
        T0_RF[1,3] = T0_TSY[1,3]+self.Ttsy_rar_home[1,3]
        T0_RF[2,3] = T0_TSY[2,3]+self.Ttsy_rar_home[2,3]+self.crouch

        hlfoot = misc.DrawAxes(self.env,T0_LF,1)
        hrfoot =misc.DrawAxes(self.env,T0_RF,1)
        
        return [T0_LF, T0_RF]
    
    def BendTheKnees(self,howMuch=0.1):
        keepFeetParallel = False
        
        # TODO
        if(keepFeetParallel):
            pass
        #    self.robotid.SetDOFValues([],[])
        else:
            # LKP: 14, RKP: 39
            self.robotid.SetDOFValues([2*howMuch,2*howMuch],[14,39])
            # LHP: 13, RHP: 38
            self.robotid.SetDOFValues([-howMuch,-howMuch],[13,38])
            # LAP: 15, RAP: 40
            self.robotid.SetDOFValues([-howMuch,-howMuch],[15,40])
            

    def Plan(self, handles=[], radius=None, manipulator=None, direction="CW", valveType=None, taskStage=None):

        if(radius != None):
            self.r_Wheel = radius
        
        if(self.optWall):
            self.AddWall()

        # Set planning variables
        self.normalsmoothingitrs = 300
        self.fastsmoothingitrs = 20

        # keep manipulator objects for easy access
        self.robotManips = self.robotid.GetManipulators()
        self.valveManip = self.crankid.GetManipulators()

        # Valve Transform End Effector in World Coordinates
        # This is the transformation matrix of the end effector 
        self.valveTee = self.valveManip[0].GetEndEffectorTransform()

        # keep link objects for easy access
        self.robotLinks = self.robotid.GetLinks()
        self.valveLinks = self.crankid.GetLinks()

        # valve has two links: 
        # 0) pole - the blue cylinder in the model, and, 
        # 1) crank - handle
        self.valveTroot = self.valveLinks[0].GetTransform()

        # keep valve's only joint index for mimicking later
        self.valveJointInd = 0

        # add cbirrt problems
        self.SetProblems()

        # Keep Active Joint Indices
        self.activedofs = []
        for m in self.robotManips:
            self.activedofs.extend(m.GetArmIndices())

        self.robotid.SetActiveDOFs(self.activedofs)
             
        # Sort Active Joint Indices
        self.activedofs.sort()
        print "activedofs:"
        print self.activedofs

        if( taskStage == 'GETREADY' ):
            error_code = self.GetReady(manipulator, valveType)
        elif( taskStage == 'TURNVALVE' ):
            if( manipulator == "LH" ):
                error_code = self.LeftHand(radius, valveType, direction)
            elif( manipulator == "RH" ):
                error_code = self.RightHand(radius, valveType, direction)
            elif( manipulator == "BH" ):
                error_code = self.BothHands(radius, valveType, direction)
        elif( taskStage == 'END' ):
            error_code = self.EndTask(manipulator, valveType)

        print "Planning, done."

        return error_code    

if __name__ == "__main__":
    # One can run this script from terminal passing a radius value in
    # Otherwise use the default value
    r = None
    play = False
    taskwall = False
    wall = False
    demo = False

    if(len(sys.argv) >= 2):
        for index in range(1,len(sys.argv)):
            if(sys.argv[index] == "-radius" and index+1<len(sys.argv)):
                r = float(sys.argv[index+1])
            elif(sys.argv[index] == "-play"):
                play = True
            elif(sys.argv[index] == "-taskwall"):
                taskwall = True
            elif(sys.argv[index] == "-wall"):
                wall = True
            elif(sys.argv[index] == "-demo"):
                demo = True

    planner = DrcHuboWheelTurning()   

    planner.optPlay = play
    planner.optTaskWall = taskwall
    planner.optWall = wall
    planner.optDemo = demo

    planner.SetViewer(True)

    handles = []    
    
    if play:
        planner.SetStopKeyStrokes(True)
        planner.StartViewerAndSetValvePos( handles )
        planner.SetProblems()
        planner.Playback(True)        
    else:
        planner.SetStopKeyStrokes(True)
        if taskwall:
            planner.InitFromTaskWallEnv()

    planner.KillOpenrave()



