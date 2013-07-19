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

class DrcHuboWheelTurning( BaseWheelTurning ):

    def __init__(self,
                 HuboModelPath = roslib.packages.get_pkg_dir("drchubo-v2")+'/robots/drchubo-v2.robot.xml',
                 WheelModelPath = roslib.packages.get_pkg_dir("wpi_drc_sim")+'/../models/driving_wheel_tiny.robot.xml' ):

        BaseWheelTurning.__init__( self, HuboModelPath, WheelModelPath )        

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

    def SetProblems(self):
        self.probs_cbirrt = RaveCreateModule(self.env,'CBiRRT')
        self.probs_crankmover = RaveCreateModule(self.env,'CBiRRT')

        try:
            self.env.AddModule(self.probs_cbirrt,self.robotid.GetName())
            self.env.AddModule(self.probs_crankmover,self.crankid.GetName())
        except openrave_exception, e:
            print e

    def SetHandDOFs(self,hand,vals):
        if(hand == "lh"):
            self.robotid.SetDOFValues(multiply(ones(len(self.lhanddofs)),vals),self.lhanddofs)
        elif(hand == "rh"):
            self.robotid.SetDOFValues(multiply(ones(len(self.rhanddofs)),vals),self.rhanddofs)
        elif(hand == "bh"):
            self.robotid.SetDOFValues(multiply(ones(len(self.lhanddofs)),vals),self.lhanddofs)
            self.robotid.SetDOFValues(multiply(ones(len(self.rhanddofs)),vals),self.rhanddofs)

    def OpenHands(self,hand,fname="openHandsHere"):
        # Wait 4 seconds to fully open the hands when exporting for ach
        waitThisMuch = 6.0 # seconds.
        rave2realhubo.openHandsHere(self.robotid,25.0,waitThisMuch,hand,fname)
        print "waiting for "+hand+" to open..."
        time.sleep(waitThisMuch)
        self.SetHandDOFs(hand,self.bothhandsopenval)
        

    def CloseHands(self,hand,fname="closeHandsHere",fake=False):
        # Wait 4 seconds to fully close the hands when exporting for ach
        waitThisMuch = 6.0 # seconds.
        rave2realhubo.closeHandsHere(self.robotid,25.0,waitThisMuch,hand,fname)
        print "waiting for "+hand+" to close..."
        time.sleep(waitThisMuch)
        if(fake):
            self.SetHandDOFs(hand,self.bothhandsopenval)
        else:
            self.SetHandDOFs(hand,self.bothhandscloseval)
        
    def BothHands(self,radius,valveType,direction):
        
        self.RemoveFiles()

        # This is a list of handles of the objects that are
        # drawn on the screen in OpenRAVE Qt-Viewer.
        # Keep appending to the end, and pop() if you want to delete.
        handles = [] 

        normalsmoothingitrs = 300;
        fastsmoothingitrs = 20;        

        # Wheel Joint Index  
        crankjointind = 0
        # Set the wheel joints back to 0 for replanning
        self.crankid.SetDOFValues([0],[crankjointind])
        self.crankid.GetController().Reset(0)

        manips = self.robotid.GetManipulators()
        crankmanip = self.crankid.GetManipulators()
        
        self.SetProblems()

        print "Getting Loaded Problems"
        probs = self.env.GetLoadedProblems()

        # Keep Active Joint Indices
        activedofs = []
        for m in manips:
            activedofs.extend(m.GetArmIndices())

        # Sort Active Joint Indices
        activedofs.sort()

        ############################## DRCHUBO V2  ############################
        
        # elbows: Left Elbow Pitch: 3; Right Elbow Pitch: 29
        self.robotid.SetDOFValues([-0.45,-0.45],[3,29]) 
        self.robotid.SetActiveDOFs(activedofs)

        leg_height = self.robotid.GetManipulators()[2].GetEndEffectorTransform()[2,3]
        # Find a trajectory from 0 to initconfig
        if( self.StopAtKeyStrokes ):
            print "Leg height :  " + str(leg_height)
            sys.stdin.readline()

        # polyscale: changes the scale of the support polygon
        # polytrans: shifts the support polygon around
        footlinknames = ' Body_RAR Body_LAR polyscale 0.5 0.5 0 ' #polytrans -0.015 0 0.0 '

        # Center of Gravity Target
        # cogtarg = [-0.05, 0.085, 0]
        cogtarg = [0, 0, 0]

        print "activedofs:"
        print activedofs

        # Current configuration of the robot is its initial configuration
        initconfig = self.robotid.GetActiveDOFValues()
    

        # List of Robot Links
        links = self.robotid.GetLinks()

        # List of Wheel (Crank Links)
        cranklinks = self.crankid.GetLinks()

        # End Effector Transforms
        Tee = []
        for i in range(len(manips)):
            # Returns End Effector Transform in World Coordinates
            Tlink = manips[i].GetEndEffectorTransform()
            # print Tlink
            Tee.append(Tlink)

        # Wheel Joint Index  
        crankjointind = 0

        # Get Transformation Matrix for the Wheel
        # Note that crank's links are not rotated
        # If you want use the wheel's end effector's transformation
        # matrix (which is 23 degrees tilted) then see
        # CTee matrix below.
        #
        # crank has two links: 
        # 0) pole - the blue cylinder in the model, and, 
        # 1) crank - the driving wheel itself.
        jointtm = cranklinks[0].GetTransform()
        # handles.append(misc.DrawAxes(self.env,matrix(jointtm),1))


        # We can also get the transformation matrix
        # with the following command as a string
        # jointtm_str = self.probs_cbirrt.SendCommand('GetJointTransform name crank jointind '+str(crankjointind))
        # And then we can convert the string to a 1x12 array
        # jointtm_str = jointtm_str.replace(" ",",")
        # jointtm_num = # eval('['+jointtm_str+']')

        # In this script we will use jointtm.
        # jointtm_str and jointtm_num are given as example.

        # Crank Transform End Effector in World Coordinates
        # This is the transformation matrix of the end effector 
        # named "dummy" in the xml file.
        # Note that dummy is tilted 23 degress around its X-Axis
        CTee = crankmanip[0].GetEndEffectorTransform()

        tilt_angle_deg = acos(dot(linalg.inv(CTee),jointtm)[1,1])*180/pi
        tilt_angle_rad = acos(dot(linalg.inv(CTee),jointtm)[1,1])

        # What is this?
        handrot = rodrigues([0,-pi/2,0])

        # Translation Offset from the wheel center for the hands
        transoffset = [0, 0.15, 0];

        # Figure out where to put the left hand on the wheel
        temp = dot(CTee, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))
        temp = dot(temp, MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0]))))

        # Left Hand Pose in World Coordinates
        T0_LH1 = dot(temp, MakeTransform(rodrigues([0,0,pi/4]),transpose(matrix([-0.02,self.r_Wheel+0.005,0]))))

        # Uncomment if you want to see where T0_LH1 is 
        handles.append(misc.DrawAxes(self.env,matrix(T0_LH1),1))

        # Figure out where to put the right hand on the wheel
        temp = dot(CTee, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))
        temp = dot(temp, MakeTransform(rodrigues([0,pi,0]),transpose(matrix([0,0,0]))))
        temp = dot(temp, MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0]))))

        # Right Hand Pose in World Coordinates
        T0_RH1 = dot(temp, MakeTransform(rodrigues([0,0,pi/4]),transpose(matrix([-0.02,self.r_Wheel+0.005,0]))))

        # Uncomment if you want to see where T0_RH1 is 
        handles.append(misc.DrawAxes(self.env,matrix(T0_RH1),1))

        # Define Task Space Region strings
        # Left Hand
        TSRString1 = SerializeTSR(0,'NULL',T0_LH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # Right Hand
        TSRString2 = SerializeTSR(1,'NULL',T0_RH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # Left Foot
        TSRString3 = SerializeTSR(2,'NULL',Tee[2],eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))
        
        # Right Foot
        TSRString4 = SerializeTSR(3,'NULL',Tee[3],eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))

        # Head
        # Grasp transform in Head coordinates
        Tw0_eH = eye(4) 
        # How much freedom do we want to give to the Head
        # [x,x,y,y,z,z,R,R,P,P,Y,Y]
        Bw0H = matrix([-0.05,0.05,-0.1,0.1,-100,100,-pi,pi,-pi,pi,-pi,pi])
        TSRString5 = SerializeTSR(4,'NULL',Tee[4],Tw0_eH,Bw0H)

        TSRChainStringFeetandHead_home2init = SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])
        TSRChainStringFeetandHead_init2home = deepcopy(TSRChainStringFeetandHead_home2init)

        
        [T0_LFTarget, T0_RFTarget] = self.GetFeetTargets()

        TSRString3 = SerializeTSR(2,'NULL',T0_LFTarget,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRString4 = SerializeTSR(3,'NULL',T0_RFTarget,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # We defined Task Space Regions. Now let's concatenate them.
        TSRChainStringGrasping = SerializeTSRChain(0,1,0,1,TSRString1,'NULL',[])+' '+SerializeTSRChain(0,1,0,1,TSRString2,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString4,'NULL',[])#+' '+SerializeTSRChain(0,1,0,1,TSRString5,'NULL',[])

        # Go home
        if( self.StopAtKeyStrokes ):
            print "Press Enter to go home"
            sys.stdin.readline()

        

        self.BendTheKnees()

        arg1 = str(cogtarg).strip("[]").replace(', ',' ')

        initik = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames+' movecog '+arg1+'nummanips 2 maniptm 2 '+trans_to_str(T0_LFTarget)+' maniptm 3 '+trans_to_str(T0_RFTarget))

        sys.stdin.readline()

        self.robotid.SetDOFValues(zeros(len(self.robotid.GetJoints())),range(len(self.robotid.GetJoints())))
        home = self.robotid.GetActiveDOFValues()

        # Find a trajectory from 0 to initconfig
        if( self.StopAtKeyStrokes ):
            print "Press Enter to plan home --> initconfig "
            sys.stdin.readline()

        # goaljoints = initconfig
        goaljoints = deepcopy(initik)
        goaljoints = str2num(goaljoints)

        try:
            answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetandHead_home2init)
            print "RunCBiRRT answer: ",str(answer)
        except openrave_exception, e:
            print "Cannot send command RunCBiRRT: "
            print e

        try:
            os.rename("cmovetraj.txt","movetraj0.txt")
            traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj0.txt','r').read())

            # print "conf spec for start2goal:"
            cs = traj.GetConfigurationSpecification()

            drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
            # print "drchubo-v2 joint values offset"
            # print drchuboJointValsGroup.offset

            drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
            # print "drchubo-v2 joint velocities offset"
            # print drchuboJointVelocitiesGroup.offset

            deltatimeGroup = cs.GetGroupFromName("deltatime")
            # print "deltatime offset"
            # print deltatimeGroup.offset

            rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj0",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
        except OSError, e:
            # No file cmovetraj
            print e

        try:
            answer=self.probs_cbirrt.SendCommand('traj movetraj0.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e

        self.robotid.GetController().Reset(0)
        time.sleep(2)

        # print "TSRChainStringGrasping"
        # print TSRChainStringGrasping

        if( self.StopAtKeyStrokes ):
            print "Press Enter to go to startik"
            sys.stdin.readline()

        arg1 = str(cogtarg).strip("[]").replace(', ',' ')
        startik = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames+' movecog '+arg1+'nummanips 2 maniptm 0 '+trans_to_str(T0_LH1)+' maniptm 1 '+trans_to_str(T0_RH1))

        if(startik == ''):
            print "Error: could not find startik"
        else:
            print "found startik"
            # print startik
            
            # GeneralIK does not check for collision, so let's make sure that we find IKs that don't collide
            # with the environment
            self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)

            self.robotid.SetActiveManipulator('leftArm')
            leftArmIkmodel = databases.inversekinematics.InverseKinematicsModel(self.robotid,iktype=IkParameterizationType.Transform6D)
            leftArmIkmodel.load()
            # check if ik solutions exist
            sol0=leftArmIkmodel.manip.FindIKSolution(array(T0_LH1),IkFilterOptions.CheckEnvCollisions)
            print "sol0"
            print len(sol0)
            print sol0
            assert(sol0 is not None)
            self.robotid.SetDOFValues(sol0,self.robotid.GetManipulators()[0].GetArmIndices())
            sys.stdin.readline()

            self.robotid.SetActiveManipulator('rightArm')
            rightArmIkmodel = databases.inversekinematics.InverseKinematicsModel(self.robotid,iktype=IkParameterizationType.Transform6D)
            rightArmIkmodel.load()
            # check if ik solutions exist
            sol1=rightArmIkmodel.manip.FindIKSolution(array(T0_RH1),IkFilterOptions.CheckEnvCollisions)
            print "sol1"
            print len(sol1)
            print sol1
            assert(sol1 is not None)
            self.robotid.SetDOFValues(sol1,self.robotid.GetManipulators()[1].GetArmIndices())
            sys.stdin.readline()

            startik = self.robotid.GetActiveDOFValues()

            #print "Checking support in startik: "
            #print self.probs_cbirrt.SendCommand('CheckSupport')

            # Rotate the wheel's transform to a suitable pose
            # for the Left Hand
            # T0_w0L stands for: 
            # left hand's transform on wheel in world coordinates

            T0_w0L = MakeTransform(rodrigues([0,tilt_angle_rad-self.tiltDiff+self.worldPitch,0]),transpose(matrix(jointtm[0:3,3])))

            #T0_w0L = MakeTransform(rodrigues([acos(self.crankid.GetManipulators()[0].GetTransform()[1,1]),0,0]),transpose(matrix(jointtm[0:3,3])))
            # This is what's happening: 
            #
            # Tw0L_0 = linalg.inv(T0_w0L)
            # Tw0L_LH1 = Tw0L_0*T0_LH1
            #
            # Left hand's transform in wheel's coordinates
            Tw0L_LH1 = dot(linalg.inv(T0_w0L),T0_LH1)
            # Transform of the left hand's end effector in wheel's coords.
            # Required by CBiRRT
            Tw0_eL = Tw0L_LH1
            # How much freedom do we want to give to the left hand
            Bw0L = matrix([0,0,0,0,0,0,0,pi,0,0,0,0])

            # Right Hand's transforms:
            T0_crankcrank = self.crankid.GetManipulators()[0].GetTransform()
            T0_w0R = MakeTransform(rodrigues([tilt_angle_rad,0,0]),transpose(matrix([0,0,0])))
            #T0_w0R = MakeTransform(rodrigues([acos(self.crankid.GetManipulators()[0].GetTransform()[1,1]),0,0]),transpose(matrix([0,0,0])))
            # End effector transform in wheel coordinates
            Tw0_eR = dot(linalg.inv(T0_crankcrank),T0_RH1)

            # handles.append(misc.DrawAxes(self.env,matrix(Tw0_eR),1))

            # How much freedom? (note: in frame of crank)
            Bw0R = matrix([0,0,0,0,0,0,0,0,0,0,0,0])

            # Head's transforms:
            T0_w0H =  Tee[4]
            Tw0_eH = eye(4);
            Bw0H = matrix([-0.05,0.05,-0.1,0.1,-100,100,-pi,pi,-pi,pi,-pi,pi])


            # Define Task Space Regions
            # Left Hand
            TSRString1 = SerializeTSR(0,'NULL',T0_w0L,Tw0_eL,Bw0L)
            # Right Hand
            TSRString2 = SerializeTSR(1,'crank crank',T0_w0R,Tw0_eR,Bw0R)
            # Left Foot
            # TSRString3 = SerializeTSR(2,'NULL',Tee[2],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
            TSRString3 = SerializeTSR(2,'NULL',T0_LFTarget,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
            # Right Foot
            # TSRString4 = SerializeTSR(3,'NULL',Tee[3],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
            TSRString4 = SerializeTSR(3,'NULL',T0_RFTarget,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
            # Head
            TSRString5 = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H)

            TSRChainStringFeetandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

            TSRChainString = SerializeTSRChain(0,0,1,1,TSRString1,'crank',matrix([crankjointind]))+' '+SerializeTSRChain(0,0,1,1,TSRString2,'NULL',matrix([]))+' '+TSRChainStringFeetandHead_goal2start

            # Calculate hand transforms after rotating the wheel (they will help us find the goalik):
            # How much do we want to rotate the wheel?
            if(direction == "ccw"):
                multiplier = -1
            elif(direction == "cw"):
                multiplier = 1

            crank_rot = (multiplier)*(pi/10)

            # Which joint do we want the CBiRRT to mimic the TSR for?
            TSRChainMimicDOF = 1

            # Create the transform for the wheel that we would like to reach to
            Tcrank_rot = MakeTransform(rodrigues([crank_rot,0,0]),transpose(matrix([0,0,0])))

            # What is this?
            temp = MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0])))

            # Rotate the left hand's transform on the wheel in world transform "crank_rot" radians around it's Z-Axis
            T0_cranknew = dot(T0_w0L,Tcrank_rot)

            # Where will the left hand go after turning the wheel?
            # This is what's happening:
            #
            # Tcranknew_LH2 = dot(Tw0L_0,T0_LH1) --> Left hand in wheel's coordinate
            # T0_LH2 = dot(T0_cranknew,Tcranknew_LH2) --> Left hand rotated around wheel's origin
            T0_LH2 = dot(T0_cranknew,dot(linalg.inv(T0_w0L),T0_LH1))

            # Uncomment to see T0_LH2
            handles.append(misc.DrawAxes(self.env,matrix(T0_LH2),1))

            # Where will the right hand go after turning the wheel?
            T0_RH2 = dot(T0_crankcrank,dot(temp,dot(linalg.inv(T0_crankcrank),T0_RH1)))

            # Uncomment to see T0_RH2
            handles.append(misc.DrawAxes(self.env,matrix(T0_RH2),1))

            if( self.StopAtKeyStrokes ):
                print "Press Enter to find a goalIK"
                sys.stdin.readline()

            self.crankid.SetDOFValues([crank_rot],[crankjointind])

            arg1 = str(cogtarg).strip("[]").replace(', ',' ')
            arg2 = trans_to_str(T0_LH2)
            arg3 = trans_to_str(T0_RH2)
            arg4 = trans_to_str(T0_LFTarget)
            arg5 = trans_to_str(T0_RFTarget)


            goalik = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames+' movecog '+arg1+' nummanips 4 maniptm 0 '+arg2+' maniptm 1 '+arg3+' maniptm 2 '+arg4+' maniptm 3 '+arg5)

            if(goalik == ''):
                print "Error: No goalik found!"
            else:
                print "found goalik"
                # print goalik

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to go to goalik"
                    sys.stdin.readline()

                self.robotid.SetActiveDOFValues(str2num(goalik))
                self.crankid.SetDOFValues([crank_rot],[crankjointind])


                self.crankid.SetDOFValues([0],[crankjointind])
                # self.robotid.SetActiveDOFValues(initconfig)
                self.robotid.SetActiveDOFValues(str2num(initik))

                # bh: both hands
                # lh: left hands
                # rh: right hands
                self.OpenHands("bh","movetraj0_openhands")
                # self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
                # self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
                

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan initconfig --> startik"
                    sys.stdin.readline()

                # Get a trajectory from initial configuration to grasp configuration
                
                goaljoints = deepcopy(startik)
                #goaljoints = str2num(goaljoints)
                
                with self.robotid:
                    try:
                        answer = self.probs_cbirrt.SendCommand('RunCBiRRT psample 0.2 supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringGrasping)
                        # answer = self.probs_cbirrt.SendCommand('RunCBiRRT psample 0.2 supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' '+TSRChainStringGrasping)
                        print "RunCBiRRT answer: ",str(answer)
                    except openrave_exception, e:
                        print "Cannot send command RunCBiRRT: "
                        print e

                try:
                    os.rename("cmovetraj.txt","movetraj1.txt")

                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj1.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj1",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
                except OSError, e:
                    # No file cmovetraj
                    print e

                # The following is the same as commented out try-except section
                traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj1.txt','r').read())   

                self.robotid.GetController().SetPath(traj) 
                self.robotid.WaitForController(0)
                self.robotid.GetController().Reset(0) 

                self.CloseHands("bh","movetraj1_closehands",True)
                # self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
                # self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan startik --> goalik "
                    sys.stdin.readline()

                # Get a trajectory from goalik to grasp configuration
                goaljoints = deepcopy(goalik)
                for i in range(TSRChainMimicDOF):
                    goaljoints += ' 0'

                goaljoints = str2num(goaljoints)

                try:
                    answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(fastsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainString)
                    print "RunCBiRRT answer: ",str(answer)
                except openrave_exception, e:
                    print "Cannot send command RunCBiRRT: "
                    print e

                try:
                    os.rename("cmovetraj.txt","movetraj2.txt")
                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj2.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    # crankJointValsGroup = cs.GetGroupFromName("joint_values crank")
                    # print "crank joint values offset:"
                    # print crankJointValsGroup.offset

                    # crankJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities crank")
                    # print "crank joint velocities offset"
                    # print crankJointVelocitiesGroup.offset



                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj2",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)

                except OSError, e:
                    # No file cmovetraj
                    print e

                try:
                    answer= self.probs_cbirrt.SendCommand('traj movetraj2.txt');
                    answer= self.probs_crankmover.SendCommand('traj movetraj2.txt');
                    self.robotid.WaitForController(0)
                    # debug
                    print "traj call answer: ",str(answer)
                except openrave_exception, e:
                    print e

                self.robotid.GetController().Reset(0)
                self.OpenHands("bh","movetraj2_openhands")
                #self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
                #self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)

                time.sleep(2)

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan goalik --> startik "
                    sys.stdin.readline()

                # goaljoints = str2num(startik)
                goaljoints = deepcopy(startik)

                try:
                    answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetandHead_goal2start)
                    print "RunCBiRRT answer: ",str(answer)
                except openrave_exception, e:
                    print "Cannot send command RunCBiRRT: "
                    print e

                try:
                    os.rename("cmovetraj.txt","movetraj3.txt")
                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj3.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj3",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
                except OSError, e:
                    # No file cmovetraj
                    print e

                try:
                    answer= self.probs_cbirrt.SendCommand('traj movetraj3.txt');
                    self.robotid.WaitForController(0)
                    # debug
                    print "traj call answer: ",str(answer)
                except openrave_exception, e:
                    print e

                self.robotid.GetController().Reset(0)

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan startik --> initconfig "
                    sys.stdin.readline()

                # goaljoints = initconfig
                goaljoints = deepcopy(initik)
                goaljoints = str2num(goaljoints)

                print goaljoints
                try:
                    answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetandHead_goal2start)
                    print "RunCBiRRT answer: ",str(answer)
                except openrave_exception, e:
                    print "Cannot send command RunCBiRRT: "
                    print e

                try:
                    os.rename("cmovetraj.txt","movetraj4.txt")
                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj4.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj4",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
                except OSError, e:
                    # No file cmovetraj
                    print e

                try:
                    answer= self.probs_cbirrt.SendCommand('traj movetraj4.txt');
                    self.robotid.WaitForController(0)
                    # debug
                    print "traj call answer: ",str(answer)
                except openrave_exception, e:
                    print e

                self.robotid.GetController().Reset(0)

                self.CloseHands("bh","movetraj4_closehands")
                # self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
                # self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan initconfig --> home "
                    sys.stdin.readline()

                goaljoints = home

                try:
                    answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetandHead_init2home)
                    print "RunCBiRRT answer: ",str(answer)
                except openrave_exception, e:
                    print "Cannot send command RunCBiRRT: "
                    print e

                try:
                    os.rename("cmovetraj.txt","movetraj5.txt")
                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj5.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj5",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
                except OSError, e:
                    # No file cmovetraj
                    print e

                try:
                    answer=self.probs_cbirrt.SendCommand('traj movetraj5.txt');
                    self.robotid.WaitForController(0)
                    # debug
                    print "traj call answer: ",str(answer)
                except openrave_exception, e:
                    print e

                self.robotid.GetController().Reset(0)
                time.sleep(2)

        ########################################################

    def LeftHand(self,radius,valveType,direction):
        self.robotid.SetActiveManipulator('leftArm')

        self.RemoveFiles()

        # This is a list of handles of the objects that are
        # drawn on the screen in OpenRAVE Qt-Viewer.
        # Keep appending to the end, and pop() if you want to delete.
        handles = [] 

        normalsmoothingitrs = 300;
        fastsmoothingitrs = 20;        

        # Wheel Joint Index  
        crankjointind = 0
        # Set the wheel joints back to 0 for replanning
        self.crankid.SetDOFValues([0],[crankjointind])
        self.crankid.GetController().Reset(0)

        manips = self.robotid.GetManipulators()
        crankmanip = self.crankid.GetManipulators()
        
        self.SetProblems()

        print "Getting Loaded Problems"
        probs = self.env.GetLoadedProblems()

        # Keep Active Joint Indices
        activedofs = []
        for m in manips:
            activedofs.extend(m.GetArmIndices())

        # Sort Active Joint Indices
        activedofs.sort()

        ############################## DRCHUBO V2  ############################
        
        # elbows: Left Elbow Pitch: 3; Right Elbow Pitch: 29
        self.robotid.SetDOFValues([-0.45,-0.45],[3,29]) 
        self.robotid.SetActiveDOFs(activedofs)

        leg_height = self.robotid.GetManipulators()[2].GetEndEffectorTransform()[2,3]
        # Find a trajectory from 0 to initconfig
        if( self.StopAtKeyStrokes ):
            print "Leg height :  " + str(leg_height)
            sys.stdin.readline()

        # polyscale: changes the scale of the support polygon
        # polytrans: shifts the support polygon around
        footlinknames = ' Body_RAR Body_LAR polyscale 0.5 0.5 0 ' #polytrans -0.015 0 0.0 '

        # Center of Gravity Target
        # cogtarg = [-0.05, 0.085, 0]
        cogtarg = [0, 0, 0]

        print "activedofs:"
        print activedofs

        # Current configuration of the robot is its initial configuration
        initconfig = self.robotid.GetActiveDOFValues()
    

        # List of Robot Links
        links = self.robotid.GetLinks()

        # List of Wheel (Crank Links)
        cranklinks = self.crankid.GetLinks()

        # End Effector Transforms
        Tee = []
        for i in range(len(manips)):
            # Returns End Effector Transform in World Coordinates
            Tlink = manips[i].GetEndEffectorTransform()
            # print Tlink
            Tee.append(Tlink)

        # Wheel Joint Index  
        crankjointind = 0

        # Get Transformation Matrix for the Wheel
        # Note that crank's links are not rotated
        # If you want use the wheel's end effector's transformation
        # matrix (which is 23 degrees tilted) then see
        # CTee matrix below.
        #
        # crank has two links: 
        # 0) pole - the blue cylinder in the model, and, 
        # 1) crank - the driving wheel itself.
        jointtm = cranklinks[0].GetTransform()
        # handles.append(misc.DrawAxes(self.env,matrix(jointtm),1))


        # We can also get the transformation matrix
        # with the following command as a string
        # jointtm_str = self.probs_cbirrt.SendCommand('GetJointTransform name crank jointind '+str(crankjointind))
        # And then we can convert the string to a 1x12 array
        # jointtm_str = jointtm_str.replace(" ",",")
        # jointtm_num = # eval('['+jointtm_str+']')

        # In this script we will use jointtm.
        # jointtm_str and jointtm_num are given as example.

        # Crank Transform End Effector in World Coordinates
        # This is the transformation matrix of the end effector 
        # named "dummy" in the xml file.
        # Note that dummy is tilted 23 degress around its X-Axis
        CTee = crankmanip[0].GetEndEffectorTransform()

        tilt_angle_deg = acos(dot(linalg.inv(CTee),jointtm)[1,1])*180/pi
        tilt_angle_rad = acos(dot(linalg.inv(CTee),jointtm)[1,1])

        # What is this?
        handrot = rodrigues([0,-pi/2,0])

        # Translation Offset from the wheel center for the hands
        transoffset = [0, 0.15, 0];

        # Figure out where to put the left hand on the wheel
        temp = dot(CTee, MakeTransform(rodrigues([0,0,pi/2]),transpose(matrix([0,0,0]))))
        temp = dot(temp, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))

        # Left Hand Pose in World Coordinates
        if(valveType == "l"): # if lever, hold it from the tip of the handle
            T0_LH1 = dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.01,self.r_Wheel-0.005]))))
        if(valveType == "w"): # if it's a small wheel, hold it from the center but back off a little
            T0_LH1 = dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.02,0]))))

        # TODO: Error checking! What if we accidentally send a huge valve to 1 hand?


        # Uncomment if you want to see where T0_LH1 is 
        handles.append(misc.DrawAxes(self.env,matrix(T0_LH1),1))

        # Uncomment if you want to see where T0_RH1 is 
        # handles.append(misc.DrawAxes(self.env,matrix(T0_RH1),1))

        # Define Task Space Region strings
        # Left Hand
        TSRString1 = SerializeTSR(0,'NULL',T0_LH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # Right Hand
        TSRString2 = SerializeTSR(1,'NULL',Tee[1],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # Left Foot
        TSRString3 = SerializeTSR(2,'NULL',Tee[2],eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))
        
        # Right Foot
        TSRString4 = SerializeTSR(3,'NULL',Tee[3],eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))

        # Head
        # Grasp transform in Head coordinates
        Tw0_eH = eye(4) 
        # How much freedom do we want to give to the Head
        # [x,x,y,y,z,z,R,R,P,P,Y,Y]
        Bw0H = matrix([-0.05,0.05,-0.1,0.1,-100,100,-pi,pi,-pi,pi,-pi,pi])
        TSRString5 = SerializeTSR(4,'NULL',Tee[4],Tw0_eH,Bw0H)

        TSRChainStringFeetandHead_home2init = SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])
        TSRChainStringFeetandHead_init2home = deepcopy(TSRChainStringFeetandHead_home2init)

        
        [T0_LFTarget, T0_RFTarget] = self.GetFeetTargets()

        TSRString3 = SerializeTSR(2,'NULL',T0_LFTarget,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRString4 = SerializeTSR(3,'NULL',T0_RFTarget,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # We defined Task Space Regions. Now let's concatenate them.
        TSRChainStringGrasping = SerializeTSRChain(0,1,0,1,TSRString1,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString2,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString4,'NULL',[])#+' '+SerializeTSRChain(0,1,0,1,TSRString5,'NULL',[])

        # Go home
        if( self.StopAtKeyStrokes ):
            print "Press Enter to go home"
            sys.stdin.readline()

        self.BendTheKnees()

        arg1 = str(cogtarg).strip("[]").replace(', ',' ')

        initik = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames+' movecog '+arg1+'nummanips 2 maniptm 2 '+trans_to_str(T0_LFTarget)+' maniptm 3 '+trans_to_str(T0_RFTarget))

        sys.stdin.readline()

        self.robotid.SetDOFValues(zeros(len(self.robotid.GetJoints())),range(len(self.robotid.GetJoints())))
        home = self.robotid.GetActiveDOFValues()

        # Find a trajectory from 0 to initconfig
        if( self.StopAtKeyStrokes ):
            print "Press Enter to plan home --> initconfig "
            sys.stdin.readline()

        # goaljoints = initconfig
        goaljoints = deepcopy(initik)
        goaljoints = str2num(goaljoints)

        try:
            answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetandHead_home2init)
            print "RunCBiRRT answer: ",str(answer)
        except openrave_exception, e:
            print "Cannot send command RunCBiRRT: "
            print e

        try:
            os.rename("cmovetraj.txt","movetraj0.txt")
            traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj0.txt','r').read())

            # print "conf spec for start2goal:"
            cs = traj.GetConfigurationSpecification()

            drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
            # print "drchubo-v2 joint values offset"
            # print drchuboJointValsGroup.offset

            drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
            # print "drchubo-v2 joint velocities offset"
            # print drchuboJointVelocitiesGroup.offset

            deltatimeGroup = cs.GetGroupFromName("deltatime")
            # print "deltatime offset"
            # print deltatimeGroup.offset

            rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj0",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
        except OSError, e:
            # No file cmovetraj
            print e

        try:
            answer=self.probs_cbirrt.SendCommand('traj movetraj0.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e

        self.robotid.GetController().Reset(0)
        time.sleep(2)

        T0_RH1 = self.robotid.GetManipulators()[1].GetEndEffectorTransform()
        handles.append(misc.DrawAxes(self.env,T0_RH1,1))
        # print "TSRChainStringGrasping"
        # print TSRChainStringGrasping

        if( self.StopAtKeyStrokes ):
            print "Press Enter to go to startik"
            sys.stdin.readline()

        arg1 = str(cogtarg).strip("[]").replace(', ',' ')
        
        # GeneralIK does not check for collision, so let's make sure that we find IKs that don't collide
        # with the environment
        # self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
        self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
        
        self.robotid.SetActiveManipulator('leftArm')
        leftArmIkmodel = databases.inversekinematics.InverseKinematicsModel(self.robotid,iktype=IkParameterizationType.Transform6D)
        leftArmIkmodel.load()
        # check if ik solutions exist
        sol0=leftArmIkmodel.manip.FindIKSolution(array(T0_LH1),IkFilterOptions.CheckEnvCollisions)
        print "sol0"
        print len(sol0)
        print sol0
        assert(sol0 is not None)
        self.robotid.SetDOFValues(sol0,self.robotid.GetManipulators()[0].GetArmIndices())
        sys.stdin.readline()
         
        startik = self.robotid.GetActiveDOFValues()
        
        if(startik == ''):
            print "Error: could not find startik"
        else:
            print "found startik"
            # print startik

            ##################
            
            #print "Checking support in startik: "
            #print self.probs_cbirrt.SendCommand('CheckSupport')

            # Rotate the wheel's transform to a suitable pose
            # for the Left Hand
            # T0_w0L stands for: 
            # left hand's transform on wheel in world coordinates

            T0_w0L = MakeTransform(rodrigues([0,tilt_angle_rad-self.tiltDiff+self.worldPitch,0]),transpose(matrix(jointtm[0:3,3])))
            #T0_w0L = dot(temp,MakeTransform(rodrigues([0,0,0]),transpose(matrix(jointtm[0:3,3]))))

            #T0_w0L = MakeTransform(rodrigues([acos(self.crankid.GetManipulators()[0].GetTransform()[1,1]),0,0]),transpose(matrix(jointtm[0:3,3])))
            # This is what's happening: 
            #
            # Tw0L_0 = linalg.inv(T0_w0L)
            # Tw0L_LH1 = Tw0L_0*T0_LH1
            #
            # Left hand's transform in wheel's coordinates
            Tw0L_LH1 = dot(linalg.inv(T0_w0L),T0_LH1)
            # Transform of the left hand's end effector in wheel's coords.
            # Required by CBiRRT
            Tw0_eL = Tw0L_LH1
            # How much freedom do we want to give to the left hand
            Bw0L = matrix([0,0,0,0,0,0,-pi,pi,0,0,0,0])

            # Right Hand's transforms:
            T0_crankcrank = self.crankid.GetManipulators()[0].GetTransform()

            # Head's transforms:
            T0_w0H =  Tee[4]
            Tw0_eH = eye(4);
            Bw0H = matrix([-0.05,0.05,-0.1,0.1,-100,100,-pi,pi,-pi,pi,-pi,pi])

            # Define Task Space Regions
            # Left Hand
            TSRString1 = SerializeTSR(0,'NULL',T0_w0L,Tw0_eL,Bw0L)
            # Right Hand
            TSRString2 = SerializeTSR(1,'NULL',T0_RH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
            # Left Foot
            # TSRString3 = SerializeTSR(2,'NULL',Tee[2],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
            TSRString3 = SerializeTSR(2,'NULL',T0_LFTarget,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
            # Right Foot
            # TSRString4 = SerializeTSR(3,'NULL',Tee[3],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
            TSRString4 = SerializeTSR(3,'NULL',T0_RFTarget,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
            # Head
            TSRString5 = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H)

            TSRChainStringFeetHeadandRightHand_start2init = SerializeTSRChain(0,0,1,1,TSRString2,'NULL',[])+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

            TSRChainStringFeetRightHandandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString2,'NULL',[])+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

            TSRChainStringFeetandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])
            
            TSRChainString = SerializeTSRChain(0,0,1,1,TSRString1,'crank',matrix([crankjointind]))+' '+SerializeTSRChain(0,0,1,1,TSRString2,'NULL',matrix([]))+' '+TSRChainStringFeetandHead_goal2start

            # Calculate hand transforms after rotating the wheel (they will help us find the goalik):
            # How much do we want to rotate the wheel?
            if(direction == "ccw"):
                multiplier = -1
            elif(direction == "cw"):
                multiplier = 1

            crank_rot = (multiplier)*pi/4

            # Which joint do we want the CBiRRT to mimic the TSR for?
            TSRChainMimicDOF = 1

            # Create the transform for the wheel that we would like to reach to
            Tcrank_rot = MakeTransform(rodrigues([crank_rot,0,0]),transpose(matrix([0,0,0])))

            # What is this?
            temp = MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0])))

            # Rotate the left hand's transform on the wheel in world transform "crank_rot" radians around it's Z-Axis
            T0_cranknew = dot(T0_w0L,Tcrank_rot)

            # Where will the left hand go after turning the wheel?
            # This is what's happening:
            #
            # Tcranknew_LH2 = dot(Tw0L_0,T0_LH1) --> Left hand in wheel's coordinate
            # T0_LH2 = dot(T0_cranknew,Tcranknew_LH2) --> Left hand rotated around wheel's origin
            T0_LH2 = dot(T0_cranknew,dot(linalg.inv(T0_w0L),T0_LH1))

            # Uncomment to see T0_LH2
            handles.append(misc.DrawAxes(self.env,matrix(T0_LH2),1))

            ################################

            handles.append(misc.DrawAxes(self.env,matrix(T0_LH2),1))

            # check if ik solutions exist
            sol1=leftArmIkmodel.manip.FindIKSolution(array(T0_LH2),IkFilterOptions.CheckEnvCollisions)
            print "sol1"
            print len(sol1)
            print sol1
            assert(sol1 is not None)
            self.robotid.SetDOFValues(sol1,self.robotid.GetManipulators()[0].GetArmIndices())
            sys.stdin.readline()
        
            goalik = self.robotid.GetActiveDOFValues()

            if(goalik == ''):
                print "Error: No goalik found!"
            else:
                print "found goalik"
                # print goalik

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to go to goalik"
                    sys.stdin.readline()


                self.robotid.SetActiveDOFValues(goalik)
                self.crankid.SetDOFValues([crank_rot],[crankjointind])


                self.crankid.SetDOFValues([0],[crankjointind])
                # self.robotid.SetActiveDOFValues(initconfig)
                self.robotid.SetActiveDOFValues(str2num(initik))

                # bh: both hands
                # lh: left hands
                # rh: right hands
                self.OpenHands("lh","movetraj0_openhands")
                # self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
                # self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
                

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan initconfig --> startik"
                    sys.stdin.readline()

                # Get a trajectory from initial configuration to grasp configuration
                
                goaljoints = deepcopy(startik)
                #goaljoints = str2num(goaljoints)
                
                with self.robotid:
                    try:
                        answer = self.probs_cbirrt.SendCommand('RunCBiRRT psample 0.2 supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringGrasping)
                        # answer = self.probs_cbirrt.SendCommand('RunCBiRRT psample 0.2 supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' '+TSRChainStringGrasping)
                        print "RunCBiRRT answer: ",str(answer)
                    except openrave_exception, e:
                        print "Cannot send command RunCBiRRT: "
                        print e

                try:
                    os.rename("cmovetraj.txt","movetraj1.txt")

                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj1.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj1",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
                except OSError, e:
                    # No file cmovetraj
                    print e

                # The following is the same as commented out try-except section
                traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj1.txt','r').read())   

                self.robotid.GetController().SetPath(traj) 
                self.robotid.WaitForController(0)
                self.robotid.GetController().Reset(0) 

                self.CloseHands("lh","movetraj1_closehands",True)
                # self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
                # self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan startik --> goalik "
                    sys.stdin.readline()

                # Get a trajectory from goalik to grasp configuration
                goaljoints = deepcopy(goalik)

                goaljoints = goaljoints.tolist()

                for i in range(TSRChainMimicDOF):
                    goaljoints.append(0)
                
                try:
                    answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(fastsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainString)
                    print "RunCBiRRT answer: ",str(answer)
                except openrave_exception, e:
                    print "Cannot send command RunCBiRRT: "
                    print e

                try:
                    os.rename("cmovetraj.txt","movetraj2.txt")
                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj2.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    # crankJointValsGroup = cs.GetGroupFromName("joint_values crank")
                    # print "crank joint values offset:"
                    # print crankJointValsGroup.offset

                    # crankJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities crank")
                    # print "crank joint velocities offset"
                    # print crankJointVelocitiesGroup.offset



                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj2",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)

                except OSError, e:
                    # No file cmovetraj
                    print e

                try:
                    answer= self.probs_cbirrt.SendCommand('traj movetraj2.txt');
                    answer= self.probs_crankmover.SendCommand('traj movetraj2.txt');
                    self.robotid.WaitForController(0)
                    # debug
                    print "traj call answer: ",str(answer)
                except openrave_exception, e:
                    print e

                self.robotid.GetController().Reset(0)
                self.OpenHands("lh","movetraj2_openhands")
                #self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
                #self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)

                time.sleep(2)

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan goalik --> startik "
                    sys.stdin.readline()

                # goaljoints = str2num(startik)
                goaljoints = deepcopy(startik)

                try:
                    answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetRightHandandHead_goal2start)
                    print "RunCBiRRT answer: ",str(answer)
                except openrave_exception, e:
                    print "Cannot send command RunCBiRRT: "
                    print e

                try:
                    os.rename("cmovetraj.txt","movetraj3.txt")
                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj3.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj3",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
                except OSError, e:
                    # No file cmovetraj
                    print e

                try:
                    answer= self.probs_cbirrt.SendCommand('traj movetraj3.txt');
                    self.robotid.WaitForController(0)
                    # debug
                    print "traj call answer: ",str(answer)
                except openrave_exception, e:
                    print e

                self.robotid.GetController().Reset(0)

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan startik --> initconfig "
                    sys.stdin.readline()

                # goaljoints = initconfig
                goaljoints = deepcopy(initik)
                goaljoints = str2num(goaljoints)

                print goaljoints
                try:
                    answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetHeadandRightHand_start2init)
                    print "RunCBiRRT answer: ",str(answer)
                except openrave_exception, e:
                    print "Cannot send command RunCBiRRT: "
                    print e

                try:
                    os.rename("cmovetraj.txt","movetraj4.txt")
                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj4.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj4",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
                except OSError, e:
                    # No file cmovetraj
                    print e

                try:
                    answer= self.probs_cbirrt.SendCommand('traj movetraj4.txt');
                    self.robotid.WaitForController(0)
                    # debug
                    print "traj call answer: ",str(answer)
                except openrave_exception, e:
                    print e

                self.robotid.GetController().Reset(0)

                self.CloseHands("lh","movetraj4_closehands")
                # self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
                # self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan initconfig --> home "
                    sys.stdin.readline()

                goaljoints = home

                try:
                    answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetandHead_init2home)
                    print "RunCBiRRT answer: ",str(answer)
                except openrave_exception, e:
                    print "Cannot send command RunCBiRRT: "
                    print e

                try:
                    os.rename("cmovetraj.txt","movetraj5.txt")
                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj5.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj5",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
                except OSError, e:
                    # No file cmovetraj
                    print e

                try:
                    answer=self.probs_cbirrt.SendCommand('traj movetraj5.txt');
                    self.robotid.WaitForController(0)
                    # debug
                    print "traj call answer: ",str(answer)
                except openrave_exception, e:
                    print e

                self.robotid.GetController().Reset(0)
                time.sleep(2)

        ########################################################
    
    def RightHand(self,radius,valveType,direction):
        self.robotid.SetActiveManipulator('rightArm')

        self.RemoveFiles()

        # This is a list of handles of the objects that are
        # drawn on the screen in OpenRAVE Qt-Viewer.
        # Keep appending to the end, and pop() if you want to delete.
        handles = [] 

        normalsmoothingitrs = 300;
        fastsmoothingitrs = 20;        

        # Wheel Joint Index  
        crankjointind = 0
        # Set the wheel joints back to 0 for replanning
        self.crankid.SetDOFValues([0],[crankjointind])
        self.crankid.GetController().Reset(0)

        manips = self.robotid.GetManipulators()
        crankmanip = self.crankid.GetManipulators()
        
        self.SetProblems()

        print "Getting Loaded Problems"
        probs = self.env.GetLoadedProblems()

        # Keep Active Joint Indices
        activedofs = []
        for m in manips:
            activedofs.extend(m.GetArmIndices())

        # Sort Active Joint Indices
        activedofs.sort()

        ############################## DRCHUBO V2  ############################
        
        # elbows: Left Elbow Pitch: 3; Right Elbow Pitch: 29
        self.robotid.SetDOFValues([-0.45,-0.45],[3,29]) 
        self.robotid.SetActiveDOFs(activedofs)

        leg_height = self.robotid.GetManipulators()[2].GetEndEffectorTransform()[2,3]
        # Find a trajectory from 0 to initconfig
        if( self.StopAtKeyStrokes ):
            print "Leg height :  " + str(leg_height)
            sys.stdin.readline()

        # polyscale: changes the scale of the support polygon
        # polytrans: shifts the support polygon around
        footlinknames = ' Body_RAR Body_LAR polyscale 0.5 0.5 0 ' #polytrans -0.015 0 0.0 '

        # Center of Gravity Target
        # cogtarg = [-0.05, 0.085, 0]
        cogtarg = [0, 0, 0]

        print "activedofs:"
        print activedofs

        # Current configuration of the robot is its initial configuration
        initconfig = self.robotid.GetActiveDOFValues()
    

        # List of Robot Links
        links = self.robotid.GetLinks()

        # List of Wheel (Crank Links)
        cranklinks = self.crankid.GetLinks()

        # End Effector Transforms
        Tee = []
        for i in range(len(manips)):
            # Returns End Effector Transform in World Coordinates
            Tlink = manips[i].GetEndEffectorTransform()
            # print Tlink
            Tee.append(Tlink)

        # Wheel Joint Index  
        crankjointind = 0

        # Get Transformation Matrix for the Wheel
        # Note that crank's links are not rotated
        # If you want use the wheel's end effector's transformation
        # matrix (which is 23 degrees tilted) then see
        # CTee matrix below.
        #
        # crank has two links: 
        # 0) pole - the blue cylinder in the model, and, 
        # 1) crank - the driving wheel itself.
        jointtm = cranklinks[0].GetTransform()
        # handles.append(misc.DrawAxes(self.env,matrix(jointtm),1))


        # We can also get the transformation matrix
        # with the following command as a string
        # jointtm_str = self.probs_cbirrt.SendCommand('GetJointTransform name crank jointind '+str(crankjointind))
        # And then we can convert the string to a 1x12 array
        # jointtm_str = jointtm_str.replace(" ",",")
        # jointtm_num = # eval('['+jointtm_str+']')

        # In this script we will use jointtm.
        # jointtm_str and jointtm_num are given as example.

        # Crank Transform End Effector in World Coordinates
        # This is the transformation matrix of the end effector 
        # named "dummy" in the xml file.
        # Note that dummy is tilted 23 degress around its X-Axis
        CTee = crankmanip[0].GetEndEffectorTransform()

        tilt_angle_deg = acos(dot(linalg.inv(CTee),jointtm)[1,1])*180/pi
        tilt_angle_rad = acos(dot(linalg.inv(CTee),jointtm)[1,1])

        # What is this?
        handrot = rodrigues([0,-pi/2,0])

        # Translation Offset from the wheel center for the hands
        transoffset = [0, 0.15, 0];

        # Figure out where to put the left hand on the wheel
        temp = dot(CTee, MakeTransform(rodrigues([0,0,pi/2]),transpose(matrix([0,0,0]))))
        temp = dot(temp, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))

        # Left Hand Pose in World Coordinates
        if(valveType == "l"): # if lever, hold it from the tip of the handle
            T0_RH1 = dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.01,self.r_Wheel-0.005]))))
        if(valveType == "w"): # if it's a small wheel, hold it from the center but back off a little
            T0_RH1 = dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.02,0]))))

        # TODO: Error checking! What if we accidentally send a huge valve to 1 hand?


        # Uncomment if you want to see where T0_RH1 is 
        handles.append(misc.DrawAxes(self.env,matrix(T0_RH1),1))

        # Define Task Space Region strings
        # Left Hand
        TSRString1 = SerializeTSR(0,'NULL',Tee[0],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # Right Hand
        TSRString2 = SerializeTSR(1,'NULL',T0_RH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # Left Foot
        TSRString3 = SerializeTSR(2,'NULL',Tee[2],eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))
        
        # Right Foot
        TSRString4 = SerializeTSR(3,'NULL',Tee[3],eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))

        # Head
        # Grasp transform in Head coordinates
        Tw0_eH = eye(4) 
        # How much freedom do we want to give to the Head
        # [x,x,y,y,z,z,R,R,P,P,Y,Y]
        Bw0H = matrix([-0.05,0.05,-0.1,0.1,-100,100,-pi,pi,-pi,pi,-pi,pi])
        TSRString5 = SerializeTSR(4,'NULL',Tee[4],Tw0_eH,Bw0H)

        TSRChainStringFeetandHead_home2init = SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])
        TSRChainStringFeetandHead_init2home = deepcopy(TSRChainStringFeetandHead_home2init)

        
        [T0_LFTarget, T0_RFTarget] = self.GetFeetTargets()

        TSRString3 = SerializeTSR(2,'NULL',T0_LFTarget,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRString4 = SerializeTSR(3,'NULL',T0_RFTarget,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # We defined Task Space Regions. Now let's concatenate them.
        TSRChainStringGrasping = SerializeTSRChain(0,1,1,1,TSRString1,'NULL',[])+' '+SerializeTSRChain(0,1,0,1,TSRString2,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString4,'NULL',[])#+' '+SerializeTSRChain(0,1,0,1,TSRString5,'NULL',[])

        # Go home
        if( self.StopAtKeyStrokes ):
            print "Press Enter to go home"
            sys.stdin.readline()

        self.BendTheKnees()

        arg1 = str(cogtarg).strip("[]").replace(', ',' ')

        initik = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+footlinknames+' movecog '+arg1+'nummanips 2 maniptm 2 '+trans_to_str(T0_LFTarget)+' maniptm 3 '+trans_to_str(T0_RFTarget))

        sys.stdin.readline()

        self.robotid.SetDOFValues(zeros(len(self.robotid.GetJoints())),range(len(self.robotid.GetJoints())))
        home = self.robotid.GetActiveDOFValues()

        # Find a trajectory from 0 to initconfig
        if( self.StopAtKeyStrokes ):
            print "Press Enter to plan home --> initconfig "
            sys.stdin.readline()

        # goaljoints = initconfig
        goaljoints = deepcopy(initik)
        goaljoints = str2num(goaljoints)

        try:
            answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetandHead_home2init)
            print "RunCBiRRT answer: ",str(answer)
        except openrave_exception, e:
            print "Cannot send command RunCBiRRT: "
            print e

        try:
            os.rename("cmovetraj.txt","movetraj0.txt")
            traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj0.txt','r').read())

            # print "conf spec for start2goal:"
            cs = traj.GetConfigurationSpecification()

            drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
            # print "drchubo-v2 joint values offset"
            # print drchuboJointValsGroup.offset

            drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
            # print "drchubo-v2 joint velocities offset"
            # print drchuboJointVelocitiesGroup.offset

            deltatimeGroup = cs.GetGroupFromName("deltatime")
            # print "deltatime offset"
            # print deltatimeGroup.offset

            rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj0",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
        except OSError, e:
            # No file cmovetraj
            print e

        try:
            answer=self.probs_cbirrt.SendCommand('traj movetraj0.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e

        self.robotid.GetController().Reset(0)
        time.sleep(2)

        T0_LH1 = self.robotid.GetManipulators()[0].GetEndEffectorTransform()
        handles.append(misc.DrawAxes(self.env,T0_LH1,1))
        # print "TSRChainStringGrasping"
        # print TSRChainStringGrasping

        if( self.StopAtKeyStrokes ):
            print "Press Enter to go to startik"
            sys.stdin.readline()

        arg1 = str(cogtarg).strip("[]").replace(', ',' ')
        
        # GeneralIK does not check for collision, so let's make sure that we find IKs that don't collide
        # with the environment
        self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
        #self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
        
        self.robotid.SetActiveManipulator('rightArm')
        rightArmIkmodel = databases.inversekinematics.InverseKinematicsModel(self.robotid,iktype=IkParameterizationType.Transform6D)
        rightArmIkmodel.load()
        # check if ik solutions exist
        sol0=rightArmIkmodel.manip.FindIKSolution(array(T0_RH1),IkFilterOptions.CheckEnvCollisions)
        print "sol0"
        print len(sol0)
        print sol0
        assert(sol0 is not None)
        self.robotid.SetDOFValues(sol0,self.robotid.GetManipulators()[1].GetArmIndices())
        sys.stdin.readline()
         
        startik = self.robotid.GetActiveDOFValues()
        
        if(startik == ''):
            print "Error: could not find startik"
        else:
            print "found startik"
            # print startik

            ##################

            T0_w0R = MakeTransform(rodrigues([0,tilt_angle_rad-self.tiltDiff+self.worldPitch,0]),transpose(matrix(jointtm[0:3,3])))

            Tw0R_RH1 = dot(linalg.inv(T0_w0R),T0_RH1)

            Tw0_eR = Tw0R_RH1


            Bw0R = matrix([0,0,0,0,0,0,-pi,pi,0,0,0,0])

            # Right Hand's transforms:
            T0_crankcrank = self.crankid.GetManipulators()[0].GetTransform()

            # Head's transforms:
            T0_w0H =  Tee[4]
            Tw0_eH = eye(4);
            Bw0H = matrix([-0.05,0.05,-0.1,0.1,-100,100,-pi,pi,-pi,pi,-pi,pi])

            # Define Task Space Regions
            # Left Hand
            TSRString1 = SerializeTSR(0,'NULL',T0_LH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
            # Right Hand
            TSRString2 = SerializeTSR(1,'NULL',T0_w0R,Tw0_eR,Bw0R)
            # Left Foot
            TSRString3 = SerializeTSR(2,'NULL',T0_LFTarget,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
            # Right Foot
            TSRString4 = SerializeTSR(3,'NULL',T0_RFTarget,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
            # Head
            TSRString5 = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H)

            TSRChainStringFeetHeadandLeftHand_start2init = SerializeTSRChain(0,0,1,1,TSRString1,'NULL',[])+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

            TSRChainStringFeetandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])
            
            TSRChainStringFeetLeftHandandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString1,'NULL',[])+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

            TSRChainString = SerializeTSRChain(0,0,1,1,TSRString1,'NULL',matrix([]))+' '+SerializeTSRChain(0,0,1,1,TSRString2,'crank',matrix([crankjointind]))+' '+TSRChainStringFeetandHead_goal2start

            
            if(direction == "ccw"):
                multiplier = -1
            elif(direction == "cw"):
                multiplier = 1

            crank_rot = (multiplier)*pi/4

            # Which joint do we want the CBiRRT to mimic the TSR for?
            TSRChainMimicDOF = 1

            # Create the transform for the wheel that we would like to reach to
            Tcrank_rot = MakeTransform(rodrigues([crank_rot,0,0]),transpose(matrix([0,0,0])))

            # What is this?
            temp = MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0])))

            # Rotate the left hand's transform on the wheel in world transform "crank_rot" radians around it's Z-Axis
            T0_cranknew = dot(T0_w0R,Tcrank_rot)

            T0_RH2 = dot(T0_cranknew,dot(linalg.inv(T0_w0R),T0_RH1))

            # Uncomment to see T0_RH2
            handles.append(misc.DrawAxes(self.env,matrix(T0_RH2),1))

            ################################

            # check if ik solutions exist
            sol1=rightArmIkmodel.manip.FindIKSolution(array(T0_RH2),IkFilterOptions.CheckEnvCollisions)
            print "sol1"
            print len(sol1)
            print sol1
            assert(sol1 is not None)
            self.robotid.SetDOFValues(sol1,self.robotid.GetManipulators()[1].GetArmIndices())
            sys.stdin.readline()
        
            goalik = self.robotid.GetActiveDOFValues()

            if(goalik == ''):
                print "Error: No goalik found!"
            else:
                print "found goalik"
                # print goalik

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to go to goalik"
                    sys.stdin.readline()


                self.robotid.SetActiveDOFValues(goalik)
                self.crankid.SetDOFValues([crank_rot],[crankjointind])


                self.crankid.SetDOFValues([0],[crankjointind])
                # self.robotid.SetActiveDOFValues(initconfig)
                self.robotid.SetActiveDOFValues(str2num(initik))

                # bh: both hands
                # lh: left hands
                # rh: right hands
                self.OpenHands("rh","movetraj0_openhands")
                # self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
                # self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
                

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan initconfig --> startik"
                    sys.stdin.readline()

                # Get a trajectory from initial configuration to grasp configuration
                
                goaljoints = deepcopy(startik)
                #goaljoints = str2num(goaljoints)
                
                with self.robotid:
                    try:
                        answer = self.probs_cbirrt.SendCommand('RunCBiRRT psample 0.2 supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringGrasping)
                        # answer = self.probs_cbirrt.SendCommand('RunCBiRRT psample 0.2 supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' '+TSRChainStringGrasping)
                        print "RunCBiRRT answer: ",str(answer)
                    except openrave_exception, e:
                        print "Cannot send command RunCBiRRT: "
                        print e

                try:
                    os.rename("cmovetraj.txt","movetraj1.txt")

                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj1.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj1",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
                except OSError, e:
                    # No file cmovetraj
                    print e

                # The following is the same as commented out try-except section
                traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj1.txt','r').read())   

                self.robotid.GetController().SetPath(traj) 
                self.robotid.WaitForController(0)
                self.robotid.GetController().Reset(0) 

                self.CloseHands("rh","movetraj1_closehands",True)
                # self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
                # self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan startik --> goalik "
                    sys.stdin.readline()

                # Get a trajectory from goalik to grasp configuration
                goaljoints = deepcopy(goalik)

                goaljoints = goaljoints.tolist()

                for i in range(TSRChainMimicDOF):
                    goaljoints.append(0)
                
                try:
                    answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(fastsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainString)
                    print "RunCBiRRT answer: ",str(answer)
                except openrave_exception, e:
                    print "Cannot send command RunCBiRRT: "
                    print e

                try:
                    os.rename("cmovetraj.txt","movetraj2.txt")
                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj2.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    # crankJointValsGroup = cs.GetGroupFromName("joint_values crank")
                    # print "crank joint values offset:"
                    # print crankJointValsGroup.offset

                    # crankJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities crank")
                    # print "crank joint velocities offset"
                    # print crankJointVelocitiesGroup.offset



                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj2",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)

                except OSError, e:
                    # No file cmovetraj
                    print e

                try:
                    answer= self.probs_cbirrt.SendCommand('traj movetraj2.txt');
                    answer= self.probs_crankmover.SendCommand('traj movetraj2.txt');
                    self.robotid.WaitForController(0)
                    # debug
                    print "traj call answer: ",str(answer)
                except openrave_exception, e:
                    print e

                self.robotid.GetController().Reset(0)
                self.OpenHands("rh","movetraj2_openhands")
                #self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
                #self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)

                time.sleep(2)

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan goalik --> startik "
                    sys.stdin.readline()

                # goaljoints = str2num(startik)
                goaljoints = deepcopy(startik)

                try:
                    answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetLeftHandandHead_goal2start)
                    print "RunCBiRRT answer: ",str(answer)
                except openrave_exception, e:
                    print "Cannot send command RunCBiRRT: "
                    print e

                try:
                    os.rename("cmovetraj.txt","movetraj3.txt")
                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj3.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj3",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
                except OSError, e:
                    # No file cmovetraj
                    print e

                try:
                    answer= self.probs_cbirrt.SendCommand('traj movetraj3.txt');
                    self.robotid.WaitForController(0)
                    # debug
                    print "traj call answer: ",str(answer)
                except openrave_exception, e:
                    print e

                self.robotid.GetController().Reset(0)

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan startik --> initconfig "
                    sys.stdin.readline()

                # goaljoints = initconfig
                goaljoints = deepcopy(initik)
                goaljoints = str2num(goaljoints)

                print goaljoints
                try:
                    answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetHeadandLeftHand_start2init)
                    print "RunCBiRRT answer: ",str(answer)
                except openrave_exception, e:
                    print "Cannot send command RunCBiRRT: "
                    print e

                try:
                    os.rename("cmovetraj.txt","movetraj4.txt")
                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj4.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj4",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
                except OSError, e:
                    # No file cmovetraj
                    print e

                try:
                    answer= self.probs_cbirrt.SendCommand('traj movetraj4.txt');
                    self.robotid.WaitForController(0)
                    # debug
                    print "traj call answer: ",str(answer)
                except openrave_exception, e:
                    print e

                self.robotid.GetController().Reset(0)

                self.CloseHands("rh","movetraj4_closehands")
                # self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
                # self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)

                if( self.StopAtKeyStrokes ):
                    print "Press Enter to plan initconfig --> home "
                    sys.stdin.readline()

                goaljoints = home

                try:
                    answer = self.probs_cbirrt.SendCommand('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFeetandHead_init2home)
                    print "RunCBiRRT answer: ",str(answer)
                except openrave_exception, e:
                    print "Cannot send command RunCBiRRT: "
                    print e

                try:
                    os.rename("cmovetraj.txt","movetraj5.txt")
                    traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj5.txt','r').read())

                    # print "conf spec for start2goal:"
                    cs = traj.GetConfigurationSpecification()

                    drchuboJointValsGroup = cs.GetGroupFromName("joint_values drchubo-v2")
                    # print "drchubo-v2 joint values offset"
                    # print drchuboJointValsGroup.offset

                    drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities drchubo-v2")
                    # print "drchubo-v2 joint velocities offset"
                    # print drchuboJointVelocitiesGroup.offset

                    deltatimeGroup = cs.GetGroupFromName("deltatime")
                    # print "deltatime offset"
                    # print deltatimeGroup.offset

                    rave2realhubo.traj2ach(self.env,self.robotid,traj,"movetraj5",drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
                except OSError, e:
                    # No file cmovetraj
                    print e

                try:
                    answer=self.probs_cbirrt.SendCommand('traj movetraj5.txt');
                    self.robotid.WaitForController(0)
                    # debug
                    print "traj call answer: ",str(answer)
                except openrave_exception, e:
                    print e

                self.robotid.GetController().Reset(0)
                time.sleep(2)

        ########################################################

    def MaxBend(self):
        pass
    
    def MaxCrouch(self):
        pass

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
        sys.stdin.readline()
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
            
        sys.stdin.readline()

    def Run(self,handles=[],radius=None,manipulator=None,direction="cw",valveType=None):

        if(radius != None):
            self.r_Wheel = radius

        self.StartViewerAndSetValvePos( handles, valveType)

        if(self.optWall):
            self.AddWall()

        if(manipulator=="lh"):
            self.LeftHand(radius,valveType,direction)
        elif(manipulator=="rh"):
            self.RightHand(radius,valveType,direction)
        elif(manipulator=="bh"):
            self.BothHands(radius,valveType,direction)
        return self.Playback()

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
    
    if demo:
        # Let's show off our task wall
        planner.InitFromTaskWallEnv()

        # get the valves defined in the env file
        valves = planner.env.GetRobots()
        T1_1 = valves[2].GetTransform()
        T2_1 = valves[3].GetTransform()
        T3_1 = valves[4].GetTransform()
        T4_1 = valves[5].GetTransform()
        T5_1 = valves[6].GetTransform()
        T6_1 = valves[7].GetTransform()
        T7_1 = valves[8].GetTransform()

        # Put the wheel in 8 different poses (4 flat, 4 straight-up)
        #
        # Set robot's transform accordingly
        #
        # demonstrate RH - lever
        planner.SetWheelPoseFromTransform(T7_1)
        planner.Run(handles, radius=0.2, manipulator="rh", valveType="l")

        # demonstrate RH - wheel
        planner.SetWheelPoseFromTransform(T2_1)
        planner.Run(handles, radius=0.2, manipulator="rh", valveType="w")

        # demonstrate LH - wheel
        planner.SetWheelPoseFromTransform(T3_1)
        planner.Run(handles, radius=0.05, manipulator="lh", valveType="w")

        # demonstrate LH - lever
        planner.SetWheelPoseFromTransform(T4_1)
        planner.Run(handles, radius=0.02, manipulator="lh", valveType="l")

        # demonstrate both hands - wheel
        planner.SetWheelPoseFromTransform(T5_1)
        planner.Run(handles, radius=0.1, manipulator="bh", valveType="w")
        
        
    else:
        if play:
            planner.SetStopKeyStrokes(True)
            planner.StartViewerAndSetValvePos( handles )
            planner.SetProblems()
            planner.Playback(True)        
        else:
            planner.SetStopKeyStrokes(True)
            if taskwall:
                planner.InitFromTaskWallEnv()

            # Uncomment the following for the main valve code
            # planner.Run(handles,r,manipulator="bh",valveType="w")

            #################################################################################
            # # Uncomment the following for the vertical ball valve code (left hand - clockwise)
            # planner.wheelHeightFromTSY = 0.1
            # planner.TSYHeight = planner.robotid.GetLinks()[12].GetTransform()[2,3]
            # planner.wheelHeight = planner.TSYHeight + planner.wheelHeightFromTSY - planner.crouch
            # wheelY = 0.0
            # planner.worldPitch = 0.0
            # # Find the difference of angle between the wheel's end effector and the link (23 degrees)
            # # We should only do this if we're using the logitech wheel.
            # planner.tiltDiff = acos(dot(linalg.inv(planner.crankid.GetManipulators()[0].GetEndEffectorTransform()),planner.crankid.GetLinks()[0].GetTransform())[1,1])

            # planner.SetWheelPoseFromTransform(MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2-planner.tiltDiff+planner.worldPitch,0,0])),transpose(matrix([planner.wheelDistFromTSY+0.05, wheelY, planner.wheelHeight]))))
            # planner.Run(handles,0.1,manipulator="lh",direction="cw",valveType="l")
            ####################################################################################

            #################################################################################
            # # Uncomment the following for the vertical ball valve code (left hand - counter clockwise)
            # planner.wheelHeightFromTSY = 0.1
            # planner.TSYHeight = planner.robotid.GetLinks()[12].GetTransform()[2,3]
            # planner.wheelHeight = planner.TSYHeight + planner.wheelHeightFromTSY - planner.crouch
            # wheelY = 0.0
            # planner.worldPitch = 0.0
            # # Find the difference of angle between the wheel's end effector and the link (23 degrees)
            # # We should only do this if we're using the logitech wheel.
            # planner.tiltDiff = acos(dot(linalg.inv(planner.crankid.GetManipulators()[0].GetEndEffectorTransform()),planner.crankid.GetLinks()[0].GetTransform())[1,1])

            # planner.SetWheelPoseFromTransform(MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2-planner.tiltDiff+planner.worldPitch,0,0])),transpose(matrix([planner.wheelDistFromTSY+0.05, wheelY, planner.wheelHeight]))))
            # planner.Run(handles,0.1,manipulator="lh",direction="ccw",valveType="l")
            ####################################################################################

            # #################################################################################
            # # Uncomment the following for the vertical tiny valve code (left hand - clockwise)
            # planner.wheelHeightFromTSY = 0.1
            # planner.TSYHeight = planner.robotid.GetLinks()[12].GetTransform()[2,3]
            # planner.wheelHeight = planner.TSYHeight + planner.wheelHeightFromTSY - planner.crouch
            # wheelY = 0.0
            # planner.worldPitch = 0.0
            # # Find the difference of angle between the wheel's end effector and the link (23 degrees)
            # # We should only do this if we're using the logitech wheel.
            # planner.tiltDiff = acos(dot(linalg.inv(planner.crankid.GetManipulators()[0].GetEndEffectorTransform()),planner.crankid.GetLinks()[0].GetTransform())[1,1])

            # planner.SetWheelPoseFromTransform(MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2-planner.tiltDiff+planner.worldPitch,0,0])),transpose(matrix([planner.wheelDistFromTSY+0.05, wheelY, planner.wheelHeight]))))
            # planner.Run(handles,0.02,manipulator="lh",direction="cw",valveType="w")
            # ####################################################################################

            # #################################################################################
            # # Uncomment the following for the vertical tiny valve code (left hand - counterclockwise)
            # planner.wheelHeightFromTSY = 0.1
            # planner.TSYHeight = planner.robotid.GetLinks()[12].GetTransform()[2,3]
            # planner.wheelHeight = planner.TSYHeight + planner.wheelHeightFromTSY - planner.crouch
            # wheelY = 0.0
            # planner.worldPitch = 0.0
            # # Find the difference of angle between the wheel's end effector and the link (23 degrees)
            # # We should only do this if we're using the logitech wheel.
            # planner.tiltDiff = acos(dot(linalg.inv(planner.crankid.GetManipulators()[0].GetEndEffectorTransform()),planner.crankid.GetLinks()[0].GetTransform())[1,1])

            # planner.SetWheelPoseFromTransform(MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2-planner.tiltDiff+planner.worldPitch,0,0])),transpose(matrix([planner.wheelDistFromTSY+0.05, wheelY, planner.wheelHeight]))))
            # planner.Run(handles,0.02,manipulator="lh",direction="ccw",valveType="w")
            # ####################################################################################

            #################################################################################
            # # Uncomment the following for the vertical ball valve code (right hand - clockwise) 
            # planner.wheelHeightFromTSY = 0.1
            # planner.TSYHeight = planner.robotid.GetLinks()[12].GetTransform()[2,3]
            # planner.wheelHeight = planner.TSYHeight + planner.wheelHeightFromTSY - planner.crouch
            # wheelY = 0.0
            # planner.worldPitch = 0.0
            # # Find the difference of angle between the wheel's end effector and the link (23 degrees)
            # # We should only do this if we're using the logitech wheel.
            # planner.tiltDiff = acos(dot(linalg.inv(planner.crankid.GetManipulators()[0].GetEndEffectorTransform()),planner.crankid.GetLinks()[0].GetTransform())[1,1])

            # planner.SetWheelPoseFromTransform(MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2-planner.tiltDiff+planner.worldPitch,0,0])),transpose(matrix([planner.wheelDistFromTSY+0.05, wheelY, planner.wheelHeight]))))
            # planner.Run(handles,0.1,manipulator="rh",direction="cw",valveType="l")
            ####################################################################################

            #################################################################################
            # # Uncomment the following for the vertical ball valve code (right hand - counterclockwise) 
            # planner.wheelHeightFromTSY = 0.1
            # planner.TSYHeight = planner.robotid.GetLinks()[12].GetTransform()[2,3]
            # planner.wheelHeight = planner.TSYHeight + planner.wheelHeightFromTSY - planner.crouch
            # wheelY = 0.0
            # planner.worldPitch = 0.0
            # # Find the difference of angle between the wheel's end effector and the link (23 degrees)
            # # We should only do this if we're using the logitech wheel.
            # planner.tiltDiff = acos(dot(linalg.inv(planner.crankid.GetManipulators()[0].GetEndEffectorTransform()),planner.crankid.GetLinks()[0].GetTransform())[1,1])

            # planner.SetWheelPoseFromTransform(MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2-planner.tiltDiff+planner.worldPitch,0,0])),transpose(matrix([planner.wheelDistFromTSY+0.05, wheelY, planner.wheelHeight]))))
            # planner.Run(handles,0.1,manipulator="rh",direction="ccw",valveType="l")
            ####################################################################################


            # #################################################################################
            # # Uncomment the following for the vertical tiny valve code (right hand - clockwise)
            # planner.wheelHeightFromTSY = 0.1
            # planner.TSYHeight = planner.robotid.GetLinks()[12].GetTransform()[2,3]
            # planner.wheelHeight = planner.TSYHeight + planner.wheelHeightFromTSY - planner.crouch
            # wheelY = 0.0
            # planner.worldPitch = 0.0
            # # Find the difference of angle between the wheel's end effector and the link (23 degrees)
            # # We should only do this if we're using the logitech wheel.
            # planner.tiltDiff = acos(dot(linalg.inv(planner.crankid.GetManipulators()[0].GetEndEffectorTransform()),planner.crankid.GetLinks()[0].GetTransform())[1,1])

            # planner.SetWheelPoseFromTransform(MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2-planner.tiltDiff+planner.worldPitch,0,0])),transpose(matrix([planner.wheelDistFromTSY+0.05, wheelY, planner.wheelHeight]))))
            # planner.Run(handles,0.02,manipulator="rh",direction="cw",valveType="w")
            # ####################################################################################

            # #################################################################################
            # # Uncomment the following for the vertical tiny valve code (right hand - counterclockwise)
            planner.wheelHeightFromTSY = 0.1
            planner.TSYHeight = planner.robotid.GetLinks()[12].GetTransform()[2,3]
            planner.wheelHeight = planner.TSYHeight + planner.wheelHeightFromTSY - planner.crouch
            wheelY = 0.0
            planner.worldPitch = 0.0
            # Find the difference of angle between the wheel's end effector and the link (23 degrees)
            # We should only do this if we're using the logitech wheel.
            planner.tiltDiff = acos(dot(linalg.inv(planner.crankid.GetManipulators()[0].GetEndEffectorTransform()),planner.crankid.GetLinks()[0].GetTransform())[1,1])

            planner.SetWheelPoseFromTransform(MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2-planner.tiltDiff+planner.worldPitch,0,0])),transpose(matrix([planner.wheelDistFromTSY+0.05, wheelY, planner.wheelHeight]))))
            planner.Run(handles,0.02,manipulator="rh",direction="ccw",valveType="w")
            # ####################################################################################
            
            # Uncomment the following for the vertical ball valve code (right hand)
            # planner.SetWheelPoseFromTransform()
            # planner.Run(handles,r,manipulator="rh",valveType="l")

            # Uncomment the following for the horizontal ball valve code (left hand)
            # planner.SetWheelPoseFromTransform()
            # planner.Run(handles,r,manipulator="lh",valveType="l")

            # Uncomment the following for the horizontal ball valve code (right hand)
            # planner.SetWheelPoseFromTransform()
            # planner.Run(handles,r,manipulator="rh",valveType="l")

            

    planner.KillOpenrave()

