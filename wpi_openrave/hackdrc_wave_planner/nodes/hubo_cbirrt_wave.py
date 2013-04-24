#!/usr/bin/env python
# Ben Suay, RAIL
# November 2012
# Worcester Polytechnic Institute
#


from openravepy import *
import sys
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
    import numpy
import time
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *
import os # for file operations

def trans_to_str(T):
    myStr = ""
    for c in range(0,3):
        for r in range(0,3):
            myStr += str(T[r,c])+" "
    
    for r in range(0,3):
        myStr += str(T[r,3])+" "

    #print "Tee string : " 
    #print myStr
    return myStr

def run():
    normalsmoothingitrs = 150;
    fastsmoothingitrs = 20;

    env = Environment()
    RaveSetDebugLevel(DebugLevel.Info) # set output level to debu
    robotid = env.ReadRobotURI('/home/jmainpri/workspace/drc_hubo/openHubo/huboplus/rlhuboplus_mit.robot.xml')
    env.Add(robotid)
    env.SetViewer('qtcoin')

    probs_cbirrt = RaveCreateModule(env,'CBiRRT')

    manips = robotid.GetManipulators()

    footlinknames = ' Body_RAR Body_LAR ';
    cogtarg = [-0.05, 0.085, 0];

    try:
        env.AddModule(probs_cbirrt,'rlhuboplus')
    except openrave_exception, e:
        print e

    print "Getting Loaded Problems"
    probs = env.GetLoadedProblems()

    activedofs = range(0,57)
    
    robotid.SetActiveDOFs(activedofs)

    #robotid.SetDOFValues([-0.85,-1.00,-1.57,-1.00,-1.00,0.548],[27,29,30,32,33,34])
    #robotid.SetDOFValues([0.85,1.00,-1.57,-1.00,1.00,0.548],[15,17,18,20,21,22])
    #robotid.SetActiveDOFs([15,16,17,18,19,20,21,22,27,28,29,30,31,32,33,34])

    initconfig = robotid.GetActiveDOFValues()
    
    #print "Press enter to continue 1..."
    #sys.stdin.readline()
    
    links = robotid.GetLinks()

    #rhanddofs = [34] # Right gripper links here
    #rhandclosevals = [0.35] # What are the closed joint values for the right hand's links?--> Obtained experimentally
    #rhandopenvals = [0.548] # What are the open joint values for the right hand's links? --> Obtained experimentally
    #lhanddofs = [22] # Left gripper links here
    #lhandclosevals = [0.35] # Same as rhandclosevals for the left gripper --> Obtained experimentally
    #lhandopenvals = [0.548] # Same as rhandopenvals for the left gripper --> Obtained experimentally

    rhanddofs = range(27,42);
    rhandclosevals = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.2];
    rhandopenvals = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.08]; 
    lhanddofs = range(42,57);
    lhandclosevals = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.2];
    lhandopenvals =  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.08];

    robotid.SetDOFValues(rhandclosevals,rhanddofs)
    robotid.SetDOFValues(lhandclosevals,lhanddofs)
    
    #arg1 = str(GetRot(T0_LH1)).strip("[]")+str(GetTrans(T0_LH1)).strip("[]")
    #arg1 = arg1.replace("\n"," ")
   
    #arg2 = str(GetRot(T0_RH1)).strip("[]")+str(GetTrans(T0_RH1)).strip("[]")
    #arg2 = arg2.replace("\n"," ")   

    # All endeffectors (except head)
    Tee = [] 
    for i in range(0,4):
        Tlink = manips[i].GetEndEffectorTransform()
        Tee.append(Tlink)
        #Tlink = MakeTransform(links(1:9,manips{i}.eelink+1),links(10:12,manips{i}.eelink+1));
        #Tgrasp = [manips{i}.Tgrasp;[0 0 0 1]];
        #Tee.append(dot(Tlink,Tgrasp));
        print "Tee : " + str(i) 
        print Tee[i]


    T0_RH1 = Tee[1]
    T0_LF = Tee[2]
    print T0_RH1
    print T0_LF
    arg1 = str(cogtarg).strip("[]").replace(","," ")
    #arg2 = str(GetRot(T0_RH1)).strip("[]") + str(GetTrans(T0_RH1)).strip("[]")
    #arg3 = str(GetRot(T0_LF)).strip("[]") + str(GetTrans(T0_LF)).strip("[]")
    

    T0_RH1[0,3] += 0.4
    T0_RH1[1,3] += 0
    T0_RH1[2,3] += 0.45

    Rotate_RH1 = MakeTransform(matrix(xyz_rotation([pi/2,-pi/3,pi/5])),transpose(matrix([0,0,0])))
    
    T0_RH1 = dot(T0_RH1,Rotate_RH1)

    h1 = misc.DrawAxes(env,matrix(T0_RH1),1)

    arg2 = trans_to_str(T0_RH1)
    arg3 = trans_to_str(T0_LF)

    #arg1 = arg1.replace("\n"," ")
    #arg2 = arg2.replace("\n"," ")
    #arg3 = arg3.replace("\n"," ")

    print "arg1 : "
    print arg1 
    print "arg2 : "
    print arg2 
    print "arg3 : "
    print arg3

    #sys.stdin.readline()
     
    startik = probs[0].SendCommand('DoGeneralIK exec supportlinks 2 ' + footlinknames + ' movecog ' +arg1+ ' nummanips 2 maniptm 1 ' + arg2 + ' maniptm 2 ' + arg3  );
    #startik = probs[0].SendCommand('DoGeneralIK exec nummanips 1  maniptm 4 '+arg2)

    print "Got startik:"
    print startik
    print "------"
    
    robotid.SetDOFValues(str2num(startik)) # Note: startik: T0_LH1 and T0_RH1
    print "go to startik"
    #sys.stdin.readline()

    T0_RH2 = T0_RH1
    #T0_RH2 = Tee[1]
    T0_RH2[0,3] -= 0.1
    T0_RH2[1,3] -= 0.2
    
    Rotate_RH2 = MakeTransform(matrix(xyz_rotation([0,-pi/4,-pi/3])),transpose(matrix([0,0,0])))
    
    T0_RH2 = dot(T0_RH2,Rotate_RH2)

    h2 = misc.DrawAxes(env,matrix(T0_RH2),1)
    #T0_RH2[2,3] += 0.01
    #arg1 = str(cogtarg).strip("[]")
    arg2 = trans_to_str(T0_RH2)
    arg3 = trans_to_str(T0_LF)
    #arg2 = str(GetRot(T0_RH2)).strip("[]") + str(GetTrans(T0_RH2)).strip("[]")
    #arg3 = str(GetRot(Tee[3])).strip("[]") + str(GetTrans(Tee[3])).strip("[]")

    #arg1 = arg1.replace("\n"," ")
    #arg2 = arg2.replace("\n"," ")
    #arg3 = arg3.replace("\n"," ")

    goalik = probs[0].SendCommand('DoGeneralIK exec supportlinks 2 ' + footlinknames + ' movecog ' +arg1+ ' nummanips 2 maniptm 1 ' + arg2 + ' maniptm 2 ' + arg3  );
    
    print "Got goalik:"
    print goalik
    print "------"
    #robotid.SetActiveDOFValues(str2num(goalik)) # Note: goalik is T0_LH2 and T0_RH2
    robotid.SetDOFValues(str2num(goalik))
    print "go to goalik"
    #sys.stdin.readline()

    for i in range(2):
        try:
            print "Removing movetraj"+str(i)+".txt"
            os.remove("movetraj"+str(i)+".txt")
        except OSError, e:
            print e    
            
    TSRString_G1 = SerializeTSR(0,'NULL',T0_RH2,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    TSRString_G2 = SerializeTSR(2,'NULL',T0_LF,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    TSRChainString_G = SerializeTSRChain(0,1,0,1,TSRString_G1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString_G2,'NULL',[]) 

    robotid.SetDOFValues(initconfig)
    robotid.SetDOFValues(rhandclosevals,rhanddofs)
    robotid.SetDOFValues(lhandclosevals,lhanddofs)
    print "go to startik prepare for cbirrt call"
    #sys.stdin.readline()

    # Set the current goal to RH2
    goaljoints = str2num(startik);
    
    try:
        answer = probs[0].SendCommand('RunCBiRRT psample 0.2 supportlinks 2 '+footlinknames+' smoothingitrs 150 jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_G))
        print "runcbirrt successful"
        print answer
    except openrave_exception, e:
        print "Cannot send command runcbirrt"
        print e
    
    try:
        os.rename("cmovetraj.txt","movetraj0.txt")
        
        print "Executing trajectory 0"
        try:
            answer= probs[0].SendCommand('traj movetraj0.txt');
            # debug
            print "traj call successful"
            print answer 
        except openrave_exception, e:
            print e
        
        robotid.WaitForController(0)
        #sys.stdin.readline()
    except OSError, e:
        print e

    robotid.SetDOFValues(str2num(startik))
    print "go to startik prepare for cbirrt call"
    #sys.stdin.readline()

     # Set the current goal to RH2
    goaljoints = str2num(goalik);

    try:
        answer = probs[0].SendCommand('RunCBiRRT psample 0.2 supportlinks 2 '+footlinknames+' smoothingitrs 150 jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_G))
        print "runcbirrt successful"
        print answer
    except openrave_exception, e:
        print "Cannot send command runcbirrt"
        print e

    try:
        os.rename("cmovetraj.txt","movetraj1.txt")
        
        print "Executing trajectory 1"
        try:
            answer= probs[0].SendCommand('traj movetraj1.txt');
            # debug
            print "traj call successful"
            print answer 
        except openrave_exception, e:
            print e
        
        robotid.WaitForController(0)
        #sys.stdin.readline()
    except OSError, e:
        print e

    env.Destroy()

if __name__ == "__main__":
    run()
