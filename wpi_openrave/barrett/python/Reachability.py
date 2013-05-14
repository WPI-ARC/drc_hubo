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

class ReachabilitySphere(object):
    def __init__(self):
        self.radius = 0.025
        self.neighbors = []
        self.shapeHandle = None
        self.axisHandle = []
        self.diameter = self.radius*2
        self.reachability = 0 # int: +1 for each direction the manipulator can approach
        self.directions = [] # string: +/-x, +/-y, +/-z
        self.comOffset = [] # How much does reaching to this point add to the center of mass of the robot?
        self.T = [] # transform of the sphere wrt the base of the manipulator. Azimuth is always +z in world coordinates, so rotation would be calculated in world coordinate frame.
        self.alpha = 0.5
        self.color = array((0,1,1,self.alpha))
        self.visible = True
        self.axisLength = 0.1
        self.configs = []

    def show(self,myEnv):
        # print self.T[0][0:3,3]
        self.shapeHandle = myEnv.plot3(points=self.T[0][0:3,3],
                                    pointsize=self.radius, # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                    colors=self.color, # This changes the transparency
                                    drawstyle=1)
        self.axisHandle = misc.DrawAxes(myEnv,self.T[0],0.01)
        
class ReachabilityMapParams(object):
    def __init__(self):
        self.xmax=1.0
        self.xmin=-1.0
        self.ymax=1.0
        self.ymin=-1.0
        self.zmax=1.0
        self.zmin=-1.0
        self.free_joint_val = 0.0 
        self.free_joint_index = None
        self.r = 0
        self.g = 0
        self.b = 0

class ReachabilityMap(object):
    def __init__(self, iksolver, robot, manip):
        self.handles = []
        self.map = []
        self.indices = {}
        self.candidates = []
        self.name = "default"

        # This is the coordinate system of the map, usually attached to the base of the manipulator.
        # T0 stands for the world coordinate frame
        self.T0_base = manip.GetBase().GetTransform() 
        
        # list of rotation matrices that we will evaluate around a point in space
        self.rm3D=[rodrigues([pi/2,0,0]),rodrigues([-pi/2,0,0]),rodrigues([0,pi/2,0]),rodrigues([0,-pi/2,0]),rodrigues([0,0,pi/2]),rodrigues([0,0,-pi/2])]
        # limits
        self.xmax=1.0
        self.xmin=-1.0
        self.ymax=1.0
        self.ymin=-1.0
        self.zmax=1.0
        self.zmin=-1.0

        # increments for left arm
        self.dx = ReachabilitySphere().diameter
        self.dy = ReachabilitySphere().diameter
        self.dz = ReachabilitySphere().diameter

        # If this manipulator has a free joint for its Fast IK Solver
        self.free_joint_val = 0.0 
        self.free_joint_index = None

        self.solver = iksolver
        self.robot = robot
        self.manip = manip
        self.armJoints = manip.GetArmJoints()

        self.r = 0
        self.g = 0
        self.b = 0

        # Make a list of Tsphere_neigbors transforms (26 of them)
        self.Tsphere_neighbors = []
        # 1
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-ReachabilitySphere().diameter,ReachabilitySphere().diameter,ReachabilitySphere().diameter]))))
        # 2
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,ReachabilitySphere().diameter,ReachabilitySphere().diameter]))))
        # 3
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([ReachabilitySphere().diameter,ReachabilitySphere().diameter,ReachabilitySphere().diameter]))))
        # 4
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-ReachabilitySphere().diameter,0.0,ReachabilitySphere().diameter]))))
        # 5
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,ReachabilitySphere().diameter]))))
        # 6
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([ReachabilitySphere().diameter,0.0,ReachabilitySphere().diameter]))))
        # 7
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-ReachabilitySphere().diameter,-ReachabilitySphere().diameter,ReachabilitySphere().diameter]))))
        # 8
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,-ReachabilitySphere().diameter,ReachabilitySphere().diameter]))))
        # 9
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([ReachabilitySphere().diameter,-ReachabilitySphere().diameter,ReachabilitySphere().diameter]))))
        # 10
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-ReachabilitySphere().diameter,ReachabilitySphere().diameter,0.0]))))
        # 11
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,ReachabilitySphere().diameter,0.0]))))
        # 12
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([ReachabilitySphere().diameter,ReachabilitySphere().diameter,0.0]))))
        # 13
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-ReachabilitySphere().diameter,0.0,0.0]))))
        # 14
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([ReachabilitySphere().diameter,0.0,0.0]))))
        # 15
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-ReachabilitySphere().diameter,-ReachabilitySphere().diameter,0.0]))))
        # 16
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,-ReachabilitySphere().diameter,0.0]))))
        # 17
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([ReachabilitySphere().diameter,-ReachabilitySphere().diameter,0.0]))))
        # 18
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-ReachabilitySphere().diameter,ReachabilitySphere().diameter,-ReachabilitySphere().diameter]))))
        # 19
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,ReachabilitySphere().diameter,-ReachabilitySphere().diameter]))))
        # 20
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([ReachabilitySphere().diameter,ReachabilitySphere().diameter,-ReachabilitySphere().diameter]))))
        # 21
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-ReachabilitySphere().diameter,0.0,-ReachabilitySphere().diameter]))))
        # 22
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,-ReachabilitySphere().diameter]))))
        # 23
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([ReachabilitySphere().diameter,0.0,-ReachabilitySphere().diameter]))))
        # 24
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-ReachabilitySphere().diameter,-ReachabilitySphere().diameter,-ReachabilitySphere().diameter]))))
        # 25
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,-ReachabilitySphere().diameter,-ReachabilitySphere().diameter]))))
        # 26
        self.Tsphere_neighbors.append(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([ReachabilitySphere().diameter,-ReachabilitySphere().diameter,-ReachabilitySphere().diameter]))))
        

    def go_to(self, sphere, transform):

        configuration = self.map[sphere].configs[transform]

        self.robot.SetDOFValues(configuration,self.armJoints)
        
    def frange(self,start,stop,inc):
        i=start
        a=[]
        a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
        while i < stop:
            i += inc
            a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
        return a

    def show(self,myEnv):
        # draw all, append to handles
        self.handles=[]
        print "In show - map length: ",str(len(self.map))
        for idx, s in enumerate(self.map):
            for direction in range(s.reachability):
                Tbase_s = s.T[direction]
                # if(Tbase_s[0,3] > 0):
                #    print "warning!!!"
                T0_ee =  dot(self.T0_base, Tbase_s)
                if(s.shapeHandle == None):
                    s.shapeHandle = myEnv.plot3(points=T0_ee[0:3,3],
                                                pointsize=s.radius*0.5, # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                                colors=array((self.r,self.g,self.b,0.166*s.reachability)), # This changes the transparency
                                                drawstyle=1
                                                )

                    self.handles.append(s.shapeHandle)
                    
                s.axisHandle.append(misc.DrawAxes(myEnv,dot(self.T0_base,Tbase_s),0.01))
                
            
    def update_indices(self):
        print "Updating reachability sphere indices..."
        self.indices = {}
        for idx, s in enumerate(self.map):
            # Convert the Tbase_sphere translation
            # into string and keep it in the dictionary
            myKey = str(round(s.T[0][0,3],2)),",",str(round(s.T[0][1,3],2)),",",str(round(s.T[0][2,3],2))
            self.indices[myKey] = idx
        print "Index update done."
        
    def find_neighbors(self):
        print "Finding neighbors..."
        # For all spheres in the map do:
        for index, sphere in enumerate(self.map):
            sphere.neighbors = []
            # For each possible neighbor do:
            for Tsphere_neighbor in self.Tsphere_neighbors:
                # Erase rotation (we don't care), keep transformation.
                Tbase_sphere = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([sphere.T[0][0,3],sphere.T[0][1,3],sphere.T[0][2,3]])))

                # Convert the transform into robot's base coordinates
                Tbase_neighbor = dot(Tbase_sphere,Tsphere_neighbor)

                # Convert the coordinates into a string
                key = str(round(Tbase_neighbor[0,3],2)),",",str(round(Tbase_neighbor[1,3],2)),",",str(round(Tbase_neighbor[2,3],2))

                # If there is a reachability sphere at these coordinates
                if( key in self.indices ):
                    # Look-up the index of the neighbor and keep it
                    sphere.neighbors.append(self.indices[key])
        print "Finding neighbors, done."
            
    def list(self):
        print "list of spheres in this reachability map"
        for i, s in enumerate(self.map):
            print "sphere #: ",str(i)
            print "reachability: ",str(s.reachability)
            for j, t in enumerate(s.T):
                print "transform: ",str(j)
                print t #Tbase_s

    def hide(self):
        # destroy handles
        self.handles=[]
        for idx, s in enumerate(self.map):
            s.shapeHandle = None
            s.axisHandle = []

    def update(self):
        pass

    # A Method that crops the reachability map
    # bounds are defined in base coordinates
    def crop(self, bounds):
        
        xmin = bounds[0]
        xmax = bounds[1]
        ymin = bounds[2]
        ymax = bounds[3]
        zmin = bounds[4]
        zmax = bounds[5]

        print "cropping... bounds: ",str(bounds)
        toDelete = []
        for idx, s in enumerate(self.map):
            # print  "x: ",str(s.T[0][0,3])
            # print  "y: ",str(s.T[0][1,3])
            # print  "z: ",str(s.T[0][2,3])
            # print str(idx)," in ",str(self.indices.values())," :"
            # print idx in self.indices.values()

            ###################################################
            if((xmin > s.T[0][0,3] or s.T[0][0,3] > xmax) or
               (ymin > s.T[0][1,3] or s.T[0][1,3] > ymax) or
               (zmin > s.T[0][2,3] or s.T[0][2,3] > zmax)):
                # # remove it from self.indices dict, so it doesn't mess up
                # # find_neighbors results
                # myKey = str(round(s.T[0][0,3],2)),",",str(round(s.T[0][1,3],2)),",",str(round(s.T[0][2,3],2))
                # del self.indices[myKey]
                # remove the sphere from the map
                toDelete.append(idx)
            ##################################################
            # if(s.T[0][0,3] > 0):
            #     myKey = str(round(s.T[0][0,3],2)),",",str(round(s.T[0][1,3],2)),",",str(round(s.T[0][2,3],2))
            #     del self.indices[myKey]
            #     toDelete.append(idx)
        
        print "toDelete: ",str(len(toDelete))
        for d, idx in enumerate(toDelete):
            adjustedIdx = idx-(d)
            del self.map[adjustedIdx]

        # Finished removing unwanted spheres.
        # Update sphere indices in the map
        self.update_indices()
        
        # Update the neighbors' indices
        self.find_neighbors()

        # Report
        print "crop done."
    
    
    # Trobot_start: transform of the robot's base wrt
    #               start transform of the manipulation
    # returns the index of the reachability sphere that
    # matches the Tbase_start
    def select(self, Tbase_start):
        for sIdx, s in enumerate(self.map):
            for tIdx, t in enumerate(s.T):
                # t is actually Tbase_ee
                if(allclose(t,Tbase_start)):
                    return [sIdx, tIdx]

        # If we're here, it's because we couldn't find a result
        return -1
    
    # Trims the reachability spheres that are farther than 
    # a certain radius (if x=y=z, then the end result is a sphere)
    def trim(self,x,y,z):
        r2 = pow(x,2)+pow(y,2)+pow(z,2)
        r = pow(r2,0.5)
        print "trimming..."
        for idx, s in enumerate(self.map):
            rs2 = pow(s.T[0][0,3],2)+pow(s.T[0][1,3],2)+pow(s.T[0][2,3],2)
            rs = pow(rs2,0.5)
            if(rs > r):
                self.map.pop(idx)
        print "trimming done."

    def save(self):
        # Save map
        tempMap = []
        for idx, s in enumerate(self.map):
            s.axisHandle=[]
            s.shapeHandle=None
            tempMap.append(s)
        output = open(self.name+'_map.pkl', 'wb')
        pickle.dump(tempMap, output)
        output.close()

        params = ReachabilityMapParams()
        params.xmax = self.xmax
        params.xmin = self.xmin
        params.ymax = self.ymax
        params.ymin = self.ymin
        params.zmax = self.zmax
        params.zmin = self.zmin

        params.r = self.r
        params.g = self.g
        params.b = self.b

        output = open(self.name+'_params.pkl', 'wb')
        pickle.dump(params, output)
        output.close()


    def load(self, fname):
        if(fname != ''):
            pkl_file = open(fname+'_map.pkl', 'rb')
            self.map = pickle.load(pkl_file)
            pkl_file.close()
            
            pkl_file = open(fname+'_params.pkl', 'rb')
            params = pickle.load(pkl_file)
            pkl_file.close()
            
            # Note to self: change this params throughout the class with self.params
            self.xmax = params.xmax
            self.xmin = params.xmin
            self.ymax = params.ymax
            self.ymin = params.ymin
            self.zmax = params.zmax
            self.zmin = params.zmin
            
            self.r = params.r
            self.g = params.g
            self.b = params.b

            return 1
        else:
            print "Error: specify a filename."
            return None

    def manip_reset(self):
        q=[]
        for j in range(len(self.armJoints)):
            q.append(0.0)
        self.robot.SetDOFValues(q,self.armJoints)

    def generate(self,env):
        self.handles.append(misc.DrawAxes(env,self.T0_base,0.4))
        # This is the main loop
        self.xarray = self.frange(self.xmin,self.xmax,self.dx)
        self.yarray = self.frange(self.ymin,self.ymax,self.dy)
        self.zarray = self.frange(self.zmin,self.zmax,self.dz)

        self.totalNumPoints = len(self.xarray)*len(self.yarray)*len(self.zarray)

        current_point_ind = 0
        for x in self.xarray:
            for y in self.yarray:
                for z in self.zarray:
                    # x, y, z are in manipulator's base coordinate frame
                    s = ReachabilitySphere()
                    s.reachability = 0
                    for rm in self.rm3D:
                        there_exists_at_least_one_good_solution = False # for this rotation
                        # rodrigues returns a numpy ndarray, we should convert it to a list before extracting data.
                        r00 = rm[0,0] 
                        r01 = rm[0,1]
                        r02 = rm[0,2]
                        
                        r10 = rm[1,0]
                        r11 = rm[1,1]
                        r12 = rm[1,2]
                         
                        r20 = rm[2,0]
                        r21 = rm[2,1]
                        r22 = rm[2,2]

                        # requested transform in manipulator's base coord frame
                        Tbase_req = MakeTransform(rm, transpose(matrix([x,y,z]))) 

                        # T0 stands for world coordinate frame
                        # T0_req: pose of the requested frame in world coordinate frame
                        T0_req =  dot(self.T0_base,Tbase_req)

                        # Show in qt-viewer where the the manipulator is trying to reach
                        h = misc.DrawAxes(env,T0_req,0.2)

                        # All rotations and translations passed in should be in the coordinate frames of the base of the manipulator
                        # x, y, z are in manipulator's base coordinate frame
                        cmd = self.solver + ' ' + str(r00) + ' ' + str(r01) + ' ' + str(r02) + ' ' + str(x) + ' ' + str(r10) + ' ' + str(r11) + ' ' + str(r12) + ' ' + str(y) + ' ' + str(r20) + ' ' + str(r21) + ' ' + str(r22) + ' ' + str(z)
                        


                        if(self.free_joint_index != None):
                            cmd = cmd + ' ' + str(self.free_joint_val)
                             
                        solutions_str = commands.getoutput(cmd)
                        solutions_float = []

                        if(solutions_str.find("Failed") != 0):
                            words = solutions_str.split()

                            for w in range(len(words)): 
                                if(words[w] == 'Found'):
                                    num_solutions = int(words[w+1])
                                elif(words[w] == '(free=0):'): # configuration comes after this word
                                    q = []
                                    for j in range(len(self.armJoints)):
                                        # the following will strip the comma in the end of the joint value and convert it into a float
                                        if(j == self.free_joint_index): # do a smarter comparison here. Get the joint name, and index, and see if it matches to the one that we actually set free in the ik solver.
                                            q.append(self.free_joint_val) # Set the free joint to zero
                                        else:
                                            #print "joint ",str(j),": ",str(float(words[w+1+j][0:len(words[w+1+j])-1]))
                                            q.append(float(words[w+1+j][0:len(words[w+1+j])-1]))
                                            # now we have a configuration for 7 joints for this solution
                                            # print q
                                            solutions_float.append(q)
                                             
                        if(solutions_float != []):
                            for sol in range(len(solutions_float)):
                                q = solutions_float[sol]
                                #print "self arm joints"
                                #print self.armJoints
                                #print "setting to: "
                                #print q
                                self.robot.SetDOFValues(q,self.armJoints)
                                # manipulator's end effector transform in world coord. frame
                                T0_ee = self.manip.GetEndEffectorTransform() # End effector in world coordinate frame
                                Tbase_ee = dot(linalg.inv(self.T0_base),T0_ee) # End effector in manipulator base coordinate frame
                                
                                 
                                close_enough = True
                                if(not allclose(Tbase_req,Tbase_ee)):
                                    close_enough = False
                                    
                                # for r in range(3): # rows
                                #     for c in range(4): # columns
                                #         if(not allclose(Tbase_req[r,3],Tbase_ee[r,3])):
                                #             close_enough = False
                                #             break

                                if(close_enough):
                                    if((not env.CheckCollision(self.robot)) and (not self.robot.CheckSelfCollision())):
                                        # Transform of the sphere in manipulator's base coordinate frame
                                        s.T.append(Tbase_ee)
                                        s.configs.append(q)
                                        there_exists_at_least_one_good_solution = True
                                        break
                                    
                        
                        
                        if(there_exists_at_least_one_good_solution):
                            s.reachability += 1
                            # print "comparing: "
                            # print "T0_ee: "
                            # print T0_ee
                            # print "Tbase_ee: "
                            # print Tbase_ee
                            # print "Tbase_req: "
                            # print Tbase_req
                            # sys.stdin.readline()

                    if(s.reachability>0):
                        for direction in range(s.reachability):
                            Tbase_s = s.T[direction]
                            # Transform of the sphere in world coordinate frame
                            T0_ee = dot(self.T0_base, Tbase_s)

                            if(s.shapeHandle == None):
                                # Add shapeHandle only once, if it doesn't exist already
                                s.shapeHandle = env.plot3(points=T0_ee[0:3,3],
                                                          pointsize=(s.radius*0.5), # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                                          colors=array((self.r,self.g,self.b,0.166*s.reachability)), # This changes the transparency
                                                          drawstyle=1
                                                          )
                            
                            s.axisHandle.append(misc.DrawAxes(env,dot(self.T0_base,Tbase_s),0.01))

                        # After adding all the handles, add sphere in the reachability map
                        self.map.append(s)
                        
                        # Convert the Tbase_sphere translation
                        # into string and keep it in the dictionary
                        myKey = str(round(s.T[0][0,3],2)),",",str(round(s.T[0][1,3],2)),",",str(round(s.T[0][2,3],2))
                        myIndex = len(self.map)-1
                        self.indices[myKey] = myIndex

                    current_point_ind += 1
                    if((current_point_ind%100)==0):
                        print str(current_point_ind),"/",str(self.totalNumPoints)
        # Finished generating the map.
        # Now find neighbors
        self.find_neighbors()

        # All done. Reset the manipulator.
        self.manip_reset()

class PathElement:
    def __init__(self,sphereIndex,transformIndex):
        self.sIdx = sphereIndex
        self.tIdx = transformIndex

class SearchPattern:
    # transforms: a list of 
    def __init__(self,transforms):
        self.T0_p = None # Transform of the first sphere in world coord frame
        self.pattern = [] # a list of reachability spheres
        self.handles = []
        self.delta = 0.05 # discretization value in milimeters
        self.prePatternTransforms = transforms
        self.delta = 2*(ReachabilitySphere().radius)
        self.generate(transforms) # will discretize and generate a search pattern from the list of transforms

    def generate(self,transforms):
        # Put the first sphere where the first transform is
        for t in range(len(transforms)):
            s = ReachabilitySphere()
            if(t==0): # Put a reachability sphere on the first transform
                s.T = transforms[t]
                self.pattern.append(s)
            else: # Otherwise do mapping (discretization)
                r = self.map([transforms[0],transforms[t]])
                if(r != None):
                    s.T = r
                    self.pattern.append(s)

    def sign(self,num):
        if(num >= 0):
            return 1
        elif(num < 0):
            return -1
    
    def map(self,tCouple):
        debug = False
        # find the relative transform between the couple
        Trel = dot(linalg.inv(tCouple[0]),tCouple[1])

        # First, discretize the distance
        relx = Trel.tolist()[0][3]
        rely = Trel.tolist()[1][3]
        relz = Trel.tolist()[2][3]

        mx = relx%(self.sign(relx)*ReachabilitySphere().diameter)
        dx = round(relx/ReachabilitySphere().diameter)
        if(abs(mx)>ReachabilitySphere().radius):
            dx+=sign(relx)         
            
        my = rely%(self.sign(rely)*ReachabilitySphere().diameter)
        dy = round(rely/ReachabilitySphere().diameter)
        if(abs(my)>ReachabilitySphere().radius):
            dy+=sign(rely)

        mz = relz%(self.sign(relz)*ReachabilitySphere().diameter)
        dz = round(relz/ReachabilitySphere().diameter)
        if(mz>ReachabilitySphere().radius):
            dz+=sign(relz)

        if(dx == 0 and dy == 0 and dz == 0):
            Tnew = None
        else:
            Tnew = array(MakeTransform(Trel[0:3,0:3],transpose(matrix([tCouple[0][0,3]+dx*ReachabilitySphere().diameter, tCouple[0][1,3]+dy*ReachabilitySphere().diameter, tCouple[0][2,3]+dz*ReachabilitySphere().diameter]))))

        if(debug):
            print "Trel"
            print Trel
            print "mx: ",str(mx)
            print "dx: ",str(dx)
            print "my: ",str(my)
            print "dy: ",str(dy)
            print "mz: ",str(mz)
            print "dz: ",str(dz)
            print "Tnew: "
            print Tnew
            

        return Tnew
        
    
    def show(self,myEnv):
        # draw all reachability spheres and keep their handles
        for s in self.pattern:
            s.shapeHandle = myEnv.plot3(points=s.T[0:3,3],
                                        pointsize=s.radius, # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                        colors=s.color, # This changes the transparency
                                        drawstyle=1)
            s.axisHandle = misc.DrawAxes(myEnv,s.T,0.01)

    
        for p in self.prePatternTransforms:
            self.handles.append(misc.DrawAxes(myEnv,p,ReachabilitySphere().axisLength))
        
    def hide(self,what):
        if(what == "trajectory"):
            # undraw all reachability spheres
            self.handles = []
        elif(what == "spheres"):
            for s in self.pattern:
                s.shapeHandle = None
        elif(what == "all"):
            for s in self.pattern:
                s.shapeHandle = None
                s.axisHandle = []
            self.handles = []
    
    def setColor(self,color):
        # change the color of all reachability spheres
        for s in self.pattern:
            s.color = color
        pass
 
def satisfy():
    # candidates = []
    # for m in range(len(paths)):
    #     for p in paths[m]:
    #         for s in p:
    #             for n in range(m+1,len(paths)):
    #                 for o in paths[n]:
    #                     for t in o:
    #                         for c in constraints:
    #                             if(dot(s.T,t.T)==c):
    pass

def my_function2(start,idx,myPattern,rmap):
    myList = []
    # For each sphere in the pattern,
    # (except the first and the last)
    #
    # Set the current sphere of interest
    # "sphere of interest" is the sphere of which 
    # we're trying to find the neighbor
    SoI = start
    print str(idx)
    Tbase_SoI = start.T[idx]

    for n in range(1,len(myPattern)):
        # sys.stdin.readline()

        # All spheres in the search pattern
        # are saved in the pattern's coordinate
        # frame, of which the first sphere.T
        # is the origin.
        #
        # To go neighbor by neighbor, we need
        # to find the relative transform between
        # the pattern's consecutive spheres.
        #
        # Get the relative transform 
        # between two consecutive spheres
        # of the pattern:
        Tp_p1 = myPattern[n-1].T
        Tp_p2 = myPattern[n].T
        Tp1_p2 = dot(linalg.inv(Tp_p1),Tp_p2)
                                
        # Go through the neighbors of SoI
        # print "sphere of interest has ",str(len(SoI.neighbors))," neighbors."
        for neighbor in SoI.neighbors:
            found = False
            # SoI.neighbors is a list of integers,
            # the list keeps the indices of the 
            # neighbors of SoI
            
            # See if any of SoI's neighbors
            # has the same relative transform
            # as Tp1_p2
            # print "neighbor with index: ",neighbor," has ",len(rmap[neighbor].T)," transforms."
            for tIdx, Tbase_neighbor in enumerate(rmap[neighbor].T):                                        
                TSoI_neighbor = dot(linalg.inv(Tbase_SoI),Tbase_neighbor)
                if(allclose(TSoI_neighbor,Tp1_p2)):
                    found = True
                    rmap[neighbor].shapeHandle = None

                    # keep the index of the sphere in the reachability map
                    myList.append(PathElement(neighbor,tIdx))

                    # Assign the next sphere of interest
                    SoI = rmap[neighbor]
                    Tbase_SoI = Tbase_neighbor
                    break
                
            if(found):
                break

        if(not found):
            # if we tested all neighbors and couldn't find a match
            # then these start and goal candidates are not
            # connected. Return error.
            return None

    return myList

def my_function(start,idx,myPattern,rm):
    myList = []
    # For each sphere in the pattern,
    # (except the first and the last)
    #
    # Set the current sphere of interest
    # "sphere of interest" is the sphere of which 
    # we're trying to find the neighbor
    SoI = start
    Tbase_SoI = start.T[idx]

    for n in range(1,len(myPattern)-1):
        # sys.stdin.readline()

        # All spheres in the search pattern
        # are saved in the pattern's coordinate
        # frame, of which the first sphere.T
        # is the origin.
        #
        # To go neighbor by neighbor, we need
        # to find the relative transform between
        # the pattern's consecutive spheres.
        #
        # Get the relative transform 
        # between two consecutive spheres
        # of the pattern:
        Tp_p1 = myPattern[n-1].T
        Tp_p2 = myPattern[n].T
        Tp1_p2 = dot(linalg.inv(Tp_p1),Tp_p2)
                                
        # Go through the neighbors of SoI
        for neighbor in SoI.neighbors:
            found = False
            # SoI.neighbors is a list of integers,
            # the list keeps the indices of the 
            # neighbors of SoI
            
            # See if any of SoI's neighbors
            # has the same relative transform
            # as Tp1_p2
            for tIdx, Tbase_neighbor in enumerate(rm.map[neighbor].T):                                        
                TSoI_neighbor = dot(linalg.inv(Tbase_SoI),Tbase_neighbor)
                if(allclose(TSoI_neighbor,Tp1_p2)):
                    found = True
                    rm.map[neighbor].shapeHandle = None

                    # keep the index of the sphere in the reachability map
                    myList.append(PathElement(neighbor,tIdx))

                    # Assign the next sphere of interest
                    SoI = rm.map[neighbor]
                    Tbase_SoI = Tbase_neighbor
                    break
                
            if(found):
                break

        if(not found):
            # if we tested all neighbors and couldn't find a match
            # then these start and goal candidates are not
            # connected. Return error.
            return None

    return myList

# def find_a_random_candidate_in(patterns,rmaps,numCandidates,lowBound,upBound):
#     pass
    
def find_random_candidates(patterns,rmaps,numCandidates):
    #print "Finding ",str(numCandidates)," random candidates for each reachability map."
    # Keep and return all path candidates
    candidates = []
    for i, rm in enumerate(rmaps):
        Tstart_goal = patterns[i].pattern[-1].T # the last element
        distStartGoal = pow(pow(Tstart_goal[0,3],2)+pow(Tstart_goal[1,3],2)+pow(Tstart_goal[2,3],2),0.5)
        paths = []
        iters = 0
        while(len(paths) < numCandidates):
            path = []
            s1Idx = randint(0,len(rm.map)-1)
            #print "s1Idx: ",str(s1Idx)
            # print "Iter#: ",str(iters)
            s1 = rm.map[s1Idx]
            
            # For each transform in s1 do:
            for l, Tbase_s1 in enumerate(s1.T):
                # Get the spheres if a solution exists
                steps = my_function2(s1,l,patterns[i].pattern,rm) 
                # See if a solution exists
                if(steps != None):
                     # Append the first sphere in the candidate path
                    path.append(PathElement(s1Idx,l))
                    # if start and goal candidates are connected a solution
                    # exists. The variable steps is a list. We don't want path to be nested, that's why we extend instead of appending.
                    path.extend(steps)
                    #print "path: "
                    #for e in path:
                    #    print e.sIdx
                    paths.append(path)
                    #print "Hit!, ",str(iters)
                    #print "Found ",str(len(paths))," of ", str(numCandidates)," so far..."
                    break
                # endif
            #endfor
            iters += 1
        # Display the list of candidate paths
        #print "candidate paths: "
        #for potentialPath in paths:
        #    for e in potentialPath:
        #        print e
        # append all possible paths for this manipulator to the bigger list of candidates
        rmaps[i].candidates.append(paths)
        candidates.append(paths)
    return candidates

def find_all_candidates(patterns, rmaps):
    # Keep and return all path candidates
    candidates = []
    print "you called Reachability.solve with 2 arguments:"
    print patterns
    print rmaps
    # sys.stdin.readline()
    
    if(len(patterns) != len(rmaps)):
        print "Error 1!"
        return None

    # The relative transform between the search pattern's spheres
    # should exist in the reachability map
    # 
    # See if Tstart_goal exist, and then try to find
    # the ones that lay in between.
    #
    # Go through the reachability maps
    for i, rm in enumerate(rmaps):
        # For this reachability map, keep the possible paths in the following list
        # print "Search pattern's transform:"
        # print Tstart_goal
        # Trasform of the goal point in start point's coordinate frame
        Tstart_goal = patterns[i].pattern[-1].T # the last element
        distStartGoal = pow(pow(Tstart_goal[0,3],2)+pow(Tstart_goal[1,3],2)+pow(Tstart_goal[2,3],2),0.5)

        paths = []
        # For each sphere in the map do
        for j, s1 in enumerate(rm.map):
            if((j%100)==0):
                print "Tstart: ",str(j),"/",str(len(rm.map))
            for k, s2 in enumerate(rm.map):
                # Find s2 in s1's coord frame just to 
                # calculate the distance between them
                # at this point we don't care about the T's orientation
                myT = dot(linalg.inv(s1.T[0]),s2.T[0])
                distS1S2 = pow(pow(myT[0,3],2)+pow(myT[1,3],2)+pow(myT[2,3],2),0.5)
                if(distS1S2 == distStartGoal):
                    for l, Tbase_s1 in enumerate(s1.T):
                        path = []
                        for m, Tbase_s2 in enumerate(s2.T):
                            Ts1_s2 = dot(linalg.inv(Tbase_s1),Tbase_s2)
                            # print "Current transform: "
                            # print Ts1_s2

                            if(allclose(Ts1_s2,Tstart_goal)):
                                print "in reachability map ",str(i),":"
                                print "spheres ",str(j)," and ",str(k)
                                print "(transforms ",str(l)," and ",str(m)," )"
                                print "is a match to search pattern's Tstart_goal"
                                s1.shapeHandle = None
                                s2.shapeHandle = None

                                # Now go trought the pattern's spheres:
                                path.append(PathElement(j,l))
                                steps = my_function(s1,l,patterns[i].pattern,rm) 
                                if(steps != None):
                                    # if start and goal candidates are connected
                                    # we don't want this list to be nested. Extend instead of appending.
                                    path.extend(steps)
                                    path.append(PathElement(k,m))
                                    print "path: "
                                    for e in path:
                                        print e.sIdx
                                    break
                                else:
                                    # if start and goal candidates are not connected
                                    path = []
                        # Append this path to the array of candidates
                        if (path != []):
                            paths.append(path)

        # Display the list of candidate paths
        print "candidate paths: "
        for potentialPath in paths:
            for e in potentialPath:
                print e
        # append all possible paths for this manipulator to the bigger list of candidates
        rmaps[i].candidates.append(paths)
        candidates.append(paths)
        
    #
    # candidates = satisfy(constraints, paths)
    #
    # sys.stdin.readline()
    return candidates


# Note to self: for find_candidates, there is no point
# in searching for s2 spheres that are far away.
# If the Euclidean distance between s1 and s2 is greater
# than the Euclidean distance search pattern's Tstart - Tgoal
# don't bother evaluating that s2 as a Tgoal candidate.
#
# This distance doesn't depend on the transform of the sphere.
# We just need the relative XYZ translation.
#
# let x,y,z be the translation of s2 in s1's coordinate frame
#
# We can find the euclidean distance with the following equation:
#
# distInMap = pow(pow(x,2)+pow(y,2)+pow(z,2),0.5)
# 
# We already know the distance between Tstart_goal
#
# distSearchPattern = pow(pow(x,2)+pow(y,2)+pow(z,2),0.5)
#
# if(disInMap <= distSearch):
#    s2 might have a Tgoal candidate in it.


# Do search over multiple maps considering the boundaries
def search(reachabilityMaps, mapTs, patterns, patternTs, myEnv): 
    # Get a deep copy of the maps, we don't want to
    # mess with the originals
    rm = []
    rmT = []
    p = []
    pT = []
    howMany = 200
    candidates = []
    paths0 = []
    paths1 = []
    for idx, m in enumerate(reachabilityMaps):
        rm.append(deepcopy(m.map))
        p.append(deepcopy(patterns[idx].pattern))
        if idx > 0:
            rmT.append(deepcopy(mapTs[idx-1]))
            pT.append(deepcopy(patternTs[idx-1]))

    for m in range(1,len(rm)):
        # Transform of map_i+1 with reference to map_i
        Ti_j= rmT[m-1]

        # Convert all reachability spheres of map_i+1
        # into map_i's base tranform
        for sIdx, s in enumerate(rm[m]):
            for tIdx, Tj_s in enumerate(s.T):
                Ti_s = array(dot(Ti_j,Tj_s)) # dot() returns a matrix
                s.T[tIdx] = Ti_s

        # TO-DO
        # Cut out the parts of the reachability maps that are unwanted.
        # The definition of "unwanted" depends on:
        #  i) the search pattern, and,
        # ii) the reachability maps.
        # TO-DO

        # # Show all reachability spheres
        # for sIdx, s in enumerate(rm[m-1]):
        #     # robot on the left
        #     print "robot0"
        #     print type(s.T[0])
        #     s.color = array((0,0,1,0.5))
        #     s.show(myEnv)
            
        # sys.stdin.readline()

        # for sIdx, s in enumerate(rm[m]):
        #     # robot on the right
        #     print "robot1"
        #     print type(s.T[0])
        #     s.color = array((1,0,0,0.5))
        #     s.show(myEnv)
            
        # # Wait
        # sys.stdin.readline()

        # Now map_i+1's reachability spheres are
        # all defined in map_i's base transform

        # Transform of pattern_i+1 with reference to pattern_i
        pTi_j = patternTs[m-1]

        # Convert all search pattern elements of pattern_i+1
        # into pattern_i's base transform
        for idx, s in enumerate(p[m]):
            pTj_s = s.T
            pTi_s = dot(pTi_j,pTj_s)
            s.T = pTi_s

        # Get the euclidean distance between pattern's root's
        pD2 = pow(pTi_j[0,3],2)+pow(pTi_j[1,3],2)+pow(pTi_j[2,3],2)
        pD = pow(pD2,0.5)

        # Find all pairs of reachability spheres that are 
        # exactly eD apart from each other
        pairs = []
        print "finding pairs..."
        # NOTE, DO A SMARTER SEARCH HERE. 
        # WE DON'T WANT A COMPLEXITY OF N^2
        s1Init = int(ceil(random()*(len(rm[m-1])-1)))
        s2Init = int(ceil(random()*(len(rm[m])-1)))
        # done = False
        # while(not done):
        #     s1Init = int(ceil(random()*(len(rm[m-1])-1)))
        #     #print s1Init
        #     allHigh = True
        #     for n in rm[m-1][s1Init].neighbors:
        #         if rm[m-1][n].reachability != 6:
        #             allHigh = False
        #             break
        #     if(allHigh):
        #         done = True
        # done = False
        # while(not done):
        #     s2Init = int(ceil(random()*(len(rm[m])-1)))
        #     #print s2Init
        #     allHigh = True
        #     for n in rm[m][s2Init].neighbors:
        #         if rm[m][n].reachability != 6:
        #             allHigh = False
        #             break
        #     if(allHigh):
        #         done = True
        for s1Idx, s1 in enumerate(rm[m-1][s1Init:-1]):
            for s2Idx, s2 in enumerate(rm[m][s2Init:-1]):
                sD = euclidean_distance(s1.T[0],s2.T[0])
                # If the euclidean distance of the pattern transforms
                # is equal to spheres' distance, then keep the pair
                if(sD == pD):
                    # Find which transforms of these spheres
                    # satisfy the transform between search patterns
                    transformsMatch = False
                    for t1Idx, Ti_s1 in enumerate(s1.T):
                        for t2Idx, Ti_s2 in enumerate(s2.T):
                            # if there's a match keep the pair
                            Ts1_s2 = dot(linalg.inv(Ti_s1),Ti_s2)
                            if(allclose(pTi_j,Ts1_s2)):
                                transformsMatch = True
                                pair1 = PathElement(s1Idx,t1Idx)
                                pair2 = PathElement(s2Idx,t2Idx)
                                pairs.append([pair1,pair2])
                                print "found a pair: ",str(len(pairs))
                                print str(s1Idx)," : ",str(t1Idx)
                                print str(s2Idx)," : ",str(t2Idx)
                                break
                            else:
                                # else, discard, even if the distance
                                # is close enough
                                pass
                        if(transformsMatch):
                            break
                if len(pairs) == howMany :
                    break
            if len(pairs) == howMany:
                break
                
        print "finding candidates.."
        for pair in pairs:
            print "Trying: "
            print str(pair[0].sIdx)," : ",str(pair[0].tIdx)
            print str(pair[1].sIdx)," : ",str(pair[1].tIdx)
            steps0 = my_function2(rm[m-1][pair[0].sIdx],pair[0].tIdx,p[m-1],rm[m-1])
            if(steps0 != None):
                path0 = []
                path0.append(PathElement(pair[0].sIdx,pair[0].tIdx))
                path0.extend(steps0)
                break

            
            steps1 = my_function2(rm[m][pair[1].sIdx],pair[1].tIdx,p[m],rm[m])
            if(steps1 != None):
                path1 = []
                path1.append(PathElement(pair[1].sIdx,pair[1].tIdx))
                path1.extend(steps1)
                break

            if(steps0 != None and steps1 != None):
                print "start indices: ",str(pair[0].sIdx),", ",str(pair[1].sIdx)
                # path is in a list because we might 
                # have more than one path candidates
                paths0.append(path0) 
                paths1.append(path1)

        if(paths0 != [] and paths1 != []):
            candidates.append(paths0)
            candidates.append(paths1)

    print "found ",str(len(candidates[0]))," candidates."
    if(candidates != []):
        # for c in candidates[0]:
        #     for pe in c:
        #         print pe.sIdx
        #         print pe.tIdx
        return candidates
    else:
        return None

def euclidean_distance(t1,t2):
    return pow(pow(t1[0,3]-t2[0,3],2)+pow(t1[1,3]-t2[1,3],2)+pow(t1[2,3]-t2[2,3],2),0.5)
