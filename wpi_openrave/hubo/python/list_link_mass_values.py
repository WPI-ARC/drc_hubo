#!/usr/bin/env python

## OPENRAVE ##
from openravepy import *
import sys
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

from openravepy.misc import OpenRAVEGlobalArguments
import os # for file operations

def run(args):
    error = True
    env = Environment()
    RaveSetDebugLevel(DebugLevel.Info)
    for aIdx, a in enumerate(args):
        if a == '--robot':
            try:
                # '../../../openHubo/jaemi/humanoids2013.jaemiHubo.planning.robot.xml'
                robot = env.ReadRobotURI(sys.argv[aIdx+1])
                for l in robot.GetLinks():
                    print str(l.GetName()),str(': '),str(l.GetMass())
                error = False
            except:
                break
    if(error):
        print ""
        print "Error!!!"
        print ""
        print "Usage: "
        print ""
        print "python list_link_mass_values.py <path_to_your_robot_file>.robot.xml"
        print ""
    env.Destroy()
    RaveDestroy()

if __name__ == "__main__":
    run(sys.argv)
            


