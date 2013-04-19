#!/bin/bash
python makeTraj.py movetraj0.txt
mv out.traj valve0.traj
python makeTraj.py movetraj1.txt
mv out.traj valve1.traj
python makeTraj.py movetraj2.txt
mv out.traj valve2.traj
python makeTraj.py movetraj3.txt
mv out.traj valve3.traj

