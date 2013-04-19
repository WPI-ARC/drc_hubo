#! /bin/bash

time_stamp=$(date +%Y%m%d_%H%M%S)
mkdir ../hubo-read-trajectory/trajectories/openrave_format/$time_stamp
cp ../../wpi_openrave/hubo/matlab/movetraj0.txt ../hubo-read-trajectory/trajectories/openrave_format/$time_stamp
cp ../../wpi_openrave/hubo/matlab/movetraj1.txt ../hubo-read-trajectory/trajectories/openrave_format/$time_stamp
cp ../../wpi_openrave/hubo/matlab/movetraj2.txt ../hubo-read-trajectory/trajectories/openrave_format/$time_stamp
cp ../../wpi_openrave/hubo/matlab/movetraj3.txt ../hubo-read-trajectory/trajectories/openrave_format/$time_stamp
