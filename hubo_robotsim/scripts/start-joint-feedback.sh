#!/bin/sh
# First follow the installation procedures for the drc_hubo pipline
# and for the hubo-ach/RobotSim connexion using the procedure at :
# http://dasl.mem.drexel.edu/drcwiki/index.php/RobotSim_Software_Package
# the RobotSim necessary files can be found in hubo_robotsim
# configure RVIZ with the file found in hubo_rviz_config
# Start hubo-ach and RobotSim in simulation mode then :
roslaunch hubo_launch full_body_feedback_node.launch sim:=true
#roslaunch hubo_launch display_drchubo_state.launch
#rosrun valve_localization valve_localizer.py &
#roslaunch hubo_planner valve_planner.launch &
#rosrun rviz rviz &
