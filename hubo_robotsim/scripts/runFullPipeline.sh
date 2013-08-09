#!/usr/bin/env bash

export command1="./start-hubo-motion-rt.sh; bash"
export command2="./start-robot-sim.sh; bash"
export command3="./start-trajectory-interface.sh; bash"
export command4="./start-joint-feedback.sh; bash"

gnome-terminal -x sh -c "$command1" --title="hubo-motion"
read -p "Press any key to continue... " -n1 -s
echo "start robot-sim..."
gnome-terminal -x sh -c "$command2" --title="robot-sim"
read -p "Press any key to continue... " -n1 -s
echo "start ros..."
gnome-terminal -x sh -c "$command4" --title="joint-feedback"
echo "start joint trajectory..."
gnome-terminal -x sh -c "$command3" --title="trajectory-interface"
