#!/usr/bin/env bash
echo "source hubo motion start"
cd $HOME/workspace/ros_workspace/src/hubo-motion-rt/build
sudo service hubo-motion stop
sudo service hubo-motion kill
sudo service hubo-motion virtual

