#!/usr/bin/env bash
# generated from catkin/cmake/templates/setup.bash.in

CATKIN_SHELL=bash

# source setup.sh from same directory as this file
CATKIN_SETUP_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
. "$CATKIN_SETUP_DIR/setup.sh"

#echo $CATKIN_SETUP_DIR

# export Gazebo paths
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$CATKIN_SETUP_DIR"/../../gazebo_models/romi/plugin/build/"
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:"/opt/ros/kinetic/lib/"
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:"/usr/lib/x86_64-linux-gnu/gazebo-8/plugins/"
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$CATKIN_SETUP_DIR"/../../gazebo_models/"

#echo $GAZEBO_PLUGIN_PATH
#echo $GAZEBO_MODEL_PATH