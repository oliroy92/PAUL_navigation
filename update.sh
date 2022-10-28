#!/bin/bash

cd /home/jetson/catkin_ws/src/paul_ros

git fetch
git reset --hard origin/main

git clean -d --force
git pull

cd /home/jetson/catkin_ws

source devel/setup.bash

catkin_make