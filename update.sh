#!/bin/bash

git fetch
git reset --hard origin/main

git clean -d --force
git pull

cd /home/jetson/catkin_ws

source devel/setup.bash