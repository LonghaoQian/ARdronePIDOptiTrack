ARdrone With PID Position Control and OptiTrack Feedback (C++)

Longhao Qian Mar 15th 2019

This package works with ardrone_autonomy package.

website: https://ardrone-autonomy.readthedocs.io/en/latest/

installation: https://ardrone-autonomy.readthedocs.io/en/latest/installation.html

Binary install:

$ apt-get install ros-*-ardrone-autonomy

Compile from source:

$ cd ~/catkin_ws/src

$ git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b indigo-devel

$ cd ~/catkin_ws

$ rosdep install --from-paths src -i

$ catkin_make

Steps for settiing up the connections:

1. Connect the battery of the ARdrone, connect computer to the ARdrone wifi

2. Power up OptiTrack system, open Tracking Tools

3. In Tracking Tools, define the trackable as ARdrone

4. Turn on the OpTiTrack to stream vrpn data.

On Windows Machine(Computer Connected to OptiTrack):

local interface 169.254.1.1

check VRPN Streaming Engine-> broadcast Frame Data, check stream vrpn on port 3883

check Trackd Stream Frame Data 

Steps for launching the package:

1. launch the ardrone_autonomy package

$ roslaunch ardrone_autonomy ardrone.launch 

2. launch the control

$ ROS_HOME=`pwd` roslaunch ARdronePIDOptiTrack ARdronePID.launch

