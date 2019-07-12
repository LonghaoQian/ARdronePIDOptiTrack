### ARdrone With PID Position Control and OptiTrack Feedback (C++)
## Longhao Qian July 11th 2019

This package requires ardrone_autonomy package:

> website: https://ardrone-autonomy.readthedocs.io/en/latest/

> installation: https://ardrone-autonomy.readthedocs.io/en/latest/installation.html

Binary install:

> `$ apt-get install ros-*-ardrone-autonomy`

Compile from source:

> `$ cd ~/catkin_ws/src`

> `$ git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b indigo-devel`

> `$ cd ~/catkin_ws`

> `$ rosdep install --from-paths src -i`

> `$ catkin_make`

Steps for settiing up the wireless connections:

1. Connect the battery of the ARdrone, connect computer to the ARdrone wifi

2. Power up the OptiTrack system, and open Motive

3. In Motive, define the trackable as ARdrone

4. Turn on the OpTiTrack to stream vrpn data.

On Windows Machine(Computer Connected to OptiTrack):

> `Data Stream IP: 192.168.1.230`

check VRPN Streaming Engine-> broadcast Frame Data, check stream vrpn on port 3883

Steps for launching the package:

1. launch the ardrone_autonomy package

> `$ roslaunch ardrone_autonomy ardrone.launch`

2. launch the control

> `$ ROS_HOME=`pwd` roslaunch ARdronePIDOptiTrack ARdronePID.launch`

`ROS_HOME=`pwd` is used so that the recorded file will be placed in the same folder as the node.

