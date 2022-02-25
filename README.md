# morphlabproject

The goal of this project is to build a two-link planar oscillating robotic arm. This can be used to train a sleeve for a human's arm that will counteract tremours in the patient so they can live their life as normally as possible.

The project is split into the following stages:
1. Develop a simulation of the two-link, planar robotic arm using ROS.
2. Build the robot and test it.
3. Build a sleeve for the robot and train it to counteract the robot's tremours. 

We are a team of three students, Camilla Giulia Billari, Yuken Ge and Hannah Knight, working at the Morphlab, Imperial College London, under the supervision of Professor Thrishantha Nanayakkara.

For more details about Imperial College's Morphlab:
- https://www.imperial.ac.uk/morph-lab/
- https://thrish.org/

### Setting up the simulation

This repository contains the code for the simulated robot.

Setup:
1. Open repoisitory in Linux environment with ROS installed.
2. In one terminal, type "roscore". _Initializing ROS_
3. In another terminal, type "rosrun gazebo_ros gazebo". _Launching gazebo_
4. In another terminal, go to the root of this repository and type "source devel/setup.bash" if you haven't added this to the .bash file.
5. In the same terminal as step (4), type "roslaunch twolink_v0 twolink_v0.launch".
6. To see the robot's control (highly oscillatory) type "cd [root directory]/src/twolink_v0/src" followed by "python3 kinematics.py full" 

### The motors

Two iPower GBM4108H-120T Gimbal Motors will be used.

https://docs.simplefoc.com/torque_mode describes how torque control is implemented.

### Connecting ROS and Arduino

https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros

https://maker.pro/arduino/tutorial/how-to-control-a-robot-arm-with-ros-and-arduino

--> write later
