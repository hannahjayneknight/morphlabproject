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

### The hardware

Two iPower GBM4108H-120T Gimbal Motors will be used (BLDC): https://www.robotshop.com/uk/ipower-gbm4108h-120t-gimbal-motor.html

_Which motor drivers will be used?_

_Which joint sensors will be used?_

### The software

This tutorial provides a nice explanation of connecting ROS and Arduino: https://maker.pro/arduino/tutorial/how-to-control-a-robot-arm-with-ros-and-arduino

SimpleFOC is used to control the motors: https://docs.simplefoc.com/code

SimpleFOC was downloaded as a library to the Arduino IDE: https://docs.simplefoc.com/library_download

## Project structure

.
├── build
│   ├── atomic_configure
│   │   ├── env.sh
│   │   ├── local_setup.bash
│   │   ├── local_setup.sh
│   │   ├── local_setup.zsh
│   │   ├── setup.bash
│   │   ├── setup.sh
│   │   ├── _setup_util.py
│   │   └── setup.zsh
│   ├── bin
│   ├── catkin
│   │   └── catkin_generated
│   │       └── version
│   │           └── package.cmake
│   ├── catkin_generated
│   │   ├── env_cached.sh
│   │   ├── generate_cached_setup.py
│   │   ├── installspace
│   │   │   ├── env.sh
│   │   │   ├── local_setup.bash
│   │   │   ├── local_setup.sh
│   │   │   ├── local_setup.zsh
│   │   │   ├── setup.bash
│   │   │   ├── setup.sh
│   │   │   ├── _setup_util.py
│   │   │   └── setup.zsh
│   │   ├── order_packages.cmake
│   │   ├── order_packages.py
│   │   ├── setup_cached.sh
│   │   └── stamps
│   │       └── Project
│   │           ├── interrogate_setup_dot_py.py.stamp
│   │           ├── order_packages.cmake.em.stamp
│   │           ├── package.xml.stamp
│   │           └── _setup_util.py.stamp
│   ├── CATKIN_IGNORE
│   ├── catkin_make.cache
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   │   ├── 3.16.3
│   │   │   ├── CMakeCCompiler.cmake
│   │   │   ├── CMakeCXXCompiler.cmake
│   │   │   ├── CMakeDetermineCompilerABI_C.bin
│   │   │   ├── CMakeDetermineCompilerABI_CXX.bin
│   │   │   ├── CMakeSystem.cmake
│   │   │   ├── CompilerIdC
│   │   │   │   ├── a.out
│   │   │   │   ├── CMakeCCompilerId.c
│   │   │   │   └── tmp
│   │   │   └── CompilerIdCXX
│   │   │       ├── a.out
│   │   │       ├── CMakeCXXCompilerId.cpp
│   │   │       └── tmp
│   │   ├── clean_test_results.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── cmake.check_cache
│   │   ├── CMakeDirectoryInformation.cmake
│   │   ├── CMakeError.log
│   │   ├── CMakeOutput.log
│   │   ├── CMakeRuleHashes.txt
│   │   ├── CMakeTmp
│   │   ├── download_extra_data.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── doxygen.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── Makefile2
│   │   ├── Makefile.cmake
│   │   ├── progress.marks
│   │   ├── run_tests.dir
│   │   │   ├── build.make
│   │   │   ├── cmake_clean.cmake
│   │   │   ├── DependInfo.cmake
│   │   │   └── progress.make
│   │   ├── TargetDirectories.txt
│   │   └── tests.dir
│   │       ├── build.make
│   │       ├── cmake_clean.cmake
│   │       ├── DependInfo.cmake
│   │       └── progress.make
│   ├── cmake_install.cmake
│   ├── CTestConfiguration.ini
│   ├── CTestCustom.cmake
│   ├── CTestTestfile.cmake
│   ├── gtest
│   │   ├── CMakeFiles
│   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   └── progress.marks
│   │   ├── cmake_install.cmake
│   │   ├── CTestTestfile.cmake
│   │   ├── googlemock
│   │   │   ├── CMakeFiles
│   │   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   │   ├── gmock.dir
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   ├── depend.make
│   │   │   │   │   ├── flags.make
│   │   │   │   │   ├── link.txt
│   │   │   │   │   ├── progress.make
│   │   │   │   │   └── src
│   │   │   │   ├── gmock_main.dir
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   ├── depend.make
│   │   │   │   │   ├── flags.make
│   │   │   │   │   ├── link.txt
│   │   │   │   │   ├── progress.make
│   │   │   │   │   └── src
│   │   │   │   └── progress.marks
│   │   │   ├── cmake_install.cmake
│   │   │   ├── CTestTestfile.cmake
│   │   │   └── Makefile
│   │   ├── googletest
│   │   │   ├── CMakeFiles
│   │   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   │   ├── gtest.dir
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   ├── depend.make
│   │   │   │   │   ├── flags.make
│   │   │   │   │   ├── link.txt
│   │   │   │   │   ├── progress.make
│   │   │   │   │   └── src
│   │   │   │   ├── gtest_main.dir
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   ├── depend.make
│   │   │   │   │   ├── flags.make
│   │   │   │   │   ├── link.txt
│   │   │   │   │   ├── progress.make
│   │   │   │   │   └── src
│   │   │   │   └── progress.marks
│   │   │   ├── cmake_install.cmake
│   │   │   ├── CTestTestfile.cmake
│   │   │   └── Makefile
│   │   ├── lib
│   │   └── Makefile
│   ├── Makefile
│   ├── test_results
│   ├── threelink
│   │   ├── catkin_generated
│   │   │   ├── installspace
│   │   │   │   ├── threelinkConfig.cmake
│   │   │   │   ├── threelinkConfig-version.cmake
│   │   │   │   └── threelink.pc
│   │   │   ├── ordered_paths.cmake
│   │   │   ├── package.cmake
│   │   │   ├── pkg.develspace.context.pc.py
│   │   │   ├── pkg.installspace.context.pc.py
│   │   │   └── stamps
│   │   │       └── threelink
│   │   │           ├── package.xml.stamp
│   │   │           └── pkg.pc.em.stamp
│   │   ├── CMakeFiles
│   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   ├── progress.marks
│   │   │   ├── roscpp_generate_messages_cpp.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── roscpp_generate_messages_eus.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── roscpp_generate_messages_lisp.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── roscpp_generate_messages_nodejs.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── roscpp_generate_messages_py.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── rosgraph_msgs_generate_messages_cpp.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── rosgraph_msgs_generate_messages_eus.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── rosgraph_msgs_generate_messages_lisp.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── rosgraph_msgs_generate_messages_nodejs.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── rosgraph_msgs_generate_messages_py.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── std_msgs_generate_messages_cpp.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── std_msgs_generate_messages_eus.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── std_msgs_generate_messages_lisp.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   ├── std_msgs_generate_messages_nodejs.dir
│   │   │   │   ├── build.make
│   │   │   │   ├── cmake_clean.cmake
│   │   │   │   ├── DependInfo.cmake
│   │   │   │   └── progress.make
│   │   │   └── std_msgs_generate_messages_py.dir
│   │   │       ├── build.make
│   │   │       ├── cmake_clean.cmake
│   │   │       ├── DependInfo.cmake
│   │   │       └── progress.make
│   │   ├── cmake_install.cmake
│   │   ├── CTestTestfile.cmake
│   │   └── Makefile
│   └── twolink_v0
│       ├── catkin_generated
│       │   ├── installspace
│       │   │   ├── twolink_v0Config.cmake
│       │   │   ├── twolink_v0Config-version.cmake
│       │   │   └── twolink_v0.pc
│       │   ├── ordered_paths.cmake
│       │   ├── package.cmake
│       │   ├── pkg.develspace.context.pc.py
│       │   ├── pkg.installspace.context.pc.py
│       │   └── stamps
│       │       └── twolink_v0
│       │           ├── package.xml.stamp
│       │           └── pkg.pc.em.stamp
│       ├── CMakeFiles
│       │   ├── CMakeDirectoryInformation.cmake
│       │   └── progress.marks
│       ├── cmake_install.cmake
│       ├── CTestTestfile.cmake
│       └── Makefile
├── devel
│   ├── cmake.lock
│   ├── env.sh
│   ├── lib
│   │   └── pkgconfig
│   │       ├── threelink.pc
│   │       └── twolink_v0.pc
│   ├── local_setup.bash
│   ├── local_setup.sh
│   ├── local_setup.zsh
│   ├── setup.bash
│   ├── setup.sh
│   ├── _setup_util.py
│   ├── setup.zsh
│   └── share
│       ├── threelink
│       │   └── cmake
│       │       ├── threelinkConfig.cmake
│       │       └── threelinkConfig-version.cmake
│       └── twolink_v0
│           └── cmake
│               ├── twolink_v0Config.cmake
│               └── twolink_v0Config-version.cmake
└── src
    ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
    ├── threelink
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   └── controller_settings.yaml
    │   ├── include
    │   │   └── threelink
    │   ├── launch
    │   │   └── threelink.launch
    │   ├── package.xml
    │   ├── src
    │   │   ├── kinematics.py
    │   │   └── test_points
    │   │       ├── dk_points.csv
    │   │       ├── fk_points.csv
    │   │       ├── ik_points.csv
    │   │       ├── points.csv
    │   │       └── workspace_points.csv
    │   └── urdf
    │       └── robot_model_gazebo.xacro
    └── twolink_v0
        ├── CMakeLists.txt
        ├── config
        │   └── controller_settings.yaml
        ├── include
        │   └── twolink_v0
        ├── launch
        │   └── twolink_v0.launch
        ├── package.xml
        ├── robot-arm-control-arduino
        │   └── robot-arm-control-arduino
        │       └── robot-arm-control-arduino.ino
        ├── src
        │   ├── kinematics.py
        │   └── test_points
        │       ├── dk_points.csv
        │       ├── fk_points.csv
        │       ├── ik_points.csv
        │       ├── points.csv
        │       └── workspace_points.csv
        └── urdf
            └── robot_model_gazebo.xacro

