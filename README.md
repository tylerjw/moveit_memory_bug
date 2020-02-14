# MoveIt Memory Bug

This repo is a minimal example of a bug that I think relates to the memory allocation in the RobotState.

In this PR https://github.com/ros-planning/moveit/pull/1382 @negril posted a stack trace that is very similar to the one produced by this repo.

## Installation

OpenVINO is an intel library that uses interposition to redefine malloc and in a way that breaks assumptions made by the code in RobotState.

This is not a contrived example, OpenVINO is a library that we are currently trying to use in the same node as we are using MoveIt.

### OpenVINO toolkit

The vision system also utilizes Intel OpenVINO toolkit for processing the image data (neural net inference). Use the
following commands to install pre-built libraries.
```
wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB | sudo apt-key add -
echo "deb https://apt.repos.intel.com/openvino/2019/ all main" | sudo tee -a /etc/apt/sources.list.d/intel-openvino-2019.list > /dev/null
sudo apt update
sudo apt --yes install intel-openvino-dev-ubuntu18-2019.3.376
```
After installing the package, remember to source the environment variables script. Remember to add it to your `.bashrc` script.
```
source /opt/intel/openvino/bin/setupvars.sh
```

For a thorough documentation of different installation options, see
[the official instructions for installing the OpenVINO toolkit](https://docs.openvinotoolkit.org/latest/_docs_install_guides_installing_openvino_linux.html).

### Building the package

*Before following these instructions make sure you have followed the above instructions to install OpenVINO.*

1. Install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) for Ubuntu 18.04. This package primarily
 targets melodic.

1. Install the following build tools:

        sudo apt-get install python-wstool python-catkin-tools

1. Re-use or create a catkin workspace:

        export ROS_WS=~/ws_ros/
        mkdir -p $ROS_WS/src
        cd $ROS_WS/src

1. Download the required repositories and install any dependencies:

        wstool init .  # Create a 'master' rosinstall file in src folder
        git clone https://github.com/tylerjw/moveit_memory_bug.git
        wstool merge -t . moveit_memory_bug/upstream.rosinstall
        wstool merge -t . moveit_memory_bug/moveit_memory_bug.rosinstall
        wstool update -t .  # this will download all the files from git as described in the .rosinstall file
        rosdep install --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -r -y  # This installs system dependencies via apt

1. Configure workspace:

        cd $ROS_WS
        catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Debug

1. Compile workspace (including full vision stack):

        catkin build

1. Source the workspace.

        source devel/setup.bash

## Run the test

To build and run the test use these commands:

        cd $ROS_WS/src/moveit_memory_bug
        catkin run_tests --this --no-deps

After the test is built you can run it with just this command:

        rostest moveit_memory_bug unit_test.test --text

To run in GDB add this to the test tag in test/unit_test.test then run using rostest:

        launch-prefix="gdb -ex run --args"

## The error

Eigen produces this error when you run the test in this packge:

```
/usr/include/eigen3/Eigen/src/Core/DenseStorage.h:128: Eigen::internal::plain_array<T, Size, MatrixOrArrayOptions, 32>::plain_array() [with T = double; int Size = 16; int MatrixOrArrayOptions = 0]: Assertion `(internal::UIntPtr(eigen_unaligned_array_assert_workaround_gcc47(array)) & (31)) == 0 && "this assertion is explained here: " "http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html" " **** READ THIS WEB PAGE !!! ****"' failed.
```
