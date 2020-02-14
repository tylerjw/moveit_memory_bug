# MoveIt Memory Bug

This repo is a minimal example of a bug that I think relates to the memory allocation in the RobotState.

In this PR https://github.com/ros-planning/moveit/pull/1382 @negril posted a stack trace that is very similar to the one produced by this repo.

## Installation

OpenVINO is an intel library for processing the image data (neural net inference).  This is not a contrived example, OpenVINO is a library that we are currently trying to use in the same node as we are using MoveIt.

### OpenVINO toolkit

Use the following commands to install pre-built libraries.
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

## Back Trace

When run in GDB this is the back trace:

```
unit_test: /usr/include/eigen3/Eigen/src/Core/DenseStorage.h:128: Eigen::internal::plain_array<T, Size, MatrixOrArrayOptions, 32>::plain_array() [with T = double; int Size = 16; int MatrixOrArrayOptions = 0]: Assertion `(internal::UIntPtr(eigen_unaligned_array_assert_workaround_gcc47(array)) & (31)) == 0 && "this assertion is explained here: " "http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html" " **** READ THIS WEB PAGE !!! ****"' failed.

Thread 1 "unit_test" received signal SIGABRT, Aborted.
__GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:51
51  ../sysdeps/unix/sysv/linux/raise.c: No such file or directory.
(gdb) bt
#0  __GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:51
#1  0x00007ffff5683801 in __GI_abort () at abort.c:79
#2  0x00007ffff567339a in __assert_fail_base (fmt=0x7ffff57fa7d8 "%s%s%s:%u: %s%sAssertion `%s' failed.\n%n", 
    assertion=assertion@entry=0x5555555c8150 "(internal::UIntPtr(eigen_unaligned_array_assert_workaround_gcc47(array)) & (31)) == 0 && \"this assertion is explained here: \" \"http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.htm"..., 
    file=file@entry=0x5555555c8118 "/usr/include/eigen3/Eigen/src/Core/DenseStorage.h", line=line@entry=128, 
    function=function@entry=0x5555555c8c00 <Eigen::internal::plain_array<double, 16, 0, 32>::plain_array()::__PRETTY_FUNCTION__> "Eigen::internal::plain_array<T, Size, MatrixOrArrayOptions, 32>::plain_array() [with T = double; int Size = 16; int MatrixOrArrayOptions = 0]")
    at assert.c:92
#3  0x00007ffff5673412 in __GI___assert_fail (
    assertion=0x5555555c8150 "(internal::UIntPtr(eigen_unaligned_array_assert_workaround_gcc47(array)) & (31)) == 0 && \"this assertion is explained here: \" \"http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.htm"..., 
    file=0x5555555c8118 "/usr/include/eigen3/Eigen/src/Core/DenseStorage.h", line=128, 
    function=0x5555555c8c00 <Eigen::internal::plain_array<double, 16, 0, 32>::plain_array()::__PRETTY_FUNCTION__> "Eigen::internal::plain_array<T, Size, MatrixOrArrayOptions, 32>::plain_array() [with T = double; int Size = 16; int MatrixOrArrayOptions = 0]") at assert.c:101
#4  0x00005555555b42cd in Eigen::internal::plain_array<double, 16, 0, 32>::plain_array (this=0x55555585ae70)
    at /usr/include/eigen3/Eigen/src/Core/DenseStorage.h:128
#5  0x00005555555b3952 in Eigen::DenseStorage<double, 16, 4, 4, 0>::DenseStorage (this=0x55555585ae70)
    at /usr/include/eigen3/Eigen/src/Core/DenseStorage.h:187
#6  0x00005555555b28da in Eigen::PlainObjectBase<Eigen::Matrix<double, 4, 4, 0, 4, 4> >::PlainObjectBase (this=0x55555585ae70)
    at /usr/include/eigen3/Eigen/src/Core/PlainObjectBase.h:484
#7  0x00005555555b1646 in Eigen::Matrix<double, 4, 4, 0, 4, 4>::Matrix (this=0x55555585ae70)
    at /usr/include/eigen3/Eigen/src/Core/Matrix.h:259
#8  0x00005555555b0596 in Eigen::Transform<double, 3, 1, 0>::Transform (this=0x55555585ae70)
    at /usr/include/eigen3/Eigen/src/Geometry/Transform.h:257
#9  0x00007ffff6e01fd2 in moveit::core::LinkModel::LinkModel (this=0x55555585acd0, name="panda_link0")
    at /home/tyler-pick/workspace/ws_bug/src/moveit/moveit_core/robot_model/src/link_model.cpp:53
#10 0x00007ffff6e20a02 in moveit::core::RobotModel::constructLinkModel (this=0x555555869920, urdf_link=0x555555860690)
    at /home/tyler-pick/workspace/ws_bug/src/moveit/moveit_core/robot_model/src/robot_model.cpp:972
#11 0x00007ffff6e1f4c3 in moveit::core::RobotModel::buildRecursive (this=0x555555869920, parent=0x0, urdf_link=0x555555860690, 
    srdf_model=...) at /home/tyler-pick/workspace/ws_bug/src/moveit/moveit_core/robot_model/src/robot_model.cpp:784
#12 0x00007ffff6e17de1 in moveit::core::RobotModel::buildModel (this=0x555555869920, urdf_model=..., srdf_model=...)
    at /home/tyler-pick/workspace/ws_bug/src/moveit/moveit_core/robot_model/src/robot_model.cpp:101
#13 0x00007ffff6e172b8 in moveit::core::RobotModel::RobotModel (this=0x555555869920, urdf_model=
    std::shared_ptr<urdf::ModelInterface> (use count 2, weak count 0) = {...}, 
    srdf_model=std::shared_ptr<const srdf::Model> (use count 4, weak count 0) = {...})
    at /home/tyler-pick/workspace/ws_bug/src/moveit/moveit_core/robot_model/src/robot_model.cpp:60
#14 0x00007ffff35375d5 in robot_model_loader::RobotModelLoader::configure (this=0x555555845f40, opt=...)
    at /home/tyler-pick/workspace/ws_bug/src/moveit/moveit_ros/planning/robot_model_loader/src/robot_model_loader.cpp:100
#15 0x00007ffff3536cf8 in robot_model_loader::RobotModelLoader::RobotModelLoader (this=0x555555845f40, 
    robot_description="robot_description", load_kinematics_solvers=true)
    at /home/tyler-pick/workspace/ws_bug/src/moveit/moveit_ros/planning/robot_model_loader/src/robot_model_loader.cpp:48
#16 0x00007ffff7344007 in __gnu_cxx::new_allocator<robot_model_loader::RobotModelLoader>::construct<robot_model_loader::RobotModelLoader, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&> (this=0x7fffffffba87, __p=0x555555845f40, 
    __args#0="robot_description") at /usr/include/c++/7/ext/new_allocator.h:136
#17 0x00007ffff733eb58 in std::allocator_traits<std::allocator<robot_model_loader::RobotModelLoader> >::construct<robot_model_loader::RobotModelLoader, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&> (__a=..., __p=0x555555845f40, 
    __args#0="robot_description") at /usr/include/c++/7/bits/alloc_traits.h:475
#18 0x00007ffff73392b2 in std::_Sp_counted_ptr_inplace<robot_model_loader::RobotModelLoader, std::allocator<robot_model_loader::RobotModelLoader>, (__gnu_cxx::_Lock_policy)2>::_Sp_counted_ptr_inplace<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&> (this=0x555555845f30, __a=...) at /usr/include/c++/7/bits/shared_ptr_base.h:526
#19 0x00007ffff7332ce7 in std::__shared_count<(__gnu_cxx::_Lock_policy)2>::__shared_count<robot_model_loader::RobotModelLoader, std::allocator<robot_model_loader::RobotModelLoader>, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&> (
    this=0x7fffffffbc68, __a=...) at /usr/include/c++/7/bits/shared_ptr_base.h:637
#20 0x00007ffff732affc in std::__shared_ptr<robot_model_loader::RobotModelLoader, (__gnu_cxx::_Lock_policy)2>::__shared_ptr<std::allocator<robot_model_loader::RobotModelLoader>, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&> (
    this=0x7fffffffbc60, __tag=..., __a=...) at /usr/include/c++/7/bits/shared_ptr_base.h:1295
#21 0x00007ffff732035d in std::shared_ptr<robot_model_loader::RobotModelLoader>::shared_ptr<std::allocator<robot_model_loader::RobotModelLoader>, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&> (this=0x7fffffffbc60, __tag=..., __a=...)
    at /usr/include/c++/7/bits/shared_ptr.h:344
#22 0x00007ffff7312f36 in std::allocate_shared<robot_model_loader::RobotModelLoader, std::allocator<robot_model_loader::RobotModelLoader>, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&> (__a=..., __args#0="robot_description")
    at /usr/include/c++/7/bits/shared_ptr.h:691
#23 0x00007ffff7307c74 in std::make_shared<robot_model_loader::RobotModelLoader, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&> (__args#0="robot_description") at /usr/include/c++/7/bits/shared_ptr.h:707
#24 0x00007ffff72e89d3 in planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor (this=0x555555848680, 
    scene=std::shared_ptr<planning_scene::PlanningScene> (empty) = {...}, robot_description="robot_description", tf_buffer=
    std::shared_ptr<tf2_ros::Buffer> (use count 1, weak count 0) = {...}, name="planning_scene_monitor")
    at /home/tyler-pick/workspace/ws_bug/src/moveit/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp:126
#25 0x00007ffff72e894f in planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor (this=0x555555848680, 
    robot_description="robot_description", tf_buffer=std::shared_ptr<tf2_ros::Buffer> (use count 1, weak count 0) = {...}, 
    name="planning_scene_monitor")
    at /home/tyler-pick/workspace/ws_bug/src/moveit/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp:119
#26 0x00007ffff76b29f5 in moveit::planning_interface::MoveItCpp::loadPlanningSceneMonitor (this=0x5555558458c0, options=...)
    at /home/tyler-pick/workspace/ws_bug/src/moveit/moveit_ros/planning_interface/moveit_cpp/src/moveit_cpp.cpp:128
---Type <return> to continue, or q <return> to quit---
#27 0x00007ffff76b1790 in moveit::planning_interface::MoveItCpp::MoveItCpp (this=0x5555558458c0, options=..., 
    tf_buffer=std::shared_ptr<tf2_ros::Buffer> (empty) = {...})
    at /home/tyler-pick/workspace/ws_bug/src/moveit/moveit_ros/planning_interface/moveit_cpp/src/moveit_cpp.cpp:69
#28 0x00005555555b7688 in __gnu_cxx::new_allocator<moveit::planning_interface::MoveItCpp>::construct<moveit::planning_interface::MoveItCpp, moveit::planning_interface::MoveItCpp::Options&, ros::NodeHandle&> (this=0x7fffffffc2f7, __p=0x5555558458c0, __args#0=..., __args#1=...)
    at /usr/include/c++/7/ext/new_allocator.h:136
#29 0x00005555555b62e4 in std::allocator_traits<std::allocator<moveit::planning_interface::MoveItCpp> >::construct<moveit::planning_interface::MoveItCpp, moveit::planning_interface::MoveItCpp::Options&, ros::NodeHandle&> (__a=..., __p=0x5555558458c0, __args#0=..., __args#1=...)
    at /usr/include/c++/7/bits/alloc_traits.h:475
#30 0x00005555555b5537 in std::_Sp_counted_ptr_inplace<moveit::planning_interface::MoveItCpp, std::allocator<moveit::planning_interface::MoveItCpp>, (__gnu_cxx::_Lock_policy)2>::_Sp_counted_ptr_inplace<moveit::planning_interface::MoveItCpp::Options&, ros::NodeHandle&> (
    this=0x5555558458b0, __a=...) at /usr/include/c++/7/bits/shared_ptr_base.h:526
#31 0x00005555555b49ff in std::__shared_count<(__gnu_cxx::_Lock_policy)2>::__shared_count<moveit::planning_interface::MoveItCpp, std::allocator<moveit::planning_interface::MoveItCpp>, moveit::planning_interface::MoveItCpp::Options&, ros::NodeHandle&> (this=0x7fffffffc508, 
    __a=...) at /usr/include/c++/7/bits/shared_ptr_base.h:637
#32 0x00005555555b3ef0 in std::__shared_ptr<moveit::planning_interface::MoveItCpp, (__gnu_cxx::_Lock_policy)2>::__shared_ptr<std::allocator<moveit::planning_interface::MoveItCpp>, moveit::planning_interface::MoveItCpp::Options&, ros::NodeHandle&> (this=0x7fffffffc500, 
    __tag=..., __a=...) at /usr/include/c++/7/bits/shared_ptr_base.h:1295
#33 0x00005555555b32cd in std::shared_ptr<moveit::planning_interface::MoveItCpp>::shared_ptr<std::allocator<moveit::planning_interface::MoveItCpp>, moveit::planning_interface::MoveItCpp::Options&, ros::NodeHandle&> (this=0x7fffffffc500, __tag=..., __a=...)
    at /usr/include/c++/7/bits/shared_ptr.h:344
#34 0x00005555555b2400 in std::allocate_shared<moveit::planning_interface::MoveItCpp, std::allocator<moveit::planning_interface::MoveItCpp>, moveit::planning_interface::MoveItCpp::Options&, ros::NodeHandle&> (__a=..., __args#0=..., __args#1=...)
    at /usr/include/c++/7/bits/shared_ptr.h:691
#35 0x00005555555b1184 in std::make_shared<moveit::planning_interface::MoveItCpp, moveit::planning_interface::MoveItCpp::Options&, ros::NodeHandle&> (__args#0=..., __args#1=...) at /usr/include/c++/7/bits/shared_ptr.h:707
#36 0x00005555555adc31 in ExampleServerTest_ExampleUnitTests_Test::TestBody (this=0x55555583f220)
    at /home/tyler-pick/workspace/ws_bug/src/moveit_memory_bug/test/unit_test.cpp:37
#37 0x00007ffff799ac03 in testing::internal::HandleSehExceptionsInMethodIfSupported<testing::Test, void> (object=0x55555583f220, 
    method=&virtual testing::Test::TestBody(), location=0x7ffff79aad93 "the test body") at /usr/src/googletest/googletest/src/gtest.cc:2402
#38 0x00007ffff7994b8b in testing::internal::HandleExceptionsInMethodIfSupported<testing::Test, void> (object=0x55555583f220, 
    method=&virtual testing::Test::TestBody(), location=0x7ffff79aad93 "the test body") at /usr/src/googletest/googletest/src/gtest.cc:2438
#39 0x00007ffff79780f4 in testing::Test::Run (this=0x55555583f220) at /usr/src/googletest/googletest/src/gtest.cc:2474
#40 0x00007ffff7978a2e in testing::TestInfo::Run (this=0x55555583a530) at /usr/src/googletest/googletest/src/gtest.cc:2656
#41 0x00007ffff79790d1 in testing::TestCase::Run (this=0x555555816c00) at /usr/src/googletest/googletest/src/gtest.cc:2776
#42 0x00007ffff798007a in testing::internal::UnitTestImpl::RunAllTests (this=0x55555583a780)
    at /usr/src/googletest/googletest/src/gtest.cc:4651
#43 0x00007ffff799bdc9 in testing::internal::HandleSehExceptionsInMethodIfSupported<testing::internal::UnitTestImpl, bool> (
    object=0x55555583a780, 
    method=(bool (testing::internal::UnitTestImpl::*)(testing::internal::UnitTestImpl * const)) 0x7ffff797fda8 <testing::internal::UnitTestImpl::RunAllTests()>, location=0x7ffff79ab5d0 "auxiliary test code (environments or event listeners)")
    at /usr/src/googletest/googletest/src/gtest.cc:2402
#44 0x00007ffff79959c7 in testing::internal::HandleExceptionsInMethodIfSupported<testing::internal::UnitTestImpl, bool> (
    object=0x55555583a780, 
    method=(bool (testing::internal::UnitTestImpl::*)(testing::internal::UnitTestImpl * const)) 0x7ffff797fda8 <testing::internal::UnitTestImpl::RunAllTests()>, location=0x7ffff79ab5d0 "auxiliary test code (environments or event listeners)")
    at /usr/src/googletest/googletest/src/gtest.cc:2438
#45 0x00007ffff797ec0b in testing::UnitTest::Run (this=0x7ffff7bcd7c0 <testing::UnitTest::GetInstance()::instance>)
    at /usr/src/googletest/googletest/src/gtest.cc:4259
#46 0x00005555555b01fe in RUN_ALL_TESTS () at /usr/src/googletest/googletest/include/gtest/gtest.h:2233
#47 0x00005555555add7b in main (argc=2, argv=0x7fffffffcbe8)
    at /home/tyler-pick/workspace/ws_bug/src/moveit_memory_bug/test/unit_test.cpp:43

```
