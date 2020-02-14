/*
   Author: Tyler Weaver, PickNik Consulting
   Desc: Unit test for demonstrating bug
*/

// C++
#include <memory>

// MoveIt
#include <moveit/moveit_cpp/moveit_cpp.h>

// Testing
#include <gtest/gtest.h>

// ROS
#include <ros/ros.h>

namespace {
// Comment out this function that is never called to see the bug go away
void test_function(const planning_scene_monitor::PlanningSceneMonitorPtr
                       &planning_scene_monitor) {
  const moveit::core::RobotStatePtr &kinematic_state =
      planning_scene_monitor->getStateMonitor()->getCurrentState();
  const Eigen::Isometry3d &command_to_camera_frame_transform =
      kinematic_state->getGlobalLinkTransform("base_link").inverse() *
      kinematic_state->getGlobalLinkTransform("camera");
}
}

TEST(ExampleServerTest, ExampleUnitTests) {
  ros::NodeHandle nh("~");
  moveit::planning_interface::MoveItCpp::Options moveit_cpp_options(nh);

  // Initialize MoveIt planning stack using MoveItCpp
  std::shared_ptr<moveit::planning_interface::MoveItCpp> moveit_cpp =
      std::make_shared<moveit::planning_interface::MoveItCpp>(
          moveit_cpp_options, nh);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "unit_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
