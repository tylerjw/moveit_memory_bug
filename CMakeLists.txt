cmake_minimum_required(VERSION 2.8.3)
project(moveit_memory_bug)

# C++14
set(CMAKE_CXX_STANDARD 14)

# System dependencies are found with CMake's conventions
find_package(InferenceEngine 2.0)
if(NOT InferenceEngine_FOUND)
  message(FATAL_ERROR "\n\n Intel OpenVINO toolkit is missing, please install it using the instructions in README.md\n\n")
endif()

# Ammend InterfaceEngine variables for cpu extension target
set(InferenceEngine_LIBRARIES "IE::ie_cpu_extension")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
)

###################################
## Catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    moveit_ros_planning_interface
    roscpp
  DEPENDS
    InferenceEngine
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${InferenceEngine_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Mark config files for installation
install(
  DIRECTORY 
    config 
  DESTINATION 
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    unit_test 
    test/unit_test.test 
    test/unit_test.cpp
  )

  target_link_libraries(unit_test
    ${catkin_LIBRARIES}
    ${InferenceEngine_LIBRARIES}
  )
endif()
