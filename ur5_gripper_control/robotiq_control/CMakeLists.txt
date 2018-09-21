cmake_minimum_required(VERSION 2.8.3)
project(robotiq_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roscpp_serialization
  rospy
  ur_modern_driver
  ur_msgs
  std_msgs
  message_generation
  ropi_msgs
)
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gripper_ur_control _robotiq_gripper_wrapper_cpp
  CATKIN_DEPENDS rosccp rospy ur_modern_driver ur_msgs
)

###########
## Build ##
###########
add_service_files(
  FILES
  GripperControl.srv
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_library(gripper_ur_control
	src/gripper_ur_control.cpp
)
target_link_libraries(gripper_ur_control ${catkin_LIBRARIES})

# add_library(_robotiq_gripper_wrapper_cpp
# 	src/gripper_wrapper.cpp
# )
# target_link_libraries(_robotiq_gripper_wrapper_cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES} gripper_ur_control)

install(TARGETS gripper_ur_control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
   	   PATTERN ".svn" EXCLUDE
   )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(sample_node src/sample_node.cpp)
target_link_libraries(sample_node ${catkin_LIBRARIES} gripper_ur_control)

add_executable(gripper_server src/gripper_server.cpp)
target_link_libraries(gripper_server ${catkin_LIBRARIES} gripper_ur_control)
add_dependencies(gripper_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} robotiq_control_gencpp)

add_executable(gripper_fake_joint_states src/fake_joint_states.cpp)
target_link_libraries(gripper_fake_joint_states ${catkin_LIBRARIES})

# Don't prepend wrapper library name with lib and add to Python libs.
# set_target_properties(_robotiq_gripper_wrapper_cpp PROPERTIES
#         PREFIX ""
#         LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
#         )
