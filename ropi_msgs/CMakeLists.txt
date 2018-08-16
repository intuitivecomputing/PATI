cmake_minimum_required(VERSION 2.8.3)
project(ropi_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin_simple REQUIRED)

catkin_simple()

## System dependencies are found with CMake's conventions
# use_boost()
# use_eigen()

# cs_add_library(my_lib src/${PROJECT_NAME}/test.cpp)

# cs_add_executable(${PROJECT_NAME}_node src/pc_seg.cpp)
#cs_add_executable(plane_seg src/plane_seg.cpp)
#target_link_libraries(plane_seg ${Boost_LIBRARIES} ${PCL_LIBRARIES} )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME pc_seg PREFIX "")


cs_install()

# cs_install_scripts(scripts/my_script.py)

cs_export()

