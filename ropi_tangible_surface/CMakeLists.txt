cmake_minimum_required(VERSION 2.8.3)
project(ropi_tangible_surface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)

catkin_simple()

include_directories(3rdparty 3rdparty/oscpack)
## System dependencies are found with CMake's conventions
use_boost()
# use_eigen()

set(TUIO_PATH 3rdparty/TUIO)
set(OSC_PATH 3rdparty/oscpack)
# cs_add_library(my_lib src/${PROJECT_NAME}/test.cpp)
set(COMMON_TUIO_SOURCES ${TUIO_PATH}/TuioTime.cpp ${TUIO_PATH}/TuioPoint.cpp ${TUIO_PATH}/TuioContainer.cpp ${TUIO_PATH}/TuioObject.cpp ${TUIO_PATH}/TuioCursor.cpp ${TUIO_PATH}/TuioBlob.cpp ${TUIO_PATH}/TuioDispatcher.cpp ${TUIO_PATH}/TuioManager.cpp  ${TUIO_PATH}/OneEuroFilter.cpp)
set(SERVER_TUIO_SOURCES ${TUIO_PATH}/TuioServer.cpp ${TUIO_PATH}/UdpSender.cpp ${TUIO_PATH}/TcpSender.cpp ${TUIO_PATH}/WebSockSender.cpp ${TUIO_PATH}/FlashSender.cpp)
set(CLIENT_TUIO_SOURCES ${TUIO_PATH}/TuioClient.cpp ${TUIO_PATH}/OscReceiver.cpp ${TUIO_PATH}/UdpReceiver.cpp ${TUIO_PATH}/TcpReceiver.cpp)
set(OSC_SOURCES ${OSC_PATH}/osc/OscTypes.cpp ${OSC_PATH}/osc/OscOutboundPacketStream.cpp ${OSC_PATH}/osc/OscReceivedElements.cpp ${OSC_PATH}/osc/OscPrintReceivedElements.cpp ${OSC_PATH}/ip/posix/NetworkingUtils.cpp ${OSC_PATH}/ip/posix/UdpSocket.cpp)

cs_add_executable(tuio_node src/tuio_server.cpp ${COMMON_TUIO_SOURCES} ${SERVER_TUIO_SOURCES} ${OSC_SOURCES})
# target_link_libraries(${PROJECT_NAME}_node my_lib)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")


# cs_install()

# cs_install_scripts(scripts/my_script.py)

# cs_export()

