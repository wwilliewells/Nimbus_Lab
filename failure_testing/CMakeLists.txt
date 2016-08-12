cmake_minimum_required(VERSION 2.8.3)
project(flight_testing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  collab_msgs
  collab_test
  #mavros
  message_generation
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
   Sensor.msg
   Mavlink.msg
   Arducopter.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   collab_msgs std_msgs geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  #include
  flight_testing/include
  parse_serial/include
  communication/include
  #/opt/ros/jade/share/mavros
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS ${include_directories}
#  LIBRARIES flight_testing
  CATKIN_DEPENDS collab_msgs collab_test message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(flight_testing_headers
  flight_testing/include/flight_testing.h
  parse_serial/include/parse_serial.h
  communication/include/mavlink2MIT_AscTec.h
  communication/include/mavlink2ros.h
)

set(flight_testing_srcs
  flight_testing/src/flight_testing.cpp
  flight_testing/src/flight_testing_init.cpp
  flight_testing/src/flight_testing_callback.cpp
  flight_testing/src/flight_testing_thread.cpp
)

## Declare a cpp library
set(parse_serial_srcs
  parse_serial/src/parse_serial.cpp
  parse_serial/src/parse_serial_init.cpp
  parse_serial/src/parse_serial_callback.cpp
)

set(mavlink2mit_srcs
  communication/src/mavlink2mit_asctec.cpp
  communication/src/mavlink2mit_asctec_init.cpp
  communication/src/mavlink2mit_asctec_callback.cpp
  communication/src/mavlink2mit_asctec_thread.cpp
)

set(mavlink2ros_srcs
  communication/src/mavlink2ros.cpp
  communication/src/mavlink2ros_init.cpp
  communication/src/mavlink2ros_callback.cpp
)

## Declare a cpp executable
add_executable(flight_testing ${flight_testing_srcs})
add_executable(parse_serial ${parse_serial_srcs})
add_executable(translation ${mavlink2mit_srcs})
add_executable(communication ${mavlink2ros_srcs})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(flight_testing collab_msgs_gencpp control_msgs_gencpp flight_testing_gencpp)
add_dependencies(communication collab_msgs_gencpp control_msgs_gencpp flight_testing_gencpp)
add_dependencies(translation collab_msgs_gencpp control_msgs_gencpp flight_testing_gencpp)

# roslint?
#roslint_cpp(${flight_testing_headers} ${flight_testing_srcs})

## Specify libraries to link a library or executable target against
target_link_libraries(flight_testing ${catkin_LIBRARIES})

target_link_libraries(parse_serial ${catkin_LIBRARIES})

target_link_libraries(communication ${catkin_LIBRARIES})

target_link_libraries(translation ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS flight_testing flight_testing_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_flight_testing.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
