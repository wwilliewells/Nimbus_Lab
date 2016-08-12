cmake_minimum_required(VERSION 2.8.3)
project(gps_testing)

find_package(catkin REQUIRED COMPONENTS
  collab_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
   Sensor.msg
   Flight.msg
   Move.msg
   Ack.msg
   Gps.msg
   Bytes.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES collab_msgs std_msgs geometry_msgs)

## set nodes
set(node1 gps_testing)
set(node2 parse_serial)
set(node3 basic_flight)
#set(node4 acknowledge)
set(node4 xbee)
set(node5 gps_device)
set(node6 test_flight)
set(node7 reach)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${node1}/include
  ${node2}/include
  ${node3}/include
  ${node4}/include
  ${node5}/include
  ${node6}/include
  ${node7}/include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS ${include_directories}
  CATKIN_DEPENDS collab_msgs message_runtime roscpp rospy std_msgs
)

###########
## Build ##
###########

## gps_testing srcs
set(${node1}_srcs
  ${node1}/src/${node1}.cpp
  ${node1}/src/${node1}_init.cpp
  ${node1}/src/${node1}_callback.cpp
  ${node1}/src/${node1}_thread.cpp
)

## basic_flight srcs
set(${node3}_srcs
  ${node3}/src/${node3}.cpp
  ${node3}/src/${node3}_init.cpp
  ${node3}/src/${node3}_callback.cpp
  #${node3}/src/${node3}_thread.cpp
)

## parse_serial
set(${node2}_srcs
  ${node2}/src/${node2}.cpp
  ${node2}/src/${node2}_init.cpp
  ${node2}/src/${node2}_callback.cpp
)

## reach
set(${node7}_srcs
  ${node7}/src/${node7}.cpp
  ${node7}/src/${node7}_init.cpp
  ${node7}/src/${node7}_callback.cpp
)

## rf, xbee, acknowledge
set(${node4}_srcs
  ${node4}/src/${node4}.cpp
  ${node4}/src/${node4}_init.cpp
  ${node4}/src/${node4}_callback.cpp
)

# gps devices
set(${node5}_srcs
  ${node5}/src/${node5}.cpp
  ${node5}/src/${node5}_init.cpp
  ${node5}/src/${node5}_callback.cpp
)

## test flight srcs
set(${node6}_srcs
  ${node3}/src/${node6}.cpp
  ${node3}/src/${node6}_init.cpp
  ${node3}/src/${node6}_callback.cpp
  ${node3}/src/${node6}_thread.cpp
)

## Declare a cpp executable
add_executable(${node1} ${gps_testing_srcs})
add_executable(${node2} ${parse_serial_srcs})
add_executable(${node3} ${basic_flight_srcs})
#add_executable(${node4} ${acknowledge_srcs})
add_executable(${node4} ${xbee_srcs})
add_executable(${node5} ${gps_device_srcs})
add_executable(${node6} ${test_flight_srcs})
add_executable(${node7} ${reach_srcs})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${node1} collab_msgs_gencpp ${node1}_gencpp)
add_dependencies(${node3} collab_msgs_gencpp ${node1}_gencpp)
add_dependencies(${node6} collab_msgs_gencpp ${node1}_gencpp)
add_dependencies(${node4} ${node1}_gencpp)
add_dependencies(${node2} ${node1}_gencpp)
add_dependencies(${node5} ${node1}_gencpp)
add_dependencies(${node7} ${node1}_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${node1} ${catkin_LIBRARIES})

target_link_libraries(${node2} ${catkin_LIBRARIES})

target_link_libraries(${node3} ${catkin_LIBRARIES})

target_link_libraries(${node4} ${catkin_LIBRARIES})

target_link_libraries(${node5} ${catkin_LIBRARIES})

target_link_libraries(${node6} ${catkin_LIBRARIES})

target_link_libraries(${node7} ${catkin_LIBRARIES})
