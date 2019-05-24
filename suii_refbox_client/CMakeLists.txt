cmake_minimum_required(VERSION 2.8.7)
project(suii_refbox_client)


find_package(
  Boost REQUIRED COMPONENTS system filesystem thread)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    protobuf_comm
    atwork_pb_msgs
    atwork_ros_msgs
    std_msgs
    geometry_msgs
    message_generation
)
find_package(Protobuf)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -DHAVE_LIBCRYPTO")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  DIRECTORY msg
  FILES
  atwork_client_msgs.msg
  atwork_action.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  atwork_ros_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    protobuf_comm
    atwork_pb_msgs
    atwork_ros_msgs
    atwork_client_msgs
    atwork_action
)

add_executable(suii_refbox_client
   src/suii_refbox_client_node.cpp
   src/suii_refbox_client.cpp
)

target_link_libraries(suii_refbox_client
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
)


add_dependencies(suii_refbox_client atwork_ros_msgs_generate_messages_cpp)
add_dependencies(suii_refbox_client suii_refbox_client_generate_messages_cpp)
add_dependencies(suii_refbox_client ${catkin_EXPORTED_TARGETS})
#add_dependencies(suii_refbox_client suii_refbox_client_ros_msgs_generate_messages_cpp)


install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  TARGETS suii_refbox_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


