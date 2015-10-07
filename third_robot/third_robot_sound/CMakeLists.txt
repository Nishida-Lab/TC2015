cmake_minimum_required(VERSION 2.8.3)
project(third_robot_sound)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg rospeex_if roslib)

## Declare ROS messages and services
#add_message_files(FILES sound_message.msg)
add_service_files(FILES sound_service.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

find_package(Boost REQUIRED COMPONENTS system)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospeex_if roslib
  DEPENDS system_lib
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

# add_executable(talker src/talker.cpp)
# target_link_libraries(talker ${catkin_LIBRARIES})
# add_dependencies(talker sound_generate_messages_cpp)

# add_executable(listener src/listener.cpp)
# target_link_libraries(listener ${catkin_LIBRARIES})
# add_dependencies(listener sound_generate_messages_cpp)

add_executable(third_robot_sound_server src/sound_server.cpp)
target_link_libraries(third_robot_sound_server ${catkin_LIBRARIES})
add_dependencies(third_robot_sound_server third_robot_sound_gencpp)

add_executable(third_robot_sound_client src/sound_client.cpp)
target_link_libraries(third_robot_sound_client ${catkin_LIBRARIES})
add_dependencies(third_robot_sound_client third_robot_sound_gencpp)

install(TARGETS third_robot_sound_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)