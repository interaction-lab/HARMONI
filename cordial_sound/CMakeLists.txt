cmake_minimum_required(VERSION 2.4.6)

project(cordial_sound)

catkin_python_setup()

find_package(catkin REQUIRED message_generation geometry_msgs)

add_message_files(
  FILES
  SoundRequest.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS},
  include
)

include_directories(include ${catkin_INCLUDE_DIRS}, include)

install(DIRECTORY scripts src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(test test.cpp)

install(FILES soundplay_node.launch test.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


set(ROS_BUILD_TYPE RelWithDebInfo)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


