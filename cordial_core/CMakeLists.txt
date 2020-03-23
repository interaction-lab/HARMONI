cmake_minimum_required(VERSION 2.4.6)
project(cordial_core)

catkin_python_setup()

find_package(catkin REQUIRED actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_action_files(FILES Behavior.action Player.action) 

generate_messages(DEPENDENCIES actionlib_msgs)

install(DIRECTORY scripts src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)






