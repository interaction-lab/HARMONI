cmake_minimum_required(VERSION 2.4.6)
project(cordial_tts)

catkin_python_setup()


find_package(catkin REQUIRED
std_msgs
actionlib_msgs
message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)





generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package()

install(DIRECTORY scripts src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


