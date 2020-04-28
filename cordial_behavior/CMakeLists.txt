cmake_minimum_required(VERSION 2.4.6)
project(cordial_behavior)


find_package(catkin REQUIRED 
roscpp
std_msgs
actionlib_msgs
audio_common_msgs
message_generation
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  Behavior.msg
)



generate_messages(
  DEPENDENCIES
  std_msgs
  audio_common_msgs
  actionlib_msgs
)


install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)






