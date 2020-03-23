cmake_minimum_required(VERSION 2.4.6)
project(cordial_tablet)

find_package(catkin REQUIRED message_generation geometry_msgs)

add_message_files(
  DIRECTORY
  msg
  
)

add_service_files(DIRECTORY srv)

generate_messages(DEPENDENCIES geometry_msgs)


catkin_package(CATKIN_DEPENDS message_runtime)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)




