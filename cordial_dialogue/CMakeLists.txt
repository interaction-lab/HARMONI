cmake_minimum_required(VERSION 2.4.6)
project(cordial_dialogue)

catkin_python_setup()


find_package(catkin REQUIRED COMPONENTS
  aws_common
  aws_ros1_common
  roscpp
  std_msgs
  actionlib_msgs
  message_generation
)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   actionlib_msgs
# )

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES cordial_dialogue
  CATKIN_DEPENDS aws_ros1_common roscpp std_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${AWSSDK_INCLUDE_DIR}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY scripts src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


