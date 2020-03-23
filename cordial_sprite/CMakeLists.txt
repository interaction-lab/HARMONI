cmake_minimum_required(VERSION 2.4.6)
project(cordial_sprite)


find_package(catkin REQUIRED actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_action_files(FILES KeyframePlayer.action) 

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package()

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)







