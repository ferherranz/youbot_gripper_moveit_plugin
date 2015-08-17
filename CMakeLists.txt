cmake_minimum_required(VERSION 2.8.3)
project(youbot_gripper_moveit_plugin)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  control_msgs
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
    moveit_core
    )

include_directories(include)

add_library(youbot_gripper_moveit_plugin src/youbot_gripper_moveit_plugin.cpp)
target_link_libraries(youbot_gripper_moveit_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS youbot_gripper_moveit_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES youbot_gripper_moveit_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )
