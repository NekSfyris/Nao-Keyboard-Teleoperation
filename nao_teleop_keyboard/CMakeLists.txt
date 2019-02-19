cmake_minimum_required(VERSION 2.8.3)
project(nao_teleop_keyboard)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostime
  std_msgs
  message_generation
  roscpp_serialization
  roslib
  geometry_msgs
)

#find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


catkin_package(CATKIN_DEPENDS roscpp geometry_msgs message_runtime std_msgs naoqi_bridge_msgs sensor_msgs ${ROS_CXX_DEPENDENCIES})


add_executable(teleop_nao src/teleop_nao_keyboard_node.cpp)
target_link_libraries(teleop_nao ${catkin_LIBRARIES})
add_dependencies(teleop_nao ${catkin_EXPORTED_TARGETS})

install(TARGETS teleop_nao
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
