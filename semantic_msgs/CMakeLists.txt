cmake_minimum_required(VERSION 3.10)
project(semantic_msgs)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

  
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Point2d.msg
  Point3d.msg
  Ellipse.msg
  Circle.msg
  Polygon.msg
  Cylinder.msg
  Polyhedron.msg
  SemanticArray.msg
  )
 
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  )


catkin_package(
  CATKIN_DEPENDS
  message_runtime
  DEPENDS
  )
  
