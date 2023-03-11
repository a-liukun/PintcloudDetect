cmake_minimum_required(VERSION 3.0.2)
project(louti)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
louti  src/loutiwode.cpp  src/preanalysis.cpp src/regions.cpp src/segmentPatch.cpp src/regiongrowing.cpp src/voxSAC.cpp src/splitmerge.cpp src/planeshape.cpp src/Stairs.cpp src/StairVector.cpp src/recognition.cpp
)

target_link_libraries(louti
${catkin_LIBRARIES}
${PCL_LIBRARIES}

)



