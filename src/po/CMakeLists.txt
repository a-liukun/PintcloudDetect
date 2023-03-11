cmake_minimum_required(VERSION 3.0.2)
project(po)
set (CMake policy CMP0074)
set(CMAKE_CXX_STANDARD 11)
 

 
#add_executable(TestVisualizer demo.cpp)
#target_link_libraries(TestVisualizer ${Open3D_LIBRARIES})
 
#target_include_directories(TestVisualizer PUBLIC ${Open3D_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
po  src/po.cpp
)

target_link_libraries(po
${catkin_LIBRARIES}
${PCL_LIBRARIES}

##${OpenCV_LIBRARIES}
)




