cmake_minimum_required(VERSION 2.8.3)
project(nano_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  apriltags_ros
  apriltag_msgs
)



find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nano_slam
  CATKIN_DEPENDS roscpp visualization_msgs apriltags_ros
DEPENDS Eigen3
)



# include_directories(include)
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})
