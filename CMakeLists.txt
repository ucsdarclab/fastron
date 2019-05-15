cmake_minimum_required(VERSION 2.8.3)
project(fastron)

## Compile as C++11, supported in ROS Kinetic and newer. turn on optimization
add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  interactive_markers
)
find_package(Eigen3 REQUIRED)
find_package(OMPL)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    moveit_visual_tools
    interactive_markers
  DEPENDS EIGEN3
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(fastronColCheckTests7D src/fastronCollisionCheckingTests7D.cpp src/fastron.cpp src/interactive_robot.cpp src/imarker.cpp src/pose_string.cpp)
target_link_libraries(fastronColCheckTests7D ${catkin_LIBRARIES} ${Boost_LIBRARIES} interactive_markers)

add_executable(fastronColCheckTestsMultiobject src/fastronCollisionCheckingTestsMultiobject.cpp src/fastron.cpp src/interactive_robot.cpp src/imarker.cpp src/pose_string.cpp)
target_link_libraries(fastronColCheckTestsMultiobject ${catkin_LIBRARIES} ${Boost_LIBRARIES} interactive_markers)

add_executable(fastronColCheckTestsMultiobject2 src/fastronCollisionCheckingTestsMultiobject2.cpp src/fastron.cpp src/interactive_robot.cpp src/imarker.cpp src/pose_string.cpp)
target_link_libraries(fastronColCheckTestsMultiobject2 ${catkin_LIBRARIES} ${Boost_LIBRARIES} interactive_markers)

add_executable(fastronColCheckTestsMultiobject3 src/fastronCollisionCheckingTestsMultiobject3.cpp src/fastron.cpp src/interactive_robot.cpp src/imarker.cpp src/pose_string.cpp)
target_link_libraries(fastronColCheckTestsMultiobject3 ${catkin_LIBRARIES} ${Boost_LIBRARIES} interactive_markers)

add_executable(fastronColCheckTestsMultiobject4 src/fastronCollisionCheckingTestsMultiobject4.cpp src/fastron.cpp src/interactive_robot.cpp src/imarker.cpp src/pose_string.cpp)
target_link_libraries(fastronColCheckTestsMultiobject4 ${catkin_LIBRARIES} ${Boost_LIBRARIES} interactive_markers)

add_executable(fastronMotionPlanningTests src/fastronMotionPlanningTests.cpp src/fastron.cpp src/interactive_robot.cpp src/imarker.cpp src/pose_string.cpp)
target_link_libraries(fastronMotionPlanningTests ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(fastronMotionPlanningMultiobjectTests src/fastronMotionPlanningMultiobjectTests.cpp src/fastron.cpp src/interactive_robot.cpp src/imarker.cpp src/pose_string.cpp)
target_link_libraries(fastronMotionPlanningMultiobjectTests ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(fastronMotionPlanningVisualizeTests src/fastronMotionPlanningVisualizeTests.cpp src/fastron.cpp src/interactive_robot.cpp src/imarker.cpp src/pose_string.cpp)
target_link_libraries(fastronMotionPlanningVisualizeTests ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES})
