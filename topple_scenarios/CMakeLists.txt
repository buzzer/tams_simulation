cmake_minimum_required(VERSION 2.8.3)
project(topple_scenarios)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(catkin REQUIRED message_generation)
find_package(catkin REQUIRED COMPONENTS race_simulation_run)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/topple_episode_monitor.cpp)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#add_executable(onegoal src/onegoal.cpp)
add_executable(cmd_goals src/cmd_goals.cpp)
#add_dependencies(cmd_goals ${PROJECT_NAME}_gencpp)
#add_executable(topple_episode_monitor src/topple_episode_monitor.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#target_link_libraries(onegoal ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(cmd_goals ${Boost_LIBRARIES} ${catkin_LIBRARIES}
  race_simulation_run)
