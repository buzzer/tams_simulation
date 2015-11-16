cmake_minimum_required(VERSION 2.8.3)
project(race_imagination)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")

#rosbuild_find_ros_package(race_simulation_run)
#rosbuild_find_ros_package(actionlib_msgs)
#find_package(catkin REQUIRED)
#find_package(catkin REQUIRED COMPONENTS race_simulation_run)
#include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
find_package(catkin REQUIRED COMPONENTS actionlib_msgs)
find_package(catkin REQUIRED COMPONENTS geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
    ParamList.msg SimState.msg AtomDbg.msg Atom.msg ConfidenceList.msg
    DurationList.msg RobotState.msg WorldState.msg
)

add_action_files(
  DIRECTORY action
  FILES Imagine.action
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  DEPENDS message_runtime
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#catkin_package(DEPENDS message_runtime)
#uncomment if you have defined services
#rosbuild_gensrv()
#catkin_package(DEPENDS service_runtime)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
find_package(Boost REQUIRED COMPONENTS thread)
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(sim_dispatcher src/sim_dispatcher.cpp)
add_dependencies(sim_dispatcher ${PROJECT_NAME}_gencpp)
add_executable(sim_client src/sim_client.cpp)
add_dependencies(sim_client ${PROJECT_NAME}_gencpp)
#add_executable(test_pub src/test_pub.cpp)
add_executable(sim_wrapper src/sim_wrapper.cpp)
add_dependencies(sim_wrapper ${PROJECT_NAME}_gencpp)
#add_dependencies(sim_wrapper ${PROJECT_NAME}_gencpp)
#add_executable(sim_wrapper_testclient src/sim_wrapper_testclient.cpp)

#target_link_libraries(example ${PROJECT_NAME})
#target_link_libraries(sim_client race_simulation_run)
#rosbuild_link_boost(sim_dispatcher thread)
#rosbuild_link_boost(sim_client thread)
target_link_libraries(sim_dispatcher ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(sim_client ${Boost_LIBRARIES} ${catkin_LIBRARIES})
