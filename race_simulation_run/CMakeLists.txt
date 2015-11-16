cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")

rosbuild_find_ros_package(race_pr2_markers)
rosbuild_find_ros_package(robot_motion_param)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(sim_manipulator src/sim_manipulator.cpp)
rosbuild_add_library(${PROJECT_NAME} src/sim_creator.cpp)
rosbuild_add_library(${PROJECT_NAME} src/sim_manipulator.cpp)
rosbuild_add_library(${PROJECT_NAME} src/parser_yaml.cpp)
rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(sim_run src/sim_run.cpp)
rosbuild_add_executable(sim_run_race_common src/sim_run_race_common.cpp)
rosbuild_add_executable(sim_run_semantic_dispatcher src/sim_run_semantic_dispatcher.cpp)
#rosbuild_add_executable(sim_test src/sim_test.cpp)
rosbuild_add_executable(race_run src/race_run.cpp)
rosbuild_add_executable(race_run_demo3 src/race_run_demo3.cpp)
rosbuild_add_executable(race_world_pr2 src/race_world_pr2.cpp)
rosbuild_add_executable(sim_amcl_update src/sim_amcl_update.cpp)
rosbuild_add_executable(spawn_on_tray src/spawn_on_tray.cpp)
rosbuild_add_executable(sim_object_listener src/sim_object_listener.cpp)
rosbuild_add_executable(object_monitor src/object_monitor.cpp)
rosbuild_add_executable(object_reset src/object_reset.cpp)
#rosbuild_add_executable(test_simtime src/test_simtime.cpp)
#rosbuild_add_executable(amcl_test src/amcl_test.cpp)

target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_link_libraries(sim_run ${PROJECT_NAME})
target_link_libraries(sim_run_semantic_dispatcher ${PROJECT_NAME})
target_link_libraries(sim_run_race_common ${PROJECT_NAME})
target_link_libraries(sim_amcl_update ${PROJECT_NAME})
target_link_libraries(spawn_on_tray ${PROJECT_NAME})
target_link_libraries(object_reset ${PROJECT_NAME})
target_link_libraries(sim_object_listener ${PROJECT_NAME})
#target_link_libraries(sim_test ${PROJECT_NAME})
#target_link_libraries(amcl_test ${PROJECT_NAME})

