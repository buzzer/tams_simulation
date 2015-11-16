cmake_minimum_required(VERSION 2.8.3)
project(race_simulation_run)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin REQUIRED roscpp)


catkin_package(
  INCLUDE_DIRS include # exported includes
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

#add_library(${PROJECT_NAME} src/sim_creator.cpp src/sim_manipulator.cpp src/parser_yaml.cpp)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

add_executable(sim_run src/sim_run.cpp)
add_executable(sim_run_race_common src/sim_run_race_common.cpp)
add_executable(sim_run_semantic_dispatcher src/sim_run_semantic_dispatcher.cpp)

#rosbuild_add_executable(sim_test src/sim_test.cpp)

add_executable(race_run src/race_run.cpp)
#add_executable(race_run_demo3 src/race_run_demo3.cpp)
add_executable(race_world_pr2 src/race_world_pr2.cpp)
add_executable(sim_amcl_update src/sim_amcl_update.cpp)
add_executable(spawn_on_tray src/spawn_on_tray.cpp)
add_executable(sim_object_listener src/sim_object_listener.cpp)
add_executable(object_monitor src/object_monitor.cpp)
add_executable(object_reset src/object_reset.cpp)
add_library(parser_yaml src/parser_yaml.cpp)

#target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_link_libraries(sim_run ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(sim_run_semantic_dispatcher ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(sim_run_race_common ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(sim_amcl_update ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(race_world_pr2 ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(spawn_on_tray ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(object_reset ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(sim_object_listener ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(race_run ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(object_monitor ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#target_link_libraries(sim_test ${PROJECT_NAME})
#target_link_libraries(amcl_test ${PROJECT_NAME})
