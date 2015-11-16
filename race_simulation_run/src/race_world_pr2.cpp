/*
 * 2014-04-25 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 *
 * Schedules the launch of the RACE Gazebo world
 */
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include "ros/ros.h"
#include "ros/package.h"

//bool gazeboIsRunning ( void )
bool isRunning ( const std::string p )
{
  const std::string cmd = "ps h -C ";
  const std:: string cmd_f = cmd + p;
  bool result = ! system(cmd_f.c_str());

  return result;
}

// ROS is not alive anymore here
bool stopRos(void)
{
  bool success = true;

  if ( isRunning("gzclient") )
    success &= system("killall gzclient &>> /dev/null");

  if ( isRunning("gzserver") )
    success &= system("killall gzserver &>> /dev/null");

  //// kill all nodes since we don't know which ones
  //ROS_WARN("Killing all ROS nodes..");
  //success &= system("rosnode kill -a &>> /dev/null");
  //sleep(5);
  //ROS_WARN("..done");

  return success;
}

int main(int argc, char **argv)
{
  boost::thread_group th_group;
  std::string args;

  ros::init(argc, argv, "race_world_pr2");
  ros::NodeHandle nh;

  for (int i=1; i < argc; i++)
    args.append(" " + std::string(argv[i]));

  boost::replace_all(args, "=", ":=");
  ROS_INFO("Gazebo arguments: %s", args.c_str());

  if (ros::ok())
    while (! isRunning("gzserver") )
    {
      th_group.create_thread(boost::bind(&system,
            "roslaunch race_gazebo_worlds race_world.launch gui:=false furniture:=false"));
      sleep(4);
    }

  if (ros::ok())
  {
    std::string items = "roslaunch race_gazebo_worlds spawn_scenario_items.launch " + args;
    ROS_INFO("%s",items.c_str());
    th_group.create_thread(boost::bind(&system, items.c_str()));
    sleep(1);
  }

  if (ros::ok())
  {
    if (args.find("gui:=true") != std::string::npos)
      th_group.create_thread(boost::bind(&system, "rosrun gazebo gui"));
    sleep(5);
  }

  if (ros::ok())
  {
    std::string spawnpr2 = "roslaunch race_gazebo_worlds spawn_pr2.launch " + args;
    ROS_INFO("%s", spawnpr2.c_str());
    th_group.create_thread(boost::bind(&system, spawnpr2.c_str()));
    sleep(5);
  }

  if (ros::ok())
    th_group.create_thread(boost::bind(&system,
          "rosrun dynamic_reconfigure dynparam set /gazebo max_update_rate 1000"));

  if (ros::ok())
    th_group.create_thread(boost::bind(&system, "rosservice call /gazebo/unpause_physics '{}'"));

  ros::spin();

  stopRos();
  th_group.join_all();

  return 0;
}
