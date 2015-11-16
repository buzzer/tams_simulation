#include "ros/ros.h"
#include "sim_manipulator.h"

int main(int argc, char **argv)
{
  std::string gazebo_name = "race_pr2";
  ros::init(argc, argv, "amcl_test");
  ros::NodeHandle nh;
  //ros::NodeHandle nh_("~"); // node private NodeHandle

  SimManipulator simManipulator(nh);

  while (ros::ok())
  {
    simManipulator.updateAmclFromSimulation(&gazebo_name);
    ros::spinOnce();
    ros::Duration(3.0).sleep();
  }

  return 0;
}
