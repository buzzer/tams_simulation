#include "sim_manipulator.h"
#include "sim_creator.h"
#include "ros/package.h"
#include <fstream>
#include <boost/thread.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~"); // node private NodeHandle

  SimManipulator simManipulator(nh_);
  //SimManipulator simManipulator;

  std::stringstream pr2_file_content;
  simManipulator.readModelFile(
      "/urdf/race_pr2.urdf",
      &pr2_file_content);

  std::string robot_description = pr2_file_content.str();
  nh_.setParam("robot_description", robot_description);

  std::string rd_from_param;
  if (nh_.hasParam("robot_description"))
      nh_.getParam("robot_description", rd_from_param);
  else
    ROS_ERROR("robot_descritpion not found on param server");

  //simManipulator.unpause();



  //ros::Duration(8.0).sleep();
  //sleep(3);

  //Spawn PR2
  gazebo_msgs::SpawnModel * spawn_pr2 = new gazebo_msgs::SpawnModel();
  spawn_pr2->request.model_name = "race_pr2";
  //std::stringstream pr2_file_content;
  //simManipulator.readModelFile(
      //"/urdf/race_pr2.urdf",
      //&pr2_file_content);
  //ROS_INFO("Modelfile content: %s", pr2_file_content.str().c_str());
  //spawn_pr2->request.model_xml = pr2_file_content.str();
  ROS_INFO("Modelfile content: %s", rd_from_param.c_str());
  spawn_pr2->request.model_xml = rd_from_param;
  spawn_pr2->request.robot_namespace = "/";
  //spawn_pr2->request.reference_frame = "world";
  spawn_pr2->request.initial_pose.position.x = 0.00;
  spawn_pr2->request.initial_pose.position.y = 0.00;
  spawn_pr2->request.initial_pose.position.z = 0.70;
  spawn_pr2->request.initial_pose.orientation.x = 0;
  spawn_pr2->request.initial_pose.orientation.y = 0;
  spawn_pr2->request.initial_pose.orientation.z = 0;
  spawn_pr2->request.initial_pose.orientation.w = 1;
  while ( ! ros::service::waitForService("/gazebo/spawn_urdf_model", ros::Duration(5.0)) );
  simManipulator.spawnUrdfObject(spawn_pr2);

  // Spawn mug
  gazebo_msgs::SpawnModel * spawn_mug = new gazebo_msgs::SpawnModel();
  spawn_mug->request.model_name = "race_mug_5";
  std::stringstream mug_file_content;
  simManipulator.readModelFile(
      "/urdf/race_coffee_cup.urdf",
      &mug_file_content);
  ROS_INFO("Modelfile content: %s", mug_file_content.str().c_str());
  spawn_mug->request.model_xml = mug_file_content.str();
  //spawn_mug->request.model_xml = rd_from_param;
  spawn_mug->request.robot_namespace = "/";
  //spawn_mug->request.reference_frame = "world";
  spawn_mug->request.initial_pose.position.x = 0.00;
  spawn_mug->request.initial_pose.position.y = 0.00;
  spawn_mug->request.initial_pose.position.z = 0.70;
  spawn_mug->request.initial_pose.orientation.x = 0;
  spawn_mug->request.initial_pose.orientation.y = 0;
  spawn_mug->request.initial_pose.orientation.z = 0;
  spawn_mug->request.initial_pose.orientation.w = 1;
  while ( ! ros::service::waitForService("/gazebo/spawn_urdf_model", ros::Duration(5.0)) );
  simManipulator.spawnUrdfObject(spawn_mug);

  return 0;
}
