/*
 * 2013-08-26 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 *
 * Spawns an object at the center pose of the robot's tray
 */
#include "ros/ros.h"
#include "sim_manipulator.h"

int main(int argc, char **argv)
{
  std::string pr2_gazebo_name = "race_pr2"; //TODO retrieve name automatically
  // default object name in Gazebo
  std::string spawn_object = "race_flowers1";
  // Pose of object relative to the robots pose
  tf::Transform robot_tray = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.3, 0.0, 0.4));

  if (argc >= 2)
    pr2_gazebo_name = argv[1];
  if (argc >= 3)
    spawn_object = argv[2];

  ros::init(argc, argv, spawn_object + "on_tray");
  ros::NodeHandle nh;

  // Set Gazebo to RACE map transform
  SimManipulator simManipulator(nh,
    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-12.28, -10.20, 0.0)));

  // Update the AMCL localization with the pose from the simulation
  //simManipulator.updateAmclFromSimulation(pr2_gazebo_name);
  // Get robot pose
  gazebo_msgs::ModelState robot_model;
  robot_model.model_name = pr2_gazebo_name;
  simManipulator.getModelPose(robot_model);

  //convert  gazebo_msgs::ModelState to  tf::Pose
  tf::Transform gaz_robot = tf::Transform(
      tf::Quaternion(
        robot_model.pose.orientation.x,
        robot_model.pose.orientation.y,
        robot_model.pose.orientation.z,
        robot_model.pose.orientation.w),
      tf::Vector3(
        robot_model.pose.position.x,
        robot_model.pose.position.y,
        robot_model.pose.position.z)
      );

  //transform object pose
  tf::Transform gaz_tray = gaz_robot * robot_tray;

  //
  if (simManipulator.modelExistsInSimulation(spawn_object))
  {
    //move object
    gazebo_msgs::ModelState move_object;
    move_object.model_name = spawn_object;
    move_object.pose.orientation.x = gaz_tray.getRotation().getX();
    move_object.pose.orientation.y = gaz_tray.getRotation().getY();
    move_object.pose.orientation.z = gaz_tray.getRotation().getZ();
    move_object.pose.orientation.w = gaz_tray.getRotation().getW();
    move_object.pose.position.x = gaz_tray.getOrigin().getX();
    move_object.pose.position.y = gaz_tray.getOrigin().getY();
    move_object.pose.position.z = gaz_tray.getOrigin().getZ();
    simManipulator.moveObject(move_object);
  }
  else
  {
    //spawn object
    FluentSimObject spawn_model;
    spawn_model.name = spawn_object;
    spawn_model.pose_map = gaz_tray;
    simManipulator.spawnFromFluent(spawn_model);
  }

  return 0;
}
