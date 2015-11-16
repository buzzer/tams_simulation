/*
 * 2014-04-07 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 */
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//TODO check why header is not found.
//#include <race_simulation_run/sim_manipulator.h>
#include <../../race_simulation_run/include/sim_manipulator.h>
#include "tf/tf.h"
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <algorithm>    // std::random_shuffle
#include <vector>       // std::vector
#include <cstdlib>      // std::rand, std::srand

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool initGoalsLinear(std::vector<move_base_msgs::MoveBaseGoal> & goals)
{
  move_base_msgs::MoveBaseGoal goal;
  tf::Quaternion rot;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  ////home goal
  //goal.target_pose.pose.position.x = 12.27;
  //goal.target_pose.pose.position.y = 9.89;
  //goal.target_pose.pose.position.z = 0.0;
  //goal.target_pose.pose.orientation.x = 0.0;
  //goal.target_pose.pose.orientation.y = 0.0;
  //goal.target_pose.pose.orientation.z = 1.0;
  //goal.target_pose.pose.orientation.w = 0.0;

  //goals.push_back(goal);

  //race:posepreManipulationAreaNorthTable1
  //goal.target_pose.pose.position.x = 9.89;
  goal.target_pose.pose.position.x = 6.89;
  //goal.target_pose.pose.position.y = 9.77;
  goal.target_pose.pose.position.y = 11.77;
  goal.target_pose.pose.position.z = 0.0;
  //rot.setRPY(0, 0, -1.57);
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 1;
  goal.target_pose.pose.orientation.w = 0;

  goals.push_back(goal);

  return true;
}

//int myrandom (int i)
//{
  //return std::rand()%i;
//}

bool driveTo( MoveBaseClient & ac,
    std::vector<move_base_msgs::MoveBaseGoal> & goals)
{
  for (auto & goal : goals)
  {
    ROS_INFO("Sending goal..");
    ac.sendGoal(goal);

    ROS_INFO("Waiting for result..");
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved to the goal");
    else
      ROS_INFO("The base failed to move to goal. State is %s", ac.getState().toString().c_str());
  }
  return true;
}

//bool initVel(std::vector<float> & vel_trans)
//{
//  float vel = 0.15;
//  //vel_trans.push_back(vel);

//  vel += 0.1;
//  vel_trans.push_back(vel);

//  vel += 0.1;
//  vel_trans.push_back(vel);

//  vel += 0.1;
//  vel_trans.push_back(vel);

//  vel += 0.1;
//  vel_trans.push_back(vel);

//  return true;
//}

//bool setVel(float & vel)
//{
//  dynamic_reconfigure::ReconfigureRequest srv_req;
//  dynamic_reconfigure::ReconfigureResponse srv_resp;
//  dynamic_reconfigure::DoubleParameter double_param;
//  dynamic_reconfigure::Config conf;

//  double_param.name = "max_trans_vel";
//  double_param.value = vel;
//  conf.doubles.push_back(double_param);

//  double_param.name = "max_rot_vel";
//  double_param.value = 0.5;
//  conf.doubles.push_back(double_param);

//  srv_req.config = conf;

//  ROS_INFO("Setting to velocity (trans): %4.2f", vel);
//  ros::service::call("/move_base_node/DWAPlannerROS/set_parameters", srv_req, srv_resp);

//  return true;
//}

gazebo_msgs::ModelState goal2gz(move_base_msgs::MoveBaseGoal goal)
{
  gazebo_msgs::ModelState gz;

  gz.pose.orientation.x = goal.target_pose.pose.position.x;
  gz.pose.orientation.y = goal.target_pose.pose.position.y;
  gz.pose.orientation.z = goal.target_pose.pose.position.z;
  gz.pose.orientation.w = goal.target_pose.pose.orientation.x;
  gz.pose.position.x =    goal.target_pose.pose.orientation.y;
  gz.pose.position.y =    goal.target_pose.pose.orientation.z;
  gz.pose.position.z =    goal.target_pose.pose.orientation.w;

  return gz;
}

gazebo_msgs::ModelState getRobotStart()
{
  std::string spawn_object = "race_pr2";
  //move object
  gazebo_msgs::ModelState move_object;
  move_object.model_name = spawn_object;
  move_object.pose.orientation.x = 0;
  move_object.pose.orientation.y = 0;
  move_object.pose.orientation.z = 1;
  move_object.pose.orientation.w = 0;
  move_object.pose.position.x    = 11.80;
  //move_object.pose.position.y    = 9.77;
  move_object.pose.position.y    = 11.77;
  move_object.pose.position.z    = 0.0;
  //simManipulator.moveObject(move_object);
  return move_object;
}

int main(int argc, char** argv)
{
  //tf::Transform pose;
  //gazebo_msgs::Transform pose;
  std::string pr2_gazebo_name = "race_pr2";
  std::vector<move_base_msgs::MoveBaseGoal> goals;
  //std::vector<float> vel_trans;

  ros::init(argc, argv, "scenario_onegoal");
  ros::NodeHandle nh;

  // Set Gazebo to RACE map transform
  SimManipulator simManipulator(nh,
    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-12.28, -10.20, 0.0)));

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(10.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Init goal
  initGoalsLinear(goals);

  while (ros::ok())
  {
    // Reset PR2 Gazebo pose
    simManipulator.moveObject(getRobotStart());

    // Update the AMCL localization with the pose from the simulation
    simManipulator.updateAmclFromSimulation(pr2_gazebo_name);

    driveTo(ac, goals);
    ros::spinOnce();
    ros::Duration(1,0).sleep();
  }

  return 0;
}
