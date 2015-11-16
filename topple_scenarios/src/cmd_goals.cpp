/*
 * 2014-04-14 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 * Validation of simulation: toppling repeatability.
 * Acceleration and stopping of the base to start toppling.
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "sim_manipulator.h"
#include <move_base_msgs/MoveBaseAction.h>

const std::string pr2_gazebo_name = "race_pr2"; // Gazebo robot name

class TwistCmd
{
  protected:

    ros::Publisher * twist_pub;

  public:

    TwistCmd(ros::Publisher * marker)
    {
      twist_pub = marker;
    }

    gazebo_msgs::ModelState getRobotStart()
    {
      //move object
      gazebo_msgs::ModelState move_object;
      move_object.model_name = pr2_gazebo_name;
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
    void set_twist(geometry_msgs::Twist & twist)
    {
      //twist.linear.x  = 0.55;
      //twist.linear.x  = 1.00;
      twist.linear.x  = 0.75;
      twist.linear.y  = 0.00;
      twist.linear.z  = 0.00;
      twist.angular.x = 0.00;
      twist.angular.y = 0.00;
      twist.angular.z = 0.00;
    }
    void stop_twist(geometry_msgs::Twist & twist)
    {
      twist.linear.x  = 0.00;
      twist.linear.y  = 0.00;
      twist.linear.z  = 0.00;
      twist.angular.x = 0.00;
      twist.angular.y = 0.00;
      twist.angular.z = 0.00;

    }
    bool driveTo( std::vector<move_base_msgs::MoveBaseGoal> & goals)
    {
      geometry_msgs::Twist twist;
      ros::Duration dur = ros::Duration(1.0);
      ros::Time now = ros::Time::now();
      ros::Time end = now + dur;

      set_twist(twist);

      //for (auto & goal : goals)
      //{
        while (now < end && ros::ok())
        {
          twist_pub->publish(twist);
          ros::Duration(0.001).sleep(); //1000 Hz
          now = ros::Time::now();
        }
        //stop_twist(twist);
        //twist_pub->publish(twist);
      //}

      return true;
    }
};

int main( int argc, char** argv )
{
  std::vector<move_base_msgs::MoveBaseGoal> goals;

  ros::init(argc, argv, "pr2_cmd_goals");
  ros::NodeHandle nh;

  // Set Gazebo to RACE map transform
  SimManipulator simManipulator(nh,
    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-12.28, -10.20, 0.0)));

  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/navigation/cmd_vel", 1);

  TwistCmd twist_cmd(&twist_pub);

  // Init goal
  twist_cmd.initGoalsLinear(goals);

  while (ros::ok())
  {
    // Reset PR2 Gazebo pose
    simManipulator.moveObject(twist_cmd.getRobotStart());

    // Update the AMCL localization with the pose from the simulation
    simManipulator.updateAmclFromSimulation(pr2_gazebo_name);

    twist_cmd.driveTo(goals);

    //driveTo(ac, goals);
    ros::spinOnce();
    ros::Duration(1,0).sleep();
  }

  return 0;
}
