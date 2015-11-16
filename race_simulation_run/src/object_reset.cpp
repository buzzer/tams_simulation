/*
 * 2014-03-11 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 */
#include <ros/ros.h>
#include "sim_manipulator.h"
#include <std_msgs/Empty.h>
#include <robot_motion_param/MotionParam.h>
#include <robot_motion_param/GetMotionParam.h>

struct MotionParam
{
  float max_vel;
  float max_acc;
  float max_jerk;
};

const float topple_angle = 1.56; // radians, defines topple state
const float topple_angle_min = 0.5; // radians, defines min topple angle. will fall down from here..
const float topple_dev = 0.1; // meter, defines linear dev for topple state
const MotionParam param_fallback = {0.55,2.5,0}; // in case of lost values take these
const MotionParam param_stop = {0,0,0}; // set in case the robot shall stop
const ros::Duration stop_timer = ros::Duration(1.5); // how long should we wait
std::string object_name_gaz = "race_flowers1";

class ObjectReset
{
  protected:
    std::string spawn_object;
    std::string pr2_gazebo_name;
    ros::NodeHandle nh;
    ros::Publisher reset_pub;

    ros::ServiceClient set_par;
    ros::ServiceClient get_par;

    bool toppling; // object is to fall
    bool timer_armed;

  public:
    ObjectReset(ros::Publisher & pub,
        std::string obj="race_flowers1", std::string rob="race_pr2")
    {
      reset_pub = pub;
      spawn_object = obj;
      pr2_gazebo_name = rob;
      set_par = nh.serviceClient<robot_motion_param::MotionParam>("set_motion_param");
      get_par = nh.serviceClient<robot_motion_param::GetMotionParam>("get_motion_param");
      toppling = true;
      timer_armed = true;
    }

    void resetOnTray(std::string obj="race_flowers1", std::string rob="race_pr2")
    {
      // Pose of object relative to the robots pose
      tf::Transform robot_tray = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.28, 0.0, 0.35));

      pr2_gazebo_name = rob;
      spawn_object = obj;

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
    }

    bool getMotParam(MotionParam &param)
    {
      robot_motion_param::GetMotionParam mot_par;

      // call service
      if (get_par.call(mot_par))
        ROS_INFO("Got motion parameters: %4.2f, %4.2f, %4.2f",
            mot_par.response.max_vel,
            mot_par.response.max_acc,
            mot_par.response.max_jerk);
      else
        ROS_ERROR("Failed to call service for getting motion parameters");

      param.max_vel = mot_par.response.max_vel;
      param.max_acc = mot_par.response.max_acc;
      param.max_jerk = mot_par.response.max_jerk;

      return true;
    }

    bool setMotParam(const MotionParam &param)
    {
      robot_motion_param::MotionParam mot_par;

      mot_par.request.max_vel = param.max_vel;
      mot_par.request.max_acc = param.max_acc;
      mot_par.request.max_jerk = param.max_jerk;
      // call service
      if (set_par.call(mot_par))
        ROS_INFO("Set motion parameters to: %4.2f, %4.2f, %4.2f",
          mot_par.request.max_vel,
          mot_par.request.max_acc,
          mot_par.request.max_jerk);
      else
        ROS_ERROR("Failed to call service for setting motion parameters");

      return true;
    }
    /*
     * on update reset the object, when:
     * -angular deviation > than some angle (radians)
     * -linear deviation  > than some distance (m)
     *
     *  This function is called at around 20Hz!
     */
    void callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
      static MotionParam param_old = {0,0,0};
      ros::Time now = ros::Time::now();
      static ros::Time stop_finish = now;

      // starting toppling, Will fall down from here on
      if(msg->vector.x > topple_angle_min || msg->vector.y > topple_dev)
      {
        if(toppling == false && timer_armed == false)
          toppling = true;
      }

      // has fallen down here already
      if(msg->vector.x > topple_angle || msg->vector.y > topple_dev)
      {
        resetOnTray(spawn_object);
        reset_pub.publish(std_msgs::Empty());
      }
      if (toppling)
      {
         getMotParam(param_old);
         //setMotParam(param_stop);
         ROS_WARN("Not (re-)setting motion parameters to 0!!!");
         stop_finish = now + stop_timer;
         timer_armed = true;
         toppling = false;
      }
      if (stop_finish < now && timer_armed)
      {
        // in case 0.0 is set and the robot does not move anymore:
        if (param_old.max_vel > 0.0)
        {
          //setMotParam(param_old);
         ROS_WARN("Not (re-)setting motion parameters to normal values!!!");
        }
        else
        {
          //setMotParam(param_fallback);
          ROS_WARN("Motion parameters set to default, as velocity would be 0.0!");
          ROS_WARN("Not (re-)setting motion parameters to fall back values!!!");
        }

        timer_armed = false;
      }
    }
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "gazebo_object_reset");
  ros::NodeHandle n;

  ros::Publisher reset_pub = n.advertise<std_msgs::Empty>("obj_reset", 1);

  if (argc < 2)
  {
    ROS_WARN("Missing gazebo model! Setting default: %s",
        object_name_gaz.c_str() );
  }
  else
  {
    object_name_gaz = argv[1];
  }

  // Wait for PR2 torso up and tuck arms
  ros::Duration(10).sleep();

  ObjectReset object_reset(reset_pub, object_name_gaz);
  ros::Subscriber sub = n.subscribe("/object_monitor/obj_dev", 1000, &ObjectReset::callback, &object_reset);

  ros::spin();

  return 0;
}
