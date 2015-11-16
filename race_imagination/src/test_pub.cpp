#include <ros/ros.h>
#include <race_imagination/SimState.h>

int main( int argc, char** argv )
{
  std::string topic = "/sim_dispatcher/sim_state1";
  ros::init(argc, argv, "sim_pub_tester");
  ros::NodeHandle n;

  ros::Publisher sim_state_pub = n.advertise<race_imagination::SimState>(topic , 1000);
  ROS_INFO("Publishing on topic %s", topic.c_str());

  race_imagination::SimState msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/map";
  msg.action = "move_base_param";

  race_msgs::ParamList pl;
  race_msgs::ParamList pl2;
  pl.param_list.push_back("premanipulationareanorthtable1");
  pl2.param_list.push_back("slow");

  msg.parameter.push_back(pl);
  msg.parameter.push_back(pl2);

  while ( ros::ok())
  {
    sim_state_pub.publish(msg);

    ros::spinOnce();
    ros::Duration(2).sleep();
  }

  return 0;
}

