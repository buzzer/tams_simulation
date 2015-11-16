#include "ros/ros.h"
#include "race_imagination/Imagine.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_wrapper_testclient");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<race_imagination::Imagine>("imagination_srv");
  race_imagination::Imagine srv_msg;

  std::vector<race_msgs::Atom> updated_fluents;
  race_msgs::Atom robotat;
  robotat.type = "RobotAt"; // to be tested?
  //robotat.type = "robotat"; // ko?
  robotat.args.push_back("premanipulationareaeastcounter1");
  updated_fluents.push_back(robotat);
  race_msgs::Atom hasleftposture;
  hasleftposture.type = "hasArmPosture";
  hasleftposture.args.push_back("leftArm1");
  hasleftposture.args.push_back("armTuckedPosture1");
  updated_fluents.push_back(hasleftposture);
  race_msgs::Atom leftposture;
  leftposture.type = "Instance";
  leftposture.args.push_back("ArmTuckedPosture");
  leftposture.args.push_back("armTuckedPosture1");
  updated_fluents.push_back(leftposture);
  race_msgs::Atom hasrightposture;
  hasrightposture.type = "hasArmPosture";
  hasrightposture.args.push_back("rightArm1");
  hasrightposture.args.push_back("armTuckedPosture2");
  updated_fluents.push_back(hasrightposture);
  race_msgs::Atom rightposture;
  rightposture.type = "Instance";
  rightposture.args.push_back("ArmTuckedPosture");
  rightposture.args.push_back("armTuckedPosture2");
  updated_fluents.push_back(rightposture);
  //race_msgs::Atom hastorsoposture;
  //hastorsoposture.type = "hasTorsoPosture";
  //hastorsoposture.args.push_back("torso1");
  //hastorsoposture.args.push_back("torsoMiddlePosture1");
  //updated_fluents.push_back(hastorsoposture);
  //race_msgs::Atom torsoposture;
  //torsoposture.type = "Instance";
  //torsoposture.args.push_back("TorsoMiddlePosture");
  //torsoposture.args.push_back("torsoMiddlePosture1");
  //updated_fluents.push_back(robotat);

  // Action
  srv_msg.request.action = "imagine";
  //srv_msg.request.action = "move_base_param";
  race_msgs::ParamList plist1;
  race_msgs::ParamList plist2;
  race_msgs::ParamList plist3;
  // Parameter
  plist1.param_list.push_back("!move_base_param");
  plist2.param_list.push_back("premanipulationareanorthtable1");
  plist3.param_list.push_back("slow");
  plist3.param_list.push_back("fast");
  srv_msg.request.parameter.push_back(plist1);
  srv_msg.request.parameter.push_back(plist2);
  srv_msg.request.parameter.push_back(plist3);
  // Fluents
  srv_msg.request.updated_fluents = updated_fluents;

  ROS_INFO("Test plan executor: Calling Imagination..");
  if (client.call(srv_msg))
  {
    ROS_INFO("Success");
  }
  else
  {
    ROS_ERROR("ERROR");
    return 1;
  }
  return 0;
}
