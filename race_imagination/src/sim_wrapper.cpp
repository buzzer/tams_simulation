#include "ros/ros.h"
#include "race_imagination/Imagine.h"
#include "race_imagination/ImagineAction.h"
#include "race_imagination/AtomDbg.h"
#include "race_msgs/GenerateImagineState.h"
#include <actionlib/client/simple_action_client.h>
#include <boost/algorithm/string.hpp>

class SimWrapper
{
  protected:

    ros::NodeHandle nh_;
    ros::Publisher atom_pub;
    ros::Publisher ws_pub;
    ros::Publisher rs_pub;
    actionlib::SimpleActionClient<race_imagination::ImagineAction> *ac;
    ros::ServiceClient srv;

  public:

    SimWrapper(ros::Publisher & pub,
               ros::Publisher & ws_p,
               ros::Publisher & rs_p)
    {
      atom_pub = pub;
      ws_pub = ws_p;
      rs_pub = rs_p;
      ac = new actionlib::SimpleActionClient<race_imagination::ImagineAction>("sim_dispatcher", true);
      srv = nh_.serviceClient<race_msgs::GenerateImagineState>("generate_imagine_state");
    }

    ~SimWrapper(void) { free(ac); }

    bool callback(race_imagination::Imagine::Request  &req,
                  race_imagination::Imagine::Response &res)
    {
      bool success = true;
      race_imagination::ImagineGoal goal;
      race_msgs::GenerateImagineState srv_msg;
      race_imagination::AtomDbg msg;

      for (auto i : req.updated_fluents)
        msg.fluents.push_back(i);

      // publish debug fluents
      atom_pub.publish(msg);

      // call Translator
      srv_msg.request.updated_fluents = req.updated_fluents;

      ROS_INFO("Calling Translator..");
      if (srv.call(srv_msg))
      {
        ROS_INFO(" Successfully called service generate_imagine_state");
      }
      else
      {
        ROS_ERROR(" Failed to call service generate_imagine_state");
        return false;
      }
      ROS_INFO("Results will be published.");
      // publish debug world state msg
      ws_pub.publish(srv_msg.response.world_state);
      // publish debug robot state msg
      rs_pub.publish(srv_msg.response.robot_state);

      //call imagine action and block
      ROS_INFO("Waiting for imagination action server to start.");
      // wait for the action server to start
      ac->waitForServer(ros::Duration(10.0)); // will wait for
      ROS_INFO("Imagination action server started, sending goal.");

      // Prepare imagination action goal
      goal.action = req.action;
      goal.parameter = req.parameter;
      goal.world_state = srv_msg.response.world_state;
      goal.robot_state = srv_msg.response.robot_state;

      ac->sendGoal(goal);
      //bool finished_before_timeout = ac->waitForResult(ros::Duration(200.0));
      bool finished_before_timeout = ac->waitForResult();

      if (finished_before_timeout)
      {
        actionlib::SimpleClientGoalState state = ac->getState();
        ROS_INFO("Action finished with result: %s",state.toString().c_str());

        race_imagination::ImagineResult res_ac;
        res_ac = *ac->getResult();

        // return the results in the service
        res.parameter = res_ac.parameter;
        res.durations = res_ac.durations;
        res.confidences = res_ac.confidences;

        success = true;
      }
      else
      {
        ROS_WARN("Action did not finish before the time out.");
        success = false;
      }

      return success;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_wrapper");
  ros::NodeHandle n;

  ros::Publisher atom_pub = n.advertise<race_imagination::AtomDbg>("/imagination_wrapper/fluents", 1);
  ros::Publisher ws_pub = n.advertise<race_msgs::WorldState>("/imagination_wrapper/world_state", 1);
  ros::Publisher rs_pub = n.advertise<race_msgs::RobotState>("/imagination_wrapper/robot_state", 1);

  SimWrapper sw(atom_pub, ws_pub, rs_pub);

  ros::ServiceServer service = n.advertiseService("imagination_srv", &SimWrapper::callback, &sw);

  ROS_INFO("Imagination wrapper ready.");
  ros::spin();

  return 0;
}
