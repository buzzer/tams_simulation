/*
 * 2014-07-18 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 *
 * Provides the action server.
 * Publishes world states to client simulations, waits and collects the results
 */
#include <ros/ros.h>
#include <race_imagination/SimState.h>
#include <race_imagination/ImagineAction.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread.hpp>

const std::string peppermill_name = "race_peppermill1";
// add worst case timout for simulations
const ros::Duration sim_timeout(300.0);
const ros::Duration dummy_duration(30.0);
const float dummy_confidence = 1.0;
//TODO make dynamic:
const int max_clients = 3;
const ros::Duration reply_wait(3.0); // wait for reply from client
const float event_sleep = 0.1;

class SimDispatcher
{
  protected:

    ros::NodeHandle nh_;
    std::vector<ros::Publisher> sim_state_pubs;

    actionlib::SimpleActionServer<race_imagination::ImagineAction> as_;
    std::string action_name_;
    //race_imagination::ImagineResult result_;

    unsigned int max_sim_count;
    unsigned int sim_count;
    std::vector<bool> sims_finished;
    std::vector<bool> sims_initialized;
    std::vector<bool> sims_used;
    std::vector<float> sims_result;
    std::vector<ros::Duration> sims_dur;
    std::vector<ros::Time> sims_start;

    boost::mutex sim_mutex;
    std::vector<race_imagination::SimState> sim_states;

  public:

    SimDispatcher(std::string name, std::vector<ros::Publisher> &pubs ) :
      as_(nh_, name, boost::bind(&SimDispatcher::execute, this, _1), false),
      action_name_(name)
    {
      // TODO get from launchfile as argument
      max_sim_count = max_clients;
      sim_state_pubs = pubs;

      // initialize empty fields
      resetData();

      as_.start();
    }

    void resetData(void)
    {
      // initialize empty fields
      sim_count = 0;
      sims_finished.clear();
      sims_initialized.clear();
      sims_used.clear();
      sims_start.clear();
      sims_dur.clear();
      sims_result.clear();

      for (unsigned int i=0; i<max_sim_count; i++)
      {
        sims_finished.push_back(false);
        sims_initialized.push_back(false);
        sims_used.push_back(false);
        sims_start.push_back(ros::Time(0.0));
        sims_dur.push_back(ros::Duration(0.0));
        sims_result.push_back(false);
      }
    }

    unsigned int update_sims(void)
    {
      unsigned int active_sims = 0;

      ROS_ERROR("Sending sim request messages");
      // send empty messages to sims
      for (unsigned int i=0; i<sim_state_pubs.size(); i++)
      {
        // Set to uninitialized
        sims_initialized.at(i) = false;
        // Send empty message
        race_imagination::SimState msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "";
        ROS_WARN("Sending sim request message to sim %d", i);
        sim_state_pubs.at(i).publish(msg);
      }
      // wait for replies
      reply_wait.sleep();

      for (unsigned int i=0; i<sim_state_pubs.size(); i++)
      {
        if (sims_initialized.at(i) == true)
        {
          ROS_WARN("Sim %d active", i);
          active_sims++;
        }
        else
        {
          ROS_WARN("Sim %d NOT active", i);
        }
      }
      return active_sims;
    }

    bool processSimReply(const race_imagination::SimState::ConstPtr& msg, unsigned int id)
    {
      if (msg->header.frame_id.compare("") != 0)
      {
        sim_mutex.lock();
          sims_finished.at(id) = true;
          if (sim_states.size() > id)
            sim_states.at(id) = *msg;
          else
            sim_states.push_back(*msg);
        sim_mutex.unlock();

        // get time from simulation
        sims_dur.at(id) =  msg->header.stamp - sims_start.at(id);
        ROS_WARN("Got Sim '%d' finished event with parameter '%s', stamp '%4.2f's, duration: '%4.2f's and confidence %4.2f",
            id, msg->parameter.back().param_list.back().c_str(),
            msg->header.stamp.toSec(), sims_dur.at(id).toSec(),
            msg->confidence);

        //ROS_INFO("System time is '%4.2f's", ros::Time::now().toSec());
        ROS_ASSERT(sim_states.at(id).parameter.size() >= 1);

        sims_result.at(id) = msg->confidence;
      }
      else // Got a init message
      {
        sims_initialized.at(id) = true;
        // store time as start time. Get it from simulation
        sims_start.at(id) = msg->header.stamp;
        ROS_INFO("Sim '%d' init message received with stamp '%4.2f's",
            id, sims_start.at(id).toSec());
        //ROS_INFO("System time is '%4.2f's", ros::Time::now().toSec());
      }
      return true;
    }

    void callback1(const race_imagination::SimState::ConstPtr& msg)
    {
      processSimReply(msg, 0);
    }
    void callback2(const race_imagination::SimState::ConstPtr& msg)
    {
      processSimReply(msg, 1);
    }
    void callback3(const race_imagination::SimState::ConstPtr& msg)
    {
      processSimReply(msg, 2);
    }
    /*
     * Returns the best sim id of all finished sims
     * The best execution is calculated as follows:
     * results == confidences, duration
     * Currently only confidence is considered.
     */
    unsigned int bestSim (
        std::vector<float> & results,
        std::vector<ros::Duration> & sims_dur,
        std::vector<race_imagination::SimState> & sim_states)
    {
      unsigned int sim_id = 0;
      float best_result = 0.0;

      // check for the highest confidence
      for (unsigned int i=0; i<results.size(); i++)
      {
        if (results.at(i) > best_result)
        {
          best_result = results.at(i);
          sim_id = i;
        }
      }

      return sim_id;
    }

    void printParamList(const std::vector<race_msgs::ParamList> & p_list)
    {
      for (unsigned int i=0; i<p_list.size(); i++)
      {
        ROS_INFO("Param list of lists %d of %d:", (int)i+1, (int)p_list.size());
        for (unsigned int j=0; j<p_list.at(i).param_list.size(); j++)
        {
          ROS_INFO("  Param %d of %d:", (int)j+1, (int)p_list.at(i).param_list.size());
          ROS_INFO("    %s", p_list.at(i).param_list.at(j).c_str());
        }
      }
    }
    void printDurationList(const std::vector<race_msgs::DurationList> & d_list)
    {
      for (unsigned int i=0; i<d_list.size(); i++)
      {
        ROS_INFO("Duration list of lists %d of %d:", (int)i+1, (int)d_list.size());
        for (unsigned int j=0; j<d_list.at(i).dur_list.size(); j++)
        {
          ROS_INFO("  Duration %d of %d:", (int)j+1, (int)d_list.at(i).dur_list.size());
          ROS_INFO("    %4.2f", d_list.at(i).dur_list.at(j).data.toSec());
        }
      }
    }
    void printConfidenceList(const std::vector<race_msgs::ConfidenceList> & c_list)
    {
      for (unsigned int i=0; i<c_list.size(); i++)
      {
        ROS_INFO("Confidence list of lists %d of %d:", (int)i+1, (int)c_list.size());
        for (unsigned int j=0; j<c_list.at(i).conf_list.size(); j++)
        {
          ROS_INFO("  Confidence %d of %d:", (int)j+1, (int)c_list.at(i).conf_list.size());
          ROS_INFO("    %4.2f", c_list.at(i).conf_list.at(j));
        }
      }
    }

    unsigned int getSimUsedCount()
    {
      unsigned int count = 0;

      for (unsigned int i=0; i<sims_used.size(); i++)
        if (sims_used.at(i) == true)
          ++count;

      return count;
    }

    void execute(const race_imagination::ImagineGoalConstPtr& goal)
    {
      ROS_ASSERT(goal->parameter.size() >= 2);
      // Reset sim_states
      sim_states.clear();
      // Dispatch parameters
      // build a list of parameters to be simulated
      std::vector<std::string> param_l;

      ROS_WARN("sim_dispatcher: Imagination called with following parameters:");
      printParamList(goal->parameter);

      // take the last list
      // Here the variation values should be: slow, fast..
      for (unsigned int i=0; i<goal->parameter.back().param_list.size(); i++)
      {
        param_l.push_back(goal->parameter.back().param_list.at(i));
      }
      ROS_INFO("sim_dispatcher: got param list of %d", (int) param_l.size());

      // Update list of active simulations
      sim_count = update_sims();
      //TODO sim_count and sims_initialized are redundant!!
      ROS_ERROR("%d simulation(s) active", sim_count);
      // Publish world state and action on simulation topic(s)
      //publish only on available topics/sims
      for (unsigned int i=0; i<sim_count; i++)
      {
        // check if sim is available
        if (sims_initialized.at(i) == true)
        {
          // initialize an empty sim state
          sim_states.push_back(race_imagination::SimState());
          race_imagination::SimState msg;
          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = goal->world_state.header.frame_id;
          //ROS_INFO("Got frame id: %s", msg.header.frame_id.c_str());
          msg.action = goal->action;

          // take the first of the first two lists, thus loop until second last
          // element
          for (unsigned int i=0; i<(-1+goal->parameter.size()); i++)
            msg.parameter.push_back(goal->parameter.at(i));

          // add a last element for one parameter
          msg.parameter.push_back(race_msgs::ParamList());
          /*
           * seperate parameter per simu here
           * stop when there are no parameters anymore
           */
          if (param_l.size() > 0)
          {
            sims_used.at(i) = true;
            // add one parameter at the end of the list
            msg.parameter.back().param_list.push_back(param_l.back());
            // delete that parameter from the ones to be tested
            param_l.pop_back();

            msg.world_state = goal->world_state;
            msg.robot_state = goal->robot_state;

            //Store the start time
            ROS_WARN("Trigger sim '%d' on system time '%4.2f's with parameter '%s'",
                //i, ros::Time::now().toSec(), msg.parameter.back().param_list.back().c_str());
                i, sims_start.at(i).toSec(), msg.parameter.back().param_list.back().c_str());

            ROS_INFO("Publishing real goal..");
            sim_state_pubs.at(i).publish(msg);
            ROS_INFO("done. Waiting for result..");
          }
          else
          {
            ROS_WARN("More simulations active than parameters to evaluate. Ignoring simulation %d", i);
          }
        }
      }
      // TODO if still parameters are to be simulated do it in sequence
      if (param_l.size() > 0)
      {
        ROS_WARN("%d parameters still to be evaluated. Skipping:", (int) param_l.size());
        for (unsigned int i=0; i<param_l.size(); i++)
          ROS_WARN(" %s",param_l.at(i).c_str());
      }

      ROS_INFO("Timeout set to %4.2f sec", sim_timeout.toSec());
      ros::Time start = ros::Time::now();

      bool all_act_sim_fin = false;

      // ============== IDLE LOOP =======================
      ROS_WARN("Entering idle loop. Waiting for %d simulation(s) to finish..", getSimUsedCount());
      while(((ros::Time::now() - start) <= sim_timeout) && ros::ok())
      {
        all_act_sim_fin = true;
        //for (unsigned int i=0; i<sim_count; i++)
        for (unsigned int i=0; i < getSimUsedCount(); i++)
          all_act_sim_fin &= sims_finished.at(i);

        if (all_act_sim_fin == true)
          break;

        ros::Duration(event_sleep).sleep();
      }
      ROS_WARN("..finished");
      // ============== IDLE LOOP =======================

      race_imagination::ImagineResult result_;

      if (all_act_sim_fin == false)
      {
        ROS_ERROR("Timeout occured. Imagination interrupted. Aborting..");
        as_.setAborted();
      }
      else if ((all_act_sim_fin == true) && (sim_count > 0))
      {
        ROS_WARN("All active simulations finished. Processing results..");

        unsigned int sim_id = bestSim(sims_result, sims_dur, sim_states);

        ROS_INFO("Best simulation found with index: %d and results:", sim_id);
        printParamList(sim_states.at(sim_id).parameter);

        // return only values for best rated run
        // PARAMETER
        for (unsigned int i=0; i<sim_states.at(sim_id).parameter.size(); i++)
        {
          race_msgs::ParamList par_list;
          // only consider the first parameter of each list!
          ROS_ASSERT(sim_states.at(sim_id).parameter.at(i).param_list.size() >= 1);
          par_list.param_list.push_back(sim_states.at(sim_id).parameter.at(i).param_list.at(0));
          result_.parameter.push_back(par_list);
        }

        // DURATIONS
        race_msgs::DurationList dur_list1;
        std_msgs::Duration dur_list;
        dur_list.data = sims_dur.at(sim_id);
        dur_list1.dur_list.push_back(dur_list);
        // fill the list of lists
        for (unsigned int i=0; i<3; i++)
          result_.durations.push_back(dur_list1);

        // CONFIDENCES
        race_msgs::ConfidenceList con_list1;
        con_list1.conf_list.push_back(sim_states.at(sim_id).confidence);
        // fill the list of lists
        for (unsigned int i=0; i<3; i++)
          result_.confidences.push_back(con_list1);

        ROS_WARN("Returning actual results.");
        as_.setSucceeded(result_);

        ROS_WARN("Imagination finished with following results:");
        printParamList(result_.parameter);
        printDurationList(result_.durations);
        printConfidenceList(result_.confidences);
      }
      else
      {
        // return dummy result for testing execution ..
        ROS_ASSERT(goal->parameter.size() > 2);

        //prepare fake parameters
        race_msgs::ParamList p_list1;
        // the area, e.g. premanipulationareasouthtable1
        p_list1.param_list.push_back(goal->parameter.at(0).param_list.at(0));
        ROS_INFO("Param1: %s", goal->parameter.at(0).param_list.at(0).c_str());
        result_.parameter.push_back(p_list1);

        race_msgs::ParamList p_list2;
        // the motion parameter, i.e. slow
        p_list2.param_list.push_back(goal->parameter.at(1).param_list.at(0));
        ROS_INFO("Param2: %s", goal->parameter.at(1).param_list.at(0).c_str());
        result_.parameter.push_back(p_list2);

        race_msgs::ParamList p_list3;
        // the motion parameter, i.e. slow
        p_list3.param_list.push_back(goal->parameter.at(2).param_list.at(0));
        ROS_INFO("Param3: %s", goal->parameter.at(2).param_list.at(0).c_str());
        result_.parameter.push_back(p_list3);

        // add elements to result structure
        race_msgs::DurationList dur_list1;
        std_msgs::Duration dur;
        dur.data = dummy_duration;
        dur_list1.dur_list.push_back(dur);
        // fill the list of lists
        for (unsigned int i=0; i<3; i++)
          result_.durations.push_back(dur_list1);

        race_msgs::ConfidenceList con_list1;
        con_list1.conf_list.push_back(dummy_confidence);
        // fill the list of lists
        for (unsigned int i=0; i<3; i++)
          result_.confidences.push_back(con_list1);

        ROS_WARN("Returning DUMMY results.");
        as_.setSucceeded(result_);
      }

      // initialize empty fields
      resetData();
    }
};

int main( int argc, char** argv )
{
  //bool latch_on = true;
  bool latch_on = false;
  ros::init(argc, argv, "sim_dispatcher");
  ros::NodeHandle n;
  // List of all publishers
  std::vector<ros::Publisher> sim_state_pubs;
  // List of all subsribers
  std::vector<ros::Subscriber> sim_state_subs;
  // base topic name
  std::string topic_name_pub = ros::this_node::getName()+"/sim_state";
  std::string topic_name_sub = "/sim_client/sim_state";
  // Max amount of sim clients!!

  if (argc >= 3)
  {
    topic_name_pub = argv[1];
    topic_name_sub = argv[2];
  }

  //TODO create dynamic multiple topics
  //ros::Publisher sim_state_pub = n.advertise<race_imagination::SimState>(
  for (int i=0; i<max_clients; i++)
  {
    sim_state_pubs.push_back( n.advertise<race_imagination::SimState>( topic_name_pub+std::to_string(i+1), 1, latch_on));
  }

  SimDispatcher sim_dispatcher(ros::this_node::getName(), sim_state_pubs);
  //SimDispatcher sim_dispatcher(ros::this_node::getName());

  //TODO create dynamic multiple topics
  sim_state_subs.push_back( n.subscribe(topic_name_sub+std::to_string(1),
      10, &SimDispatcher::callback1, &sim_dispatcher));
  sim_state_subs.push_back( n.subscribe(topic_name_sub+std::to_string(2),
      10, &SimDispatcher::callback2, &sim_dispatcher));
  sim_state_subs.push_back( n.subscribe(topic_name_sub+std::to_string(3),
      10, &SimDispatcher::callback3, &sim_dispatcher));

  ROS_INFO("%s publishes on:", ros::this_node::getName().c_str());
  for (unsigned int i=0; i<sim_state_pubs.size(); i++)
    ROS_INFO("%s",sim_state_pubs.at(i).getTopic().c_str());
  ROS_INFO("%s subsribed to:", ros::this_node::getName().c_str());
  for (unsigned int i=0; i<sim_state_subs.size(); i++)
    ROS_INFO("%s",sim_state_subs.at(i).getTopic().c_str());

  ros::spin();

  return 0;
}
