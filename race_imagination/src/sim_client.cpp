/*
 * 2014-07-18 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 *
 * Listens to world state messages, sets up the simulation and executes the
 * action. Finally returns a result message.
 */
#include <ros/ros.h>
#include <race_imagination/SimState.h>
#include "sim_manipulator.h"
#include "parser_yaml.h" // FluentSimObject
#include <boost/thread.hpp>
#include "race_msgs/ExecuteTask.h"
#include "race_msgs/ExecuteTaskAction.h"
#include <actionlib/client/simple_action_client.h>
#include "tf/tf.h"

const std::string pr2_gazebo_name = "race_pr2";
struct ToppleState {
  unsigned int notopple;
  unsigned int semitopple;
  unsigned int topple;
  void reset (void) { notopple=semitopple=topple=0; }
};
const float topple_angle = 0.5; // radians
const float semitopple_angle = 0.01; // radians
const float event_sleep = 0.1; // event loop 1/f
const float max_conf = 1.0; // maximum confidence
const float sim_speed = 0.3; // empirical ideal speed for navigation
const ros::Duration action_wait(300); // wait for action to finish
const float c_t = 100; // coefficient for topple events
const float c_s = 10; // coefficient for shacking events
const float c_n = 1; // coefficient for no topple events

class SimClient
{
  protected:

    ros::NodeHandle nh_;
    std::vector<ros::Publisher> sim_state_pubs;
    ros::ServiceClient mdl_srv;
    bool start_execution; // action execution has started
    bool is_executing; // action is being executed right now
    boost::mutex ex_mtx;
    boost::mutex isex_mtx;
    boost::mutex ac_mtx;
    std::string cur_action;
    std::vector<std::string> cur_param;
    SimManipulator sim_man;
    bool init_phase;
    boost::mutex init_mtx;
    ToppleState topple_cnt;
    boost::mutex topple_mtx;
    boost::thread_group th_group;
    race_msgs::WorldState ws_;
    race_msgs::RobotState rs_;
    boost::mutex ws_mtx;
    boost::mutex rs_mtx;
    actionlib::SimpleActionClient<race_msgs::ExecuteTaskAction> *ac;

  public:

    SimClient(std::string name, std::vector<ros::Publisher> &pubs, ros::ServiceClient & srv,
        SimManipulator & simManipulator )
    {
      sim_state_pubs = pubs;
      mdl_srv = srv;
      sim_man = simManipulator;
      setStartExec(false);
      setIsExec(false);
      setInit(false);
      topple_mtx.lock();
        topple_cnt.reset(); // reset topple counter
      topple_mtx.unlock();
      ac = new actionlib::SimpleActionClient<race_msgs::ExecuteTaskAction>("execute_action", true);
      //actionlib::SimpleActionClient<race_msgs::ExecuteTaskAction> ac("execute_action", true);
    }
    ~SimClient()
    {
      free(ac);
    }
    /*
     * Translation between blackboard and Gazebo names
     * Blackboard --> Gazebo
     */
    std::string getModelName( const std::string & sym_name)
    {
      /* table to get the model name from the fluent name*/
      std::map<std::string, std::string> dictionary;

      //dictionary["counter1"] = "race_table_counter1";
      dictionary["counter1"] = "race_counter1";
      //dictionary["table1"] = "race_table_table1";
      //dictionary["table2"] = "race_table_table1";
      dictionary["table1"] = "race_table1";
      dictionary["table2"] = "race_table2";
      dictionary["table4"] = "race_table_140x70";
      dictionary["table6"] = "race_table_140x70";
      dictionary["table7"] = "race_table_140x70";
      dictionary["table8"] = "race_table_140x70";
      dictionary["board1"] = "race_board";
      dictionary["chair1"] = "race_chair1";
      dictionary["chair2"] = "race_chair1";
      dictionary["chair3"] = "race_chair1";
      dictionary["chair4"] = "race_chair1";
      dictionary["PR21"] = "race_pr2";
      //dictionary["mug1"] = "race_coffee_cup";
      //dictionary["mug2"] = "race_coffee_cup";
      dictionary["mug1"] = "coffee_cup1";
      dictionary["mug2"] = "coffee_cup2";
      //TODO hack for getting objects from household database spawn correctly
      dictionary["race_coffee_cup"] = "race_coffee_cup";
      dictionary["race_book_c"] = "race_book_c";
      dictionary["race_fork1"] = "race_fork1";
      dictionary["race_knife1"] = "race_knife1";
      dictionary["race_spoon1"] = "race_spoon1";
      dictionary["race_bowl"] = "race_bowl";
      dictionary["race_flowers"] = "race_flowers";
      dictionary["race_oilservers1"] = "race_oilservers1";
      dictionary["race_iphone5"] = "race_iphone5";
      dictionary["peppermill1"] = "race_peppermill1";

      return dictionary[sym_name.c_str()];
    }

    bool prepareSimulation(const race_msgs::WorldState & ws)
    {
      for (unsigned int i=0; i<ws.name.size(); i++)
      {
        // check for object already in simulation
        ROS_WARN("Checking model name: %s", getModelName(ws.name[i]).c_str());
        //ROS_ERROR("Checking model name: %s", ws.name[i].c_str());
        // if yes modify pose, if not spawn
        if (sim_man.modelExistsInSimulation(getModelName(ws.name[i])))
        {
          gazebo_msgs::ModelState ms;
          ms.model_name = getModelName(ws.name[i]);
          ms.pose = ws.pose[i];
          //modify pose
          sim_man.moveObject(ms);
        }
        else
        {
          FluentSimObject so;
          so.name = getModelName(ws.name[i]);
          so.pose_map.setOrigin(tf::Vector3(
                ws.pose[i].position.x,
                ws.pose[i].position.y,
                ws.pose[i].position.z));
          so.pose_map.setRotation(tf::Quaternion(
                ws.pose[i].orientation.x,
                ws.pose[i].orientation.y,
                ws.pose[i].orientation.z,
                ws.pose[i].orientation.w));
          //spawn
          sim_man.spawnFromFluent(so);
        }
      }

      return true;
    }

    /*
     * Calls the middle layer to execute a task
     * blocks until action (!) is finished
     */
    bool callService(race_msgs::ExecuteTask & request)
    {
      bool success = true;

      //TODO start in own thread
      if (mdl_srv.call(request)) // blocks until action (!) is finished
      {
        ROS_INFO("Execution successful with result: %s", request.response.result.c_str());
        success = true;
      }
      else
      {
        ROS_ERROR("Execution failed with result: %s", request.response.result.c_str());
        success = false;
      }

      return success;
    }

    bool prepareRobot(const race_msgs::RobotState & rs)
    {
      std::string torso_state;
      std::string head_state;
      std::string left_arm_state;
      std::string right_arm_state;

      ROS_INFO("Got robot state with pose:");
      for (unsigned int i=0; i<rs.name.size(); i++)
      {
        ROS_INFO("%s", rs.name[i].c_str());
        ROS_INFO("%s", rs.pose[i].c_str());

        if (rs.name[i].compare("torso_pose") == 0)
          torso_state = rs.pose[i];
        if (rs.name[i].compare("head") == 0)
          head_state = rs.pose[i];
        if (rs.name[i].compare("right_arm") == 0)
          right_arm_state = rs.pose[i];
        if (rs.name[i].compare("left_arm") == 0)
          left_arm_state = rs.pose[i];
      }
      ROS_INFO("Got torso state: %s", torso_state.c_str());
      ROS_INFO("Got head state: %s", head_state.c_str());
      ROS_INFO("Got right_arm state: %s", left_arm_state.c_str());
      ROS_INFO("Got left_arm state: %s", right_arm_state.c_str());

      // move torso
      if (torso_state.compare("TorsoUpPosture") == 0)
      {
        // move torso
        race_msgs::ExecuteTask srv_r;
        race_msgs::TaskAtom task;
        //middlelayer service call
        task.task = "move_torso";
        task.args = {"torsoupposture"};
        ROS_WARN("Execute move_torso up posture..");
        srv_r.request.task = task;

        callService(srv_r);
      }
      else if (torso_state.compare("TorsoDownPosture") == 0)
      {
        // move torso
        race_msgs::ExecuteTask srv_r;
        race_msgs::TaskAtom task;
        //middlelayer service call
        task.task = "move_torso";
        task.args = {"torsodownposture"};
        ROS_WARN("Execute down up posture..");
        srv_r.request.task = task;

        callService(srv_r);
      }
      else if (torso_state.compare("TorsoMiddlePosture") == 0)
      {
        // move torso
        race_msgs::ExecuteTask srv_r;
        race_msgs::TaskAtom task;
        //middlelayer service call
        task.task = "move_torso";
        task.args = {"torsomiddleposture"};
        ROS_WARN("Execute move_torso middle posture..");
        srv_r.request.task = task;

        callService(srv_r);
      }

      if (left_arm_state.compare("ArmTuckedPosture") == 0 &&
          right_arm_state.compare("ArmTuckedPosture") == 0)
      {
        // move arms
        race_msgs::ExecuteTask srv_r;
        race_msgs::TaskAtom task;
        //middlelayer service call
        task.task = "tuck_arms";
        task.args = {"armtuckedposture", "armtuckedposture"};
        ROS_WARN("Execute tuck_arms..");
        srv_r.request.task = task;

        callService(srv_r);
      }
      //TODO move head
      return true;
    }

    bool resetActionParams()
    {
      cur_action.clear();
      cur_param.clear();

      return true;
    }
    void setInit(bool init)
    {
      init_mtx.lock();
        init_phase = init;
      init_mtx.unlock();
    }
    bool isInInit(void)
    {
      bool value;
      init_mtx.lock();
        value = init_phase;
      init_mtx.unlock();
      return value;
    }
    void setIsExec(bool execute)
    {
      isex_mtx.lock();
        is_executing = execute;
      isex_mtx.unlock();
    }
    bool isExecuting(void)
    {
      bool value;
      isex_mtx.lock();
        value = is_executing;
      isex_mtx.unlock();
      return value;
    }
    void setStartExec(bool execute)
    {
      ex_mtx.lock();
        start_execution = execute;
      ex_mtx.unlock();
    }

    bool hasStartedExec()
    {
      bool value;
      ex_mtx.lock();
        value = start_execution;
      ex_mtx.unlock();
      return value;
    }

    std::string getCurAction()
    {
      std::string action;
      ac_mtx.lock();
      action = cur_action;
      ac_mtx.unlock();

      return action;
    }

    std::vector<std::string> getCurParams(void)
    {
      std::vector<std::string> param_list;

      ac_mtx.lock();
        param_list = cur_param;
      ac_mtx.unlock();

      return param_list;
    }

    /*
     * The returning confidence value is calculated as follows:
     * c = 1/ (c_topple * c_t + c_semi * c_s + c_no * c_n)
     */
    double getConfidence(void)
    {
      double confidence = 1.0;
      unsigned int event_cnt = 0;

      topple_mtx.lock();
        event_cnt = topple_cnt.topple+topple_cnt.semitopple+topple_cnt.notopple;
        confidence = 1 / (
           (topple_cnt.topple * c_t +
            topple_cnt.semitopple * c_s +
            topple_cnt.notopple * c_n)
           / event_cnt);
      topple_mtx.unlock();

      // normalize
      //if (confidence > max_conf)
        //confidence = max_conf;

      ROS_WARN("Got topple: %d, semitopple: %d, notopple: %d",
          topple_cnt.topple,
          topple_cnt.semitopple,
          topple_cnt.notopple);
      ROS_WARN("Execution confidence is %4.2f", confidence);

      return confidence;
    }
    void sendHello(void)
    {
      race_imagination::SimState msg;

      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "";
      sim_state_pubs.back().publish(msg);
      ROS_ERROR("Received init message. Sent empty response message.");
      setInit(false);
    }

    /*
     * This method starts task execution by calling the middlelayer and waits
     * upon completion.
     * I fills the result msg and publishes it.
     * Contents of it are the current world and robot state as well as the
     * parameters this simulation has been set up with.
     */
    void executeAction(void)
    {
      //actionlib::SimpleActionClient<race_msgs::ExecuteTaskAction> ac("execute_action", true);
      race_msgs::ExecuteTaskGoal ag; // action goal
      bool finished_within_time = false;
      race_imagination::SimState msg;
      race_msgs::TaskAtom task;
      //middlelayer service call
      // translate, see #1222: remove unnecessary 'imagine' task from beginning
      std::vector<std::string> param_list = getCurParams();
      std::string action = param_list.front();
      param_list.erase(param_list.begin());
      boost::replace_all(action, "!", "");

      //TODO put it in constructor
      // wait for action server
      ROS_INFO("Waiting for server..");
      ac->waitForServer();
      ROS_INFO("Connected to server");

      // set up the action
      ag.task.task = action;
      ag.task.args = param_list;

      ROS_INFO("Call middlelayer with task '%s' and args:", ag.task.task.c_str());
      for (unsigned int i=0; i<ag.task.args.size(); i++)
        ROS_INFO("'%s'", ag.task.args.at(i).c_str());

      //send init message to trigger accurate time measurement
      sendHello();

      // Send action goal
      ac->sendGoal(ag);

      finished_within_time = ac->waitForResult(action_wait);
      bool success = true;
      if (!finished_within_time)
      {
        ROS_ERROR("Timed out achieving action goal. Canceling..");
        ac->cancelGoal();
        ROS_ERROR("..Canceled");
      }
      else
      {
        actionlib::SimpleClientGoalState state = ac->getState();
        success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
        if(success)
          ROS_INFO("Action finished: %s",state.toString().c_str());
        else
          ROS_INFO("Action failed: %s",state.toString().c_str());
      }

      //pause simulation
      //sim_man.pause(); // Stop simulation time

      // query worldstate from simulation
      std::vector<gazebo_msgs::ModelState> models;
      models = sim_man.getModelPoses();

      // fill in the result msg structure
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "/map";
      msg.action = ag.task.task;
      // put the action as first argument
      race_msgs::ParamList p_list;
      p_list.param_list.push_back(ag.task.task);
      msg.parameter.push_back(p_list);
      // put all other arguments as parameter lists
      for (unsigned int i=0; i<ag.task.args.size(); i++)
      {
        race_msgs::ParamList p_list;
        p_list.param_list.push_back(ag.task.args.at(i));
        msg.parameter.push_back(p_list);
      }

      // Confidence is dependent on the action result
      if (success)
        msg.confidence = getConfidence();
      else
        msg.confidence = 0.0;

      // put the world state
      for (unsigned int i=0; i<models.size(); i++)
      {
        msg.world_state.name.push_back(models.at(i).model_name);
        msg.world_state.pose.push_back(models.at(i).pose);
      }

      sim_state_pubs.back().publish(msg);

      ROS_ERROR("Sent actual result message for '%s' with confidence '%4.2f'.",
          msg.parameter.back().param_list.front().c_str(), msg.confidence);

      setIsExec(false); // set not executing flag
    }

    /*
     * Resets the move_base_node collision map to prevent artefacts from
     * position changes of the robot
     */
    bool resetCollMap (void)
    {
      std_srvs::Empty clear_msg;
      ros::ServiceClient service_client;
      std::string sc_name = "/move_base_node/clear_costmaps";
      bool success = true;

      service_client = nh_.serviceClient<std_srvs::Empty>(sc_name);

      ROS_WARN("Calling service %s", service_client.getService().c_str());
      if (service_client.call(clear_msg))
      {
        ROS_INFO("%s service returned successfully", service_client.getService().c_str());
        success = true;
      }
      else
      {
        ROS_ERROR("%s service call failed", service_client.getService().c_str());
        success = false;
      }
      return success;
    }

    void setWorldState(const race_msgs::WorldState & ws)
    {
      ws_mtx.lock();
        ws_ = ws;
      ws_mtx.unlock();
    }
    race_msgs::WorldState getWorldState(void)
    {
      race_msgs::WorldState ws;
      ws_mtx.lock();
        ws = ws_;
      ws_mtx.unlock();
      return ws;
    }
    void setRobotState(const race_msgs::RobotState & rs)
    {
      rs_mtx.lock();
        rs_ = rs;
      rs_mtx.unlock();
    }
    race_msgs::RobotState getRobotState(void)
    {
      race_msgs::RobotState rs;
      rs_mtx.lock();
        rs = rs_;
      rs_mtx.unlock();
      return rs;
    }
    /*
     * Prepares simulation:
     * -environment (static)
     * -environment (dynamic)
     * -robot
     */
    void prepareSim(void)
    {
      ROS_INFO("Preparing simulation..");
      //sim_man.pause(); // Stop simulation time
      prepareSimulation(getWorldState());
      //sim_man.unpause(); // Start simulation time
      //resetCollMap();
      sim_man.updateAmclFromSimulation(pr2_gazebo_name);
      prepareRobot(getRobotState());
      //resetCollMap(); // TODO check if needed
      ROS_INFO("..preparing simulation done");
    }
    /*
     * This IF is called from the sim dispatcher.
     * The provided message contains the world and robot state as well as the
     * action parameters:
     * Expects a list of lists with only two lists and one element each
     *
     * A received empty msg will be replied with an empty message as well (see
     * protocol).
     *
     * This method is non-blocking.
     */
    void callback(const race_imagination::SimState::ConstPtr& msg)
    {
      // check for initialization msg
      if (msg->header.frame_id.compare("") == 0)
      {
        ROS_WARN("Initialization request received, sending empty response..");
        setInit(true);
      }
      //TODO prevent concurrency
      //
      //only execute when there is no current execution
      else if (hasStartedExec() == false)
      {

        printParamList(msg->parameter);
        setWorldState(msg->world_state);
        setRobotState(msg->robot_state);

        // should be 3 parameter lists here, check where it comes from
        ROS_ASSERT(msg->parameter.size() >= 3);
        ac_mtx.lock();
          cur_action = msg->action;
          // fill parameter list
          for (unsigned int i=0; i<msg->parameter.size(); i++)
            cur_param.push_back(msg->parameter.at(i).param_list.front());

          int j=0;
          for (auto i: cur_param)
          {
            ROS_INFO("callback parameter: %s", i.c_str());
            j++;
          }
          ROS_INFO("callback param count: %d", j);
        ac_mtx.unlock();

        // decouple callback from action execution
        setStartExec(true);
      }
    }
    void resetToppleCnt(void)
    {
      topple_mtx.lock();
        topple_cnt.reset(); // reset topple counter
      topple_mtx.unlock();
    }

    /*
     * Checks for the topple state
     */
    void cb_obj_dev(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
      //if (isExec() == true)
      //{
        topple_mtx.lock();
          if (msg->vector.x >= topple_angle)
            topple_cnt.topple++;
          else if (msg->vector.x >= semitopple_angle)
            topple_cnt.semitopple++;
          else
            topple_cnt.notopple++;
        topple_mtx.unlock();
      //}
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
};

void spinThread() { ros::spin(); }

int main( int argc, char** argv )
{
  //boost::thread ex_action;
  boost::thread_group group_th;
  //bool latch_on = true;
  bool latch_on = false;
  ros::init(argc, argv, "sim_client");
  ros::NodeHandle n;
  // List of all publishers
  std::vector<ros::Publisher> sim_state_pubs;
  // List of all subsribers
  std::vector<ros::Subscriber> sim_state_subs;
  ros::ServiceClient mdl_srv = n.serviceClient<race_msgs::ExecuteTask>("execute_task");
  // base topic name
  // Should be set unique per node in the launchfile!
  std::string topic_name_pub = "/sim_client/sim_state";
  std::string topic_name_sub = "/sim_dispatcher/sim_state";
  const std::string obj_dev_topic = "/object_monitor/obj_dev";
  int client_id = 1;
  // Max amount of sim clients!! TODO make dynamic
  int max_clients = 1;
  // Uses ROS nodehandle, thus has to be called after (!) ros::init
  SimManipulator simManipulator(n,
      //tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-12.28, -10.20, 0.0)));
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-12.20, -10.20, 0.0)));

  if (ros::ok())
  {
    //TODO only for move_base to be faster
    //simManipulator.setUpdateRate(0.3); // empirical ideal speed for navigation
    simManipulator.unpause(); // Start simulation time
  }

  if (argc >= 2)
  {
    client_id = atoi(argv[1]);
  }
  topic_name_sub += std::to_string(client_id);
  topic_name_pub += std::to_string(client_id);

  ROS_INFO("%s publishes on %s and listens on %s", ros::this_node::getName().c_str(),
      topic_name_pub.c_str(), topic_name_sub.c_str());

  for (int i=0; i<max_clients; i++)
  {
    sim_state_pubs.push_back( n.advertise<race_imagination::SimState>(
          topic_name_pub, 1, latch_on));
  }

  SimClient sim_client(ros::this_node::getName(), sim_state_pubs, mdl_srv, simManipulator);

  //TODO test if still working if topic is not present
  sim_state_subs.push_back( n.subscribe(topic_name_sub,
      10, &SimClient::callback, &sim_client));
  // Get notified on tray object deviation
  sim_state_subs.push_back( n.subscribe(obj_dev_topic,
      100, &SimClient::cb_obj_dev, &sim_client));

  // spin a background thread
  boost::thread spin_thread(&spinThread);

  // TODO add obj_reset subscriber

  ROS_INFO("Entering execution loop");
  while (ros::ok()) // execution loop
  {
    if (true == sim_client.isInInit())
    { // send init response
      sim_client.sendHello();
    }
    else if (true == sim_client.hasStartedExec())
    { // start execution
      sim_client.setStartExec(false);
      //simManipulator.setUpdateRate(1.0);
      sim_client.prepareSim(); // blocking
      //simManipulator.setUpdateRate(sim_speed); // empirical ideal speed for navigation
      sim_client.setIsExec(true);
      // start a new thread and return
      group_th.create_thread(boost::bind(&SimClient::executeAction, &sim_client));
    } // check for execution finish
    else if (false == sim_client.hasStartedExec() &&
             false == sim_client.isExecuting() ) // when finished
    {
      group_th.join_all();
      sim_client.resetActionParams();
      sim_client.resetToppleCnt();
    }
    ros::Duration(event_sleep).sleep();
  }

  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  return 0;
}
