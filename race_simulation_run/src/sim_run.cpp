/*
 * 2013-07-29 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 *
 * This script launches the RACE system in an automated, deterministic and ordered way.
 * It depends on having all RACE components properly installed. Furthermore it
 * depends on having the blackboard locally installed on the PC this is running
 * on.
 *
 * It runs for the given amount of cycles and executes the given task. For
 * convenience real-time information as well as summaries after each runs
 * related to Fluent processing and time used is given on the terminal.
 *
 */
#include "sim_manipulator.h"
#include "sim_creator.h"
#include "ros/package.h"
#include <stdio.h>
#include <boost/thread.hpp>
#include "race_msgs/GetNumberOfFluents.h"
#include "race_msgs/GetCurrentEpisode.h"
#include "race_msgs/GetEpisodes.h"

bool use_optirun = false;
const std::string gazebo_name = "PR21"; //TODO take name from Fluent
const uint32_t timer_seconds_elapse = 300; // scenario should be finished within 5 minutes
const std::string demo = "demo_311a";

std::vector<std::string> getRepositoryEpisodes(ros::NodeHandle & nh)
{
  ros::ServiceClient get_episodes;
  std::string current_episode_name = "";
  race_msgs::GetEpisodes get_episodes_msg;
  std::vector<std::string> episodes_names;

  if ( (get_episodes =
        nh.serviceClient<race_msgs::GetEpisodes>("/blackboard/get_episodes")).exists() )
  {

    if (get_episodes.call(get_episodes_msg))
    {
      if (get_episodes_msg.response.result.result_code == 0)
      {
        episodes_names = get_episodes_msg.response.episodes;
      }
      else
      {
        printf("[ERROR]: blackboard replied with error: %s\n",
            get_episodes_msg.response.result.error_message.c_str());
      }
    }
    else
    {
      printf("[ERROR]: Calling %s service failed\n", get_episodes.getService().c_str());
    }
  }
  else
  {
    printf("[WARN]: %s service does not exist\n",
        get_episodes.getService().c_str());
  }

  get_episodes.shutdown();
  return episodes_names;
}

std::string getCurrentEpisode(ros::NodeHandle & nh)
{
  ros::ServiceClient get_current_episode;
  std::string current_episode_name = "";
  race_msgs::GetCurrentEpisode current_episode_msg;

  if ( (get_current_episode =
    nh.serviceClient<race_msgs::GetCurrentEpisode>("/blackboard/get_current_episode")).exists() )
  {

    if (get_current_episode.call(current_episode_msg))
    {
      if (current_episode_msg.response.result.result_code == 0)
      {
        current_episode_name = current_episode_msg.response.episode;
      }
      else
      {
        printf("[WARN]: blackboard replied with error: %s\n",
            current_episode_msg.response.result.error_message.c_str());
      }
    }
    else
    {
      printf("[WARN]: Calling %s service failed\n", get_current_episode.getService().c_str());
    }
  }
  else
  {
    printf("[WARN]: %s service does not exist\n",
        get_current_episode.getService().c_str());
  }

  get_current_episode.shutdown();
  return current_episode_name;
}

int32_t getNumberFluents(ros::NodeHandle & nh)
{
  ros::ServiceClient get_number_fluents;
  race_msgs::GetNumberOfFluents num_fluents_msg;
  int32_t number_fluents = -1;

  if ( (get_number_fluents =
    nh.serviceClient<race_msgs::GetNumberOfFluents>("/blackboard/get_number_of_fluents")).exists() )
  {
    if (get_number_fluents.call(num_fluents_msg))
    {
      if (num_fluents_msg.response.result.result_code == 0) // success
      {
        number_fluents = num_fluents_msg.response.size;
      }
      else
      {
        printf("[WARN]: blackboard replied with error: %s\n",
          num_fluents_msg.response.result.error_message.c_str());
      }
    }
    else
    {
      printf("[WARN]: Calling %s service failed\n", get_number_fluents.getService().c_str());
    }
  }
  else
  {
    printf("[WARN]: %s service does not exist, cannot get number of fluents\n",
        get_number_fluents.getService().c_str());
  }

  get_number_fluents.shutdown();
  return number_fluents;
}

bool spawnObjects ( std::vector<FluentSimObject> & objects_vector, SimManipulator & sim_man)
{
  bool success = true;

  for(std::vector<FluentSimObject>::iterator sim_object = objects_vector.begin();
      sim_object != objects_vector.end();
      ++sim_object)
  {
    ROS_DEBUG("Iterator object name: %s", sim_object->name.c_str());
    success &= sim_man.spawnFromFluent(*sim_object);
  }

  return success;
}

int main(int argc, char **argv)
{
  uint32_t simcount = 1;
  bool success = true;
  std::vector<std::string> episodes_names;
  std::vector<uint32_t> fluents_per_episode;
  std::string repository_name = "sim_run1";
  //stefan
  //std::string repository_name = "7";
  std::vector<std::string> repo_episodes_names;
  uint32_t count_fluents = 0;

  if (argc >= 2) { simcount = atoi(argv[1]); }

  printf("[ INFO]: Starting %d simulation(s) in a row\n", simcount);

  // run N simulations
  for (uint32_t i = 0; i < simcount; ++i)
  {
    bool stop = false;
    std::stringstream model_file_content;
    SimCreator simCreator;
    std::vector<FluentSimObject> objects_vector;
    boost::thread_group group_th;
    volatile bool timer_elapsed = false;
    double scenario_time = 0.0;
    int32_t number_fluents = 0;
    std::string scenario_instruction;
    std::string current_episode_name;

    printf("[ INFO]: Loading simulation run %d..\n", i+1);

    // Init a ROS node otherwise any creation of a ROS node will fail..
    printf("[ INFO]: Starting ROS..\n");
    group_th.create_thread(boost::bind(&system, "roslaunch race_gazebo_worlds race_world.launch furniture:=false gui:=true &>> /dev/null"));
    sleep(5);

    /********************************
     * Initialization of a ROS node *
     ********************************/

    ros::init(argc, argv, "sim_run"+i);
    ros::NodeHandle nh;

    // Uses ROS nodehandle, thus has to be called after (!) ros::init
    SimManipulator sim_man(nh,
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-12.28, -10.20, 0.0)));

    if (ros::ok())
    {
      sim_man.setUpdateRate(1.0);
      sim_man.unpause(); // Start simulation time
    }

    // Read yaml model list
    printf("[ INFO]: generating data/output.yaml\n");
    if ( system("rosrun race_simulation_run race_fluent_modification.py") )
      printf("[ERROR]: YAML fluent modifications failed\n");

    //get all Fluent objects as vector
    ParserYaml::getObjectStructs(objects_vector);


    if (ros::ok())
    { // Spawn simulation objects
      if ( ! spawnObjects(objects_vector, sim_man) )
      {
        printf("[ERROR]: Spawn objects failed, aborting simulation run..\n");
        ros::shutdown();
      }
    }

    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Launching PR2 controllers..\n", ros::Time::now().toSec());
      // needs robot_description on param server thus has to be called after
      // completed spawning
      if (use_optirun) {
        group_th.create_thread(boost::bind(system, "optirun roslaunch race_gazebo_worlds pr2_bringup.launch &>> /dev/null"));
      } else {
        group_th.create_thread(boost::bind(system, "roslaunch race_gazebo_worlds pr2_bringup.launch &>> /dev/null"));
      }

      group_th.create_thread(boost::bind(system, "roslaunch pr2_controller_configuration_gazebo pr2_default_controllers.launch &>> /dev/null"));

      ROS_INFO("Requesting model poses..");
      sim_man.getModelPoses();

      sleep(10); // wait for PR2 controllers to come up properly
    }

    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Launching PR2 navigation..\n", ros::Time::now().toSec());
      group_th.create_thread(boost::bind(system, "roslaunch race_navigation pr2_nav_tutorial_pr2.launch &>> /dev/null"));
      sleep(10); // wait for navigation to come up
      // Update the AMCL localization with the pose from the simulation
      sim_man.updateAmclFromSimulation(gazebo_name);

      sim_man.setUpdateRate(0.2);
    }

    /******************************************************
     * Simulation/PR2 systems should be running from here *
     ******************************************************/
    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Creating/re-setting local repository '%s'..\n", ros::Time::now().toSec(), repository_name.c_str());
      group_th.create_thread(boost::bind(system, std::string("rosrun race_bb reposcript.py LOCAL '"+repository_name+"' &>> /dev/null").c_str()));
    }

    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Launching tabletop manipulation..\n", ros::Time::now().toSec());
      group_th.create_thread(boost::bind(system, "roslaunch race_tabletop_object_publisher race_pr2_tabletop_manipulation.launch sim:=true stereo:=false &>> /dev/null"));
      sleep(20); // wait for manipulation to come up
    }

    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Launching blackboard..\n", ros::Time::now().toSec());
      group_th.create_thread(boost::bind(system, "rosrun race_bb blackboard.py _topics_config:=`rospack find race_bringup`/config/bb_topics.yaml _sesame_server:='http://localhost:8080/openrdf-sesame/' _sesame_repo:='sim_run1' &>> /dev/null"));
      //stefan
//      group_th.create_thread(boost::bind(system, "rosrun race_bb blackboard.py _topics_config:=`rospack find race_bringup`/config/bb_topics.yaml _sesame_server:='http://localhost:8080/openrdf-sesame/' _sesame_repo:='"+repository_name+"' &>> /dev/null"));
      sleep(5);
    }

    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Launching proprioception monitors..\n", ros::Time::now().toSec());
      group_th.create_thread(boost::bind(system, "roslaunch race_proprioception monitors.launch &>> /dev/null"));
      sleep(10);
    }

    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Loading static knowledge..\n", ros::Time::now().toSec());
      std::string pkgPath = ros::package::getPath("race_simulation_run");
      std::string file = pkgPath + "/data/output.yaml";
      std::string call = "rosrun race_static_knowledge race_static_knowledge.py " + file + " &>> /dev/null";
      success &= system(call.c_str());
      sleep(5);
    }

    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Launching planner..\n", ros::Time::now().toSec());
      group_th.create_thread(boost::bind(system, "roslaunch race_shop2_planner race_shop2_planner.launch domain:=`rospack find race_shop2_planner`/domains/race_owl_domain.lisp &>> /dev/null"));
      sleep(5);
    }

    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Launching spatial reasoner..\n", ros::Time::now().toSec());
      group_th.create_thread(boost::bind(system, "roslaunch race_spatial_reasoner race_spatial_reasoner.launch &>> /dev/null"));
      sleep(10);
    }

    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Launching new episode..\n", ros::Time::now().toSec());
      group_th.create_thread(boost::bind(system, "rosrun race_episodes episode.py &>> /dev/null"));
      sleep(2);
    }

    //// start visualization tools TODO really needed?
    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Launching visualization tools..\n", ros::Time::now().toSec());
      group_th.create_thread(boost::bind(system, "rosrun race_visualization_tools area_marker_publisher.py &>> /dev/null"));
    }

    // TODO add laser filter
    // TODO rosrun node only
    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Launching move_base_straight node..\n", ros::Time::now().toSec());
      group_th.create_thread(boost::bind(system, "roslaunch move_base_straight move_base_straight.launch"));
    }

    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Launching carryarm server..\n", ros::Time::now().toSec());
      group_th.create_thread(boost::bind(system, "rosrun carryarm_actionlib carryarm_server &>> /dev/null"));
    }

    if (ros::ok())
    {
      printf("[ INFO] [%4.2f]: Launching world state tracker..\n", ros::Time::now().toSec());
      group_th.create_thread(boost::bind(system, "rosrun race_world_state_tracker world_state_tracker.py &>> /dev/null"));
      sleep(5);
    }
    //TODO add tray pick and place
    //TODO tray monitor
    //TODO palm tf listener

    if (ros::ok())
    {
      printf("[ WARN] [%4.2f]: Launching scheduler..\n", ros::Time::now().toSec());
      group_th.create_thread(boost::bind(system, "rosrun race_plan_executor smach_plan_scheduler.py _parallel:=true _debug:=-1 _advanced:=false"));
      sleep(10);
    }

    //TODO change demo to variable
    if (ros::ok())
    {
      printf("[ WARN] [%4.2f]: Loading scenario knowledge for serve a coffee..\n", ros::Time::now().toSec());
      success &= system("roslaunch race_bringup load_static_knowledge_scenario.launch demo_11_a:=true &>> /dev/null");
      //success &= system("roslaunch race_bringup scenario.launch demo_12_a:=true &>> /dev/null");
      sleep(2);
    }
    //TODO affordances?

    if (ros::ok())
    {
      printf("[ WARN] [%4.2f]: Launching scenario..\n", ros::Time::now().toSec());
      scenario_instruction = "achieve serve_coffee_to_guest_Task guest1";
      //scenario_instruction = "achieve clear_table_Task table1 counter1";
      group_th.create_thread(boost::bind(system, std::string("rosrun race_cli race_cli.py " + scenario_instruction + " &>> /dev/null").c_str()));
    }

    /* pretend this is a node
       listen on specific topics/use specific services to gain knowledge
       of current state of the RACE system
    */
    //ros::Rate loop_rate(1); // 1Hz

    if (ros::ok())
    {
      sim_man.setUpdateRate(1.0);

      // Wait until SIGINT signal
      ROS_INFO("[ INFO]: Wait to stop ROS nodes..");
    }

    /***************************************************
     * RACE plan execution should be running from here *
     ***************************************************/
    boost::mutex mutex;

    if (ros::ok())
    {
      // Create a timer to stop run after some time
      group_th.create_thread( [&timer_elapsed, timer_seconds_elapse, &mutex]()
      {
        uint32_t seconds = timer_seconds_elapse;
        uint32_t loop = 0;
        bool thread_safe_elapsed = timer_elapsed;

        printf("[ WARN] [%4.2f]: %d seconds timer armed..\n", ros::Time::now().toSec(), seconds );
        //
        // Use rostime here as the simulation speed varies on each system
        ros::Rate r(1); // 1 hz
        // Do not sleep here and keep an event loop to prevent a sleeping thread
        // causing a segmentation fault when application is interrupted
        //while(ros::ok() && loop < seconds) // Ctrl-C is catched here
        while(false == thread_safe_elapsed && loop < seconds) // Ctrl-C is catched here
        {
          ++loop;
          r.sleep();

          mutex.lock();
          thread_safe_elapsed = timer_elapsed;
          mutex.unlock();
        }

        if (false == thread_safe_elapsed) //if it really was the timer..
        {
          mutex.lock();
          timer_elapsed = true;
          printf("[ WARN] [%4.2f]: ..%d seconds timer hit\n", ros::Time::now().toSec(), seconds);
          mutex.unlock();
        }
      });
    }

    /**********************************
     * Pre-processing of scenario run *
     **********************************/
    if (ros::ok())
    {
      current_episode_name = getCurrentEpisode(nh).c_str();
      episodes_names.push_back(current_episode_name);
      printf("[ INFO]: Current episode is: %s\n", current_episode_name.c_str());
      if (i == 0) //only in the first run
      {
        repo_episodes_names = getRepositoryEpisodes(nh);
        printf("[ INFO]: %d episodes in repository %s: ",
            (int) repo_episodes_names.size(), repository_name.c_str());
        //for ( auto &i : repo_episodes_names) { printf("'%s' ", i.c_str()); }
        printf("\n");
      }
    }

    scenario_time = ros::Time::now().toSec();
    /*************************************************
     * This node's run loop. Do periodic tasks here. *
     *************************************************/
    bool thread_safe_elapsed = timer_elapsed;
    while (ros::ok() && thread_safe_elapsed == false)
    {
      /* suggestion:
         usage of 'Timer' (see http://www.ros.org/wiki/roscpp/Overview/Timers)
         taken from http://answers.ros.org/question/11887/significance-of-rosspinonce/
      */
      /* loop_rate.sleep(); does not work, because something is wrong with sim-time and ros-time stuff!
         to see for yourself, comment in the section before, start the sim_run node
         after the blocking of the process just press 'play' in the gazebo simulator
         the process then proceeds to run but blocks forever after gazebo is killed!
         according to some rosanswers posts, this is related to usage of the '--clock' parameter
      */
      ros::spinOnce();
      sleep(1);

      // check if all key-nodes are running
      std::vector<std::string> nodes;
      nodes.push_back("/gazebo");
      nodes.push_back("/blackboard_node");
      nodes.push_back("/move_base_straight");
      nodes.push_back("/exmo/dispatcher_monitor");
      nodes.push_back("/plan_scheduler");
      if ( false == SimCreator::checkNodesRunningList(nodes))
      {
        printf("[ WARN] [%4.2f]: At least one node is not running, stopping..\n", ros::Time::now().toSec());
        stop = true;
        // TODO re-run the scenario??
      }

      // listen to specific topics (FAILURE SUCCESS)

      /* stop the system if something is broken,
         of if some criteria is met
         (i.e. failure or success of specific instruction)
         */
      if (stop)
      {
        break;
      }
      /* Here we can do some post-processing of collected data.
         That could be import/export of blackboard data etc.
         */
      if ( getNumberFluents(nh) > number_fluents )
      {
        number_fluents = getNumberFluents(nh);
        fluents_per_episode.push_back(number_fluents);
        printf("[ INFO] [%4.2f]: Current number of fluents: %d\n",
          ros::Time::now().toSec(), number_fluents);
      }

      mutex.lock();
      thread_safe_elapsed = timer_elapsed;
      mutex.unlock();
    }
    scenario_time = ros::Time::now().toSec() - scenario_time;

    mutex.lock();
    timer_elapsed = true; // stop timer
    mutex.unlock();

    /***********************************
     * ROS is dead after this line (!) *
     ***********************************/
    printf("[ WARN]: Stopping ROS nodes..\n");
    simCreator.stopRos();
    group_th.interrupt_all();
    printf("[ INFO]: ..stopped\n");

    /***********************************
     * Post-processing of scenario run *
     ***********************************/
    if (scenario_time > 0.0)
    {
      printf("\n= RUN %d SUMMARY =\n", i+1 );
      printf("=[ INFO]: Scenario episode is: '%s'\n", current_episode_name.c_str());
      printf("=[ INFO]: Scenario instruction: '%s'\n", scenario_instruction.c_str());
      printf("=[ INFO]: Total (sim) time used: %4.2f seconds\n", scenario_time);
      printf("=[ INFO]: Total number of Fluents used: %d\n", number_fluents);
      printf("=[ INFO]: Average ratio of (Fluents/sec): %4.2f\n\n", number_fluents/scenario_time);
    }
  }
  /***********************************
   * Post-processing of test run *
   ***********************************/
  if (episodes_names.size() > 1 )
  {
    printf("= TEST SUMMARY =\n");
    printf("=[ INFO]: %d episodes created during test with names: ", (int) episodes_names.size());
    for ( auto &i : episodes_names) { printf("'%s' ", i.c_str()); } printf("\n");
    for ( auto &i : fluents_per_episode) { count_fluents += i; }
    printf("=[ INFO]: Average Fluents per episode: %4.2f\n", (float) count_fluents / fluents_per_episode.size());
    printf("=[ INFO]: %d episodes are now in repository '%s'\n\n",
        (int) (repo_episodes_names.size() + episodes_names.size()), repository_name.c_str());
  }

  return 0;
}
