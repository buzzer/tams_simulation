/*
 * 2013-06-26 Sebastian Rockel, Pascal Rost
 *
 * This ROS node is responsible for:
 * - starting up the RACE system (simulation, * blackboard, planner etc.)
 * - keeping track of the simulation run (processes, events, results)
 * - stopping cleanly all related processes
 *
 * Furthermore following features shall be provided:
 * - modifying simulation objects (spawn, move, delete) according to the static
 *   knowledge provided per (simulation) run
 *
 */

#include "sim_creator.h"

SimCreator::SimCreator (){}

SimCreator::~SimCreator (){}

void SimCreator::alterKnowledge()
{
  /* TODO: ros find pkg path
     the used path should be relative or direct of this...
     */
  ROS_WARN("Adjusting initial knowledge.");
  /*system("python ./tools/race_fluent_modification.py");
TODO: change object spawn positions (spawn_race_models.launch)
*/
  ROS_WARN("Adjusting object spawn positions.");
}

//FILE* SimCreator::startGazeboWorld ()
bool SimCreator::startGazeboWorld ()
{
  bool success = true;
  std::string ns_str;

  getUniqueNamespace(&ns_str);

  // Start empty RACE world
  std::string cmd_gazebo;
  cmd_gazebo.assign("roslaunch race_gazebo_worlds race_world.launch ns:=true namespace:=\"");
  cmd_gazebo.append( ns_str.c_str() );
  //cmd_gazebo.append("\" gui:=false furniture:=false &>> /dev/null");
  cmd_gazebo.append("\" gui:=true furniture:=false &>> /dev/null");

  ROS_DEBUG("Executing: %s", cmd_gazebo.c_str());

  //FILE *stream1 = popen(cmd_gazebo.c_str(), "r");
  success &= system(cmd_gazebo.c_str());

  //return stream1;
  return success;
}

// FIXME make me random accessable
void SimCreator::getUniqueNamespace(std::string * unique_namespace)
{
  uint32_t sim_count = namespaces.size();
  std::string ns;

  if (sim_count == 0)
  {
    // Empty namespace
    ns.assign("/");
  }
  else
  {
    ns.assign("/sim" + sim_count);
  }

  // Remember it
  namespaces.push_back(ns);
  // Return it
  unique_namespace->assign(ns);

  ROS_DEBUG("Simulation: created new namespace %s", ns.c_str());
}

// Returns the last namespace in the list
void SimCreator::getLastNamespace(std::string * cur_ns)
{
  ROS_ASSERT(cur_ns != NULL);

  if (getSimCount() > 0)
  {
    cur_ns->assign( namespaces.back() );
  }
  else
  {
    // Return NULL pointer if list is empty
    cur_ns = NULL;
  }
}

// delete namespace method
bool SimCreator::deleteNamespaceByName(std::string * name_ns)
{
  bool success = false;
  std::vector<std::string>::iterator it_ns;

  ROS_ASSERT( name_ns != NULL);

  if ( (it_ns = std::find(namespaces.begin(), namespaces.end(), *name_ns)) != namespaces.end() )
  {
      namespaces.erase(it_ns);
      success = true;
  }

  return success;
}

std::vector<std::string> SimCreator::getNamespaces()
{
  // Return a copy
  return namespaces;
}

//FILE* SimCreator::startRos()
//void SimCreator::startRos(FILE* pipe )
void SimCreator::startRos()
{
  ROS_DEBUG("SimCreator: Starting ROS.");
  //sim_count++;
  // output of the ROS/RACE system should 
  // be piped to some place usefull
  //std::string roscore_cmd = "roscore >> /dev/null";
  //std::string cmd2 = "roslaunch race_plan_executor smach_plan_scheduler.launch gui:=true sesame_remote:=false sesame_repo:=\"my_repo\"";
  //FILE *roscore_pipe = popen(roscore_cmd.c_str(), "r");
  //std::string cmd3 = "rosrun turtlesim turtlesim_node";
  //std::string gazebonamespace = "run";
  //std::string cmd3 = "roslaunch race_gazebo_worlds race_world.launch gui:=false furniture:=false";
  //FILE *stream2 = popen(cmd3.c_str(), "r");
  //sleep(5); // should take way longer for actual system
  //FILE *stream2 = startGazeboWorld();
  //pipe = startGazeboWorld();
  //startGazeboWorld();
  //ROS_WARN("Waiting for roscore to be started.");
  //return stream;
  //return stream2;
}

// TODO stop only the process requested
bool SimCreator::stopRos()
{
  bool success = true;

  // ROS is not alive anymore here
  success &= system("killall gzclient &>> /dev/null");

  success &= system("killall roslaunch &>> /dev/null");

  success &= system("rosnode kill -a &>> /dev/null");
  sleep(5);

  success &= system("killall gzserver &>> /dev/null");
  sleep(5);

  success &= system("killall roscore &>> /dev/null");

  return success;
}

/*
 * Function to check if all nodes are running.
 */
//TODO add smach_plan_scheduler
bool SimCreator::checkNodesRunningList(std::vector<std::string> & nodes)
{
  bool running = true;
  std::vector<std::string> needed_nodes = nodes; //nodes that need to be running
  std::vector<std::string> running_nodes; //nodes that are running
  std::string node;

  // check ROS itself
  if ( ! ros::master::check() )
  {
    printf("[ WARN]: Cannot reach rosmaster\n");
    running = false;
  }
  if ( ! ros::ok() )
  {
    printf("[ WARN]: ROS is not running\n");
    running = false;
  }

  /* TODO Check processes instead of nodes, e.g. gazebo node can still be
   * running when the 'gazebo' process crashed
   */
  // TODO add other nodes e.g. blackboard etc.

  ros::master::getNodes(running_nodes);

  for (uint32_t i = 0; i < needed_nodes.size(); i++)
  {
    node = needed_nodes[i];
    ROS_DEBUG("is node %s running? ", node.c_str());

    if (std::find(running_nodes.begin(), running_nodes.end(), node) != running_nodes.end())
    {
      ROS_DEBUG("node %s is running", node.c_str());
    }
    else
    {
      printf("[ WARN]: node %s is not running\n", node.c_str());
      running = false;
      break;
    }
  }
  return running;
}
/*
 * Function to check if all nodes are running.
 * FIXME: deprecated
 * use SimCreator::checkNodesRunningList instead
 */
//TODO add smach_plan_scheduler
bool SimCreator::checkNodesRunning(bool use_semantic_dispatcher)
{
  bool running = true;
  std::vector<std::string> needed_nodes; //nodes that need to be running
  std::vector<std::string> running_nodes; //nodes that are running
  std::string node;

  // check ROS itself
  if ( ! ros::master::check() )
  {
    printf("[ WARN]: Cannot reach rosmaster\n");
    running = false;
  }
  if ( ! ros::ok() )
  {
    printf("[ WARN]: ROS is not running\n");
    running = false;
  }

  /* TODO Check processes instead of nodes, e.g. gazebo node can still be
   * running when the 'gazebo' process crashed
   */
  needed_nodes.push_back("/gazebo");
  
  if (use_semantic_dispatcher){
    needed_nodes.push_back("/exmo/dispatcher_monitor");
  } else {
    needed_nodes.push_back("/plan_scheduler");  
  }
  //stefan
  needed_nodes.push_back("/blackboard_node");
  needed_nodes.push_back("/move_base_straight");
  // TODO add other nodes e.g. blackboard etc.

  ros::master::getNodes(running_nodes);

  for (uint32_t i = 0; i < needed_nodes.size(); i++)
  {
    node = needed_nodes[i];
    ROS_DEBUG("is node %s running? ", node.c_str());

    if (std::find(running_nodes.begin(), running_nodes.end(), node) != running_nodes.end())
    {
      ROS_DEBUG("node %s is running", node.c_str());
    }
    else
    {
      printf("[ WARN]: node %s is not running\n", node.c_str());
      running = false;
      break;
    }
  }
  return running;
}

uint32_t SimCreator::getSimCount()
{
  return namespaces.size();
}

#ifdef TEST_SIM_MANIPULATOR/*{{{*/
int main(int argc, char **argv)
{
  uint32_t simcount = 1;

  if (argc >= 2) { simcount = atoi(argv[1]); }

  ROS_WARN("Starting %d simulation(s) in a row", simcount);

  ros::init(argc, argv, "sim_creator");
  ros::NodeHandle nh_;

  SimCreator simcreator(nh_);

  // run N simulations
  for (uint32_t i = 0; i < simcount; ++i)
  {
    // alter the knowledge and object files
    simcreator.alterKnowledge();

    // start the ROS/RACE system
    FILE *test = simcreator.startRos();

    /* pretend this is a node
       listen on specific topics/use specific services to gain knowledge
       of current state of the RACE system
      also: initiate the execution of instructions
    */
    ros::Rate loop_rate(1); // 1Hz
    bool stop = false;

    while (ros::ok())
    {
      loop_rate.sleep();
      ros::spinOnce();

      // check if all key-nodes are running
      ROS_DEBUG("checking state...");
      if (!simcreator.checkNodesRunning())
      {
        ROS_WARN("Nodes not running..");
        stop = true;
      }

      // listen to specific topics (FAILURE SUCCESS)

      /* stop the system if something is broken,
         of if some criteria is met 
         (i.e. failure or success of specific instruction)
         */
      if (stop)
      {
        simcreator.stop_ros(test);
      }
      /* Here we can do some post-processing of collected data.
         That could be import/export of blackboard data etc.
         */
      //sleep(10); //TODO use ros::Duration(10).sleep()
    }
  }

return 0;
}
#endif // TEST_SIM_MANIPULATOR/*}}}*/
