/*
 * Copyright (c) 2013, Sebastian Rockel (rockel at informatik dot uni-hamburg dot de)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   - Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *   - Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * 2013-07-16 Sebastian Rockel
 *
 * Following features shall be provided:
 * - modifying simulation objects (spawn, move, delete)
 *
 */
#include "sim_manipulator.h"

SimManipulator::SimManipulator( void )
{
  //Translation from map to Gazebo coordinates
  map_2_gazebo.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  map_2_gazebo.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1));
}

SimManipulator::SimManipulator(const ros::NodeHandle & nh) : nh_(nh)
{
  //Translation from map to Gazebo coordinates
  map_2_gazebo.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  map_2_gazebo.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1));
}

SimManipulator::SimManipulator(const ros::NodeHandle & nh, const tf::Transform & map_gazebo_tf) :
  nh_(nh), map_2_gazebo(map_gazebo_tf) {}

SimManipulator::~SimManipulator(void) {}

tf::Transform SimManipulator::getMap2GazeboTransform() { return map_2_gazebo; }

bool SimManipulator::moveObject (const gazebo_msgs::ModelState & modelstate)
{
  bool success = true;
  gazebo_msgs::SetModelState setmodelstate_msg;
  tf::Pose pose_map;
  tf::Pose pose_gaz;
  ros::ServiceClient service_client;
  std::string sc_name = "set_model_state";

  ROS_ASSERT ( &modelstate != NULL );

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return (success = false);

  pose_map.setOrigin(tf::Vector3(
      modelstate.pose.position.x,
      modelstate.pose.position.y,
      modelstate.pose.position.z));
  pose_map.setRotation(tf::Quaternion(
        modelstate.pose.orientation.x,
        modelstate.pose.orientation.y,
        modelstate.pose.orientation.z,
        modelstate.pose.orientation.w));

  // coordinate transformation from map to gazebo
  pose_gaz = map_2_gazebo * pose_map;

  setmodelstate_msg.request.model_state.model_name = modelstate.model_name;
  setmodelstate_msg.request.model_state.pose.position.x = pose_gaz.getOrigin().getX();
  setmodelstate_msg.request.model_state.pose.position.y = pose_gaz.getOrigin().getY();
  setmodelstate_msg.request.model_state.pose.position.z = pose_gaz.getOrigin().getZ();
  setmodelstate_msg.request.model_state.pose.orientation.x = pose_gaz.getRotation().getX();
  setmodelstate_msg.request.model_state.pose.orientation.y = pose_gaz.getRotation().getY();
  setmodelstate_msg.request.model_state.pose.orientation.z = pose_gaz.getRotation().getZ();
  setmodelstate_msg.request.model_state.pose.orientation.w = pose_gaz.getRotation().getW();

  if (service_client.call(setmodelstate_msg))
  {
    ROS_DEBUG("model %s moved to (%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f)",
        modelstate.model_name.c_str(),
        modelstate.pose.position.x,
        modelstate.pose.position.y,
        modelstate.pose.position.z,
        modelstate.pose.orientation.x,
        modelstate.pose.orientation.y,
        modelstate.pose.orientation.z,
        modelstate.pose.orientation.w);

    success = true;
  }
  else
  {
    ROS_ERROR("Failed to move object %s: %s",
        setmodelstate_msg.request.model_state.model_name.c_str(),
        setmodelstate_msg.response.status_message.c_str());
    success = false;
  }

  return success;
}

/*
 * URDF spawn facility
 * takes Gazebo coordinates
 */
bool SimManipulator::spawnUrdfObject (gazebo_msgs::SpawnModel & spawnmodel_msg)
{
  bool success = true;
  ros::ServiceClient service_client;
  std::string sc_name = "spawn_urdf_model";

  ROS_ASSERT( &spawnmodel_msg != NULL);

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return (success = false);

  if (service_client.call(spawnmodel_msg))
  {
    if (spawnmodel_msg.response.success == true)
    {
      ROS_DEBUG("model %s (xml: %s,ns: %s) spawned to (%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f)",
          spawnmodel_msg.request.model_name.c_str(),
          spawnmodel_msg.request.model_xml.c_str(),
          spawnmodel_msg.request.robot_namespace.c_str(),
          spawnmodel_msg.request.initial_pose.position.x,
          spawnmodel_msg.request.initial_pose.position.y,
          spawnmodel_msg.request.initial_pose.position.z,
          spawnmodel_msg.request.initial_pose.orientation.x,
          spawnmodel_msg.request.initial_pose.orientation.y,
          spawnmodel_msg.request.initial_pose.orientation.z,
          spawnmodel_msg.request.initial_pose.orientation.w);

      success = true;
    }
    else
    {
      ROS_ERROR("Failed to spawn model %s: %s",
          spawnmodel_msg.request.model_name.c_str(),
          spawnmodel_msg.response.status_message.c_str()
          );
      success = false;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service SpawnModel: %s with model %s",
        spawnmodel_msg.response.status_message.c_str(),
        spawnmodel_msg.request.model_name.c_str() );

    success = false;
  }
  return success;
}

/*
 * wait for service
 * Equivalent for the ROS method but relies on real time
 */
bool SimManipulator::waitForService(ros::ServiceClient & serv_client, uint32_t wait_sec)
{
  bool exists = false;
  uint32_t timeout = 0;

  // relying on ros::Duration is not working as the simulation might be paused (!)
  while ( ! serv_client.exists() && timeout < wait_sec )
  {
    sleep(1);
    ++timeout;
  }
  if (timeout >= wait_sec)
  {
    ROS_WARN("Timeout while waiting for %s service", serv_client.getService().c_str());
    exists = false;
  }
  else
  {
    exists = true;
  }

  return exists;
}

/*
 * Spawn Gazebo objects
 * takes Gazebo coordinates!
 */
bool SimManipulator::spawnObject (gazebo_msgs::SpawnModel & spawnmodel_msg)
{
  bool success = true;
  ros::ServiceClient service_client;
  std::string sc_name = "spawn_gazebo_model";

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return (success = false);

  if (service_client.call(spawnmodel_msg) && &spawnmodel_msg != NULL)
  {
    if (spawnmodel_msg.response.success == true)
    {
      ROS_DEBUG("model %s (xml: %s,ns: %s) spawned to (%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f)",
          spawnmodel_msg.request.model_name.c_str(),
          spawnmodel_msg.request.model_xml.c_str(),
          spawnmodel_msg.request.robot_namespace.c_str(),
          spawnmodel_msg.request.initial_pose.position.x,
          spawnmodel_msg.request.initial_pose.position.y,
          spawnmodel_msg.request.initial_pose.position.z,
          spawnmodel_msg.request.initial_pose.orientation.x,
          spawnmodel_msg.request.initial_pose.orientation.y,
          spawnmodel_msg.request.initial_pose.orientation.z,
          spawnmodel_msg.request.initial_pose.orientation.w);

      success = true;
    }
    else
    {
      ROS_ERROR("Failed to spawn model %s: %s",
          spawnmodel_msg.request.model_name.c_str(),
          spawnmodel_msg.response.status_message.c_str()
          );
      success = false;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service SpawnModel: %s with model %s",
        spawnmodel_msg.response.status_message.c_str(),
        spawnmodel_msg.request.model_name.c_str() );

    success = false;
  }
  return success;
}

/*
 * Spawns objects from Fluents into Gazebo
 * Takes both URDF and Gazebo format
 * Relies on model names match file names!
 */
//TODO add xacro spawn facility
bool SimManipulator::spawnFromFluent(const FluentSimObject & object_from_fluent)
{
  bool success = true;
  bool is_urdf = false;
  gazebo_msgs::SpawnModel spawnmodel_msg;
  std::stringstream model_file_content;
  std::string model_filename;
  std::string rel_file_path_model;
  std::string rel_file_path_urdf;
  tf::Quaternion tf_quaternion;
  tf::Pose pose_gaz;

  ROS_ASSERT( &object_from_fluent != NULL);

  // Gazebo models are expected to be under race_gazebo_worlds/[models|urdf]/*.[model|urdf]
  spawnmodel_msg.request.model_name = object_from_fluent.name;
  model_filename = ParserYaml::getModelName(object_from_fluent.name);
  // when not in dictionary still tray plain filename
  if(model_filename.length() <= 0)
  {
    model_filename = spawnmodel_msg.request.model_name;
  }

  //ROS_ASSERT(model_filename.length() > 0);
  if ( ! model_filename.length() > 0)
  {
    ROS_WARN("Ignoring empty model name!");
    return false;
  }

  rel_file_path_model = "/models/" + model_filename + ".model";
  rel_file_path_urdf = "/urdf/" + model_filename + ".urdf";

  // Try Gazebo format
  if ( readModelFile(rel_file_path_model.c_str(), model_file_content) )
  {
    is_urdf = false;
  } // Try URDF format
  else if ( readModelFile(rel_file_path_urdf.c_str(), model_file_content) )
  {
    is_urdf = true;
    if (model_filename == "race_pr2") // FIXME make this less of a dirty hack
    {
    // put xml description onto param server, only needed for the robot
      nh_.setParam("robot_description", model_file_content.str());
    }
  }

  ROS_DEBUG("Modelfile content: %s", model_file_content.str().c_str());

  spawnmodel_msg.request.model_xml = model_file_content.str();
  //spawnmodel_msg.request.robot_namespace = "/";
  //spawnmodel_msg.request.reference_frame = "world";

  // coordinate transformation from map to gazebo
  pose_gaz = map_2_gazebo * object_from_fluent.pose_map;

  spawnmodel_msg.request.initial_pose.position.x = pose_gaz.getOrigin().getX();
  spawnmodel_msg.request.initial_pose.position.y = pose_gaz.getOrigin().getY();
  spawnmodel_msg.request.initial_pose.position.z = pose_gaz.getOrigin().getZ();
  spawnmodel_msg.request.initial_pose.orientation.x = pose_gaz.getRotation().getX();
  spawnmodel_msg.request.initial_pose.orientation.y = pose_gaz.getRotation().getY();
  spawnmodel_msg.request.initial_pose.orientation.z = pose_gaz.getRotation().getZ();
  spawnmodel_msg.request.initial_pose.orientation.w = pose_gaz.getRotation().getW();

  if (is_urdf == false)
    success &= spawnObject(spawnmodel_msg);
  else
    success &= spawnUrdfObject(spawnmodel_msg);

  if (success == true)
  {
    printf("[ INFO]: model %s (ns: %s) spawned to (%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f)\n",
        spawnmodel_msg.request.model_name.c_str(),
        spawnmodel_msg.request.robot_namespace.c_str(),
        // output map coordinates (!)
        object_from_fluent.pose_map.getOrigin().getX(),
        object_from_fluent.pose_map.getOrigin().getY(),
        object_from_fluent.pose_map.getOrigin().getZ(),
        spawnmodel_msg.request.initial_pose.orientation.x,
        spawnmodel_msg.request.initial_pose.orientation.y,
        spawnmodel_msg.request.initial_pose.orientation.z,
        spawnmodel_msg.request.initial_pose.orientation.w);
  }

  return success;
}

bool SimManipulator::deleteObject (const gazebo_msgs::ModelState & modelstate)
{
  bool success = true;
  gazebo_msgs::DeleteModel deletemodel_msg;
  ros::ServiceClient service_client;
  std::string sc_name = "delete_model";

  ROS_ASSERT( &modelstate != NULL );

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return (success = false);

  deletemodel_msg.request.model_name = modelstate.model_name;

  if (service_client.call(deletemodel_msg))
  {
    ROS_INFO("model %s deleted", modelstate.model_name.c_str());
    success = true;
  }
  else
  {
    ROS_ERROR("Failed to delete object: %s", deletemodel_msg.response.status_message.c_str());
    success = false;
  }

  return success;
}

bool SimManipulator::pause (void)
{
  bool success = true;
  std_srvs::Empty pause_msg;
  ros::ServiceClient service_client;
  std::string sc_name = "pause_physics";

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<std_srvs::Empty>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return (success = false);

  service_client.call(pause_msg) ? success = true : success =false;

  return success;
}

bool SimManipulator::unpause (void)
{
  bool success = true;
  std_srvs::Empty unpause_msg;
  ros::ServiceClient service_client;
  std::string sc_name = "unpause_physics";

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<std_srvs::Empty>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return (success = false);

  service_client.call(unpause_msg) ? success = true : success =false;

  return success;
}

bool SimManipulator::resetSimulation (void)
{
  bool success = true;
  std_srvs::Empty resetsimulation_msg;
  ros::ServiceClient service_client;
  std::string sc_name = "reset_simulation";

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<std_srvs::Empty>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return (success = false);

  service_client.call(resetsimulation_msg) ? success = true : success =false;

  return success;
}

bool SimManipulator::resetWorld (void)
{
  bool success = true;
  std_srvs::Empty resetworld_msg;
  ros::ServiceClient service_client;
  std::string sc_name = "reset_world";

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<std_srvs::Empty>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return (success = false);

  service_client.call(resetworld_msg) ? success = true : success =false;

  return success;
}

/*
 * returns vector of size == 0 on error
 */
std::vector<gazebo_msgs::ModelState> SimManipulator::getModelPoses ()
{
  gazebo_msgs::GetWorldProperties worldproperties;
  gazebo_msgs::GetModelState getmodelstate;
  gazebo_msgs::ModelState model_state;
  tf::Transform pose_map;
  tf::Transform pose_gaz;
  std::vector<gazebo_msgs::ModelState> model_states;
  ros::ServiceClient service_client;
  std::string sc_name = "get_world_properties";

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return model_states;

  //get world properties
  if (service_client.call(worldproperties))
  {
    for (uint32_t i=0; i<worldproperties.response.model_names.size(); i++)
    {
      //request pose per model name
      getmodelstate.request.model_name = worldproperties.response.model_names[i];
      getmodelstate.request.relative_entity_name = "world";

      model_state.model_name = worldproperties.response.model_names[i];
      getModelPose(model_state);

      model_states.push_back(model_state);

      //printf("[ DEBUG] [%4.2f]: Id %d, name %s, pose (%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f)\n",
      //    ros::Time::now().toSec(),
      //    i,
      //    model_state.model_name.c_str(),
      //    model_state.pose.position.x,
      //    model_state.pose.position.y,
      //    model_state.pose.position.z,
      //    model_state.pose.orientation.x,
      //    model_state.pose.orientation.y,
      //    model_state.pose.orientation.z,
      //    model_state.pose.orientation.w);
    }
  }
  else
  {
    ROS_ERROR("Failed to get model poses: %s", worldproperties.response.status_message.c_str());
  }
  return model_states;
}

/*
 * Reads a file given by the path specified and returns the file's content as
 * a stringstream.
 */
bool SimManipulator::readModelFile(const std::string & file_path, std::stringstream & file_content)
{
  bool success = true;
  std::ifstream readfile;

  ROS_ASSERT(&file_content != NULL);
  ROS_ASSERT(&file_path != NULL);

  //TODO get package name dynamically
  std::string absFilePath = ros::package::getPath("race_gazebo_worlds") + file_path;
  ROS_DEBUG("Model file path: %s", absFilePath.c_str());

  // Use binary mode as it is supposed to be faster
  readfile.open(absFilePath.c_str(), std::ifstream::binary);
  if ( readfile.fail() )
  {
    success = false;
    ROS_WARN("Failed opening the file %s", absFilePath.c_str());
  }
  else
  {
    success = true;
    file_content << readfile.rdbuf();
    readfile.close();
  }

  return success;
}
/*
 * Check a model name exists already in simulation
 */
bool SimManipulator::modelExistsInSimulation(const std::string & model_name)
{
  bool existence = true;
  gazebo_msgs::GetModelState getmodelstate;
  ros::ServiceClient service_client;
  std::string sc_name = "get_model_state";

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return (existence = false);

  //request pose per model name
  getmodelstate.request.model_name = model_name;
  getmodelstate.request.relative_entity_name = "world";
  if (service_client.call(getmodelstate))
  {
    ROS_INFO("Model %s exists in simulation: %s",
        model_name.c_str(),
        getmodelstate.response.status_message.c_str());
    existence = true;
  }
  else
  {
    ROS_INFO("Model %s does not exist in simulation: %s",
        model_name.c_str(),
        getmodelstate.response.status_message.c_str());
    existence = false;
  }

  return existence;
}

/*
 * Returns the model pose in map coordinates
 * if model not found or on error returns 'false'
 */
bool SimManipulator::getModelPose(gazebo_msgs::ModelState & pose)
{
  bool success = true;
  tf::Vector3 position;
  tf::Quaternion rotation;
  gazebo_msgs::GetModelState getmodelstate;
  tf::Transform pose_map;
  tf::Transform pose_gaz;
  ros::ServiceClient service_client;
  std::string sc_name = "get_model_state";

  ROS_ASSERT(&pose != NULL);

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return (success = false);

  //request pose per model name
  getmodelstate.request.model_name = pose.model_name.c_str();
  getmodelstate.request.relative_entity_name = "world";
  if (service_client.call(getmodelstate))
  {
    pose_gaz.setOrigin(tf::Vector3(
          getmodelstate.response.pose.position.x,
          getmodelstate.response.pose.position.y,
          getmodelstate.response.pose.position.z));
    pose_gaz.setRotation(tf::Quaternion(
          getmodelstate.response.pose.orientation.x,
          getmodelstate.response.pose.orientation.y,
          getmodelstate.response.pose.orientation.z,
          getmodelstate.response.pose.orientation.w));

    // coordinate transformation from gazebo to map
    pose_map = map_2_gazebo.inverse() * pose_gaz;

    pose.pose.position.x = pose_map.getOrigin().getX();
    pose.pose.position.y = pose_map.getOrigin().getY();
    pose.pose.position.z = pose_map.getOrigin().getZ();
    pose.pose.orientation.x = pose_map.getRotation().getX();
    pose.pose.orientation.y = pose_map.getRotation().getY();
    pose.pose.orientation.z = pose_map.getRotation().getZ();
    pose.pose.orientation.w = pose_map.getRotation().getW();

    ROS_DEBUG("Got model %s pose: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
        pose.model_name.c_str(),
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w );

    ROS_DEBUG("Model %s exists in simulation: %s",
        pose.model_name.c_str(),
        getmodelstate.response.status_message.c_str());
  }
  else
  {
    ROS_WARN("Model %s does not exist in simulation: %s",
        pose.model_name.c_str(),
        getmodelstate.response.status_message.c_str());
    success = false;
  }

  return success;
}

bool SimManipulator::setAmclPose(gazebo_msgs::ModelState & pose)
{
  bool success = true;
  ros::Time time_now;
  bool latch_on = true;

  //TODO consider namespace
  //send a latched message. will be delivered when subscriber subscribes, but only until the publisher exits.
  ros::Publisher pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, latch_on);
  geometry_msgs::PoseWithCovarianceStamped pose_stamped_msg;

  ROS_INFO("Setting AMCL pose for %s to (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)",
      pose.model_name.c_str(),
      pose.pose.position.x,
      pose.pose.position.y,
      pose.pose.position.z,
      pose.pose.orientation.x,
      pose.pose.orientation.y,
      pose.pose.orientation.z,
      pose.pose.orientation.w );

  // Get current time
  while ( time_now.toSec() == 0.0 )
  {
    ros::Duration(1.0).sleep();
    time_now = ros::Time::now();
  }
  pose_stamped_msg.header.seq = 0;
  pose_stamped_msg.header.stamp = time_now;
  pose_stamped_msg.header.frame_id = "/map";
  pose_stamped_msg.pose.pose.position.x =    pose.pose.position.x;
  pose_stamped_msg.pose.pose.position.y =    pose.pose.position.y;
  pose_stamped_msg.pose.pose.position.z =    pose.pose.position.z;
  pose_stamped_msg.pose.pose.orientation.x = pose.pose.orientation.x;
  pose_stamped_msg.pose.pose.orientation.y = pose.pose.orientation.y;
  pose_stamped_msg.pose.pose.orientation.z = pose.pose.orientation.z;
  pose_stamped_msg.pose.pose.orientation.w = pose.pose.orientation.w;
  // TODO check if index is right in amcl_node.xml
  pose_stamped_msg.pose.covariance[6*0+0] = 0.0675 * 0.0675; // X²
  pose_stamped_msg.pose.covariance[6*1+1] = 0.0675 * 0.0675; // Y²
  pose_stamped_msg.pose.covariance[6*3+3] =  M_PI/12.0 * M_PI/12.0; //a²
  //pose_stamped_msg.pose.covariance[6*0+0] = 1.0 * 1.0; // X²
  //pose_stamped_msg.pose.covariance[6*1+1] = 1.0 * 1.0; // Y²
  //pose_stamped_msg.pose.covariance[6*3+3] =  M_PI/2 * M_PI/2; //a²

  ROS_DEBUG("Got time %.2f", pose_stamped_msg.header.stamp.toSec());

  if (pub)
  {
    int numSub = 0;
    uint32_t max_tries = 30;
    uint32_t cur_try = 0;

    // we need to wait for a subscriber before we can exit, otherwise the topic will be closed (even if it's latched)
    while ( numSub == 0 && cur_try < max_tries)
    {
      ROS_INFO("Waiting for subscribers to /initialpose..");
      ros::Duration(1.0).sleep(); // wait for the subscriber
      numSub = pub.getNumSubscribers();
      ++cur_try;
    }
    if ( numSub > 0 )
    {
      ROS_DEBUG("Got %d subscribers to /initialpose, publishing..", numSub);
      pub.publish(pose_stamped_msg);
      ros::spinOnce();
      // needed in order to be sure to actually have sent the message before we close the publisher
      ros::Duration(1.0).sleep();
    }
    else
    {
      ROS_ERROR("Timed out while waiting for AMCL /initialpose topic!");
      success = false;
    }
  }
  else
  {
    ROS_ERROR("Failed to create a publisher for AMCL pose");
    success = false;
  }

  return success;
}

// model name as parameter
bool SimManipulator::updateAmclFromSimulation (const std::string & gazebo_name)
{
  bool success = true;
  gazebo_msgs::ModelState pr2_pose;

  pr2_pose.model_name = gazebo_name;

  // get PR2 pose from simulation
  if ( getModelPose(pr2_pose) )
  {
    ROS_DEBUG("Got model %s pose: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
        pr2_pose.model_name.c_str(),
        pr2_pose.pose.position.x,
        pr2_pose.pose.position.y,
        pr2_pose.pose.position.z,
        pr2_pose.pose.orientation.x,
        pr2_pose.pose.orientation.y,
        pr2_pose.pose.orientation.z,
        pr2_pose.pose.orientation.w);

    setAmclPose(pr2_pose);
  }

  return success;
}

gazebo_msgs::GetPhysicsProperties SimManipulator::getPhysicsProperties (void)
{
  gazebo_msgs::GetPhysicsProperties physics_props;
  ros::ServiceClient service_client;
  std::string sc_name = "get_physics_properties";

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return physics_props;

  if (service_client.call(physics_props))
  {
    ROS_DEBUG("%s service returned successfully", service_client.getService().c_str());
  }
  else
  {
    ROS_ERROR("%s service call failed: %s", service_client.getService().c_str(),
        physics_props.response.status_message.c_str());
  }

  return physics_props;
}

/*
 * update_rate is in (0.0 .. 1.0 .. 2.0 ..)
 * 0 means as fast as possible (hardware dependend)
 */
bool SimManipulator::setUpdateRate (const float & update_rate)
{
  bool success = true;
  gazebo_msgs::SetPhysicsProperties physics_props_set;
  gazebo_msgs::GetPhysicsProperties physics_props_get;
  ros::ServiceClient service_client;
  std::string sc_name = "set_physics_properties";

  ROS_ASSERT(update_rate >= 0.0);

  if ( 0 == service_clients.count(sc_name) )
    service_clients[sc_name] = nh_.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/"+sc_name);

  service_client = service_clients[sc_name];

  if ( ! waitForService(service_client, 10) )
      return (success = false);

  // Query current properties
  physics_props_get = getPhysicsProperties();
  // Copy current values
  physics_props_set.request.time_step = physics_props_get.response.time_step;
  physics_props_set.request.max_update_rate = (1/physics_props_get.response.time_step) * update_rate;
  physics_props_set.request.gravity = physics_props_get.response.gravity;
  physics_props_set.request.ode_config = physics_props_get.response.ode_config;

  if (service_client.call(physics_props_set) )
  {
    ROS_DEBUG("%s service returned successfully", service_client.getService().c_str());
  }
  else
  {
    ROS_ERROR("%s service call failed: %s", service_client.getService().c_str(),
        physics_props_set.response.status_message.c_str());
    success = false;
  }

  return success;
}

#ifdef TEST_SIM_MANIPULATOR/*{{{*/
int main(int argc, char **argv)
{
  // TODO Read yaml model list
  // TODO read yaml models in yaml file
  // TODO spawn models
  ros::init(argc, argv, "sim_manipulator");
  ros::NodeHandle nh;

  gazebo_msgs::ModelState modelstate;

  SimManipulator simManipulator(nh);
  //SimManipulator simManipulator;

  simManipulator.unpause();
  ros::Duration(1.0).sleep();

#if 0/*{{{*/
  modelstate.model_name = "coffee_cup1";
  modelstate.pose.position.x = -7.10;
  modelstate.pose.position.y =  0.20;
  modelstate.pose.position.z = 0.76;
  simManipulator.moveObject(&modelstate);

  ros::Duration(1.0).sleep();
  modelstate.pose.position.x = -6.91;
  modelstate.pose.position.y =  0.20;
  //modelstate.pose.position.z =  0.71;
  modelstate.pose.position.z =  0.76;
  simManipulator.moveObject(&modelstate);

  ros::Duration(1.0).sleep();
  modelstate.pose.orientation.x = 0.00;
  modelstate.pose.orientation.y = 0.00;
  modelstate.pose.orientation.z = 0.79;
  simManipulator.moveObject(&modelstate);
  modelstate.model_name = "coffee_cup2";
  modelstate.pose.position.x = -7.10;
  modelstate.pose.position.y =  -0.20;
  modelstate.pose.position.z = 0.76;
  simManipulator.moveObject(&modelstate);


  ros::Duration(1.0).sleep();
  modelstate.model_name = "pr2";
  modelstate.pose.position.x = -6.00;
  modelstate.pose.position.y =  0.00;
  modelstate.pose.position.z =  0.00;
  simManipulator.moveObject(&modelstate);

  ros::Duration(1.0).sleep();
  modelstate.pose.position.y =  0.40;
  simManipulator.moveObject(&modelstate);

  ros::Duration(0.5).sleep();
  modelstate.pose.position.y =  -0.40;
  simManipulator.moveObject(&modelstate);

  ros::Duration(0.5).sleep();
  modelstate.pose.position.x =  -6.00;
  modelstate.pose.position.y =  0.00;
  modelstate.pose.position.z =  0.00;
  simManipulator.moveObject(&modelstate);

  ros::Duration(0.5).sleep();
  modelstate.pose.position.x =  -6.20;
  modelstate.pose.position.y =  0.00;
  simManipulator.moveObject(&modelstate);

  ros::Duration(0.5).sleep();
  modelstate.pose.position.x =  -5.80;
  modelstate.pose.position.y =  0.00;
  simManipulator.moveObject(&modelstate);

  ros::Duration(0.5).sleep();
  modelstate.pose.orientation.z = 0.0;
  simManipulator.moveObject(&modelstate);

  ros::Duration(1.0).sleep();
  modelstate.pose.position.x =  -5.00;
  modelstate.pose.position.z =  1.00;
  //modelstate.pose.orientation.x = -1.57;
  simManipulator.moveObject(&modelstate);

  ros::Duration(1.0).sleep();
  modelstate.pose.position.x =  -5.00;
  modelstate.pose.position.z =  2.00;
  modelstate.pose.orientation.x = 3.14;
  simManipulator.moveObject(&modelstate);
#endif/*}}}*/

  simManipulator.getModelPoses();

  //gazebo_msgs::SpawnModel * spawnmodel = new gazebo_msgs::SpawnModel();
  //spawnmodel->request.model_name = "race_book3_b";
  //std::stringstream model_file_content;
  //simManipulator.readModelFile("/homeL/rockell/ros/race/race_pr2/race_gazebo_worlds/models/race_book1.model", &model_file_content);
  //ROS_INFO("Modelfile content: %s", model_file_content.str().c_str());
  //spawnmodel->request.model_xml = model_file_content.str();
  //spawnmodel->request.robot_namespace = "/";
  ////spawnmodel->request.reference_frame = "world";
  //spawnmodel->request.initial_pose.position.x = 0.00;
  //spawnmodel->request.initial_pose.position.y = 0.00;
  //spawnmodel->request.initial_pose.position.z = 0.70;
  //spawnmodel->request.initial_pose.orientation.x = 0;
  //spawnmodel->request.initial_pose.orientation.y = 0;
  //spawnmodel->request.initial_pose.orientation.z = 0;
  //spawnmodel->request.initial_pose.orientation.w = 1;
  //simManipulator.spawnObject(spawnmodel);

  //ros::Duration(1.0).sleep();

  gazebo_msgs::SpawnModel * spawnurdfmodel = new gazebo_msgs::SpawnModel();
  spawnurdfmodel->request.model_name = "race_mug1";
  std::stringstream model_urdf_file_content;
  simManipulator.readModelFile("/homeL/rockell/ros/race/race_pr2/race_gazebo_worlds/urdf/race_coffee_cup.urdf", &model_urdf_file_content);
  //ROS_INFO("Modelfile content: %s", model_urdf_file_content.str().c_str());
  spawnurdfmodel->request.model_xml = model_urdf_file_content.str();
  spawnurdfmodel->request.robot_namespace = "/";
  //spawnurdfmodel->request.reference_frame = "world";
  spawnurdfmodel->request.initial_pose.position.x = 0.00;
  spawnurdfmodel->request.initial_pose.position.y = 0.00;
  spawnurdfmodel->request.initial_pose.position.z = 0.70;
  spawnurdfmodel->request.initial_pose.orientation.x = 0;
  spawnurdfmodel->request.initial_pose.orientation.y = 0;
  spawnurdfmodel->request.initial_pose.orientation.z = 0;
  spawnurdfmodel->request.initial_pose.orientation.w = 1;
  simManipulator.spawnUrdfObject(spawnurdfmodel);

  ros::Duration(1.0).sleep();

  simManipulator.modelExistsInSimulation("notAModel");

  ros::Duration(1.0).sleep();

  simManipulator.modelExistsInSimulation("race_table1");

  ros::Duration(1.0).sleep();

  simManipulator.pause();
  simManipulator.resetWorld();
  simManipulator.unpause();

  gazebo_msgs::SpawnModel * spawn_pr2 = new gazebo_msgs::SpawnModel();
  spawn_pr2->request.model_name = "race_pr3_5";
  std::stringstream pr2_file_content;
  simManipulator.readModelFile(
     //"/homeL/rockell/ros/race/race_pr2/race_pr2_description/robots/race_pr2.urdf.xacro",
     //"/homeL/rockell/ros/race/race_pr2/race_pr2_description/robots/race_pr2.urdf",
     "/urdf/race_pr2.urdf",
     &pr2_file_content);
  //ROS_INFO("Modelfile content: %s", pr2_file_content.str().c_str());
  spawn_pr2->request.model_xml = pr2_file_content.str();
  spawn_pr2->request.robot_namespace = "/";
  //spawn_pr2->request.reference_frame = "world";
  spawn_pr2->request.initial_pose.position.x = 0.00;
  spawn_pr2->request.initial_pose.position.y = 0.00;
  spawn_pr2->request.initial_pose.position.z = 0.70;
  spawn_pr2->request.initial_pose.orientation.x = 0;
  spawn_pr2->request.initial_pose.orientation.y = 0;
  spawn_pr2->request.initial_pose.orientation.z = 0;
  spawn_pr2->request.initial_pose.orientation.w = 1;
  //simManipulator.spawnUrdfObject(spawn_pr2);

  simManipulator.spawnPR2(spawn_pr2);

#if 0/*{{{*/
  ros::Duration(1.0).sleep();

  modelstate.model_name = "race_table1";
  //modelstate.pose.position.x = -4.55;
  //modelstate.pose.position.y =  1.29;
  //modelstate.pose.position.z =  0.001;
  modelstate.pose.position.x = -4.55;
  modelstate.pose.position.y =  0.29;
  modelstate.pose.position.z =  0.001;
  simManipulator.moveObject(&modelstate);

  ros::Duration(1.0).sleep();

  simManipulator.pause();
  simManipulator.resetSimulation();
  simManipulator.unpause();

  ros::Duration(1.0).sleep();

  modelstate.model_name = "race_table1";
  modelstate.pose.position.x = -4.55;
  modelstate.pose.position.y =  1.29;
  modelstate.pose.position.z =  0.001;
  simManipulator.moveObject(&modelstate);

  ros::Duration(1.0).sleep();

  simManipulator.pause();
  simManipulator.resetWorld();
  simManipulator.unpause();

#endif/*}}}*/
  simManipulator.pause();

  //TODO spawn person
//  ros::spin();

  return 0;
}
#endif // TEST_SIM_MANIPULATOR[>}}}<]
