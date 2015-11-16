/*
 * Pascal Rost, Sebastian Rockel
 *
 * Parse a YAML file and retrieve fluents object properties
 */
#include "parser_yaml.h"

std::string ParserYaml::getPoseName(const std::string & fluent_name, const std::string & file_path)
{
  std::ifstream fin(file_path.c_str());
  YAML::Parser parser(fin);
  YAML::Node doc;
  std::string pose_name;

  while(parser.GetNextDocument(doc))
  {
    std::vector<std::string> class_instance;
    doc["Class_Instance"] >> class_instance;
    if (class_instance[1] == fluent_name)
    {
      for(YAML::Iterator it=doc["Properties"].begin(); it!=doc["Properties"].end();++it)
      {
        std::vector<std::string> property;
        *it >> property;
        std::string filler_type(property[1]);

        if (filler_type == "Pose")
        {
          pose_name = property[2];
        }
      }
    }
  }
  return pose_name;
}

bool ParserYaml::getPoseVec(FluentSimObject & fluent_sim_obj, const std::string & file_path )
{
  bool success = true;

  std::ifstream fin(file_path.c_str());
  YAML::Parser parser(fin);
  YAML::Node doc;
  tf::Vector3 position(0,0,0);
  tf::Quaternion rotation(0,0,0,1);
  std::string pose_name;

  pose_name = getPoseName(fluent_sim_obj.name, file_path);

  while(parser.GetNextDocument(doc))
  {
    std::vector<std::string> class_instance;
    doc["Class_Instance"] >> class_instance;
    if (class_instance[1] == pose_name)
    {
      for(YAML::Iterator it=doc["Properties"].begin(); it!=doc["Properties"].end();++it)
      {
        std::vector<std::string> property;
        *it >> property;
        std::string role_filler(property[0]);

        if (role_filler == "hasX")
        {
          position.setX( boost::lexical_cast<float>(property[2]) );
        }
        else if (role_filler == "hasY")
        {
          position.setY( boost::lexical_cast<float>(property[2]) );
        }
        else if (role_filler == "hasZ")
        {
          position.setZ( boost::lexical_cast<float>(property[2]) );
        }
        else if (role_filler == "hasYaw")
        {
          // Ignoring Roll and Pitch angles ((!) TODO
          rotation = tf::createQuaternionFromRPY(0, 0, boost::lexical_cast<float>(property[2]));
        }

        fluent_sim_obj.pose_map.setOrigin(position);
        fluent_sim_obj.pose_map.setRotation(rotation);
      }
    }
  }

  //ROS_INFO("Pose name for fluent %s in %s is %s (%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f)",
  printf("[ INFO]: Pose name for fluent %s in %s is %s (%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f)\n",
      fluent_sim_obj.name.c_str(),
      //file_path.c_str(),
      "YAML file",
      pose_name.c_str(),
      fluent_sim_obj.pose_map.getOrigin().getX(),
      fluent_sim_obj.pose_map.getOrigin().getY(),
      fluent_sim_obj.pose_map.getOrigin().getZ(),
      fluent_sim_obj.pose_map.getRotation().getX(),
      fluent_sim_obj.pose_map.getRotation().getY(),
      fluent_sim_obj.pose_map.getRotation().getZ(),
      fluent_sim_obj.pose_map.getRotation().getW());

  return success;
}

std::vector<std::string> ParserYaml::getFluentNames(const std::string & file_path)
{
  std::ifstream fin(file_path.c_str());
  YAML::Parser parser(fin);
  YAML::Node doc;
  std::vector<std::string> fluent_names;

  while(parser.GetNextDocument(doc))
  {
    std::vector<std::string> class_instance;
    doc["Class_Instance"] >> class_instance;
    for(YAML::Iterator it=doc["Properties"].begin(); it!=doc["Properties"].end();++it)
    {
      std::vector<std::string> property;
      *it >> property;
      std::string filler_type(property[1]);

      if (filler_type == "Pose")
      {
        fluent_names.push_back(class_instance[1]);
      }
    }
  }
  return fluent_names;
}

std::string ParserYaml::getModelName(const std::string & fluent_name) {
  /* table to get the model name from the fluent name*/
  std::map<std::string, std::string> dictionary;

  dictionary["counter1"] = "race_table_counter1";
  dictionary["table1"] = "race_table_table1";
  dictionary["table2"] = "race_table_table1";
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
  dictionary["mug1"] = "race_coffee_cup";
  dictionary["mug2"] = "race_coffee_cup";
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

  return dictionary[fluent_name.c_str()];
}

/*
 * Returns a list of Fluent objects which have a simulation representation
 */
void ParserYaml::getObjectStructs(std::vector<FluentSimObject> & objects_vector)
{
  std::string fluentsPath;
  std::string outputPath;
  std::string package_path;
  std::vector<std::string> fluent_names;

  package_path = ros::package::getPath("race_simulation_run");

  ROS_ASSERT(&objects_vector != NULL && package_path.length() != 0);

  fluentsPath = package_path + "/data/output.yaml";
  outputPath = package_path + "/data/output.yaml";
  fluent_names = getFluentNames(fluentsPath);

  ROS_ASSERT(fluent_names.size() != 0);

  for(std::vector<std::string>::iterator fluent_name = fluent_names.begin(); fluent_name != fluent_names.end(); ++fluent_name)
  {
    FluentSimObject sim_object;

    sim_object.name = *fluent_name;

    //TODO workaround to not try to spawn objects w/o simulation representation
    // Add only objects which have a simulation representation!
    // Check that in the translation hash map..
    if ( (ParserYaml::getModelName(sim_object.name)).length() != 0)
    {
      getPoseVec(sim_object, outputPath);
      objects_vector.push_back(sim_object);
    }
  }
}

#if 0/*{{{*/
int main(int argc, char **argv)
{
  std::string pkgPath = ros::package::getPath("race_simulation_run");
  std::vector<float> pose_vector(getPoseVec("table1", pkgPath + "/src/test.yaml"));
  if (pose_vector.size() == 3) {
    std::cout << "x: " << pose_vector[0] << " y: " << pose_vector[1] 
	      << " z: " << pose_vector[2] << std::endl;
  }
  std::vector<std::string> fluent_names(getFluentNames(pkgPath + "/data/replace.yaml"));
  for(std::vector<std::string>::iterator it = fluent_names.begin(); 
      it != fluent_names.end(); ++it) {
    std::cout << *it << std::endl;
  } 


  return 0;
  }
#endif/*}}}*/
