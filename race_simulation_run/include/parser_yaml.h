#ifndef PARSER_YAML_H
#define PARSER_YAML_H

#include <fstream>
#include "yaml-cpp/yaml.h"
#include <vector>
#include <map>
#include <boost/lexical_cast.hpp>
#include "ros/package.h"
#include "ros/ros.h"
#include "tf/tf.h"

struct FluentSimObject
{
  std::string name; // Gazebo object label
  tf::Pose pose_map; // Position (origin, orientation) on the map
};

class ParserYaml
{
  protected:

    static std::string getPoseName(const std::string & fluentName, const std::string & filePath);

    static bool getPoseVec(FluentSimObject & fluentName, const std::string & filePath);

  public:

    static std::vector<std::string> getFluentNames(const std::string & filePath);

    static void getObjectStructs(std::vector<FluentSimObject> &);

    static std::string getModelName(const std::string & fluent_name);
};

#endif // PARSER_YAML_H
