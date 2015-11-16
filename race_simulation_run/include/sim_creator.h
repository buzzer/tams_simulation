#ifndef SIM_CREATOR_H
#define SIM_CREATOR_H

// ROS includes.
#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

class SimCreator
{
  protected:

    std::vector<std::string> namespaces;

    void getUniqueNamespace(std::string * unique_namespace);

    std::vector<std::string> getNamespaces ();

    void getLastNamespace(std::string * cur_ns);

    bool deleteNamespaceByName(std::string * name_ns);

  public:

    SimCreator ();

    ~SimCreator();

    void alterKnowledge();

    bool startGazeboWorld ()  __attribute__ ((deprecated));

    void startRos()  __attribute__ ((deprecated));

    bool stopRos();

    static bool checkNodesRunning(bool use_semantic_dispatcher = false) __attribute__ ((deprecated));

    static bool checkNodesRunningList(std::vector<std::string> & nodes);

    uint32_t getSimCount();
};

#endif // SIM_CREATOR_H
