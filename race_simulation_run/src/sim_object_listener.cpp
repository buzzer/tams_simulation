#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <tf/tf.h>
#include "sim_manipulator.h"

class SimObjectListener
{
  protected:

    SimManipulator sim_manipulator;
    gazebo_msgs::ModelState pr2_transform;
    tf::Transform pr2_gz_tf;
    std::map<std::string, tf::Pose> spawned_models;

  public:
    SimObjectListener(std::string pr2_label)
    {
      pr2_transform.model_name = pr2_label;
      sim_manipulator.getModelPose(pr2_transform);
      pr2_gz_tf.setOrigin(tf::Vector3(
            pr2_transform.pose.position.x,
            pr2_transform.pose.position.y,
            pr2_transform.pose.position.z
            ));
      pr2_gz_tf.setRotation(tf::Quaternion(
          pr2_transform.pose.orientation.x,
          pr2_transform.pose.orientation.y,
          pr2_transform.pose.orientation.z,
          pr2_transform.pose.orientation.w
          ));

      ROS_INFO("%s at: [%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f]",
          pr2_transform.model_name.c_str(),
          pr2_transform.pose.position.x,
          pr2_transform.pose.position.y,
          pr2_transform.pose.position.z,
          pr2_transform.pose.orientation.x,
          pr2_transform.pose.orientation.y,
          pr2_transform.pose.orientation.z,
          pr2_transform.pose.orientation.w
      );
    }

    void objectsCallback(const visualization_msgs::Marker::ConstPtr& msg)
    {
      if (msg->color.r < 1.0 && msg->color.r > 0.0 && msg->id >= 0)
      {
        ROS_INFO("Got object: [%s] id: [%d] in frame: [%s] at: [%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f] with color: [%4.2f,%4.2f,%4.2f,%4.2f]",
            msg->text.c_str(),
            msg->id,
            msg->header.frame_id.c_str(),
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w,
            msg->color.r,
            msg->color.g,
            msg->color.b,
            msg->color.a
        );
        //TODO check for clusters (no pose given) and calculate pose from
        //point(s)
        std::string object_label = getModelName(msg->id);
        if (object_label.size() > 0)
        {
          //check for spawned models
          if (0 == spawned_models.count(object_label))
          {
            // spawn a new object
            FluentSimObject sim_object;
            sim_object.name = object_label;
            sim_object.pose_map.setOrigin(tf::Vector3(
                  msg->pose.position.x,
                  msg->pose.position.y,
                  msg->pose.position.z
                  ));
            sim_object.pose_map.setRotation(tf::Quaternion(
                  msg->pose.orientation.x,
                  msg->pose.orientation.y,
                  msg->pose.orientation.z,
                  msg->pose.orientation.w
                  ));
            sim_object.pose_map = sim_object.pose_map * pr2_gz_tf;
            auto it = spawned_models.end();
            sim_manipulator.spawnFromFluent(sim_object);
            spawned_models.insert(it, std::pair<std::string, tf::Pose >(object_label, sim_object.pose_map));
          }
          else
          {
            //TODO move object instead
          }
        }
      }
    }

    /*
     * Takes a model_id and returns a label.
     * model_id has to be consistent with the household database entities!
     * To verify this you can use 'pgadmin3'.
     */
    std::string getModelName(const uint32_t model_id)
    {
      /* table to get the model name from the fluent name*/
      std::map<uint32_t, std::string> dictionary;

      dictionary[2] = "race_coffee_cup";
      dictionary[5] = "race_book_c";
      dictionary[6] = "race_fork1";
      dictionary[7] = "race_knife1";
      dictionary[8] = "race_spoon1";
      dictionary[9] = "race_bowl";
      dictionary[10] = "race_flowers";
      dictionary[11] = "race_oilservers1";
      dictionary[12] = "race_iphone5";

      return dictionary[model_id];
    }
};

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "sim_object_listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  SimObjectListener sim_object_listener("race_pr2");

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub_recognition = n.subscribe("tabletop_object_recognition_markers",
      1000, &SimObjectListener::objectsCallback, &sim_object_listener);
  //ros::Subscriber sub_segmentation = n.subscribe("tabletop_segmentation_markers",
      //1000, &SimObjectListener::segmentsCallback, &sim_object_listener);
  ros::Subscriber sub_segmentation = n.subscribe("tabletop_segmentation_markers",
      1000, &SimObjectListener::objectsCallback, &sim_object_listener);


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
