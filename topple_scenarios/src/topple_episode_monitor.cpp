// 2014-07-08 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
//
// This node listens to robot and object dynamics and deviation topics and
// agregates the information to publish it an Episode-like format
//
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <race_pr2_markers/BaseDynamics.h>
#include <topple_scenarios/ToppleEpisodePart.h>
#include <boost/thread.hpp>

const float event_sleep = 0.1; // sample frequency 1/Hz
const float topple_angle = 0.5; // radians
const float semitopple_angle = 0.01; // radians

const float vel_min = -1.0;
const float acc_min = -5.0;
const float jerk_min = -15.0; //TODO check
const float vel_max = vel_min * -1.0;
const float acc_max = acc_min * -1.0;
const float jerk_max= jerk_min* -1.0;

class ToppleEpisodeMon
{
  protected:

    ros::Publisher * dev_pub;
    boost::mutex base_dyn_mx;
    boost::mutex dev_mx;
    boost::mutex obj_dyn_mx;
    boost::mutex ep_pt_mx;
    boost::mutex base_ctl_mx;
    // data per episode interval
    race_pr2_markers::BaseDynamics base_dyn;
    geometry_msgs::Vector3Stamped obj_dev;
    race_pr2_markers::BaseDynamics obj_dyn;
    topple_scenarios::ToppleEpisodePart episode_part;
    race_pr2_markers::BaseDynamics base_ctl;

    std::string topple_state;

  public:

    ToppleEpisodeMon( ros::Publisher * dev )
    {
      dev_pub = dev;
    }

    void cb_base_dyn(const race_pr2_markers::BaseDynamics::ConstPtr& msg)
    {
      base_dyn_mx.lock();

      // Bias for higher acceleration
      //if (std::abs(msg->linear.acc) > std::abs(base_dyn.linear.acc))
      base_dyn = *msg;

      base_dyn_mx.unlock();
    }

    /*
     * Gets the low level linear/angular values sent to the base_contorller
     */
    void cb_base_ctl (const geometry_msgs::Twist::ConstPtr& msg)
    {
      ros::Time now = ros::Time::now();
      static ros::Time last_time = now;
      ros::Duration duration(1.0);
        (now == last_time) ? duration : duration = now - last_time;
      //ros::Duration duration = now - last_time;
      static float vel_l_0 = 0.0;
      static float vel_a_0 = 0.0;
      static float acc_l_0 = 0.0;
      static float acc_a_0 = 0.0;
      float vel_l = sqrt(msg->linear.x*msg->linear.x + msg->linear.y*msg->linear.y);
      float acc_l = (vel_l - vel_l_0)/duration.toSec();
      float jerk_l= (acc_l - acc_l_0)/duration.toSec();
      float vel_a = msg->angular.z;
      float acc_a = (vel_a - vel_a_0)/duration.toSec();
      float jerk_a= (acc_a - acc_a_0)/duration.toSec();

      // check for bounds
      if (vel_l <  vel_min  || vel_l > vel_max) {vel_l = vel_l_0;}
      if (acc_l <  acc_min  || acc_l > acc_max) {acc_l = acc_l_0;}
      if (jerk_l < jerk_min || jerk_l > jerk_max) {jerk_l = 0.0;}
      if (vel_a <  vel_min  || vel_a > vel_max) {vel_a = vel_a_0;}
      if (acc_a <  acc_min  || acc_a > acc_max) {acc_a = acc_a_0;}
      if (jerk_a < jerk_min || jerk_a > jerk_max) {jerk_a = 0.0;}

      base_ctl_mx.lock();
      // Bias for higher acceleration TODO not correct yet
      //if (std::abs(msg->linear.acc) > std::abs(base_dyn.linear.acc))
      base_ctl.header.stamp = now;
      base_ctl.header.frame_id = "/map";
      /* sign gets lost here! */
      base_ctl.linear.vel  = vel_l;
      base_ctl.linear.acc  = acc_l;
      base_ctl.linear.jerk = jerk_l;
      base_ctl.angular.vel = vel_a;
      base_ctl.angular.acc = acc_a;
      base_ctl.angular.jerk= jerk_a;

      base_ctl_mx.unlock();

      vel_l_0 = vel_l;
      vel_a_0 = vel_a;
      acc_l_0 = acc_l;
      acc_a_0 = acc_a;
      last_time = now;
    }

    /*
     * Checks for the topple state
     */
    void cb_obj_dev(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
      dev_mx.lock();

      // Bias for the higher deviation value
      //if (std::abs(msg->vector.x) > std::abs(obj_dev.vector.x))
      obj_dev = *msg;

      dev_mx.unlock();
    }

    void cb_obj_dyn(const race_pr2_markers::BaseDynamics::ConstPtr& msg)
    {
      obj_dyn_mx.lock();

      // Bias for higher acceleration
      //if ( std::abs(msg->linear.acc) > std::abs(obj_dyn.linear.acc))
      obj_dyn = *msg;

      obj_dyn_mx.unlock();
    }

    /*
     * TODO not completely done yet
     */
    void resetValues()
    {
      base_dyn_mx.lock(); base_dyn.linear.acc = 0.0; base_dyn_mx.unlock();
      dev_mx.lock();      obj_dev.vector.x = 0.0;    dev_mx.unlock();
      obj_dyn_mx.lock();  obj_dyn.linear.acc = 0.0;  obj_dyn_mx.unlock();
      base_ctl_mx.lock(); base_ctl.linear.acc = 0.0;   base_ctl_mx.lock();
    }

    /*
     * An episode consists of multiple parts of equal time intervals. It ends
     * when there was an interval with a 'topple' event.
     */
    void mergeData ( void )
    {
      static unsigned int counter = 0;
      static bool was_topple = false;
      topple_scenarios::ToppleEpisodePart episode_part_tmp;
      // Bias for the most recent timestamp is to be set
      // FIXME make protect by mutex
      //episode_part_tmp.header.stamp = std::max(
        //std::max(obj_dev.header.stamp, base_dyn.header.stamp), obj_dyn.header.stamp
      //);
      episode_part_tmp.header.stamp = ros::Time::now();
      // TODO write time interval in message

      episode_part_tmp.EpisodeID = counter;

      // Topple state set in obj_dev callback
      // check for topple state
      if (obj_dev.vector.x >= topple_angle &&
          was_topple == false)
      {
        topple_state = "topple"; //notopple, semitopple, topple
        counter++; // Start a new episode
        was_topple = true;
      }
      else if ( obj_dev.vector.x < topple_angle &&
          obj_dev.vector.x >= semitopple_angle )
      {
        topple_state = "semitopple"; //notopple, semitopple, topple
        was_topple = false;
      }
      else if ( obj_dev.vector.x < semitopple_angle )
      {
        topple_state = "notopple"; //notopple, semitopple, topple
        was_topple = false;
      }

      episode_part_tmp.state = topple_state;

      base_dyn_mx.lock(); episode_part_tmp.base_dyn = base_dyn; base_dyn_mx.unlock();
      dev_mx.lock();      episode_part_tmp.obj_dev = obj_dev;   dev_mx.unlock();
      obj_dyn_mx.lock();  episode_part_tmp.obj_dyn = obj_dyn;   obj_dyn_mx.unlock();
      base_ctl_mx.lock(); episode_part_tmp.base_ctl = base_ctl; base_ctl_mx.unlock();

      ep_pt_mx.lock();
      episode_part = episode_part_tmp;
      ep_pt_mx.unlock();
    }

    void publishData( void )
    {
      ep_pt_mx.lock();
      dev_pub->publish(episode_part);
      ep_pt_mx.unlock();
    }

};

int main( int argc, char** argv )
{
  // Input topics as arguments
  std::string base_dyn_topic;//= "/speedmarker/pr2_base_dynamics";
  std::string obj_dev_topic ;//= "/object_monitor/obj_dev";
  std::string obj_dyn_topic ;//= "/object_monitor/obj_dynamics";
  std::string base_ctl_topic;//= "/base_controller/command";
  float sleep_time = event_sleep;

  ros::init(argc, argv, "topple_episode_monitor");
  if (argc >= 6)
  {
    base_dyn_topic = argv[1];
    obj_dev_topic  = argv[2];
    obj_dyn_topic  = argv[3];
    base_ctl_topic = argv[4];
    sleep_time     = atof(argv[5]);
  }
  else if (argc >= 5)
  {
    base_dyn_topic = argv[1];
    obj_dev_topic  = argv[2];
    obj_dyn_topic  = argv[3];
    base_ctl_topic = argv[4];
  }
  else
  {
    ROS_ERROR("No Topics provided, exitting..");
    return -1;
  }

  ROS_INFO("%s got following arguments: ", ros::this_node::getName().c_str());
  for (int i=1; i<argc; i++)
    ROS_INFO("%s", argv[i]);

  ros::NodeHandle n;

  ros::Publisher dev_pub = n.advertise<topple_scenarios::ToppleEpisodePart>(ros::this_node::getName()+"/topple_episode_part", 1);

  ToppleEpisodeMon episode_monitor( &dev_pub);

  ros::Subscriber base_sub    = n.subscribe(base_dyn_topic, 100, &ToppleEpisodeMon::cb_base_dyn, &episode_monitor);
  ros::Subscriber obj_dev_sub = n.subscribe(obj_dev_topic,  100, &ToppleEpisodeMon::cb_obj_dev, &episode_monitor);
  ros::Subscriber obj_dyn_sub = n.subscribe(obj_dyn_topic,  100, &ToppleEpisodeMon::cb_obj_dyn, &episode_monitor);
  ros::Subscriber base_ctl_sub= n.subscribe(base_ctl_topic, 100, &ToppleEpisodeMon::cb_base_ctl, &episode_monitor);

  //ros::spin();
  while (ros::ok())
  {
    //publish episode part
    episode_monitor.mergeData();
    episode_monitor.publishData();

    // prepare for new biased sample cycle
    // if commented just the last available sample will be used
    //episode_monitor.resetValues();

    ros::spinOnce();
    ros::Duration(sleep_time).sleep();
  }

  return 0;
}
