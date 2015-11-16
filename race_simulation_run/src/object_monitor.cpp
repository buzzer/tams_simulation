/*
 * 2014-03-07 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 */
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <race_pr2_markers/BaseDynamics.h>
#include <math.h>

const tf::Transform gaz2map(tf::Quaternion(0,0,0,1),tf::Vector3(12.28,10.20,0.0));
const tf::Transform rob2try(tf::Quaternion(0,0,0,1),tf::Vector3(0.08,0.00,0.346));
const tf::Transform rob2obj(tf::Quaternion(0,0,0,1),tf::Vector3(0.3, 0.0, 0.35));
const bool absolute_values = false; // negative or only absolute values for speed for backward compatibility
//std::string object_name_gaz = "race_flowers1";
std::string object_name_gaz = "race_peppermill1";
const ros::Duration marker_lifetime(2);
const ros::Duration sleep_dur(0.05); // callback event loop sleep == 1/f
const std::string robot_gaz_name("race_pr2");
// TODO get automatically from Gazebo
struct CylMarker {
  float r; // radius in m
  float h; // height in m
  CylMarker (float a, float b) { r=a; h=b; }
};
// Tray object marker
const CylMarker cyl_marker(0.037,0.274); // from Gazebo model

class ObjectMonitor
{
  protected:

    ros::Publisher * dev_pub;
    ros::Publisher * mark_pub;
    ros::Publisher * dyn_pub;
    std::string object_name;
    tf::TransformBroadcaster br;
    tf::TransformListener * tf_lr;
    tf::Transform gaz2map_;

  public:

    /*
     * returns:
     * 0 - robot transform
     * 1 - object transform
     */
    std::vector<tf::Transform>
      getObjectPose(const gazebo_msgs::ModelStates::ConstPtr & msg, std::string obj_name)
    {
        tf::Vector3 pose(0,0,0);
        tf::Quaternion rot(0,0,0,1);
        tf::Transform obj_t(rot, pose);
        tf::Transform rob(rot, pose);
        unsigned int found = 0;
        std::vector<tf::Transform> tfs;

        unsigned int count = msg->name.size();
        // Note: backwards loop as typically these objects are at the end of the
        // array. Does not have to be!
        for (int i=count-1; i>=0; --i)
        {
          if( obj_name ==  msg->name[i] )
          {
            pose.setValue(
                  msg->pose[i].position.x,
                  msg->pose[i].position.y,
                  msg->pose[i].position.z);
            rot.setX(msg->pose[i].orientation.x);
            rot.setY(msg->pose[i].orientation.y);
            rot.setZ(msg->pose[i].orientation.z);
            rot.setW(msg->pose[i].orientation.w);

            obj_t.setOrigin(pose);
            obj_t.setRotation(rot);
            ++found;
          }
          if( robot_gaz_name == msg->name[i] )
          {
            pose.setValue(
                  msg->pose[i].position.x,
                  msg->pose[i].position.y,
                  msg->pose[i].position.z);
            rot.setX(msg->pose[i].orientation.x);
            rot.setY(msg->pose[i].orientation.y);
            rot.setZ(msg->pose[i].orientation.z);
            rot.setW(msg->pose[i].orientation.w);

            rob.setOrigin(pose);
            rob.setRotation(rot);
            ++found;
          }
          // optimization
          if(found >= 2) break;
        }

        tfs.push_back(rob);
        tfs.push_back(obj_t);
        //ROS_INFO("found pose: %4.2f, %4.2f", msg->pose[i].position.x, msg->pose[i].position.y);
        //ROS_INFO("found pose: %4.2f, %4.2f", obj_t.getOrigin().getX(), obj_t.getOrigin().getY());
        return tfs;
    }

    void callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
      // ModelStates does not have a timestamp: work around: use own time!!
      //throttle to about 20 Hz as Gazebo publishes at 1000Hz
      ros::Time time_now = ros::Time::now();
      static ros::Time time_last = time_now;
      ros::Duration dur_last = time_now - time_last;
      if (dur_last >= sleep_dur)
      {
        tf::Transform obj_t;
        static tf::Transform obj_t_old;
        tf::Transform rob;
        float vel_now;
        static float vel_old;
        float acc_now;
        static float acc_old;
        float jerk_now;
        visualization_msgs::MarkerArray marker_array;

        std::vector<tf::Transform> poses = getObjectPose(msg, object_name);
        rob = poses.at(0);
        obj_t = poses.at(1);
        //ROS_INFO("object origin: %4.2f,%4.2f",obj_t.getOrigin().getX(), obj_t.getOrigin().getY());
        //Object frame and mesh
        tf::StampedTransform tr(
              //gaz2map_ * obj_t,
              obj_t,
              //rob.inverse() * obj_t * rob2try.inverse(),
              time_now,
              //std::string("/map"),
              std::string("/gazebo_link"),
              //std::string("/base_tray_link"),
              std::string("/tray_object_1"));
        br.sendTransform(tr);
        marker_array.markers.push_back(makeMarker(tr, std::string("object_model")));

        // Reference angle
        tf::Quaternion ref_ang(0,0,0,1);
        //ROS_INFO("Z axis: %4.2f, %4.2f, %4.2f",/*{{{*/
        //    tf::Matrix3x3(ref_ang).getColumn(2).getX(),
        //    tf::Matrix3x3(ref_ang).getColumn(2).getY(),
        //    tf::Matrix3x3(ref_ang).getColumn(2).getZ());/*}}}*/
        ref_ang.setRPY(0,-M_PI/2,0);
        tf::Transform obj_ref =
              gaz2map_ * tf::Transform(ref_ang, obj_t.getOrigin());
        marker_array.markers.push_back(makeMarker(tf::StampedTransform(
              obj_ref,
              time_now,
              std::string("/map"),
              std::string("/tray_object")), std::string("ref_angle"), true));

        // actual object angle
        tf::Transform obj_ang =
              gaz2map_ * obj_t;// * tf::Transform(ref_ang, tf::Vector3(0,0,0));
        tf::Vector3 z_ax_obj(
            tf::Matrix3x3(obj_ang.getRotation()).getColumn(2).getX(),
            tf::Matrix3x3(obj_ang.getRotation()).getColumn(2).getY(),
            tf::Matrix3x3(obj_ang.getRotation()).getColumn(2).getZ());
        //ROS_INFO("Z axis obj: %4.2f, %4.2f, %4.2f",/*{{{*/
        //    z_ax_obj.getX(),
        //    z_ax_obj.getY(),
        //    z_ax_obj.getZ());/*}}}*/
        marker_array.markers.push_back(makeMarker(tf::StampedTransform(
              obj_ang * tf::Transform(ref_ang, tf::Vector3(0,0,0)),
              time_now,
              std::string("/map"),
              std::string("/tray_object")), std::string("obj_angle"), true));

        // publish the marker
        mark_pub->publish(marker_array);

        //Angle between reference vector and object vector (z)
        double ang =
          acos(
              z_ax_obj.getZ() / sqrt(
                      +z_ax_obj.getX()*z_ax_obj.getX()
                      +z_ax_obj.getY()*z_ax_obj.getY()
                      +z_ax_obj.getZ()*z_ax_obj.getZ()));
        //ROS_INFO("Angle: %4.2f", ang);
        geometry_msgs::Vector3Stamped dev_msg;
        dev_msg.header.stamp = time_now;
        dev_msg.header.frame_id = "/base_footprint";
        dev_msg.vector.x = ang;
        //relative translatory deviation object to tray
        tf::Vector3 tray_obj = obj_t.getOrigin() - (rob*rob2obj).getOrigin();
        //vector length is trans deviation
        dev_msg.vector.y = sqrt(
            +tray_obj.getX()*tray_obj.getX()
            +tray_obj.getY()*tray_obj.getY()
            +tray_obj.getZ()*tray_obj.getZ());

        dev_pub->publish(dev_msg);

        // object dynamcis
        vel_now = getVel(obj_t.getOrigin(), obj_t_old.getOrigin(), dur_last);
        vel_now = getMovingAverage(vel_now);
        acc_now = getDerivation(vel_now, vel_old, dur_last);
        jerk_now = getDerivation(acc_now, acc_old, dur_last);
        float orient = getOrient(obj_t, obj_t_old);
        //ROS_INFO("Obj. dynamics: %4.2f,%4.2f,%4.2f,%4.2f", vel_now, acc_now, jerk_now, orient);

        std::vector<float> dyn = {vel_now, acc_now, jerk_now, orient};
        publishDynamics(dyn, time_now);

        obj_t_old = obj_t;
        vel_old = vel_now;
        acc_old = acc_now;
        time_last = time_now;
      }
    }

    float getMovingAverage(float val)
    {
      float av;
      //TODO: implement
      //http://stackoverflow.com/questions/10990618/calculate-rolling-moving-average-in-c-or-c
      av = val;

      return av;
    }

    ObjectMonitor(
        const std::string & ob_name,
        ros::Publisher * pub1,
        ros::Publisher * mark,
        ros::Publisher * dyn,
        const tf::Transform & g2m)
    {
      dev_pub = pub1;
      object_name = ob_name;
      mark_pub = mark;
      dyn_pub = dyn;
      gaz2map_ = g2m;
    }

    float getVel(tf::Vector3 & pose1, tf::Vector3 & pose2, const ros::Duration & dur)
    {
      float vel;
      float delta_x = fabs(pose1.getX() - pose2.getX());
      float delta_y = fabs(pose1.getY() - pose2.getY());
      float dist = sqrt(delta_x*delta_x + delta_y*delta_y);

      vel = dist / dur.toSec();

      return vel;
    }

    float getDerivation(float val1, float val2, const ros::Duration & dur)
    {
      float der;

      if (absolute_values)
        der = fabs(val1 - val2) / dur.toSec();
      else
        der = (val1 - val2) / dur.toSec();

      return der;
    }
    //FIXME
    //calculate the absolute orientation of the object
    //
    float getOrient(tf::Transform & pose1, tf::Transform & pose2)
    {
      float orient;

      /* The orientation is the angle of (x-axis) rotation about the z-axis,
       *  under the assumption that z-axis does not rotate on x or y */
      orient = acos(
          sqrt(pow(tf::Matrix3x3(pose1.getRotation()).getColumn(0).getX(),2) + pow(tf::Matrix3x3(pose1.getRotation()).getColumn(0).getY(),2)) /
          sqrt(pow(tf::Matrix3x3(pose2.getRotation()).getColumn(0).getX(),2) + pow(tf::Matrix3x3(pose2.getRotation()).getColumn(0).getY(),2))  );

      return orient;
    }
    /*
     * Publish dynamics
     * Arguments:
     * 1-vel
     * 2-acc
     * 3-jerk
     * 4-orientation
     */
    void publishDynamics(std::vector<float> & dyn, ros::Time time)
    {
      race_pr2_markers::BaseDynamics dynamics;

      dynamics.header.frame_id = "/map";
      // use original message time stamp
      dynamics.header.stamp = time;
      // Remark: Only linear (accumulated) dynamics here, no distinction between
      // angular and linear
      dynamics.linear.vel = dyn[0];
      dynamics.linear.acc = dyn[1];
      dynamics.linear.jerk = dyn[2];
      dynamics.angular.vel = dyn[3];

      dyn_pub->publish(dynamics);
    }

    /*
     * Marker type can be:
     *  0 - mesh
     *  1 - arrow
     */
    visualization_msgs::Marker makeMarker(
        const tf::StampedTransform & tf, std::string ns="marker", bool arrow=false)
    {
      unsigned int counter = 0;
      visualization_msgs::Marker marker;
      visualization_msgs::Marker textmarker;
      unsigned int shape;

      if (arrow == false)
        //shape = visualization_msgs::Marker::MESH_RESOURCE;
        shape = visualization_msgs::Marker::CYLINDER;
      else
        shape = visualization_msgs::Marker::ARROW;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = tf.frame_id_;

      marker.header.stamp = tf.stamp_;

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = ns;

      marker.id = counter;

      // Set the marker type.  Initially this is CUBE, and cycles between that
      // and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = tf.getOrigin().x();
      marker.pose.position.y = tf.getOrigin().y();
      marker.pose.position.z = tf.getOrigin().z();
      marker.pose.orientation.x = tf.getRotation().x();
      marker.pose.orientation.y = tf.getRotation().y();
      marker.pose.orientation.z = tf.getRotation().z();
      marker.pose.orientation.w = tf.getRotation().w();

      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      if (arrow == false)
      {
        // Cylinder pivot point is center point, need another transform
        tf::Transform tf_cyl;
        tf_cyl.setOrigin(tf::Vector3(0.0, 0.0, (cyl_marker.h/2)));
        tf_cyl.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        tf_cyl = tf * tf_cyl;
        marker.pose.position.x =    tf_cyl.getOrigin().x();
        marker.pose.position.y =    tf_cyl.getOrigin().y();
        marker.pose.position.z =    tf_cyl.getOrigin().z();
        marker.pose.orientation.x = tf_cyl.getRotation().x();
        marker.pose.orientation.y = tf_cyl.getRotation().y();
        marker.pose.orientation.z = tf_cyl.getRotation().z();
        marker.pose.orientation.w = tf_cyl.getRotation().w();
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker.scale.x = marker.scale.y = (cyl_marker.r*2);
        marker.scale.z = cyl_marker.h;
      }

      marker.lifetime = marker_lifetime;

      return marker;
    }
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "gazebo_object_monitor");
  ros::NodeHandle n;

  ros::Publisher dev_pub = n.advertise<geometry_msgs::Vector3Stamped>(ros::this_node::getName()+"/obj_dev", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName()+"/obj_marker", 1);
  ros::Publisher dyn_pub =
    n.advertise<race_pr2_markers::BaseDynamics>(ros::this_node::getName()+"/obj_dynamics", 1);

  if (argc < 2)
  {
    ROS_WARN("Missing gazebo model! Setting default: %s",
        object_name_gaz.c_str() );
  }
  else
  {
    object_name_gaz = argv[1];
  }

  ObjectMonitor object_monitor(
      object_name_gaz,
      &dev_pub,
      &marker_pub,
      &dyn_pub,
      gaz2map);
  ros::Subscriber sub = n.subscribe("/gazebo/model_states", 10, &ObjectMonitor::callback, &object_monitor);

  ros::spin();

  return 0;
}
