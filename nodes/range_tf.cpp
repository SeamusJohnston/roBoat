#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/SetModelState.h"
#include "geometry_msgs/Quaternion.h"
#include "gazebo_msgs/ModelState.h"
#include <cmath>
#include "sensor_msgs/Range.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/TransformStamped.h"
#include <geometry_msgs/Pose.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "range_tf");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(2.5, 0, 2) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "range"));
    rate.sleep();
  }
  return 0;
};
