#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/SetModelState.h"
#include "geometry_msgs/Quaternion.h"
#include "gazebo_msgs/ModelState.h"
#include <cmath>
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/TransformStamped.h"
#include <geometry_msgs/Pose.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "tfListener");

  ros::NodeHandle n;


  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (n.ok()){
    tf::StampedTransform transform;
    try {
    listener.waitForTransform("base_footprint", "imu_data", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("base_footprint", "imu_data", ros::Time(0), transform);
} catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
}
   
    rate.sleep();
  }
  return 0;
};
