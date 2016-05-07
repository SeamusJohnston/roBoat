#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>
  
	



int main(int argc, char** argv){
  ros::init(argc, argv, "odom");
  
 ros::NodeHandle n_; 
 
  ros::Time current_time, last_time;
  tf::TransformBroadcaster odom_broadcaster;
  
    //Topic you want to publish
   ros::Publisher odom_pub = n_.advertise<nav_msgs::Odometry>("odom", 50);
   
  ros::ServiceClient model = n_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  while(ros::ok())
{
		gazebo_msgs::GetModelState test;
	  geometry_msgs::Twist model_twist;
	  geometry_msgs::Pose model_pose;
	  
	  test.request.model_name= "boat";
	  
  model.call(test);
  
  model_pose.position.x = test.response.pose.position.x;
  model_pose.position.y = test.response.pose.position.y;
  
  model_twist.linear.x = test.response.twist.linear.x;
  model_twist.linear.y = test.response.twist.linear.y;
  model_twist.angular.z = test.response.twist.angular.z;
  
	current_time = ros::Time::now();
double th=+ model_twist.angular.z;
/*
double w= test.response.pose.orientation.w;
double x= test.response.pose.orientation.x;
double y= test.response.pose.orientation.y;
double z= test.response.pose.orientation.z;

double r= atan2(2*(w*x+y*z),(1-2*(x*x+y*y)));
double p= asin(2*(w*y-z*x));
double yaw= atan2(2*(w*z+x*y),(1-2*(y*y+z*z)));

    //since all odometry is 6DOF we'll need a quaternion created from yaw

double a=0;*/
 geometry_msgs::Quaternion odom_quat;
 /*   	odom_quat.w =cos(a/2);
  	odom_quat.x =sin(a/2)*cos(r);
	odom_quat.y =sin(a/2)*cos(p);
	odom_quat.z =sin(a/2)*cos(yaw+M_PI);

 */
odom_quat=test.response.pose.orientation;   
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = model_pose.position.x;
    odom_trans.transform.translation.y = model_pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = model_pose.position.x;
    odom.pose.pose.position.y = model_pose.position.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = model_twist.linear.x;
    odom.twist.twist.linear.y = model_twist.linear.y;
    odom.twist.twist.angular.z = model_twist.angular.z;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    
ros::spinOnce();
}
return 0;
 
  }

