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

class SubscribeAndPublish 
{
	public:
	SubscribeAndPublish()
	{
		//Publications
		//controllers for rotating motor and spinning prop
		odom_pub = n_.advertise<nav_msgs::Odometry>("odom", 50);
		pub1_=n_.advertise<sensor_msgs::Imu>("imu_data",1000);
		
		//Subscriptions
		sub_= n_.subscribe("/imu_data", 1000, &SubscribeAndPublish::callback, this);
	}
	
	void callback(const sensor_msgs::Imu& test)
	{
		
		ros::Time current_time, last_time;
		//tf::TransformBroadcaster imu_broadcaster;
		//tf::TransformListener imu_listener;
		//int r=test.orientation.x;
		//int p=test.angular.y;
		//int y=test.angular.z;
		
		
		/*geometry_msgs::Quaternion imu;
	
	double c1 = cos((r/2)*(M_PI/180));
    double s1 = sin((r/2)*(M_PI/180));
    double c2 = cos((y/2)*(M_PI/180));
    double s2 = sin((y/2)*(M_PI/180));
    double c3 = cos((p/2)*(M_PI/180));
    double s3 = sin((p/2)*(M_PI/180));
    double c1c2 = c1*c2;
    double s1s2 = s1*s2;
    imu.w =c1c2*c3 - s1s2*s3;
  	imu.x =c1c2*s3 + s1s2*c3;
	imu.y =s1*c2*c3 + c1*s2*s3;
	imu.z =c1*s2*c3 - s1*c2*s3;
		
		current_time = ros::Time::now();
		sensor_msgs::Imu imuMessage;
		imuMessage.header.stamp = current_time;
		imuMessage.header.frame_id = "base_footprint";
		
		
		imuMessage.angular_velocity.x=r*(M_PI/180);
		imuMessage.angular_velocity.y=p*(M_PI/180);
		imuMessage.angular_velocity.z=y*(M_PI/180);
		
		imuMessage.linear_acceleration.x=0;
		imuMessage.linear_acceleration.y=0;
		imuMessage.linear_acceleration.z=0;
		imuMessage.orientation=imu;
		
		
		pub1_.publish(imuMessage);
		*/
	
	/*geometry_msgs::TransformStamped imu_trans;
    imu_trans.header.stamp = current_time;
    imu_trans.header.frame_id = "odom";
    imu_trans.child_frame_id = "base_footprint";

    imu_trans.transform.translation.x =0;
    imu_trans.transform.translation.y = 0;
    imu_trans.transform.translation.z = 0.0;
    imu_trans.transform.rotation = imu;
	
	imu_broadcaster.sendTransform(imu_trans);*/
	//imu_listener.waitForTransform("/imu_data", "base_footprint",imu_trans.header.stamp, ros::Duration(3.0));
    //send the transform
    
			nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";
		odom.pose.pose.position.x = 0.0;
		odom.pose.pose.position.y = 0.0;
		odom.pose.pose.position.z = 0.0;
		odom.pose.covariance.elems[0] = 0.2;
		odom.pose.covariance.elems[7]= 0.2;
		odom.pose.covariance.elems[14]= 0.2;
		odom.pose.covariance.elems[21]=0.1;
		odom.pose.covariance.elems[28]=0.1;
		odom.pose.covariance.elems[35]=0.1;
		odom.twist.twist.linear.x=0;
		odom.twist.twist.linear.y=0;
		odom.twist.twist.linear.z=0;
		odom.twist.twist.angular.x=0;
		odom.twist.twist.angular.y=0;
		odom.twist.twist.angular.z=0;
		odom.pose.pose.orientation=test.orientation;
	odom_pub.publish(odom);
		
	}
	private:
	ros::NodeHandle n_;
	ros::Subscriber sub_;
	ros::Publisher pub1_;
	ros::Publisher odom_pub;
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_gazebo");
	SubscribeAndPublish object;
	ros::spin();
	
	return 0;
}
