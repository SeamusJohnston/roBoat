#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <std_msgs/Float64.h>
#include "geometry_msgs/TransformStamped.h"
#include <geometry_msgs/Pose.h>

class SubscribeAndPublish 
{
	public:
	SubscribeAndPublish()
	{
		//Publications
		//controllers for rotating motor and spinning prop
		pub4_=n2_.advertise<std_msgs::Float64>("/boat/servo_controller/command",1000);
		pub5_=n2_.advertise<std_msgs::Float64>("/boat/motor_controller/command",1000);

		
		//Subscriptions
		sub2_= n2_.subscribe("/movement", 1000, &SubscribeAndPublish::callback, this);
	}
	
	void callback(const geometry_msgs::Twist& data)
	{
		std_msgs::Float64 linearData;
		std_msgs::Float64 turnData;

		linearData.data=data.linear.x;
		turnData.data=data.angular.z;

		pub4_.publish(turnData);
		pub5_.publish(linearData);
	
		
	}
	private:
	ros::NodeHandle n2_;
	ros::Subscriber sub2_;
	ros::Publisher pub4_;
	ros::Publisher pub5_;
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "remap");
	SubscribeAndPublish object;
	ros::spin();
	
	return 0;
}
