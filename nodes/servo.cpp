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
		pub1_=n_.advertise<std_msgs::Float64>("/boat/prop_housing_controller/command",1000);

		
		//Subscriptions
		sub_= n_.subscribe("/boat/servo_controller/command", 1000, &SubscribeAndPublish::callback, this);
	}
	
	void callback(const std_msgs::Float64& test)
	{
		std_msgs::Float64 pubdata;

		pubdata.data=-1*test.data;
		
		pub1_.publish(pubdata);
	
		
	}
	private:
	ros::NodeHandle n_;
	ros::Subscriber sub_;
	ros::Publisher pub1_;
	
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servo");
	SubscribeAndPublish object;
	ros::spin();
	
	return 0;
}
