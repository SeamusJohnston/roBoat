#include <ArduinoHardware.h>

#include <ros.h>

#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
ros::NodeHandle nh;


geometry_msgs::Twist msg;


ros::Publisher cmd_vel("cmd_vel", &msg);


// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values

float pitch = 0;
float roll = 0;
float yaw = 0;

  void setup()
{

nh.initNode();
nh.advertise(cmd_vel);
while(!nh.connected())
{
  nh.spinOnce();
}


   while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    nh.spinOnce();
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
}

void loop()
{
  timer = millis();
// Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;
  msg.angular.x=roll;
  msg.angular.y=pitch;
  msg.angular.z=yaw;
  cmd_vel.publish(&msg);
  nh.spinOnce();

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));


}
