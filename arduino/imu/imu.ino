#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include <MPU6050.h>
#include <geometry_msgs/Quaternion.h>
MPU6050 mpu;
ros::NodeHandle nh;


sensor_msgs::Imu imuMessage;
geometry_msgs::Quaternion imuQ;

ros::Publisher imu("/imu_data", &imuMessage);


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
nh.advertise(imu);
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
  
    double c1 = cos((roll/2)*(PI/180));
    double s1 = sin((roll/2)*(PI/180));
    double c2 = cos((yaw/2)*(PI/180));
    double s2 = sin((yaw/2)*(PI/180));
    double c3 = cos((pitch/2)*(PI/180));
    double s3 = sin((pitch/2)*(PI/180));
    double c1c2 = c1*c2;
    double s1s2 = s1*s2;
    imuQ.w =c1c2*c3 - s1s2*s3;
    imuQ.x =c1c2*s3 + s1s2*c3;
    imuQ.y =s1*c2*c3 + c1*s2*s3;
    imuQ.z =c1*s2*c3 - s1*c2*s3;

    imuMessage.header.frame_id = "base_footprint";

    imuMessage.linear_acceleration.x=0;
    imuMessage.linear_acceleration.y=0;
    imuMessage.linear_acceleration.z=0;
    imuMessage.orientation=imuQ;

    imu.publish(&imuMessage);
  
  nh.spinOnce();

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));


}

