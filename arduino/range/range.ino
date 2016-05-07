
#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>
#include <std_msgs/Header.h>

ros::NodeHandle nh;
sensor_msgs::Range rangeMeasurement;
std_msgs::Header header;
ros::Publisher range("range", &rangeMeasurement);


#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


  void setup() 
{
  
  nh.initNode();
  nh.advertise(range);


  // Initialize MPU6050
}

void loop()
{
  delay(50);
  header.frame_id="range";
  rangeMeasurement.header=header;
  rangeMeasurement.min_range=0.5;
  rangeMeasurement.max_range=20;
  rangeMeasurement.field_of_view=0.5;
  rangeMeasurement.range=sonar.ping_cm()/10.0;
  range.publish(&rangeMeasurement);
  
  nh.spinOnce();
  

  
  
}
