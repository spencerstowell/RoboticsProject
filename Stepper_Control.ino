#include <ros.h>
#include <std_msgs/Float32Array.h>

ros::NodeHandle nh;

// Callback function for the "/angle_command" topic
// Expects a std_msgs/Float32Array message
void angleCallback(const std_msgs::Float32Array& msg) {
  // Process the received angle command array
  float angle = msg.data;
  // Do something with the angle command
}

// Initialize the subscriber to the "angle_command" topic
ros::Subscriber<std_msgs::Float32> angleSub("/angle_command", angleCallback);

void setup() {
  nh.initNode();
  nh.subscribe(angleSub);
}

void loop() {
  nh.spinOnce();
  // Other code to run in the loop
}
