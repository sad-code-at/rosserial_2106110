#include <ros.h>                          // ROS serial library
#include <std_msgs/Float32MultiArray.h>    // ROS message type for the sensor data

// Define sensor pins
const int trigPin = 2;        // TRIG pin of the ultrasonic sensor
const int echoPin = 4;        // ECHO pin of the ultrasonic sensor
const int potPin = A0;        // Potentiometer signal pin

// Variables for timing
long duration;
float distanceCm;

// Variables for the potentiometer
int potValue;
float scaledPotValue;

// ROS node handle
ros::NodeHandle nh;

// ROS message to publish sensor data
std_msgs::Float32MultiArray sensor_data_msg;

// ROS publisher
ros::Publisher sensor_pub("/sensor_data", &sensor_data_msg);

void setup() {

  Serial.begin(57600);

  // Initialize serial communication for ROS
  nh.initNode();
  nh.advertise(sensor_pub);

  // Set up pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(potPin, INPUT);

  // Initialize the array size for sensor data
  sensor_data_msg.data_length = 2;           // Two elements (potentiometer, ultrasonic distance)
  sensor_data_msg.data = (float*)malloc(sizeof(float) * 2);
}

void loop() {
  // Measure distance from the ultrasonic sensor
  distanceCm = measureDistance();

  // Read the potentiometer value and scale it to 0.0 - 1.0
  potValue = analogRead(potPin);
  scaledPotValue = map(potValue, 0, 1023, 0, 100) / 100.0;

  // Populate the sensor data message
  sensor_data_msg.data[0] = scaledPotValue;  // Potentiometer value (scaled 0.0 - 1.0)
  sensor_data_msg.data[1] = distanceCm;      // Ultrasonic distance (in cm)

  // Publish the sensor data to the /sensor_data topic
  sensor_pub.publish(&sensor_data_msg);

  // Spin to keep the connection alive
  nh.spinOnce();

  // Delay for 1 second
  delay(1000);
}

// Function to measure distance using the ultrasonic sensor
float measureDistance() {
  // Clear the TRIG pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Set the TRIG pin high for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the duration from the ECHO pin
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters (duration / 2) / 29.1
  float distance = (duration * 0.0343) / 2;
  return distance;
}
