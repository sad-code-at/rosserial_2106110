#include <ros.h>
#include <geometry_msgs/Twist.h>

// Define motor control pins
const int IN1 = 2;
const int IN2 = 3;
const int ENA = 4;  // PWM pin for motor A
const int IN3 = 5;
const int IN4 = 6;
const int ENB = 7;  // PWM pin for motor B

ros::NodeHandle nh;

// Variable to store the speed for the motors
int motor_speed_A = 0;
int motor_speed_B = 0;

// Callback function for receiving cmd_vel messages
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
  // Extract linear.x for forward. only forward
  float linear = cmd_vel.linear.x;  // range 0 to 1.0
  float angular = cmd_vel.angular.z; // range 0 to 1.0

// setting the direction of the motors
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  

  // Map the linear speed to motor A and B (forward)
  motor_speed_A = linear * 255; // map speed to PWM range (0 - 255)
  motor_speed_B = linear * 255;

  // Determine the turning motion (left/right rotation)
  if (angular > 0) {  // Turning left
    digitalWrite(IN1,LOW);
    digitalWrite(IN1,HIGH); //changing the direction of motor A
    motor_speed_A = angular * 255;
    motor_speed_B = angular *255;           
  }
  

  // Set motor direction and speed for motor A
  

    analogWrite(ENA, motor_speed_A);
    analogWrite(ENA, motor_speed_B);
    
  if(linear ==0 & angular == 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
  
  
}

// ROS Subscriber
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVelCallback);

void setup() {
  // Set motor control pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize the ROS node handle
  nh.initNode();
  
  // Subscribe to the cmd_vel topic
  nh.subscribe(sub);
}

void loop() {
  // Spin ROS node to handle callbacks
  nh.spinOnce();

  // Add a small delay
  delay(10);
}
