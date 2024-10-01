#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float16MultiArray

# Callback function to process sensor data
def sensor_callback(data):
    global obstacle_detected, movement_speed
    
    # Read potentiometer value (mapped between 0 and 1)
    movement_speed = data.data[0]
    
    # Read ultrasound distance in cm
    ultrasound_distance = data.data[1]
    
    # If an obstacle is detected within 20 cm, stop forward movement
    if ultrasound_distance < 20:
        obstacle_detected = True
    else:
        obstacle_detected = False

# Function to publish movement commands
def publish_cmd_vel():
    # Create a Twist message
    twist = Twist()

    # If obstacle is detected, turn
    if obstacle_detected:
        twist.linear.x = 0.0  # Stop moving forward
        twist.angular.z = 0.5  # Start turning (adjust direction and speed as needed)
    else:
        twist.linear.x = movement_speed  # Move forward based on potentiometer value
        twist.angular.z = 0.0  # No turning

    # Publish the movement command
    cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('driver_node', anonymous=True)
        
        # Initialize global variables
        obstacle_detected = False
        movement_speed = 0.0
        
        # Subscribe to the /sensor_data topic
        rospy.Subscriber("/sensor_data", Float16MultiArray, sensor_callback)
        
        # Publisher for /cmd_vel topic
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=15)
        
        # Set loop rate
        rate = rospy.Rate(110)  # adjusting with arduino
        
        # Main loop
        while not rospy.is_shutdown():
            publish_cmd_vel()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
