#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for each sensor
SENSORS = [
    {"TRIG": 20, "ECHO": 21},
    {"TRIG": 5, "ECHO": 6},
    {"TRIG": 12, "ECHO": 16}
]

# Set up the GPIO pins
for sensor in SENSORS:
    GPIO.setup(sensor["TRIG"], GPIO.OUT)
    GPIO.setup(sensor["ECHO"], GPIO.IN)

def get_distance(trig, echo):
    # Set the Trigger to HIGH
    GPIO.output(trig, True)

    # Set the Trigger to LOW after 10 microseconds
    time.sleep(0.00001)
    GPIO.output(trig, False)

    # Save the start and stop times
    start_time = time.time()
    stop_time = time.time()

    # Record the start time
    while GPIO.input(echo) == 0:
        start_time = time.time()

    # Record the stop time
    while GPIO.input(echo) == 1:
        stop_time = time.time()

    # Calculate the time difference
    time_elapsed = stop_time - start_time
    # Calculate the distance (speed of sound is 34300 cm/s)
    distance = (time_elapsed * 34300) / 2

    return distance

def publish_sensor_data():
    rospy.init_node('ultrasonic_sensor_publisher')

    front_pub = rospy.Publisher('/ultrasonic_sensor/front', Range, queue_size=10)
    left_pub = rospy.Publisher('/ultrasonic_sensor/left', Range, queue_size=10)
    right_pub = rospy.Publisher('/ultrasonic_sensor/right', Range, queue_size=10)

    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        # Read sensor data
        front_distance = get_distance(SENSORS[0]["TRIG"], SENSORS[0]["ECHO"])
        left_distance = get_distance(SENSORS[1]["TRIG"], SENSORS[1]["ECHO"])
        right_distance = get_distance(SENSORS[2]["TRIG"], SENSORS[2]["ECHO"])

        # Debug: Print raw sensor data
        print(f"Raw front distance: {front_distance} cm")
        print(f"Raw left distance: {left_distance} cm")
        print(f"Raw right distance: {right_distance} cm")

        # Create and publish ROS messages
        front_msg = Range()
        front_msg.range = front_distance / 100  # Convert to meters
        front_pub.publish(front_msg)
        print(f"Published front distance: {front_msg.range} m")

        left_msg = Range()
        left_msg.range = left_distance / 100  # Convert to meters
        left_pub.publish(left_msg)
        print(f"Published left distance: {left_msg.range} m")

        right_msg = Range()
        right_msg.range = right_distance / 100  # Convert to meters
        right_pub.publish(right_msg)
        print(f"Published right distance: {right_msg.range} m")

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_sensor_data()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()  # Clean up the GPIO pins
