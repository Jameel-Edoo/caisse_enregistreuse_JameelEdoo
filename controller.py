#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from math import radians, sqrt, pow, pi
import tf

class GoPiGo3Controller:
    def __init__(self):
        rospy.init_node('gopigo3_controller', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/ultrasonic_sensor/front', Range, self.front_sensor_callback)
        rospy.Subscriber('/ultrasonic_sensor/left', Range, self.left_sensor_callback)
        rospy.Subscriber('/ultrasonic_sensor/right', Range, self.right_sensor_callback)

        self.rate = rospy.Rate(10)

        self.current_pose = None
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.is_turning = False  # Flag to track if the robot is currently turning
        self.consecutive_turns = 0  # Counter for consecutive turns
        self.should_move = True #State variable to determine if the robot should move
        self.no_obstacles = 0 # Counter for determining when there are no obstacles at all

    def odom_callback(self, data):
        self.current_pose = data.pose.pose

    def front_sensor_callback(self, data):
        self.front_distance = data.range

    def left_sensor_callback(self, data):
        self.left_distance = data.range

    def right_sensor_callback(self, data):
        self.right_distance = data.range

    def get_yaw_from_pose(self, pose):
        orientation_q = pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw
    
    def move_forward(self):
        move_cmd = Twist()
        move_cmd.linear.x = -0.1
        self.cmd_vel_pub.publish(move_cmd)

    def stop(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(move_cmd)


    def move_backward(self, distance):
        if self.current_pose is None:
            rospy.logwarn("Current pose is not available. Cannot move backward.")
            return

        initial_x = self.current_pose.position.x
        initial_y = self.current_pose.position.y

        move_cmd = Twist()
        move_cmd.linear.x = 0.2

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.logwarn("Current pose is not available. Stopping.")
                break

            distance_moved = sqrt(pow((self.current_pose.position.x - initial_x), 2) + pow((self.current_pose.position.y - initial_y), 2))
            if distance_moved >= distance:
                break
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

        move_cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(move_cmd)

    def turn_right(self, angle):
        self.is_turning = True  # Set the flag to indicate the robot is turning
        if self.current_pose is None:
            rospy.logwarn("Current pose is not available. Cannot turn right.")
            return

        initial_yaw = self.get_yaw_from_pose(self.current_pose)
        target_yaw = (initial_yaw - radians(angle)) % (2 * pi)

        rospy.loginfo(f"Turning right: initial_yaw={initial_yaw}, target_yaw={target_yaw}")

        move_cmd = Twist()
        move_cmd.angular.z = -0.5

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.logwarn("Current pose is not available. Stopping.")
                break

            current_yaw = self.get_yaw_from_pose(self.current_pose)
            rospy.loginfo(f"Current yaw: {current_yaw}")

            if abs(current_yaw - target_yaw) <= radians(1):
                break

            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

            # Timeout to prevent getting stuck in turning
            if (rospy.Time.now() - start_time).to_sec() > 2:  # 2 seconds timeout
                rospy.logwarn("Turning timeout reached. Stopping turn.")
                break

        move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        self.is_turning = False  # Reset the flag after the turn is complete

    def turn_left(self, angle):
        self.is_turning = True  # Set the flag to indicate the robot is turning
        if self.current_pose is None:
            rospy.logwarn("Current pose is not available. Cannot turn left.")
            return

        initial_yaw = self.get_yaw_from_pose(self.current_pose)
        target_yaw = (initial_yaw + radians(angle)) % (2 * pi)

        rospy.loginfo(f"Turning left: initial_yaw={initial_yaw}, target_yaw={target_yaw}")

        move_cmd = Twist()
        move_cmd.angular.z = 0.5

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.logwarn("Current pose is not available. Stopping.")
                break

            current_yaw = self.get_yaw_from_pose(self.current_pose)
            rospy.loginfo(f"Current yaw: {current_yaw}")

            if abs(current_yaw - target_yaw) <= radians(1):
                break

            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

            # Timeout to prevent getting stuck in turning
            if (rospy.Time.now() - start_time).to_sec() > 2:  # 2 seconds timeout
                rospy.logwarn("Turning timeout reached. Stopping turn.")
                break

        move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        self.is_turning = False  # Reset the flag after the turn is complete

    def slight_right(self, angle):
        self.is_turning = True  # Set the flag to indicate the robot is turning
        if self.current_pose is None:
            rospy.logwarn("Current pose is not available. Cannot turn right.")
            return

        initial_yaw = self.get_yaw_from_pose(self.current_pose)
        target_yaw = (initial_yaw - radians(angle)) % (2 * pi)

        rospy.loginfo(f"Turning right: initial_yaw={initial_yaw}, target_yaw={target_yaw}")

        move_cmd = Twist()
        move_cmd.angular.z = -0.5

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.logwarn("Current pose is not available. Stopping.")
                break

            current_yaw = self.get_yaw_from_pose(self.current_pose)
            rospy.loginfo(f"Current yaw: {current_yaw}")

            if abs(current_yaw - target_yaw) <= radians(1):
                break

            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

            # Timeout to prevent getting stuck in turning
            if (rospy.Time.now() - start_time).to_sec() > 1:  # 1 seconds timeout
                rospy.logwarn("Turning timeout reached. Stopping turn.")
                break

        move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        self.is_turning = False  # Reset the flag after the turn is complete

    def slight_left(self, angle):
        self.is_turning = True  # Set the flag to indicate the robot is turning
        if self.current_pose is None:
            rospy.logwarn("Current pose is not available. Cannot turn left.")
            return

        initial_yaw = self.get_yaw_from_pose(self.current_pose)
        target_yaw = (initial_yaw + radians(angle)) % (2 * pi)

        rospy.loginfo(f"Turning left: initial_yaw={initial_yaw}, target_yaw={target_yaw}")

        move_cmd = Twist()
        move_cmd.angular.z = 0.5

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.logwarn("Current pose is not available. Stopping.")
                break

            current_yaw = self.get_yaw_from_pose(self.current_pose)
            rospy.loginfo(f"Current yaw: {current_yaw}")

            if abs(current_yaw - target_yaw) <= radians(1):
                break

            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

            # Timeout to prevent getting stuck in turning
            if (rospy.Time.now() - start_time).to_sec() > 1:  # 2 seconds timeout
                rospy.logwarn("Turning timeout reached. Stopping turn.")
                break

        move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        self.is_turning = False  # Reset the flag after the turn is complete



    def escape_corner(self):
        rospy.loginfo("Attempting to escape corner...")
        self.move_backward(0.5)  # Move backward a little
        self.turn_right(45)  # Turn right 45 degrees
        self.consecutive_turns = 0  # Reset the consecutive turn counter

    def navigate(self):
        start_time=rospy.Time.now()
        while not rospy.is_shutdown():
            if not self.is_turning:  # Execute navigation logic only if the robot is not turning
                print(f"Front Distance: {self.front_distance:.2f} m")
                print(f"Left Distance: {self.left_distance:.2f} m")
                print(f"Right Distance: {self.right_distance:.2f} m")

                if self.front_distance < 0.6 or self.right_distance < 0.6 or self.left_distance < 0.6:
                    self.should_move = True # Set the flag to indicate the robot should move


                else:
                    #start_time = rospy.Time.now()
                    print((rospy.Time.now()-start_time).to_sec())

                    if (rospy.Time.now() - start_time).to_sec() > 5:
                        self.no_obstacles += 1
                        start_time = rospy.Time.now()
                        if self.no_obstacles <=1:
                            pass


                        else:
                            self.should_move = False

                print(self.no_obstacles)
                print(self.should_move)

                if self.should_move == True:
                    if self.front_distance < 0.2:  # Obstacle detected in front
                        self.stop()
                        if self.left_distance > self.right_distance:
                            print("Turning left...")
                            self.turn_left(90)
                        else:
                            print("Turning right...")
                            self.turn_right(90)
                        self.consecutive_turns += 1
                    elif self.left_distance < 0.1:  # Obstacle detected on the left
                        self.stop()
                        print("Obstacle on left, adjusting right...")
                        self.slight_right(15)  # Slight right adjustment
                        self.consecutive_turns += 1
                    elif self.right_distance < 0.1:  # Obstacle detected on the right
                        self.stop()
                        print("Obstacle on right, adjusting left...")
                        self.slight_left(15)  # Slight left adjustment
                        self.consecutive_turns += 1
                    else:
                        print("Moving forward...")
                        self.move_forward()  # Move forward in small increments
                        self.consecutive_turns = 0  # Reset the consecutive turn counter

                    # Check for corner case
                    if self.consecutive_turns >= 4:  # Adjust this threshold as needed
                        self.escape_corner()

                else:
                    # Stop if no obstacles are detected within 50 cm five times in a row
                    self.stop()
                    self.no_obstacles = 0 # Reset the no obstacle counter
                    #Set the robot to stop for 10 seconds
                    rospy.sleep(5)

            self.rate.sleep()

    def run(self):
        rospy.sleep(1)  # Wait for a second to ensure initialization
        self.navigate()

if __name__ == '__main__':
    try:
        controller = GoPiGo3Controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass

