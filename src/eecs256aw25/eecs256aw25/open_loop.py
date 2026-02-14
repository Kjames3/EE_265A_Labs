import rclpy
import threading
import time

import numpy as np

from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

import math

from irobot_create_msgs.action import Undock

class Turtlebot(Node):

    def __init__(self):

        # Create and initialize the Node - this is required
        super().__init__('turtlebot_move')

        self.get_logger().info("Press Ctrl + C to terminate")

        # Create publisher for publishing the velocity commands
        self.vel_pub = self.create_publisher(TwistStamped, "cmd_vel", qos_profile_sensor_data)

        # Undocking from the station
        self.undock_action_client = ActionClient(self, Undock, '/undock')

    def undock(self):
        """
        Undock TurtleBot 4 from the docking station.
        """

        # Use the action client to send the undock goal to the TurtleBot 4.
        self.undock_action_client.wait_for_server()
        undock_goal_result = self.undock_action_client.send_goal(Undock.Goal())
        if undock_goal_result.result.is_docked:
            self.get_logger().error('Undocking failed')

    # Create a TwistStamped message 
    def create_twist_stamped(self, linear_x, angular_z):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist.linear.x = linear_x
        twist_stamped.twist.angular.z = angular_z
        return twist_stamped

    def spin_for(self, duration_sec, twist_msg):
        # Get the current time
        start_time = self.get_clock().now()

        # Print the target duration
        self.get_logger().info(f"Start spin: {duration_sec:.2f}s target")

        # Loop until the target duration is reached
        while rclpy.ok() and (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration_sec:
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(twist_msg)
            # Short sleep to prevent CPU hogging, but loop condition rules validity
            time.sleep(0.01)
        
        elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
        self.get_logger().info(f"End spin: {elapsed:.4f}s elapsed")
        
        # Stop
        stop_msg = self.create_twist_stamped(0.0, 0.0)
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        self.vel_pub.publish(stop_msg)
        time.sleep(0.1)

    def run(self):
        """
        Run function. There are a variety of design patterns.
        """

        # Create a Twist object
        # Essentially it's going to publish the velocity
        vel = self.create_twist_stamped(0.5, 0.0)

        # Undock first
        self.undock()

        # Provide some time to undock
        time.sleep(1)

        self.get_logger().info("Undocking complete.")
        self.get_logger().info('Running open-loop control...')

        # TODO: Modify / Add code after this point
        # SAMPLE: Go Straight
        # for _ in range(100):
        #     self.vel_pub.publish(self.create_twist_stamped(0.5, 0.0))
        #     time.sleep(0.1)

        waypoints = [
            (3.0, -4.0),
            (6.0, 0.0),
            (3.0, 4.0),
            (0.0, 0.0)
        ]

        linear_speed = 0.5
        angular_speed = 0.5

        # Compensation for acceleration/deceleration of the robot that is messing up the timing
        LINEAR_SCALE = 2.05
        ANGULAR_SCALE = 1.60

        current_x = 0.0
        current_y = 0.0
        current_theta = 0.0

        self.get_logger().info("Starting Open Loop Control...")

        for wp_x, wp_y in waypoints:
            # Calculate the change in x and y
            dx = wp_x - current_x
            dy = wp_y - current_y
            
            # Get the distance and angle to waypoint
            distance = math.sqrt(dx**2 + dy**2)
            angle = math.atan2(dy, dx)

            # Calculate the delta angle
            turn_angle = angle - current_theta
            
            while turn_angle > math.pi:
                turn_angle -= 2 * math.pi
            while turn_angle < -math.pi:
                turn_angle += 2 * math.pi

            # Calculate the turn duration time
            turn_duration = (abs(turn_angle) / angular_speed) * ANGULAR_SCALE
            drive_duration = (distance / linear_speed) * LINEAR_SCALE

            # Print the current state
            self.get_logger().info(f"WP: ({wp_x}, {wp_y}), Curr: ({current_x}, {current_y}, {current_theta:.2f})")

            # Print the turn and drive durations
            self.get_logger().info(f"Turning {turn_angle: .2f} rad, Driving {distance: .2f} m")

            if turn_angle != 0:
                turn_msg = self.create_twist_stamped(0.0, angular_speed if turn_angle > 0 else -angular_speed)
                self.spin_for(turn_duration, turn_msg)
            
            time.sleep(0.5)

            # Drive straight
            drive_msg = self.create_twist_stamped(linear_speed, 0.0)
            self.spin_for(drive_duration, drive_msg)
            
            time.sleep(0.5)

            # Update the internal state for the next iteration
            current_x = wp_x
            current_y = wp_y
            current_theta = angle


def open_loop_entry_point_function(args=None):
    """
    Entry point function for the Open Loop control node.

    @param args - Arguments that need to be passed to ROS.
    """

    # Initialize the client library - this is required
    rclpy.init(args=args)

    # Create the publisher node
    turtlebot_node = Turtlebot()

    # Spin so that the node is busy processing
    # while no exceptions are encountered and ROS
    # is executing
    # For this lab - spin rclpy on separate thread
    thread = threading.Thread(target=rclpy.spin, args=(turtlebot_node,), daemon=True)
    thread.start()

    # Allow time for other nodes to start
    time.sleep(5)

    try:
        turtlebot_node.run()
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    turtlebot_node.destroy_node()

    # Shut down this context
    rclpy.shutdown()


if __name__ == '__main__':
    open_loop_entry_point_function()