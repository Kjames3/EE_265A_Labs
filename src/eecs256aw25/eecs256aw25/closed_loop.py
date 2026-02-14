

import rclpy
import threading
import time
import math

import numpy as np

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped, Pose2D
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion

class Controller:
    """
    Controller class for implementing the PD Controller
    """

    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = None

    def update(self, current_value, dt=None):
        # Calculate error
        error = self.set_point - current_value
        
        # P_term
        P_term = self.Kp * error
        
        # D_term
        if self.previous_error is None:
            self.previous_error = error
        
        if dt is not None and dt > 0:
            D_term = self.Kd * (error - self.previous_error) / dt
        else:
            D_term = 0.0
        
        self.previous_error = error
        return P_term + D_term

    def update_with_velocity(self, error, measured_velocity):
        # PD Control with velocity feedback: u = Kp * error - Kd * measured_velocity
        # This avoids derivative kick and noise from differentiating the error manually
        return (self.Kp * error) - (self.Kd * measured_velocity)

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = None

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

class Turtlebot(Node):

    def __init__(self):

        # Create and initialize the Node - this is required
        super().__init__('turtlebot_move')

        self.get_logger().info("Press Ctrl + C to terminate")

        # Current pose of the robot
        self.pose = Pose2D()

        # Logging counter
        self.logging_counter = 0

        # Store the robot trajectory
        self.trajectory = []

        # Create publisher for publishing the velocity commands
        self.vel_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)

        # Create a subscriber for the ground truth
        self.odom_sub = self.create_subscription(Odometry,
            "odom",
            self.odom_callback,
            100)
            
        self.current_v = 0.0
        self.current_w = 0.0

    def odom_callback(self, msg):
        """
        Callback function to get the robot odometry.

        @param msg - Message passed by the odom topic.
        """

        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory

            self.get_logger().info("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))

        # get velocity
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z

    def run(self):
        """
        Run function.
        """

        # Create a Twist object
        # Essentially it's going to publish the velocity
        vel = TwistStamped()

        # Populate the fields of the message
        vel.twist.linear.x = 0.0
        vel.twist.angular.z = 0.0

        self.get_logger().info('Running closed-loop control...')

        # TODO: Add Closed loop control code over here
        # HINT: Use the pose provided to the subscriber
        # Waypoints for the diamond shape
        waypoints = [[3.0, -4.0], 
                     [6.0, 0.0], 
                     [3.0, 4.0], 
                     [0.0, 0.0]]
        
        # Initialize Controllers
        dist_controller = Controller(P=1.0, D=0.1)
        angle_controller = Controller(P=8.0, D=0.5)
        
        # Thresholds
        dist_threshold = 0.1 # Increased from 0.05 to avoid missing waypoints and to match lab report
        angle_threshold = 0.05 
        
        for target in waypoints:
            target_x, target_y = target
            self.get_logger().info(f"New Waypoint: ({target_x}, {target_y})")
            
            # --- PHASE 1: TURN IN PLACE ---
            self.get_logger().info("Rotating to target...")
            # angle_controller.setPoint(0) # Not used directly with update_with_velocity
            
            while rclpy.ok():
                dx = target_x - self.pose.x
                dy = target_y - self.pose.y
                desired_theta = math.atan2(dy, dx)
                
                # Calculate wrapped angular error
                theta_error = desired_theta - self.pose.theta
                while theta_error > math.pi: theta_error -= 2 * math.pi
                while theta_error < -math.pi: theta_error += 2 * math.pi
                
                # If aligned, break
                if abs(theta_error) < angle_threshold:
                    self.get_logger().info("Aligned.")
                    break
                
                # Compute control output using velocity feedback
                w_cmd = angle_controller.update_with_velocity(theta_error, self.current_w)
                
                # Publish
                vel.twist.linear.x = 0.0
                vel.twist.angular.z = w_cmd
                self.vel_pub.publish(vel)
                time.sleep(0.01) # 100Hz

            # Stop rotation
            vel.twist.angular.z = 0.0
            self.vel_pub.publish(vel)
            time.sleep(0.5)

            # --- PHASE 2: MOVE TO TARGET (with heading correction) ---
            self.get_logger().info("Moving to target...")
            # dist_controller.setPoint(0)

            while rclpy.ok():
                dx = target_x - self.pose.x
                dy = target_y - self.pose.y
                dist_error = math.sqrt(dx**2 + dy**2)
                
                # Check arrival
                if dist_error < dist_threshold:
                    self.get_logger().info("Waypoint reached.")
                    break
                
                # Heading error for correction
                desired_theta = math.atan2(dy, dx)
                theta_error = desired_theta - self.pose.theta
                while theta_error > math.pi: theta_error -= 2 * math.pi
                while theta_error < -math.pi: theta_error += 2 * math.pi
                
                # Compute controls with velocity feedback
                v_cmd = dist_controller.update_with_velocity(dist_error, self.current_v)
                w_cmd = angle_controller.update_with_velocity(theta_error, self.current_w)
                
                # Saturation
                v_cmd = min(max(v_cmd, -0.5), 0.5)
                w_cmd = min(max(w_cmd, -1.0), 1.0)

                # Debug Logging (every 1 second approx)
                current_time_log = self.get_clock().now().nanoseconds
                if (current_time_log % 1000000000) < 20000000: # 1Hz log
                     self.get_logger().info(f"DistErr: {dist_error:.2f}, HeadErr: {theta_error:.2f}, V_cmd: {v_cmd:.2f}, W_cmd: {w_cmd:.2f}")

                # Publish
                vel.twist.linear.x = v_cmd
                vel.twist.angular.z = w_cmd # Use calculated angular velocity
                self.vel_pub.publish(vel)
                time.sleep(0.01)
            
            # Stop at waypoint
            vel.twist.linear.x = 0.0
            vel.twist.angular.z = 0.0
            self.vel_pub.publish(vel)
            self.get_logger().info(f"Stopped at waypoint. Final Error: {math.sqrt((target_x-self.pose.x)**2 + (target_y-self.pose.y)**2):.4f}")
            time.sleep(1.0)
        
        # Stop at the end
        vel.twist.linear.x = 0.0
        vel.twist.angular.z = 0.0
        self.vel_pub.publish(vel)
        self.get_logger().info("Finished trajectory.")

    def save_trajectory(self):
        """
        Save trajectory to disk.
        """
        import os
        cwd = os.getcwd()
        path = os.path.join(cwd, "trajectory.csv")
        self.get_logger().info(f"Saving trajectory to {path} with {len(self.trajectory)} points...")
        np.savetxt("trajectory.csv", np.array(self.trajectory), fmt='%f', delimiter=',')
        self.get_logger().info("Trajectory saved.")

def closed_loop_entry_point_function(args=None):
    """
    Entry point function for the Closed Loop control node.

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
    # time.sleep(5)

    try:
        turtlebot_node.run()
    except KeyboardInterrupt:
        turtlebot_node.get_logger().info("Keyboard Interrupt.")
    finally:
        turtlebot_node.save_trajectory()

    # Destroy the node explicitly
    turtlebot_node.destroy_node()

    # Shut down this context
    rclpy.shutdown()


if __name__ == '__main__':
    closed_loop_entry_point_function()