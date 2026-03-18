from copy import deepcopy
import math
import time
import threading

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped, Pose2D
from nav_msgs.msg import Odometry

from irobot_create_msgs.action import Undock
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
        error = self.set_point - current_value
        P_term = self.Kp * error
        if self.previous_error is None:
            self.previous_error = error
        if dt is not None and dt > 0:
            D_term = self.Kd * (error - self.previous_error) / dt
        else:
            D_term = 0.0
        self.previous_error = error
        return P_term + D_term

    def update_with_velocity(self, error, measured_velocity):
        return (self.Kp * error) - (self.Kd * measured_velocity)

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = None

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

class Turtlebot(Node):

    def __init__(self, path, num_path, wf, ws):

        # Create and initialize the Node - this is required
        super().__init__('turtlebot_move')

        self.get_logger().info("Press Ctrl + C to terminate")

        # Current pose of the robot
        self.pose = Pose2D()

        # Logging counter
        self.logging_counter = 0

        # Store the robot trajectory
        self.trajectory = []

        # Store the ground truth trajectory
        self.gt_trajectory = []

        # Create publisher for publishing the velocity commands
        self.vel_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)

        # Save passed parameters
        self.path = path
        self.wf = wf
        self.ws = ws
        self.num_path = num_path

        # Create a subscriber for the ground truth
        self.odom_sub = self.create_subscription(Odometry,
            "odom",
            self.odom_callback,
            100)

        # Create a QoS profile for subscribing
        # to ground truth odometry published by Gazebo
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Logging counter for Ground Truth
        self.gt_logging_counter = 0

        # Create a subscriber for the ground truth
        # and use the profile previously created.
        self.gt_sub = self.create_subscription(Odometry,
            "sim_ground_truth_pose",
            self.gt_callback,
            qos_profile
            )

        # Undocking from the station
        self.undock_action_client = ActionClient(self, Undock, '/undock')

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

            # display (x, y, theta) on the terminal
            self.get_logger().info("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))

        # get velocity
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z

    def smooth(self, path, alpha=1, weight_follow=0.5, weight_smooth=0.1, tolerance=0.000001):
        """
        Smooth the trajectory here.
        """
        newpath = deepcopy(path)
        change = float('inf')
        while change > tolerance:
            change = 0
            for i in range(1, len(path) - 1):
                for j in range(2): # x and y coordinates
                    aux = newpath[i][j]
                    newpath[i][j] += alpha * (
                        weight_follow * (path[i][j] - newpath[i][j]) +
                        weight_smooth * (newpath[i-1][j] + newpath[i+1][j] - 2.0 * newpath[i][j])
                    )
                    change += abs(aux - newpath[i][j])
        return newpath

    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        """
        Polynomial timescaling for 3rd order. Returns the matrix coefficients.

        @param p_start - Position of the start point.
        @param v_start - Velocity of the start point.
        @param p_end - Position of the end point.
        @param v_end - Velocity of the end point.
        @param T - the desired time to complete this segment of trajectory (in second).
        @return the coefficients of this polynomial
        """
        # Formulate T matrix for 3rd order (position and velocity at t=0 and t=T)
        # alpha_0 + alpha_1*t + alpha_2*t^2 + alpha_3*t^3
        # derivative: alpha_1 + 2*alpha_2*t + 3*alpha_3*t^2
        T_mat = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [1, T, T**2, T**3],
            [0, 1, 2*T, 3*T**2]
        ])
        
        # Solving for X constraints: [p_start, v_start, p_end, v_end]^T
        x_vec = np.array([p_start[0], v_start[0], p_end[0], v_end[0]])
        y_vec = np.array([p_start[1], v_start[1], p_end[1], v_end[1]])
        
        # Multiply by T^-1
        T_inv = np.linalg.inv(T_mat)
        alpha_x = T_inv @ x_vec.T
        alpha_y = T_inv @ y_vec.T
        
        return alpha_x, alpha_y

    def evaluate_polynomial(self, alpha_x, alpha_y, t):
        pos_x = alpha_x[0] + alpha_x[1]*t + alpha_x[2]*t**2 + alpha_x[3]*t**3
        pos_y = alpha_y[0] + alpha_y[1]*t + alpha_y[2]*t**2 + alpha_y[3]*t**3
        
        vel_x = alpha_x[1] + 2*alpha_x[2]*t + 3*alpha_x[3]*t**2
        vel_y = alpha_y[1] + 2*alpha_y[2]*t + 3*alpha_y[3]*t**2
        
        acc_x = 2*alpha_x[2] + 6*alpha_x[3]*t
        acc_y = 2*alpha_y[2] + 6*alpha_y[3]*t
        
        return pos_x, pos_y, vel_x, vel_y, acc_x, acc_y

    def move_to_point(self, current_waypoint, next_waypoint, v_start, v_end, T_interval):
        """
        Generate polynomial trajectory and move to next_waypoint using 
        non-linear unicycle trajectory tracking control law.
        """
        
        alpha_x, alpha_y = self.polynomial_time_scaling_3rd_order(current_waypoint, v_start, next_waypoint, v_end, T_interval)
        
        # Controller gains
        k_x = 1.0  
        k_y = 5.0
        k_theta = 2.0

        vel = TwistStamped()
        
        # To avoid being stuck in a single polynomial command forever,
        # iterate discretely.
        t = 0.0
        
        while rclpy.ok():
            # If we exceed polynomial defined time interval T slightly, snap to T
            if t > T_interval:
                t = T_interval
                
            # Get theoretically desired position and velocity
            x_r, y_r, vx_r, vy_r, ax_r, ay_r = self.evaluate_polynomial(alpha_x, alpha_y, t)
            
            # Extract theoretical heading and velocity speed
            v_r = math.sqrt(vx_r**2 + vy_r**2)
            theta_r = math.atan2(vy_r, vx_r)
            
            # Approximated omgea_r
            if v_r > 1e-3:
                omega_r = (vx_r * ay_r - vy_r * ax_r) / (v_r**2)
            else:
                omega_r = 0.0 
            
            # Calculate global errors
            dx = x_r - self.pose.x
            dy = y_r - self.pose.y
            dtheta = theta_r - self.pose.theta
            
            # Wrap angle
            while dtheta > math.pi: dtheta -= 2 * math.pi
            while dtheta < -math.pi: dtheta += 2 * math.pi
            
            # Transform errors to robot's LOCAL frame
            e_x = math.cos(self.pose.theta) * dx + math.sin(self.pose.theta) * dy
            e_y = -math.sin(self.pose.theta) * dx + math.cos(self.pose.theta) * dy
            e_theta = dtheta
            
            # Canonical unicycle trajectory tracking control law
            v_cmd = v_r * math.cos(e_theta) + k_x * e_x
            w_cmd = omega_r + k_y * v_r * e_y + k_theta * e_theta
            # NOTE: We use k_theta * e_theta without scaling uniformly by v_r 
            # to guarantee stable steering adjustments even when velocity approaches 0

            # Cap values safely based on physical robot limits
            # limit linear strictly to prevent driving backwards rapidly over pure tracking deviation
            if abs(e_theta) > math.pi / 2:
                v_cmd = max(0.0, v_cmd)

            vel.twist.linear.x = float(min(max(v_cmd, -0.25), 0.25))
            vel.twist.angular.z = float(min(max(w_cmd, -1.0), 1.0))
            self.vel_pub.publish(vel)
            
            if t >= T_interval:
                break
                
            t += 0.05
            time.sleep(0.05)

    def gt_callback(self, msg):
        """
        Callback function for ground truth pose.

        @param msg - Message passed by Gazebo.
        """
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(quarternion)

        self.gt_logging_counter += 1
        if self.gt_logging_counter % 10 == 0:
            self.gt_logging_counter = 0
            self.gt_trajectory.append([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.get_logger().info(f"GT: ({msg.pose.pose.position.x:.3f}, \
                                   {msg.pose.pose.position.y:.3f}, {yaw:.3f})")

    def run(self):
        """
        Run function.
        """

        self.get_logger().info('Running Polynomial Timescaling...')

        # Move between waypoints using polynomial timescaling
        self.get_logger().info('Smoothing path...')
        smooth_waypoints = self.smooth(self.path, alpha=1.0, weight_follow=self.wf, weight_smooth=self.ws)
        self.smoothed_path = deepcopy(smooth_waypoints)
        self.get_logger().info('Path smoothed. Executing Timeshaped Tracking...')
        
        waypoints = smooth_waypoints
        
        velocity_avg = 0.2
        
        for i in range(len(waypoints)-1):
            curr_pt = waypoints[i]
            next_pt = waypoints[i+1]
            
            dist = math.hypot(next_pt[0] - curr_pt[0], next_pt[1] - curr_pt[1])
            if dist < 0.01:
                continue
                
            T_static = max(dist / velocity_avg, 0.1)
            
            # Calculate starting velocity using Catmull-Rom Tangents for smooth corner spline curvature 
            if i == 0:
                v_start = [0.0, 0.0]
            else:
                prev_pt = waypoints[i-1]
                v_start_dir = [next_pt[0] - prev_pt[0], next_pt[1] - prev_pt[1]]
                mag = math.hypot(v_start_dir[0], v_start_dir[1])
                v_start = [velocity_avg * v_start_dir[0] / mag, velocity_avg * v_start_dir[1] / mag] if mag > 1e-4 else [0.0, 0.0]
                
            # Calculate ending velocity
            if i == len(waypoints) - 2:
                v_end = [0.0, 0.0]
            else:
                next_next_pt = waypoints[i+2]
                v_end_dir = [next_next_pt[0] - curr_pt[0], next_next_pt[1] - curr_pt[1]]
                mag = math.hypot(v_end_dir[0], v_end_dir[1])
                v_end = [velocity_avg * v_end_dir[0] / mag, velocity_avg * v_end_dir[1] / mag] if mag > 1e-4 else [0.0, 0.0]
                
            # Move the boundary constraint parameters to our tracking function over dynamic interval T
            self.move_to_point(curr_pt, next_pt, v_start, v_end, T_static)
        
        # Ensure safely halted
        vel = TwistStamped()
        self.vel_pub.publish(vel)

    def save_trajectory(self):
        """
        Save trajectory to disk.
        """
        np.savetxt(f"trajectory_{self.num_path}.csv", np.array(self.trajectory), fmt='%f', delimiter=',')
        np.savetxt(f"trajectory_ground_truth_{self.num_path}.csv", np.array(self.gt_trajectory), fmt='%f', delimiter=',')
        if hasattr(self, 'smoothed_path'):
            np.savetxt(f"smoothed_path_{self.num_path}.csv", np.array(self.smoothed_path), fmt='%f', delimiter=',')

def polynomial_timescaling_entry_point_function(args=None):
    """
    Entry point function for the Polynomial Timescaling node.

    @param args - Arguments that need to be passed to ROS.
    """

    # Initialize the client library - this is required
    rclpy.init(args=args)

    #path1
    path1 = [[0,0], [0,1], [0,2], [0,3], [0,4], [1,4], [2,4], [2,3], [2,2], [3,2], [4,2], [4,1], [4,0]]

    #path2
    path2 = [[0, 0], [0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],\
                        [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],\
                        [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]

    # Create the publisher node
    # turtlebot_node = Turtlebot(path1, 1, wf=0.1, ws=0.1)
    turtlebot_node = Turtlebot(path2, 2, wf=0.1, ws=0.1)

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
        turtlebot_node.get_logger().info("Keyboard Interrupt.")
    finally:
        turtlebot_node.save_trajectory()

    # Destroy the node explicitly
    turtlebot_node.destroy_node()

    # Shut down this context
    rclpy.shutdown()

if __name__ == '__main__':
    polynomial_timescaling_entry_point_function()