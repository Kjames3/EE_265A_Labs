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

        # Initial conditions
        self.previous_point = [0, 0]
        self.previous_velocity = [0, 0]
        self.vel_steering = 0.5
        self.vel = TwistStamped()

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
        if self.logging_counter == 10:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory

            # display (x, y, theta) on the terminal
            self.get_logger().info("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))

    def smooth(self, path, alpha=1, weight_follow = 0.5, weight_smooth = 0.1, tolerance = 0.000001):
        """
        Smooth the trajectory here.

        @param path: Original path
        @param alpha: Description
        @param weight_follow: Description
        @param weight_smooth: Description
        @param tolerance: Description
        """

        # Make a copy of the original path 
        newpath = deepcopy(path)

        # Large value
        change = float('inf')

        while change > tolerance:
            # Iterative update (repeat until convergence)
            change  = 0

            for i in range(1, len(path) - 1):
                for j in range(2): # x and y coordinates
                    aux = newpath[i][j]
                    newpath[i][j] += alpha * (
                        weight_follow * (path[i][j] - newpath[i][j]) +
                        weight_smooth * (newpath[i-1][j] + newpath[i+1][j] - 2.0 * newpath[i][j])
                    )
                    change += abs(aux - newpath[i][j])

        # Return smoothed path
        return newpath

    def move_to_point(self, current_waypoint, next_waypoint):
        """
        Move to the next waypoint using continuous stable non-linear point-to-point control.
        """
        k_v = 1.0
        k_w = 2.0
        
        while rclpy.ok():
            dx = next_waypoint[0] - self.pose.x
            dy = next_waypoint[1] - self.pose.y
            dist = math.hypot(dx, dy)
            
            # Reach condition: transitions to the next point within ~0.2m of the waypoint
            # which facilitates smoother cornering along the ~0.1-0.3m curved offset 
            if dist < 0.2: 
                break
                
            desired_theta = math.atan2(dy, dx)
            e_theta = desired_theta - self.pose.theta
            
            while e_theta > math.pi: e_theta -= 2 * math.pi
            while e_theta < -math.pi: e_theta += 2 * math.pi
            
            # Prevent moving forward if heading is too far off to avoid wide arcs (Oscillation at start)
            if abs(e_theta) > math.pi / 2:
                v_cmd = 0.0
                w_cmd = 1.0 if e_theta > 0 else -1.0
            else:
                # Non-linear unicycle controller: scales forward velocity by cos(error)
                # smoothly decelerates as distance approaches 0
                v_cmd = k_v * dist * math.cos(e_theta)
                w_cmd = k_w * e_theta
                
            # Limit velocities
            v_cmd = min(max(v_cmd, -0.2), 0.2)
            w_cmd = min(max(w_cmd, -1.0), 1.0)
            
            vel = TwistStamped()
            vel.twist.linear.x = float(v_cmd)
            vel.twist.angular.z = float(w_cmd)
            self.vel_pub.publish(vel)
            
            time.sleep(0.01)

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
        self.get_logger().info('Running Gradient Descent (Greedy Replanner)...')

        original_waypoints = deepcopy(self.path)
        
        # Save the very first ideal smoothed path for visualization
        self.smoothed_path = self.smooth(deepcopy(original_waypoints), alpha=1.0, weight_follow=self.wf, weight_smooth=self.ws)
        
        current_idx = 1
        while current_idx < len(original_waypoints):
            # Create a localized path for replanning: current pose + remaining original waypoints
            local_path = [[self.pose.x, self.pose.y]] + original_waypoints[current_idx:]
            
            # Smooth this local path
            smoothed_local = self.smooth(local_path, alpha=1.0, weight_follow=self.wf, weight_smooth=self.ws)
            
            # Move to the first upcoming waypoint in the smoothly replanned path
            self.move_to_point(smoothed_local[0], smoothed_local[1])
            
            current_idx += 1

        # Stop robot gracefully at goal
        vel = TwistStamped()
        self.vel_pub.publish(vel)
        self.get_logger().info('Reached goal!')

    def save_trajectory(self):
        """
        Save trajectory to disk.
        """
        np.savetxt(f"trajectory_{self.num_path}.csv", np.array(self.trajectory), fmt='%f', delimiter=',')
        np.savetxt(f"trajectory_ground_truth_{self.num_path}.csv", np.array(self.gt_trajectory), fmt='%f', delimiter=',')
        if hasattr(self, 'smoothed_path'):
            np.savetxt(f"smoothed_path_{self.num_path}.csv", np.array(self.smoothed_path), fmt='%f', delimiter=',')

    def printpaths(self, path,newpath):
        """
        Print paths to the console. This will show the initial path before smoothing.

        @param path: Original path.
        @param newpath: New path.
        """

        for old, new in zip(path, newpath):
            self.get_logger().info(
                '['+ ', '.join('%.3f'%x for x in old) + \
                '] -> ['+ ', '.join('%.3f'%x for x in new) +']')

    def plot(self, path, newpath):
        """
        Plotting function.

        @param path - Original path.
        @patam newpath - New path.
        """

        x =  [x for [x, y] in path]
        y =  [y for [ax, y] in path]

        # This plots the smooth path overlayed on the original 
        plt.plot(x, y, color='r', marker='o')
        x2 =  [x for [x, y] in newpath]
        y2 =  [y for [ax, y] in newpath]
        plt.plot(x2, y2, color='b', marker='o')
        plt.xlim(-1, 3)
        plt.ylim(-1, 3)
        plt.show()

def gradient_descent_entry_point_function(args=None):
    """
    Entry point function for the Gradient Descent node.

    @param args - Arguments that need to be passed to ROS.
    """

    # Initialize the client library - this is required
    rclpy.init(args=args)

    #path1
    path1 = [[0,0], [0,1], [0,2], [0,3], [0,4], [1,4], [2,4], [2,3], [2,2], [3,2], [4,2], [4,1], [4,0]]

    #path2
    path2 = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],
                        [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],
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
    gradient_descent_entry_point_function()