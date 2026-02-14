# EE265A - Labs

This repository contains the scripts and implementations for the ROS2 TurtleBot labs.

## Lab 1: Introduction and Teleop
This lab involved setting up the ROS2 Jazzy environment and verifying the TurtleBot 2D simulation. We ensured that the pre-installed environment worked correctly and that the robot could be controlled using teleoperation.

## Lab 2: Open Loop Control
For this lab, we developed the `open_loop.py` script. The core implementation focused on the `run(self)` function, which sends velocity commands to move the robot in an open-loop manner.

## Lab 3: Closed Loop Control
In this lab, we implemented closed-loop control in the `closed_loop.py` script. Similar to Lab 2, we focused on the `run(self)` function to implement feedback control for navigating the robot.

## Lab 4: A* Pathfinding
This lab required implementing the A* search algorithm. We wrote two main scripts:
- `point_a_star.py`: Implementation of A* for point-based pathfinding.
- `orient_a_star.py`: Implementation of A* with orientation considerations.

We also used the provided `test_a_star.py` script to verify and confirm that our A* implementations worked correctly.
