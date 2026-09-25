# 5-DOF Robotic Arm Simulation

A robotic manipulator built entirely from first principles in MATLAB — no pre-built URDF models — 
to explore the mathematical foundations of robot kinematics and motion planning.

## Features

- **Custom DH-Parameter Architecture** — Robot geometry derived directly from Denavit-Hartenberg parameters
- **Dual Inverse Kinematics** — Analytical IK engine for exact servo angle solutions, plus a numerical 
  IK solver adapted to the task-space constraints of a 5-DOF system
- **RRT Path Planning** — Rapidly-exploring Random Tree algorithm for collision-free navigation through 
  a 3D workspace with obstacles
- **Minimum-Jerk Trajectory Smoothing** — Fluid joint motion with minimized mechanical stress, ready 
  for real-world servo deployment
- **Custom 3D Visualization** — Dark-themed simulation environment with dynamic lighting, tracking 
  end-effector trajectory in real time

## Roadmap

- [ ] Port kinematics/planning logic to a ROS 2 (Jazzy) workspace
- [ ] Build custom C++ control nodes
- [ ] Physics-based simulation in Gazebo

## Demo

[Insert video/GIF of path-planning execution]
