# 2D Occupancy Grid Mapping from Scratch (ROS)

## 📌 Project Overview
This project implements a 2D mapping system that allows a mobile robot to build a map of an unknown environment. Unlike standard SLAM packages, this solution uses a custom Python-based mapping node to process raw Odometry and Laser Scan data into a global occupancy grid.

## 🛠️ Technical Workflow
1. **Data Acquisition:** Subscribes to `/scan` (LaserScan) and `/odom` (Odometry) topics.
2. **Coordinate Transformation:** Performs rotation and translation of local sensor hits into the global world frame using robot yaw and position.
3. **Grid Management:** Subdivides the environment into fixed-size cells, marking them as occupied (1.0) based on real-time sensor feedback.
4. **Visualization:** Publishes a `nav_msgs/OccupancyGrid` to be rendered in RViz.

## 🚀 Key Features
- **Custom Mapping Logic:** Hand-coded coordinate transformations using NumPy and Math libraries.
- **Real-time Processing:** Incremental map updates as the robot explores via teleoperation.
- **Simulation Environment:** Validated in Gazebo with a TurtleBot3 Burger robot.
