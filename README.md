# Simple 2D LiDAR Odometry

[![Video Link](https://i.postimg.cc/htnHxkH7/2023-10-28-5-35-55.png)](https://youtu.be/9CWUMSL1x-A)
## Overview

`Simple-2D-LiDAR-Odometry` is a straightforward implementation of 2D LiDAR-based odometry using the Generalized Iterative Closest Point (GICP) algorithm. It has been tested and verified on ROS 2 Humble. The primary libraries utilized for this project are Eigen and Point Cloud Library (PCL).

## Prerequisites

- ROS 2 Humble
- Eigen
- Point Cloud Library (PCL)

## Installation

1. First, ensure you have a working ROS 2 Humble installation. If not, follow the [official installation guide](https://docs.ros.org/en/humble/Installation.html).

2. Install the required dependencies:

```bash
sudo apt-get install libeigen3-dev libpcl-dev
```

3. Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src/
git clone https://github.com/dawan0111/Simple-2D-LiDAR-Odometry.git
```

4. Build the package:

```bash
cd ~/ros2_ws/
colcon build --packages-select simple_2d_lidar_odometry
```

## Usage

After building the package, you can run the LiDAR odometry node with:

```bash
ros2 run simple_2d_lidar_odometry lidar_odometry_node
```

Ensure your LiDAR sensor is correctly set up and publishing data to the appropriate topic.

## Implementation Details

The core of this odometry solution is the GICP algorithm, which aligns consecutive LiDAR scans to estimate the robot's motion. By using both Eigen and PCL, the implementation efficiently handles point cloud data and performs matrix operations to compute the odometry.
