# Table Detection (No Camera) and Transport Robot

Autonomous table detection and transport system using laser-based perception for warehouse environments without camera dependency.

## Overview

This project implements an autonomous robot system that identifies and transports tables using laser segmentation and robotic manipulation. The system combines SLAM navigation, Jump-Distance Segmentation, Euclidean clustering, and precision elevator control to operate in complete darkness or low-light warehouse conditions.


## Demo

### Real Robot Testing
<p align="center">
  <img src="https://i.imgflip.com/a49eoa.gif" alt="Real Robot Demo">
</p>

*Turtlebot4 with Elevator Mechanism robot detecting and transporting tables in real café environment using LiDAR-only perception*

### Simulation Testing
<p align="center">
  <img src="https://i.imgflip.com/a49dvt.gif" alt="Simulation Demo">
</p>

*Robot using LiDAR to detect trash tables in café environment and transporting them to designated trash room*

**For complete project presentation**: [Watch Full 45-min Presentation](https://www.youtube.com/watch?v=GF3cPU2OSH4&t=8175s) - Detailed walkthrough covering system architecture, technical implementation, and extensive test scenarios

## Key Features

- **Jump-Distance Segmentation**: Implements Lee (relative distance), Dietmayer (noise-aware), and Santos (angular compensation) methods
- **Table Leg Identification**: Real-time detection using voxel-grid downsampling and statistical outlier removal
- **Precision Approach**: PID-controller based alignment with ±2cm accuracy for table attachment
- **Dynamic Footprint Adjustment**: Collision boundary updates when carrying tables
- **Multi-Table Support**: Sequential transport of multiple tables with state machine control
- **Dark Environment Capable**: Operates in complete darkness (0 lux)
- **Simulation-First Development**: Full Gazebo environment for safe testing

## Performance Metrics

| Metric | Value | Conditions |
|--------|-------|------------|
| Detection Range | 0.5m - 3.5m | Indoor warehouse environment |
| Approach Precision | ±2cm | P-controller final alignment |
| Navigation Accuracy | ±5cm | AMCL localization |
| Success Rate | 95% | 50+ transport cycles |
| Cycle Time | 3 minutes | Detection to placement |
| Operating Light Level | 0 lux | Complete darkness capable |
| Table Weight Capacity | 30kg | Standard café tables |

## Technical Stack

- **Platform**: Turtlebot4 with Elevator Mechanism Robot
- **Framework**: ROS2 Humble
- **Simulation**: Gazebo Classic
- **SLAM**: Cartographer (2D LiDAR-based)
- **Localization**: AMCL with particle filters
- **Navigation**: Nav2 stack with DWB controller
- **Perception**: PCL, Jump-Distance Segmentation + Euclidean clustering
- **Control**: P-controller for approach alignment
- **State Machine**: Python-based sequential execution

## Installation

### Prerequisites
```bash
# ROS2 Humble
sudo apt install ros-humble-desktop-full

# Navigation and SLAM
sudo apt install ros-humble-navigation2 ros-humble-cartographer-ros

# Additional dependencies
sudo apt install ros-humble-tf2-geometry-msgs ros-humble-nav2-util

# Simulation
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Build from Source
```bash
# Clone repository
git clone https://github.com/username/table_transport_robot.git
cd table_transport_robot

# Download warehouse models (stored separately due to size)
./download_models.sh  # Downloads warehouse world from Google Drive

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Launch Simulation Environment
```bash
# Start Gazebo with warehouse world
ros2 launch warehouse_sim warehouse_with_tables.launch.py

# Launch robot with all sensors
ros2 launch Turtlebot4 with Elevator Mechanism_robot robot_spawn.launch.py
```

### Run Autonomous Transport
```bash
# Start SLAM and navigation
ros2 launch cartographer_slam cartographer.launch.py
ros2 launch path_planner_server navigation.launch.py

# Launch perception pipeline
ros2 launch laser_segmentation segmentation.launch.py

# Start table detection and approach
ros2 launch attach_table attach_server.launch.py

# Enable transport mode
ros2 run nav2_application robot_patrol.py

# Monitor detection (optional)
ros2 topic echo /segments/centroids

# Begin autonomous transport behavior
ros2 service call /approach_table attach_table/srv/GoToLoading "{attach_to_table: true, table_number: 1}"
```

## Repository Structure

```
table_transport_robot/
├── laser_segmentation/      # Perception algorithms
│   ├── src/
│   │   ├── segmentationJumpDistance.cpp    # JDS algorithm
│   │   └── laserSegmentation.cpp           # Main perception node
│   └── params/
├── attach_table/           # Table approach control
│   ├── src/
│   │   └── approach_service_server.cpp     # P-controller implementation
│   └── srv/
├── nav2_application/       # High-level control
│   ├── scripts/
│   │   └── robot_patrol.py                 # State machine controller
│   └── config/
├── warehouse_sim/         # Simulation environment
│   ├── worlds/
│   ├── models/           # [Large files - download separately]
│   └── launch/
├── Turtlebot4 with Elevator Mechanism_robot/            # Turtlebot4 with Elevator Mechanism robot description and control
├── cartographer_slam/    # SLAM configuration
├── localization_server/  # AMCL localization
├── path_planner_server/  # Nav2 configuration
└── slg_msgs/            # Custom message definitions
```

## Technical Implementation

### Jump-Distance Segmentation Pipeline
1. **Point Segmentation**: Adaptive threshold calculation using Dietmayer method
2. **Voxel-Grid Downsampling**: Reduces point density while preserving features
3. **Statistical Outlier Removal**: Filters noise with 0.7 threshold
4. **Segment Merging**: Combines nearby segments within threshold

### Table Detection Pipeline
1. **LiDAR Processing**: Voxel grid downsampling → Statistical outlier removal
2. **Leg Segmentation**: Jump-distance clustering with adaptive thresholds
3. **Leg Pairing**: Euclidean clustering with 0.5m distance threshold
4. **Centroid Calculation**: Compute approach point between detected legs

### Approach Strategy
- **Global Alignment**: Navigate to table vicinity using Nav2
- **Local Approach**: P-controller for precise alignment
- **Final Positioning**: Orientation correction before attachment
- **Attachment Mode**: Elevator activation and footprint update

### Multi-Table Coordination
- **State Machine Control**: Sequential task execution via Python node
- **Table Tracking**: Unique ID assignment for each detected table
- **Transport Planning**: Optimized route between pickup and drop-off points
- **Recovery Behaviors**: Automatic retry on failed attachments

## Contact

**Ritwik Rohan**  
Robotics Engineer | Johns Hopkins MSE '25  
Email: ritwikrohan7@gmail.com  
LinkedIn: [linkedin.com/in/ritwik-rohan](https://linkedin.com/in/ritwik-rohan)

---
