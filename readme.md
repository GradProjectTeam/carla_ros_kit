# CARLA ROS2 Sensor Integration Kit

This project integrates CARLA simulator's sensors (LIDAR, RADAR, IMU, Camera) with ROS2, using TCP socket connections to bridge between the CARLA Python API and ROS2.

![CARLA GUI Control Panel](Assets/Screenshot%20from%202025-06-28%2000-07-43.png)

## Features

- **Multi-Sensor Support**: LIDAR, RADAR, IMU, Camera, and Waypoint data
- **Interactive GUI Control**: Real-time vehicle control with visual feedback
- **Sensor Toggle System**: Enable/disable sensors on demand with keybindings
- **Gradual Throttle Control**: Realistic acceleration and deceleration with smooth transitions
- **Traffic Management**: Spawn and control AI traffic vehicles
- **TCP Socket Communication**: Bridge between CARLA and ROS2
- **Waypoint Navigation**: Generate and visualize waypoints for navigation assistance

## Project Structure
```
.
├── CARLA/
│   ├── Sensors/
│   │   ├── Gui_Control.py  # Main GUI control interface
│   │   ├── IMU.py          # CARLA IMU sensor
│   │   ├── Lidar.py        # CARLA LIDAR sensor
│   │   └── Radar.py        # CARLA RADAR sensor
│   └── scripts/            # Additional CARLA scripts
├── my_python_pkg/          # ROS2 package
│   ├── my_python_pkg/
│   │   ├── __init__.py
│   │   ├── imu_processor_node.py
│   │   ├── lidar_processor_node.py
│   │   ├── radar_processor_node.py
│   │   ├── camera_processor_node.py
│   │   └── waypoint_processor_node.py
│   ├── resource/
│   │   └── my_python_pkg   # Package resources
│   ├── setup.py
│   └── package.xml
├── Assets/                 # Screenshots and other assets
├── .gitignore              # Git ignore configuration
├── RVIZ2.md                # RViz2 setup instructions
└── readme.md               # This file
```

## Controls

### Vehicle Control
- **W/Up Arrow**: Accelerate forward (gradually increases throttle)
- **S/Down Arrow**: Accelerate in reverse (gradually increases reverse throttle)
- **A/Left Arrow**: Steer left
- **D/Right Arrow**: Steer right
- **Space**: Brake
- **ESC**: Exit application

### Sensor Toggle Controls
- **1**: Toggle LIDAR sensor
- **2**: Toggle RADAR sensor
- **3**: Toggle IMU sensor
- **4**: Toggle Camera sensor
- **5**: Toggle Waypoint generation

### Traffic Management
- **T**: Add a traffic vehicle
- **U**: Add random traffic vehicles
- **Y**: Remove last traffic vehicle
- **M**: Toggle roaming mode for traffic vehicles

### UI Controls
- **H**: Toggle help display

## Advanced Features

### Gradual Throttle Control
The vehicle features realistic acceleration and deceleration:
- Throttle gradually increases while acceleration key is held
- Throttle gradually decreases when acceleration key is released
- When changing direction (forward to reverse or vice versa), the vehicle first decelerates to a stop before accelerating in the new direction

### Sensor Management
Each sensor can be toggled independently:
- When toggled on, the sensor is created and data transmission begins
- When toggled off, the sensor is destroyed and data transmission stops
- TCP sockets are created and managed automatically

### Traffic Vehicle System
The system can spawn and manage AI-controlled traffic vehicles:
- Traffic vehicles use CARLA's autopilot system
- Vehicles can be set to roam freely or follow specific paths
- Traffic density can be adjusted by adding or removing vehicles

## Prerequisites

### CARLA Environment
- CARLA 0.9.12
- Python 3.7+
- numpy
- pygame
- socket

### ROS2 Environment
- ROS2 Rolling/Humble
- Python 3.8+
- numpy
- sensor_msgs
- rclpy

## Installation

1. Setup CARLA:
```bash
# Extract CARLA if not already done
cd CARLA
tar -xf CARLA_0.9.12.tar.gz

# Add CARLA Python API to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$PWD/CARLA/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg
```

2. Setup ROS2 Package:
```bash
# Build package
colcon build --packages-select my_python_pkg

# Source workspace
source install/setup.bash
```

## Usage

1. Start CARLA (Terminal 1):
```bash
cd CARLA/CARLA_0.9.12
./CarlaUE4.sh
```

2. Start GUI Control Interface (Terminal 2):
```bash
# Set Python path for CARLA
export PYTHONPATH=$PYTHONPATH:$PWD/CARLA/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg

# Run GUI Control script
cd CARLA/Sensors
python3.7 Gui_Control.py
```

3. Start ROS2 Sensor Processors (Terminal 3):
```bash
# Source ROS2
source /opt/ros/rolling/setup.bash
source install/setup.bash

# Run processor nodes
ros2 launch my_python_pkg all_sensors.launch.py
```

4. Visualize in RViz2 (Terminal 4):
```bash
rviz2 -d config/sensors_config.rviz
```

## TCP Socket Communication

The system uses TCP sockets to bridge between CARLA and ROS2:

| Sensor  | Port  |
|---------|-------|
| LIDAR   | 12349 |
| RADAR   | 12347 |
| IMU     | 12341 |
| Camera  | 12342 |
| Waypoint| 12343 |
| Control | 12344 |

## Troubleshooting

### Common Issues
1. **"ImportError: No module named 'carla'"**:
   - Check PYTHONPATH setting
   - Verify carla egg file location
   - Ensure using compatible Python version

2. **"Connection refused"**:
   - Ensure CARLA script is running
   - Check port availability
   - Verify network settings

3. **"No sensors visible in RViz"**:
   - Verify sensor is toggled ON in GUI
   - Check ROS2 topic subscription
   - Confirm frame_id settings

## Authors
- Shishtawy
- Hendy

## Project by: TechZ

## License
This project is licensed under the MIT License - see the LICENSE file for details.