# CARLA ROS2 Sensor Integration Kit

This project integrates CARLA simulator's sensors (LIDAR, RADAR, IMU, Camera) with ROS2, using TCP socket connections to bridge between the CARLA Python API and ROS2.

![CARLA GUI Control Panel](Assets/Screenshot%20from%202025-06-28%2000-07-43.png)

## System Architecture

### Core Components
```
┌─────────────────┐     ┌──────────────────┐     ┌────────────────┐
│  CARLA Server   │◄────┤  Sensor Manager  │────►│   ROS2 Nodes   │
└─────────────────┘     └──────────────────┘     └────────────────┘
                              │
                        ┌─────┴─────┐
                        │ TCP Bridge │
                        └───────────┘
```

### Sensor Data Flow
```
┌──────────┐    ┌────────────┐    ┌──────────────┐    ┌───────────┐
│ Sensors  │───►│ Data Queue │───►│ TCP Sockets  │───►│ ROS2 Msgs │
└──────────┘    └────────────┘    └──────────────┘    └───────────┘
   │  │  │
   │  │  └─► Camera (RGB)
   │  └────► RADAR (Object Detection)
   └───────► LIDAR (Point Cloud)
```

## Features

### Sensor Suite
- **LIDAR System**: High-precision 3D point cloud generation
- **RADAR System**: Object detection and velocity tracking
- **IMU Sensor**: Vehicle orientation and acceleration data
- **Camera System**: RGB image capture with configurable resolution
- **Waypoint Navigation**: Dynamic path planning and visualization

### Vehicle Control
- Advanced throttle management with smooth transitions
- Precise steering control with realistic physics
- Integrated brake system with variable force
- Automatic gear management system

### Traffic Management
- AI-driven traffic vehicle spawning
- Autonomous vehicle behavior control
- Dynamic traffic density adjustment
- Vehicle roaming mode for realistic city simulation

### Environment Control
- Dynamic weather system with multiple presets
- Time of day manipulation
- Multiple town maps support
- Real-time environment modification

## Project Structure
```
carla_ros_kit/
├── CARLA/
│   └── Sensors/
│       ├── Gui_Control.py          # Main control interface
│       │   ├── CARLASetup         # CARLA environment initialization
│       │   ├── SensorManager      # Sensor handling and data processing
│       │   ├── TrafficManager     # AI traffic control
│       │   └── CarlaControl       # Main GUI and vehicle control
│       ├── Camera.py              # Camera sensor implementation
│       └── vehicle_control.py     # Vehicle control system
├── ROS2/
│   └── carla_bridge/
│       ├── sensor_nodes/          # ROS2 sensor interface nodes
│       └── control_nodes/         # Vehicle control nodes
├── Scripts/
│   ├── start_visualization.sh     # Visualization startup
│   └── debug_tools.sh            # Debugging utilities
└── Assets/                       # Resources and documentation
```

## Technical Details

### TCP Socket Configuration
| Sensor/System | Port  | Data Format          | Update Rate |
|--------------|-------|----------------------|-------------|
| LIDAR        | 12349 | Point Cloud (Binary) | 10 Hz       |
| RADAR        | 12347 | Detection Array      | 20 Hz       |
| IMU          | 12341 | Vector3 (Float32)    | 100 Hz      |
| Camera       | 12342 | RGB Image (JPEG)     | 30 Hz       |
| Waypoint     | 12343 | Path Array           | 5 Hz        |
| Control      | 12344 | Command Structure    | 50 Hz       |

### Control System

#### Keyboard Controls
- **Vehicle Operation**
  - W/↑: Forward acceleration
  - S/↓: Reverse
  - A/←: Left steering
  - D/→: Right steering
  - SPACE: Brake
  - R: Toggle reverse gear
  - ESC: Exit application

- **Sensor Management**
  - 1: LIDAR toggle
  - 2: RADAR toggle
  - 3: IMU toggle
  - 4: Camera toggle
  - 5: Waypoint toggle
  - 6: Velocity data toggle

- **Traffic Control**
  - T: Spawn single vehicle
  - U: Add random vehicles
  - Y: Remove last vehicle
  - M: Toggle roaming mode

- **Interface**
  - H: Help display toggle

### Configuration Parameters

#### Environment Settings
```python
# Map Selection
TOWN_MAP = 'Town04'  # Options: Town01-07, Town10HD, Town11, Town12

# Weather Configuration
WEATHER_PRESET = 'ClearNoon'  # Options: ClearNoon, CloudyNoon, WetNoon, etc.

# Sensor Configuration
LIDAR_POINTS = 100000  # Points per second
RADAR_RANGE = 100.0    # Detection range in meters
CAMERA_RES = (800, 600)  # Resolution in pixels
```

## Installation Guide

### Prerequisites
- CARLA 0.9.12+
- Python 3.7+
- ROS2 Rolling/Humble
- Required Python packages:
  ```
  numpy>=1.19.0
  pygame>=2.0.0
  rclpy>=1.0.0
  sensor_msgs>=2.0.0
  ```

### Setup Steps

1. **CARLA Installation**:
   ```bash
   # Extract CARLA
   tar -xf CARLA_0.9.12.tar.gz
   
   # Set Python path
   export PYTHONPATH=$PYTHONPATH:$PWD/CARLA/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg
   ```

2. **ROS2 Workspace Setup**:
   ```bash
   # Create workspace
   mkdir -p ~/carla_ws/src
   cd ~/carla_ws/src
   
   # Clone repository
   git clone https://github.com/your-repo/carla_ros_kit.git
   
   # Build workspace
   cd ~/carla_ws
   colcon build
   ```

## Usage Instructions

1. **Start CARLA Server**:
   ```bash
   ./CarlaUE4.sh
   ```

2. **Launch Sensor Interface**:
   ```bash
   python3 CARLA/Sensors/Gui_Control.py
   ```

3. **Run ROS2 Bridge**:
   ```bash
   ros2 launch carla_bridge sensor_bridge.launch.py
   ```

## Troubleshooting Guide

### Common Issues and Solutions

1. **Sensor Connection Failures**
   - Check port availability
   - Verify CARLA server status
   - Confirm Python path settings

2. **Performance Issues**
   - Reduce sensor update rates
   - Lower LIDAR point count
   - Adjust camera resolution

3. **Vehicle Control Problems**
   - Verify keyboard input settings
   - Check control server connection
   - Confirm physics settings

## Development Team

### Project Leads
- Shishtawy ([shishtawylearning@gmail.com](mailto:shishtawylearning@gmail.com))
- Hendy ([mustafahendy@outlook.com](mailto:mustafahendy@outlook.com))

## Project by:
TechZ

## License
This project is licensed under the MIT License - see the LICENSE file for details.