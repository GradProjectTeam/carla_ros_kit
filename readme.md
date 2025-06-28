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
- **Configurable Environment**: Easily change map and weather through configuration variables

## Project Structure
```
.
├── CARLA/
│   └── Sensors/
│       ├── Gui_Control.py          # Main GUI control interface
│       ├── Camera.py               # CARLA camera sensor
│       ├── vehicle_control_client.py # Client for vehicle control
│       ├── vehicle_control_server.py # Server for vehicle control
│       └── four_sensors_with_pygame.py # Combined sensors with pygame
├── Assets/                         # Screenshots and other assets
├── start_radar_visualization.sh    # Script to start radar visualization
├── debug_radar_visualization.sh    # Script to debug radar visualization
├── .gitignore                      # Git ignore configuration
└── readme.md                       # This file
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

### Configurable Environment
The simulation environment can be easily configured by changing variables at the top of the `Gui_Control.py` file:

#### Map Selection
```python
# Available options: 'Town01' to 'Town07', 'Town10HD', 'Town11', 'Town12'
TOWN_MAP = 'Town04'  # Set this to change the map
```

#### Weather Presets
```python
# Available options include:
# - Noon conditions: 'ClearNoon', 'CloudyNoon', 'WetNoon', 'WetCloudyNoon', etc.
# - Sunset conditions: 'ClearSunset', 'CloudySunset', 'WetSunset', etc.
WEATHER_PRESET = 'ClearNoon'  # Set this to change the weather
```

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

2. Run the GUI Control Interface:
```bash
# Set Python path for CARLA
export PYTHONPATH=$PYTHONPATH:$PWD/CARLA/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg

# Run GUI Control script
cd CARLA/Sensors
python3.7 Gui_Control.py
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

### Changing Maps and Weather

To change the simulation environment:

1. Open `Gui_Control.py` in a text editor
2. Find the configuration variables at the top of the file
3. Change `TOWN_MAP` to select a different map (e.g., 'Town01', 'Town04', 'Town10HD')
4. Change `WEATHER_PRESET` to select different weather conditions (e.g., 'ClearNoon', 'RainyNoon', 'CloudySunset')
5. Save the file and run the script

The system will automatically load your selected map and apply the weather preset when started.

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

3. **"No sensors visible"**:
   - Verify sensor is toggled ON in GUI
   - Check TCP connection
   - Confirm port settings

4. **"Map or weather not changing"**:
   - Verify you've edited the correct configuration variables
   - Check for typos in map or weather preset names
   - Ensure CARLA has the requested map installed

## Authors
- Shishtawy
- Hendy

## Project by:
TechZ

## License
This project is licensed under the MIT License - see the LICENSE file for details.