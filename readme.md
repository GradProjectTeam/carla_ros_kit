# CARLA ROS2 LIDAR Integration

This project integrates CARLA simulator's LIDAR sensor with ROS2, using a socket connection to bridge between Python 3.5 (CARLA) and Python 3.12 (ROS2 Rolling).

## Project Structure
```
.
├── CARLA/
│   ├── Sensors/
│   │   └── Lidar.py         # CARLA LIDAR sensor (Python 3.5)
│   ├── scripts/
│   ├── CARLA_0.9.8/        # CARLA simulator
│   ├── AdditionalMaps_0.9.8.tar.gz
│   ├── CARLA_0.9.8.tar.gz
│   ├── carla-0.9.5-py3.5-linux-x86_64.egg
│   ├── carla-0.9.8-py3.5-linux-x86_64.egg  # CARLA Python API
│   └── Python-3.5.10/      # Python 3.5 installation
├── build/                  # ROS2 build directory
├── install/                # ROS2 install directory
├── log/                    # ROS2 log directory
├── my_python_pkg/         # ROS2 package
│   ├── my_python_pkg/
│   │   ├── __init__.py
│   │   ├── lidar_processor_node.py
│   │   ├── pub_node.py
│   │   └── sensor_reader_node.py
│   ├── setup.py
│   └── package.xml
└── readme.md
```

## Prerequisites

### CARLA Environment (Python 3.5)
- CARLA 0.9.8
- Python 3.5.10 (included)
- numpy
- socket

### ROS2 Environment (Python 3.12)
- ROS2 Rolling
- Python 3.12
- numpy
- sensor_msgs
- rclpy

## Installation

1. Setup CARLA:
```bash
# Extract CARLA if not already done
cd CARLA
tar -xf CARLA_0.9.8.tar.gz

# Extract additional maps (optional)
tar -xf AdditionalMaps_0.9.8.tar.gz

# Install CARLA Python API
export PYTHONPATH=$PYTHONPATH:$PWD/carla-0.9.8-py3.5-linux-x86_64.egg
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
cd CARLA/CARLA_0.9.8
./CarlaUE4.sh
```

2. Start LIDAR Sensor (Terminal 2):
```bash
# Set Python path for CARLA
export PYTHONPATH=$PYTHONPATH:$PWD/CARLA/carla-0.9.8-py3.5-linux-x86_64.egg

# Run LIDAR script
cd CARLA/Sensors
python3.5 Lidar.py
```

3. Start ROS2 LIDAR Processor (Terminal 3):
```bash
# Source ROS2
source /opt/ros/rolling/setup.bash
source install/setup.bash

# Run processor node
ros2 run my_python_pkg lidar_processor
```

4. Visualize (Terminal 4):
```bash
rviz2
```

## Socket Communication

The system uses TCP sockets to bridge between Python versions:

### CARLA Side (Python 3.5)
```python
# Lidar.py
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 12345))
server_socket.listen(1)
```

### ROS2 Side (Python 3.12)
```python
# lidar_processor_node.py
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('localhost', 12345))
```

## Troubleshooting

### Python Path Issues
```bash
# Set PYTHONPATH for CARLA
export PYTHONPATH=$PYTHONPATH:$PWD/CARLA/carla-0.9.8-py3.5-linux-x86_64.egg

# Check CARLA installation
python3.5 -c "import carla; print(carla.__file__)"
```

### Socket Issues
```bash
# Check if port is in use
sudo netstat -tulpn | grep 12345

# Test socket connection
nc -zv localhost 12345
```

### Common Problems
1. "ImportError: No module named 'carla'":
   - Check PYTHONPATH setting
   - Verify carla egg file location
   - Ensure using Python 3.5

2. "Connection refused":
   - Ensure CARLA script is running
   - Check port 12345 is available
   - Verify network settings

3. "ROS2 package not found":
   - Verify package is built
   - Check source setup.bash
   - Confirm package name

## Performance Notes
- LIDAR data is transmitted via TCP socket
- Processing is done on ROS2 side
- Visualization through RViz2
- Monitor system resources

## Logs
- ROS2 logs are stored in `log/` directory
- Check CARLA output for simulator issues
- Monitor socket communication errors