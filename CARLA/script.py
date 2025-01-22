#! /usr/local/bin/python3.5m
import sys
import glob
import os
import socket
import pickle
import numpy as np
import time
import json
import struct
import math
import random
import asyncio

print("Starting script...")
print("Python version:", sys.version)

try:
    # CARLA setup
    carla_path = '/home/mostafa/ROS2andCarla/CARLA'
    print("Looking for CARLA at:", carla_path)
    carla_eggs = glob.glob('{0}/carla-*{1}.{2}-{3}.egg'.format(
        carla_path,
        sys.version_info.major,
        sys.version_info.minor,
        "win-amd64" if os.name == "nt" else "linux-x86_64"
    ))
    sys.path.append(carla_eggs[0])
    print("Found CARLA egg:", carla_eggs[0])
    
    import carla
    print("CARLA imported successfully")

    # Try importing required packages
    print("Checking required packages...")
    
    print("Importing numpy...")
    import numpy as np
    print("Numpy version:", np.__version__)
    
    print("Importing socket...")
    import socket
    
    print("Importing struct...")
    import struct
    
   
    print("All imports successful!")
    
    # Initialize Pygame
   

    # Connect to CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    print("Connected to CARLA world")

    # Initialize actor list
    actor_list = []

    # Clear all existing vehicles
    for actor in world.get_actors():
        if 'vehicle' in actor.type_id:
            actor.destroy()
    print("Cleared existing vehicles")

    # Setup sockets with better buffer sizes and options
    def create_socket():
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)  # 64KB buffer
        return sock

    lidar_socket = create_socket()
    radar_socket = create_socket()
    imu_socket = create_socket()

    # Bind sockets
    host_ip = '0.0.0.0'  # or '127.0.0.1' for local only
    lidar_socket.bind((host_ip, 12345))
    radar_socket.bind((host_ip, 12346))
    imu_socket.bind((host_ip, 12347))

    for sock in [lidar_socket, radar_socket, imu_socket]:
        sock.listen(1)

    print('Listening on {}:'.format(host_ip))
    print('LIDAR: 12345\nRADAR: 12346\nIMU: 12347')

    # Initialize client connections
    lidar_client = None
    radar_client = None
    imu_client = None

    # Accept connections before starting sensors
    print("Waiting for client connections...")
    try:
        print("Waiting for LIDAR client...")
        lidar_client, addr = lidar_socket.accept()
        print("LIDAR client connected from {}".format(addr))
        
        print("Waiting for RADAR client...")
        radar_client, addr = radar_socket.accept()
        print("RADAR client connected from {}".format(addr))
        
        print("Waiting for IMU client...")
        imu_client, addr = imu_socket.accept()
        print("IMU client connected from {}".format(addr))
        
        print("All clients connected!")
    except Exception as e:
        print("Connection error: {}".format(e))
        sys.exit(1)

    # Pre-allocate numpy arrays for better performance
    MAX_POINTS = 100000
    points_buffer = np.zeros((MAX_POINTS, 4), dtype=np.float32)

    def send_sensor_data(client, data_dict):
        if not client:
            return False
        try:
            # Pack data size and timestamp first
            header = struct.pack('!II', data_dict['size'], data_dict['timestamp'])
            client.sendall(header)
            # Then send the actual data
            client.sendall(data_dict['data'])
            return True
        except Exception as e:
            print('Socket send error: {}'.format(e))
            return False

    def lidar_callback(point_cloud):
        global lidar_client
        if not lidar_client:
            return

        try:
            # Process LIDAR data
            raw_data = np.frombuffer(point_cloud.raw_data, dtype=np.float32)
            points = raw_data.reshape(-1, 4)  # x, y, z, intensity
            
            # Pack data for sending
            data = {
                'size': len(points) * 16,  # 4 float32s per point
                'timestamp': int(point_cloud.timestamp * 1000),  # ms
                'data': points.tobytes()
            }
            
            if not send_sensor_data(lidar_client, data):
                print('LIDAR client disconnected')
                lidar_client = None

        except Exception as e:
            print('LIDAR processing error: {}'.format(e))

    def radar_callback(radar_data):
        global radar_client
        if not radar_client:
            return

        try:
            # Process RADAR data
            raw_data = np.frombuffer(radar_data.raw_data, dtype=np.float32)
            points = raw_data.reshape(-1, 4)  # velocity, azimuth, altitude, depth
            
            # Pack data for sending
            data = {
                'size': len(points) * 16,
                'timestamp': int(radar_data.timestamp * 1000),
                'data': points.tobytes()
            }
            
            if not send_sensor_data(radar_client, data):
                print('RADAR client disconnected')
                radar_client = None

        except Exception as e:
            print('RADAR processing error: {}'.format(e))

    def imu_callback(imu_data):
        global imu_client
        if not imu_client:
            return

        try:
            # Pack IMU data into array
            imu_array = np.array([
                imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z,
                imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z,
                imu_data.compass
            ], dtype=np.float32)
            
            # Pack data for sending
            data = {
                'size': len(imu_array) * 4,  # float32 size
                'timestamp': int(imu_data.timestamp * 1000),
                'data': imu_array.tobytes()
            }
            
            if not send_sensor_data(imu_client, data):
                print('IMU client disconnected')
                imu_client = None

        except Exception as e:
            print('IMU processing error: {}'.format(e))

    # Add status printing
    def print_status():
        status = []
        if lidar_client:
            status.append('LIDAR connected')
        if radar_client:
            status.append('RADAR connected')
        if imu_client:
            status.append('IMU connected')
        print('\rActive connections: {}'.format(' | '.join(status)), end='')

    async def handle_connections():
        global lidar_client, radar_client, imu_client
        while True:
            if not lidar_client:
                lidar_client, _ = await lidar_socket.accept()
                print('LIDAR client connected')
            if not radar_client:
                radar_client, _ = await radar_socket.accept()
                print('RADAR client connected')
            if not imu_client:
                imu_client, _ = await imu_socket.accept()
                print('IMU client connected')
            await asyncio.sleep(0.1)

    # Get spawn points
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = spawn_points[0]
    spawn_point.location.z += 0.5
    
    # Spawn vehicle
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    actor_list.append(vehicle)
    print("\nVehicle spawned successfully")
    print("Vehicle location: x={:.2f}, y={:.2f}, z={:.2f}".format(
        vehicle.get_location().x,
        vehicle.get_location().y,
        vehicle.get_location().z
    ))

    # Add LIDAR sensor with updated configuration
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('channels', '32')  # Standard number of channels
    lidar_bp.set_attribute('points_per_second', '10000')  # Reduced for stability
    lidar_bp.set_attribute('rotation_frequency', '50')  # Standard rotation speed
    lidar_bp.set_attribute('range', '500.0')  # Reduced range for testing
    lidar_bp.set_attribute('upper_fov', '0.0')  # Reduced upper FOV
    lidar_bp.set_attribute('lower_fov', '-20.0')  # Standard lower FOV

    # Print LIDAR configuration
    print("\nLIDAR Configuration:")
    print("- Channels:", lidar_bp.get_attribute('channels').as_int())
    print("- Points per second:", lidar_bp.get_attribute('points_per_second').as_int())
    print("- Range:", lidar_bp.get_attribute('range').as_float(), "meters")
    print("- Upper FOV:", lidar_bp.get_attribute('upper_fov').as_float(), "degrees")
    print("- Lower FOV:", lidar_bp.get_attribute('lower_fov').as_float(), "degrees")
    print("- Rotation frequency:", lidar_bp.get_attribute('rotation_frequency').as_float(), "Hz")
    print("- Horizontal FOV: 360 degrees (fixed)")

    # Spawn LIDAR with adjusted position (moved forward and up for better visibility)
    lidar_location = carla.Transform(
        carla.Location(x=1.5, z=2.0),  # Moved forward and up
        carla.Rotation(pitch=0.0)  # Ensure LIDAR is level
    )
    lidar = world.spawn_actor(lidar_bp, lidar_location, attach_to=vehicle)
    actor_list.append(lidar)
    print("\nLIDAR sensor added at position: x=1.5m, z=2.0m")

    # Enable autopilot
    vehicle.set_autopilot(True)
    print("Autopilot enabled")
    # throttle = 0.5
    # vehicle.apply_control(carla.VehicleControl(throttle=throttle))
    # Set up basic autopilot parameters
    settings = world.get_settings()
    settings.synchronous_mode = True
    world.apply_settings(settings)
    print("World settings configured")

    # Initialize sensors with more conservative settings
    try:
        # IMU with lower update rate
        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_bp.set_attribute('sensor_tick', '0.1')  # 10Hz
        imu_location = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
        imu = world.spawn_actor(imu_bp, imu_location, attach_to=vehicle)
        actor_list.append(imu)
        print("IMU sensor added")

        # RADAR with conservative settings
        radar_bp = blueprint_library.find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', '30')
        radar_bp.set_attribute('vertical_fov', '10')
        radar_bp.set_attribute('range', '50')
        radar_bp.set_attribute('sensor_tick', '0.1')  # 10Hz
        radar_location = carla.Transform(carla.Location(x=2.0, z=1.0))
        radar = world.spawn_actor(radar_bp, radar_location, attach_to=vehicle)
        actor_list.append(radar)
        print("RADAR sensor added")

        # Wait for sensors to initialize
        time.sleep(0.5)

        # Configure world settings first
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.01  # 100 FPS
        world.apply_settings(settings)
        print('World settings configured with fixed_delta_seconds = {}'.format(
            settings.fixed_delta_seconds))

        # Register callbacks after world configuration
        imu.listen(imu_callback)
        radar.listen(radar_callback)
        lidar.listen(lidar_callback)
        print("Sensor callbacks registered")

        # Main loop with status updates
        print('Starting main loop...')
        while True:
            world.tick()
            print_status()
            time.sleep(0.01)  # Match fixed_delta_seconds

    except KeyboardInterrupt:
        print('\nStopping...')
    except Exception as e:
        print('Setup error: {}'.format(e))
    finally:
        print('Cleaning up...')
        try:
            # Restore original settings
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
            
            # Stop sensors first
            print('Stopping sensors...')
            for sensor in [lidar, radar, imu]:
                if sensor:
                    try:
                        sensor.stop()
                    except:
                        pass
            
            # Wait for sensors to stop
            time.sleep(0.5)
            
            # Destroy sensors
            print('Destroying sensors...')
            for sensor in [lidar, radar, imu]:
                if sensor:
                    try:
                        sensor.destroy()
                    except:
                        pass
                        
            # Close sockets
            print('Closing sockets...')
            for sock in [lidar_client, radar_client, imu_client, 
                        lidar_socket, radar_socket, imu_socket]:
                if sock:
                    try:
                        sock.close()
                    except:
                        pass
                        
            # Clean up remaining actors
            print('Destroying remaining actors...')
            for actor in actor_list:
                try:
                    actor.destroy()
                except:
                    pass
                    
            print('Cleanup complete')
            
        except Exception as e:
            print('Error during cleanup: {}'.format(e))

except ImportError as e:
    print("Import Error:", str(e))
    print("Please install the missing package")
    sys.exit(1)
except Exception as e:
    print("Error: {}".format(str(e)))
    sys.exit(1)

print("Script finished successfully")

def start_server():
    print("Initializing server...")
    # Create server socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    # Use localhost instead of hostname for better compatibility
    host = 'localhost'
    port = 12343
    
    try:
        print("Binding server to {}:{}".format(host, port))
        server_socket.bind((host, port))
        print("Server bound successfully")
        
        server_socket.listen(1)
        print("Waiting for connection...")
        
        # Accept one connection
        conn, addr = server_socket.accept()
        print("Connected to: {}".format(addr))
        
        # Receive and send back data
        data = conn.recv(1024)
        print("Received: {}".format(data.decode()))
        conn.send(b"Message received")
        
    except Exception as e:
        print("Server error: {}".format(e))
    finally:
        server_socket.close()

def start_client():
    print("Initializing client...")
    # Create client socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    host = 'localhost'
    port = 12345
    
    try:
        print("Connecting to server...")
        client_socket.connect((host, port))
        print("Connected to server")
        
        # Send data
        client_socket.send(b"Hello from client")
        
        # Receive response
        data = client_socket.recv(1024)
        print("Server response: {}".format(data.decode()))
        
    except Exception as e:
        print("Client error: {}".format(e))
    finally:
        client_socket.close()

def main():
    # Set up socket server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('localhost', 12345))
    server_socket.listen(1)
    print("Waiting for ROS2 client connection...")
    
    try:
        # Connect to CARLA
        print('Connecting to CARLA...')
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        world = client.get_world()

        # Get a random spawn point
        spawn_points = world.get_map().get_spawn_points()
        spawn_point = spawn_points[0] if spawn_points else carla.Transform()

        # Create the vehicle
        print('Spawning vehicle...')
        blueprint = world.get_blueprint_library().find('vehicle.tesla.model3')
        vehicle = world.spawn_actor(blueprint, spawn_point)

        # Make it move forward
        print('Moving vehicle...')
        vehicle.set_autopilot(False)
        vehicle.apply_control(carla.VehicleControl(throttle=0.5))

        # Accept connection from ROS2 client
        client_socket, addr = server_socket.accept()
        print("Connected to ROS2 client at {}".format(addr))
        
        # Main loop
        start_time = time.time()
        while time.time() - start_time < 10:
            # Get vehicle data
            velocity = vehicle.get_velocity()
            transform = vehicle.get_transform()
            
            # Create data dictionary
            data = {
                'position': {
                    'x': transform.location.x,
                    'y': transform.location.y,
                    'z': transform.location.z
                },
                'speed': 3.6 * np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # km/h
            }
            
            # Send data to ROS2 client
            client_socket.send(pickle.dumps(data))
            time.sleep(0.1)
    
    except Exception as e:
        print('Error:', e)
    finally:
        # Clean up
        print('Cleaning up...')
        if 'vehicle' in locals():
            vehicle.destroy()
        client_socket.close()
        server_socket.close()

if __name__ == "__main__":
    try:
        print("Checking command line arguments...")
        if len(sys.argv) > 1 and sys.argv[1] == 'client':
            print("Starting client mode...")
            time.sleep(1)  # Give server time to start
            start_client()
        else:
            print("Starting server mode...")
            start_server()
    except Exception as e:
        print("Main error: {}".format(e))