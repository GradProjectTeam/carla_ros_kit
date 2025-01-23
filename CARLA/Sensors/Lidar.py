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

print("Starting script...")
print("Python version:", sys.version)

try:
    # CARLA setup
    carla_path = '../'
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

    # Setup TCP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    host_ip = '0.0.0.0'
    port = 12345
    server_socket.bind((host_ip, port))
    server_socket.listen(1)
    print("Waiting for connection on {}:{}".format(host_ip, port))

    # Initialize client socket as global
    client_socket = None

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
    lidar_bp.set_attribute('points_per_second', '100000')  # Reduced for stability
    lidar_bp.set_attribute('rotation_frequency', '50')  # Standard rotation speed
    lidar_bp.set_attribute('range', '2000.0')  # Reduced range for testing
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

    def wait_for_connection():
        global client_socket
        while True:
            try:
                print("\nWaiting for ROS node connection...")
                client_socket, addr = server_socket.accept()
                print("Connected to:", addr)
                return
            except Exception as e:
                print("Connection failed:", str(e))
                print("Retrying in 2 seconds...")
                time.sleep(2)

    def lidar_callback(point_cloud):
        global client_socket
        try:
            # Get raw data and send directly
            raw_data = point_cloud.raw_data
            
            try:
                if client_socket is None or client_socket.fileno() == -1:
                    wait_for_connection()
                
                # Send data size first
                size_header = struct.pack('!I', len(raw_data))
                client_socket.sendall(size_header)
                # Send raw data
                client_socket.sendall(raw_data)
                
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                print("\nConnection lost:", str(e))
                if client_socket:
                    client_socket.close()
                client_socket = None
                wait_for_connection()
            
        except Exception as e:
            print("\nError in LIDAR callback:", str(e))

    # Register LIDAR callback
    lidar.listen(lidar_callback)
    print("LIDAR callback registered")
    print("\nVisualization available in RViz2. Use the following commands in separate terminals:")
    print("1. ros2 run my_python_pkg pub_node")
    print("2. rviz2")
    print("\nIn RViz2:")
    print("- Set Fixed Frame to 'lidar_link'")
    print("- Add PointCloud2 display")
    print("- Set Topic to '/carla/lidar'")

    # Initial connection
    wait_for_connection()

    try:
        while True:
            world.tick()
            time.sleep(0.1)  # 10 FPS

    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        print("Cleaning up...")
        settings.synchronous_mode = False
        world.apply_settings(settings)
        
        print("Destroying actors...")
        for actor in actor_list:
            actor.destroy()
        
        if 'client_socket' in globals():
            client_socket.close()
        server_socket.close()
        print("Clean up complete")

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