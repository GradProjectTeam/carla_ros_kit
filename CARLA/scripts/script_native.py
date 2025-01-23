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
    lidar_bp.set_attribute('points_per_second', '10000')  # Reduced for stability
    lidar_bp.set_attribute('rotation_frequency', '10')  # Standard rotation speed
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

    def detect_safe_zones_lidar(points, lidar_height=2.0):
        try:
            # Compensate for LIDAR height and angle
            adjusted_points = points.copy()
            adjusted_points[:, 2] += lidar_height  # Adjust Z coordinates for LIDAR height
            
            # Calculate distance from LIDAR
            distances = np.sqrt(
                adjusted_points[:, 0]**2 + 
                adjusted_points[:, 1]**2 + 
                adjusted_points[:, 2]**2
            )
            
            # Filter for potential safe zones with perspective compensation
            safe_mask = (
                (adjusted_points[:, 2] < 0.2) &      # Slightly above ground
                (adjusted_points[:, 2] > -0.2) &     # Slightly below ground
                (np.abs(adjusted_points[:, 1]) < 4.0 * (distances/10)) &  # Width increases with distance
                (adjusted_points[:, 0] > 2.0) &      # Not too close
                (adjusted_points[:, 0] < 25.0) &     # Not too far
                (adjusted_points[:, 3] > 0.05) &     # Valid intensity
                (distances < 30.0)                   # Maximum reliable range
            )
            safe_points = points[safe_mask]
            
            if len(safe_points) > 0:
                # Create perspective-aware grid
                resolution = 0.2  # 20cm grid
                width = 12.0     # Increased for perspective
                length = 25.0
                grid_width = int(width / resolution)
                grid_length = int(length / resolution)
                
                # Initialize grids
                height_grid = np.full((grid_length, grid_width), np.inf)
                density_grid = np.zeros((grid_length, grid_width))
                
                # Convert points to grid with perspective compensation
                cell_x = np.clip((safe_points[:, 0] / resolution).astype(int), 0, grid_length-1)
                cell_y = np.clip(((safe_points[:, 1] + width/2) / resolution).astype(int), 0, grid_width-1)
                
                # Record heights and point density
                for i in range(len(safe_points)):
                    x, y = cell_x[i], cell_y[i]
                    if height_grid[x, y] == np.inf:
                        height_grid[x, y] = safe_points[i, 2]
                    else:
                        height_grid[x, y] = (height_grid[x, y] + safe_points[i, 2]) / 2
                    density_grid[x, y] += 1
                
                # Create flatness grid
                flatness_grid = np.zeros_like(height_grid)
                for i in range(1, grid_length-1):
                    for j in range(1, grid_width-1):
                        if height_grid[i, j] != np.inf:
                            neighborhood = height_grid[i-1:i+2, j-1:j+2]
                            valid_neighbors = neighborhood[neighborhood != np.inf]
                            if len(valid_neighbors) >= 5:
                                flatness_grid[i, j] = np.std(valid_neighbors)
                
                # Identify safe zones
                safe_zones = []
                safe_mask = (
                    (flatness_grid < 0.05) &              # Flat enough
                    (density_grid > 5) &                  # Enough points
                    (height_grid != np.inf)               # Valid height data
                )
                
                # Find continuous regions
                from scipy.ndimage import label, binary_dilation
                labeled_grid, num_features = label(safe_mask)
                
                # Process each potential safe zone
                for i in range(1, num_features + 1):
                    zone_mask = labeled_grid == i
                    zone_size = np.sum(zone_mask)
                    
                    if zone_size > 20:  # Minimum size threshold
                        # Calculate center in world coordinates
                        y, x = np.where(zone_mask)
                        center_x = np.mean(x) * resolution
                        center_y = np.mean(y) * resolution - width/2
                        
                        # Calculate average height and flatness
                        zone_height = np.mean(height_grid[zone_mask])
                        zone_flatness = np.mean(flatness_grid[zone_mask])
                        
                        safe_zones.append({
                            'position': (float(center_x), float(center_y)),
                            'size': float(zone_size * resolution * resolution),
                            'height': float(zone_height),
                            'flatness': float(zone_flatness),
                            'density': float(np.mean(density_grid[zone_mask]))
                        })
            
                return safe_zones
            
            return []
            
        except Exception as e:
            print("\nSafe zone detection error:", e)
            return []

    def lidar_callback(point_cloud):
        global client_socket, leader, follower
        try:
            # Get raw data and ensure it's properly aligned
            raw_data = np.frombuffer(point_cloud.raw_data, dtype=np.float32)
            if len(raw_data) % 4 != 0:
                return
            points = raw_data.reshape(-1, 4)
            
            # Detect safe zones
            safe_zones = detect_safe_zones_lidar(points)
            
            # Visualize points and safe zones in CARLA simulator
            for point in points:
                loc = carla.Location(
                    float(point[0]),
                    float(point[1]),
                    float(point[2])
                )
                world_loc = point_cloud.transform.transform(loc)
                
                # Color coding: red for road, blue for others, green for safe zones
                is_safe_zone = any(
                    abs(point[0] - sz['position'][0]) < 1.0 and
                    abs(point[1] - sz['position'][1]) < 1.0
                    for sz in safe_zones
                )
                
                if is_safe_zone:
                    color = carla.Color(r=0, g=255, b=0)  # Green for safe zones
                elif -0.45 < point[2] < -0.25:
                    color = carla.Color(r=255, g=0, b=0)  # Red for road
                else:
                    color = carla.Color(r=0, g=0, b=255)  # Blue for others
                
                world.debug.draw_point(
                    world_loc,
                    size=0.05,
                    color=color,
                    life_time=0.1
                )
            
            # Draw safe zone boundaries
            for zone in safe_zones:
                center = carla.Location(
                    float(zone['position'][0]),
                    float(zone['position'][1]),
                    -0.3
                )
                world_center = point_cloud.transform.transform(center)
                
                # Draw a circle around safe zone
                radius = np.sqrt(zone['size'] / np.pi)
                world.debug.draw_circle(
                    world_center,
                    radius,
                    0.1,
                    carla.Color(r=0, g=255, b=0),
                    life_time=0.1
                )
            
            print("\rPoints: {} | Safe zones: {}".format(
                len(points),
                len(safe_zones)
            ), end='')
            
        except Exception as e:
            print("\nLIDAR visualization error:", e)
            if 'point' in locals():
                print("Point data:", point)

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