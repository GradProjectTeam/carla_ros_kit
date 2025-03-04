#! /usr/local/bin/python3.5m
import sys
import glob
import os
import socket
import time
import struct
import math
import random
import numpy

print("Starting Lidar script...")
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
    
    # Setup TCP client for Lidar data
    client_socket = None
    host_ip = '127.0.0.1'
    port = 12345

    print("Connecting to {}:{}...".format(host_ip, port))

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

    # Configure vehicle movement
    def configure_vehicle_movement(vehicle, world):
        try:
            # Enable autopilot with more aggressive settings
            vehicle.set_autopilot(True)
            
            # Optional: Configure traffic manager for more dynamic movement
            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_global_distance_factor(1.0)  # Adjust following distance
            traffic_manager.set_percentage_speed_difference(vehicle, 0)  # Normal speed
            
            # Optional: Set specific driving parameters
            traffic_manager.ignore_lights_percentage(vehicle, 0)  # Obey traffic lights
            traffic_manager.set_random_device_seed(42)  # Consistent randomness
            
            print("Vehicle movement configured successfully")
        except Exception as e:
            print("Error configuring vehicle movement: ",e)

    # Configure vehicle movement
    configure_vehicle_movement(vehicle, world)

    # Add Lidar sensor
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('channels', '32')  # Standard number of channels
    lidar_bp.set_attribute('points_per_second', '50000')  # Reduced for stability
    lidar_bp.set_attribute('rotation_frequency', '50')  # Standard rotation speed
    lidar_bp.set_attribute('range', '2000.0')  # Reduced range for testing
    lidar_bp.set_attribute('upper_fov', '0.0')  # Reduced upper FOV
    lidar_bp.set_attribute('lower_fov', '-20.0')  # Standard lower FOV
    
    # Spawn Lidar with adjusted position
    lidar_location = carla.Transform(
        carla.Location(x=0.0, z=2.0),  # Top of vehicle
        carla.Rotation(pitch=0.0)  # Facing forward
    )
    lidar = world.spawn_actor(lidar_bp, lidar_location, attach_to=vehicle)
    actor_list.append(lidar)
    print("\nLidar sensor added")

    map_name = world.get_map().name
    print("Current map: ", map_name)

    # Enable autopilot
    vehicle.set_autopilot(True)
    print("Autopilot enabled")

    # Set up synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True
    world.apply_settings(settings)
    print("World settings configured")

    def wait_for_connection():
        global client_socket
        while True:
            try:
                if client_socket is None:
                    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                
                print("\nWaiting for connection...")
                client_socket.connect((host_ip, port))
                print("Connected to {0}:{1}".format(host_ip, port))
                return
            except Exception as e:
                print("Connection failed:", str(e))
                if client_socket:
                    client_socket.close()
                    client_socket = None
                print("Retrying in 2 seconds...")
                time.sleep(2)

    def lidar_callback(point_cloud):
        global client_socket
        if not client_socket:
            return  # Skip if no connection
        
        try:
            # Each point is sent as 16 bytes (4 floats * 4 bytes)
            for point in point_cloud:
                # CARLA lidar points don't have direct intensity attribute
                # We can use point.intensity if available, or calculate from raw data
                intensity = 1.0  # Default intensity or calculate from raw data
                
                point_data = struct.pack('fff', point.x, point.y, point.z)
                print(point.x,point.y,point.z)
                client_socket.send(point_data)  # Sends exactly 16 bytes
        
        except Exception as e:
            print("\nUnexpected error in lidar callback: {0}".format(str(e)))
            if client_socket:
                client_socket.close()
                client_socket = None

    # Register lidar callback
    lidar.listen(lidar_callback)
    print("Lidar callback registered")
    print("\nLidar data streaming on port", port)

    # Initialize socket as None
    client_socket = None
    
    # Initial connection attempt
    wait_for_connection()

    try:
        while True:
            if client_socket is None:
                wait_for_connection()
            world.tick()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        print("Cleaning up...")
        # Disable synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        
        # Clean up lidar and vehicle
        print("Destroying actors...")
        for actor in actor_list:
            if actor is not None and actor.is_alive:
                actor.destroy()
        
        # Close socket properly
        if 'client_socket' in globals() and client_socket is not None:
            try:
                client_socket.shutdown(socket.SHUT_RDWR)
                client_socket.close()
            except:
                pass
        print("Clean up complete")

except Exception as e:
    print("Error: {0}".format(str(e)))
    # Emergency cleanup
    if 'actor_list' in locals():
        for actor in actor_list:
            if actor is not None and actor.is_alive:
                actor.destroy()
    if 'client_socket' in locals() and client_socket is not None:
        try:
            client_socket.close()
        except:
            pass
    sys.exit(1)

print("Script finished successfully")