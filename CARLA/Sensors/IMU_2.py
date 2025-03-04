#! /usr/local/bin/python3.5m
import sys
import glob
import os
import socket
import numpy as np
import time
import struct
import math
import random

print("Starting IMU script...")
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
    
    # Setup TCP client for IMU data
    client_socket = None
    host_ip = '127.0.0.1'
    port = 12346
    print("Connecting to {}:{}...".format(host_ip, port))

    # Example: send IMU data (4 floats)
   

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

    # Add IMU sensor
    imu_bp = blueprint_library.find('sensor.other.imu')
    imu_bp.set_attribute('sensor_tick', '0.1')  # 10Hz update rate
    
    # Spawn IMU with adjusted position
    imu_location = carla.Transform(
        carla.Location(x=0.0, z=1.0),  # Center of vehicle, 1m up
        carla.Rotation()
    )
    imu = world.spawn_actor(imu_bp, imu_location, attach_to=vehicle)
    actor_list.append(imu)
    print("\nIMU sensor added")

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
                # Create new socket if needed
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

    def imu_callback(imu_data):
        global client_socket
        if not client_socket:
            return  # Skip if no connection
        
        try:
            # Validate IMU data before packing
            if not all(hasattr(imu_data, attr) for attr in ['accelerometer', 'gyroscope', 'compass']):
                print("Invalid IMU data structure")
                return

            # Pack IMU data with safety checks
            try:
                data_bytes = struct.pack('!7f',
                    float(imu_data.accelerometer.x),
                    float(imu_data.accelerometer.y),
                    float(imu_data.accelerometer.z),
                    float(imu_data.gyroscope.x),
                    float(imu_data.gyroscope.y),
                    float(imu_data.gyroscope.z),
                    float(imu_data.compass)
                )
            except (AttributeError, TypeError, struct.error) as e:
                print("Error packing IMU data: {0}".format(str(e)))
                return

            # Send data with timeout
            client_socket.settimeout(1.0)  # 1 second timeout
            client_socket.sendall(data_bytes)
            client_socket.settimeout(None)  # Reset timeout
            
        except (BrokenPipeError, ConnectionResetError, OSError, socket.timeout) as e:
            print("\nConnection error: {0}".format(str(e)))
            if client_socket:
                client_socket.close()
                client_socket = None
        except Exception as e:
            print("\nUnexpected error in IMU callback: {0}".format(str(e)))

    # Register IMU callback
    imu.listen(imu_callback)
    print("IMU callback registered")
    print("\nIMU data streaming on port", port)

    # Initialize socket as None
    client_socket = None
    
    # Initial connection attempt
    wait_for_connection()

    try:
        while True:
            # Attempt reconnection if needed
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
        
        # Clean up IMU and vehicle
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