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

print("Starting Camera script...")
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

    # Setup TCP socket for camera data
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    host_ip = '0.0.0.0'
    port = 12347  # Changed port for camera
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

    # Add RGB camera for lane detection
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    # Set camera attributes for lane detection
    camera_bp.set_attribute('image_size_x', '640')
    camera_bp.set_attribute('image_size_y', '480')
    camera_bp.set_attribute('fov', '90')
    camera_bp.set_attribute('sensor_tick', '0.1')  # 10 FPS

    # Print camera configuration
    print("\nCamera Configuration:")
    print("- Resolution: {}x{}".format(
        camera_bp.get_attribute('image_size_x').as_int(),
        camera_bp.get_attribute('image_size_y').as_int()
    ))
    print("- FOV:", camera_bp.get_attribute('fov').as_float(), "degrees")
    print("- Update rate:", 1.0/camera_bp.get_attribute('sensor_tick').as_float(), "FPS")

    # Spawn camera with adjusted position for lane detection
    camera_location = carla.Transform(
        carla.Location(x=2.0, z=1.5),  # Front of car, slightly elevated
        carla.Rotation(pitch=-15.0)    # Tilted down slightly
    )
    camera = world.spawn_actor(camera_bp, camera_location, attach_to=vehicle)
    actor_list.append(camera)
    print("\nCamera sensor added at position: x=2.0m, z=1.5m, pitch=-15°")

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
                print("\nWaiting for ROS node connection...")
                client_socket, addr = server_socket.accept()
                print("Connected to:", addr)
                return
            except Exception as e:
                print("Connection failed:", str(e))
                print("Retrying in 2 seconds...")
                time.sleep(2)

    def camera_callback(image):
        global client_socket
        try:
            # Get raw image data
            raw_data = image.raw_data
            
            try:
                if client_socket is None or client_socket.fileno() == -1:
                    wait_for_connection()
                
                # Send image size first
                size_header = struct.pack('!I', len(raw_data))
                client_socket.sendall(size_header)
                # Send image data
                client_socket.sendall(raw_data)
                
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                print("\nConnection lost:", str(e))
                if client_socket:
                    client_socket.close()
                client_socket = None
                wait_for_connection()
            
        except Exception as e:
            print("\nError in camera callback:", str(e))

    # Register camera callback
    camera.listen(camera_callback)
    print("Camera callback registered")
    print("\nCamera feed available in ROS2. Use the following commands:")
    print("1. ros2 run my_python_pkg camera_node")
    print("2. rqt_image_view")

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

except Exception as e:
    print("Error: {}".format(str(e)))
    sys.exit(1)

print("Script finished successfully")