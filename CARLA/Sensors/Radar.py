#! /usr/local/bin/python3.5m
import sys
import glob
import os
import socket
import time
import struct
import math
import random

print("Starting Radar script...")
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
    
    # Setup TCP client for Radar data
    client_socket = None
    host_ip = '127.0.0.1'
    port = 12347
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

    # Add Radar sensor
    radar_bp = blueprint_library.find('sensor.other.radar')
    radar_bp.set_attribute('sensor_tick', '0.1')      # 10Hz update rate
    radar_bp.set_attribute('horizontal_fov', '30')    # ±15 degrees
    radar_bp.set_attribute('vertical_fov', '30')      # ±15 degrees
    radar_bp.set_attribute('range', '100')            # 100m max range
    radar_bp.set_attribute('points_per_second', '1500')  # Increase point density

    # Radar mounting position (adjust these values)
    radar_transform = carla.Transform(
        carla.Location(x=1.0, y=1.0, z=2.0),  # 1 meter in front of the bumper (0.5m above ground)
        carla.Rotation(pitch=0)         # Level with ground
    )

    # Spawn the radar sensor
    radar = world.spawn_actor(radar_bp, radar_transform, attach_to=vehicle)
    actor_list.append(radar)
    print("\nRadar sensor added")

    # Enable autopilot
    vehicle.set_autopilot(True)
    print("Autopilot enabled")

    # Set up synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True
    world.apply_settings(settings)
    print("World settings configured")

    # Initialize the spectator
    spectator = world.get_spectator()

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

    def radar_callback(radar_data):
        global client_socket
        if not client_socket:
            return  # Skip if no connection
        
        try:
            # Process radar data
            points = []
            for detection in radar_data:
                # Transform to correct coordinate system
                depth = max(0.1, detection.depth)  # Ensure positive depth
                azimuth = detection.azimuth
                altitude = detection.altitude
                velocity = detection.velocity

                points.append((altitude, azimuth, depth, velocity))
                if len(points) > 0:
                    print("Valid point: alt={:.2f}°, az={:.2f}°, d={:.2f}m, v={:.2f}m/s".format(
                        altitude, azimuth, depth, velocity))

            print("Number of detections: ", len(points))
            if points:
                # Pack radar data
                try:
                    data_bytes = b''  # Initialize data_bytes to an empty byte string
                    # dont send the number of points

                    # Then pack each point's data
                    for point in points:
                        data_bytes += struct.pack('!4f', *point)
                except struct.error as e:
                    print("Error packing radar data: {0}".format(str(e)))
                    return

                # Send data with timeout
                client_socket.settimeout(1.0)
                client_socket.sendall(data_bytes)
                client_socket.settimeout(None)

        except (BrokenPipeError, ConnectionResetError, OSError, socket.timeout) as e:
            print("\nConnection error: {0}".format(str(e)))
            if client_socket:
                client_socket.close()
                client_socket = None
        except Exception as e:
            print("\nUnexpected error in radar callback: {0}".format(str(e)))

    # Register radar callback
    radar.listen(radar_callback)
    print("Radar callback registered")
    print("\nRadar data streaming on port", port)

    # Initialize socket as None
    client_socket = None
    
    # Initial connection attempt
    wait_for_connection()

    try:
        while True:
            # Update the spectator's position to match the radar's position
            radar_location = radar.get_location()
            spectator.set_transform(carla.Transform(carla.Location(x=radar_location.x, y=radar_location.y, z=radar_location.z), carla.Rotation(pitch=0)))
            print("Spectator set to radar location")

            world.tick()  # Advance the simulation
            time.sleep(0.1)  # Maintain a loop frequency

    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        print("Cleaning up...")
        # Disable synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        
        # Clean up radar and vehicle
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