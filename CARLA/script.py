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

print("Starting script...")

try:
    # Print Python version info
    print("Python version:", sys.version)
    
    # CARLA setup
    carla_path = '/home/mostafa/GP/CARLA_0.9.5/PythonAPI/carla/dist'
    print("Looking for CARLA at:", carla_path)
    
    carla_eggs = glob.glob('{0}/carla-*{1}.{2}-{3}.egg'.format(
        carla_path,
        sys.version_info.major,
        sys.version_info.minor,
        "win-amd64" if os.name == "nt" else "linux-x86_64"
    ))
    
    if not carla_eggs:
        raise RuntimeError("No CARLA egg files found")
    
    print("Found CARLA egg:", carla_eggs[0])
    sys.path.append(carla_eggs[0])
    
    import carla
    print("CARLA imported successfully")

    # Connect to CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    print("Connected to CARLA world")

    # Setup TCP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    host_ip = '0.0.0.0'
    port = 12345
    server_socket.bind((host_ip, port))
    server_socket.listen(1)
    print("Waiting for connection on {}:{}".format(host_ip, port))

    # Spawn vehicle
    spawn_points = world.get_map().get_spawn_points()
    blueprint = world.get_blueprint_library().find('vehicle.tesla.model3')
    vehicle = world.spawn_actor(blueprint, spawn_points[0])
    print("Vehicle spawned")

    # Enable autopilot
    vehicle.set_autopilot(True)
    print("Autopilot enabled")

    # Set up basic autopilot parameters
    settings = world.get_settings()
    settings.synchronous_mode = True
    world.apply_settings(settings)
    print("World settings configured")

    client_socket, addr = server_socket.accept()
    print("Connected to: {}".format(addr))

    try:
        while True:
            # Get vehicle data
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            
            # Pack data efficiently (24 bytes total: 6 floats * 4 bytes)
            data = struct.pack('!ffffff',
                transform.location.x,
                transform.location.y,
                transform.location.z,
                velocity.x,
                velocity.y,
                velocity.z
            )
            
            # Send data size first (4 bytes for size)
            size_data = struct.pack('!I', len(data))
            client_socket.sendall(size_data + data)
            
            # Tick the world to advance simulation
            world.tick()
            time.sleep(0.01)  # 100Hz update rate

    except KeyboardInterrupt:
        print("Stopping...")
    
    finally:
        print("Cleaning up...")
        settings.synchronous_mode = False
        world.apply_settings(settings)
        vehicle.set_autopilot(False)
        vehicle.destroy()
        client_socket.close()
        server_socket.close()
        print("Clean up complete")

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
    port = 12345
    
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