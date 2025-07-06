#! /usr/bin/python3.7
"""
Gui_Control.py - CARLA Simulator Vehicle Control and Sensor Management System

This script provides a comprehensive interface for controlling vehicles in the CARLA simulator
and managing various sensors (LiDAR, RADAR, IMU, Camera) attached to them. It handles:
1. Vehicle spawning, control, and traffic management
2. Sensor data collection, processing, and transmission via TCP
3. GUI interface for real-time control and visualization
4. Waypoint generation for navigation assistance

Authors: Shishtawy & Hendy
Project by: TechZ
Version: 6.0 - Added traffic vehicles with autopilot in front of the main vehicle
"""

# Import required libraries for simulation, networking, GUI and data processing
import sys
import glob
import os
import socket  # For TCP communication
import pickle  # For data serialization
import numpy as np  # For numerical operations
import time
import json  # For data formatting
import struct  # For binary data packing
import math
import random
import pygame  # For GUI interface
from pygame.locals import *
import threading  # For parallel processing
from queue import Queue  # For thread-safe data queues
import traceback  # For error tracking




# Configuration Variables - Edit these to change simulation settings
# -----------------------------------------------
# TOWN_MAP: Select which CARLA town to load when starting the simulation
# Available options:
#   - 'Town01' to 'Town07': Standard maps
#   - 'Town10HD': High-definition map
#   - 'Town11': Rural environment
#   - 'Town12': Under construction map
# If an invalid map name is provided, the system will default to 'Town01'
# To change the map, simply modify the value of TOWN_MAP below
TOWN_MAP = 'Town03'  # Default town map for simulation

# WEATHER_PRESET: Select the weather condition for the simulation
# Available options:
#   - 'ClearNoon': Clear sky with noon sun
#   - 'CloudyNoon': Cloudy sky with noon sun
#   - 'WetNoon': Wet roads with noon sun
#   - 'WetCloudyNoon': Wet roads with cloudy noon sky
#   - 'MidRainyNoon': Medium rain with noon sun
#   - 'HardRainNoon': Heavy rain with noon sun
#   - 'SoftRainNoon': Light rain with noon sun
#   - 'ClearSunset': Clear sky with sunset sun
#   - 'CloudySunset': Cloudy sky with sunset sun
#   - 'WetSunset': Wet roads with sunset sun
#   - 'WetCloudySunset': Wet roads with cloudy sunset
#   - 'MidRainSunset': Medium rain with sunset sun
#   - 'HardRainSunset': Heavy rain with sunset sun
#   - 'SoftRainSunset': Light rain with sunset sun
# If an invalid weather preset is provided, the system will use 'ClearNoon'
WEATHER_PRESET = 'ClearNoon'  # Default weather setting for simulation



# Version 9: Added map & wheather control through a global configuration variable


class CARLASetup:  # Class responsible for initializing CARLA environment and Python API
    def __init__(self):
        print("Starting CARLA setup...")
        self.carla_path = '/home/shishtawy/Carla/CARLA_0.9.12/PythonAPI/carla/dist'  # Path to CARLA Python API
        # self.carla_path = '/home/mostafa/ROS2andCarla/CARLA/CARLA_0.9.8/PythonAPI/carla/dist'
        self.setup_carla()
        
    def setup_carla(self):  # Method to setup CARLA environment and import necessary modules
        print("Looking for CARLA at:", self.carla_path)
        carla_eggs = glob.glob('{0}/carla-*{1}.{2}-{3}.egg'.format(  # Find CARLA egg file for current Python version
            self.carla_path,
            sys.version_info.major,
            sys.version_info.minor,
            "win-amd64" if os.name == "nt" else "linux-x86_64"
        ))
        sys.path.append(carla_eggs[0])  # Add CARLA egg to Python path
        print("Found CARLA egg:", carla_eggs[0])
        
        global carla
        import carla # type: ignore  # Import CARLA module
        print("CARLA imported successfully")

class SensorManager:  # Class managing all vehicle sensors and their data processing
    def __init__(self, vehicle, world):  # Initialize sensor manager with vehicle and world objects
        print("\n=== Initializing Sensor Manager ===")
        self.vehicle = vehicle  # Store reference to the vehicle
        self.world = world  # Store reference to the CARLA world
        self.actor_list = []  # List to track all sensor actors
        self.map = world.get_map()  # Get current map for navigation
        
        # Waypoint configuration for navigation
        self.waypoint_distance = 2.0  # Distance between consecutive waypoints
        self.waypoints = []  # List to store generated waypoints
        
        # Thread-safe queues for sensor data
        self.lidar_queue = Queue(maxsize=1)  # Queue for LiDAR point cloud data
        self.radar_queue = Queue(maxsize=1)  # Queue for RADAR detection data
        self.imu_queue = Queue(maxsize=1)  # Queue for IMU measurements
        self.camera_queue = Queue(maxsize=1)  # Queue for camera images
        self.waypoint_queue = Queue(maxsize=1)  # Queue for navigation waypoints
        self.velocity_queue = Queue(maxsize=1)  # Queue for velocity data
        
        # TCP network configuration
        self.host_ip = '127.0.0.1'  # Local host IP for network communication
        self.lidar_port = 12349  # Port for LiDAR data streaming
        self.radar_port = 12347  # Port for RADAR data streaming
        self.imu_port = 12341  # Port for IMU data streaming
        self.camera_port = 12342  # Port for camera data streaming
        self.waypoint_port = 12343  # Port for waypoint data streaming
        self.velocity_port = 12346  # Port for velocity data streaming

        # Sensor activation flags
        self.lidar_flag = True  # Toggle for LiDAR sensor
        self.radar_flag = True  # Toggle for RADAR sensor
        self.imu_flag = True  # Toggle for IMU sensor
        self.camera_flag = False  # Toggle for camera sensor
        self.waypoint_flag = True  # Toggle for waypoint generation
        self.velocity_flag = True  # Toggle for velocity tracking
        
        # Additional waypoint settings
        self.waypoint_distance = 2.0  # Spacing between waypoints
        self.waypoint_lifetime = 0.5  # Duration waypoints remain visible
        self.waypoints = []  # Storage for waypoint path
        
        print("Initial sensor flags - LIDAR: {}, RADAR: {}, IMU: {}, CAMERA: {}, WAYPOINT: {}, VELOCITY: {}".format(
            self.lidar_flag, self.radar_flag, self.imu_flag, self.camera_flag, self.waypoint_flag, self.velocity_flag))
        
        # Thread management
        self.running = True  # Control flag for all threads
        self.lidar_thread = None  # Thread for LiDAR processing
        self.radar_thread = None  # Thread for RADAR processing
        self.imu_thread = None  # Thread for IMU processing
        self.camera_thread = None  # Thread for camera processing
        self.waypoint_thread = None  # Thread for waypoint processing
        self.waypoint_generation_thread = None  # Thread for waypoint generation
        
        # Initialize systems
        self.setup_tcp_sockets()  # Setup network connections
        self.setup_sensors()  # Initialize all sensors
        self.start_processing_threads()  # Start data processing threads
        print("=== Sensor Manager Initialization Complete ===\n")
        
    def setup_tcp_sockets(self):  # Method to initialize all TCP network connections
        # LiDAR socket setup
        if self.lidar_flag:
            try:
                self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket for LiDAR
                self.lidar_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Optimize for real-time data
                print("LiDAR TCP configured for {0}:{1}".format(self.host_ip, self.lidar_port))
                
                try:
                    self.lidar_socket.connect((self.host_ip, self.lidar_port))  # Connect to LiDAR server
                    print("LiDAR TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print("LiDAR TCP connection failed: {}. Will continue with local data only.".format(e))
            except Exception as e:
                print("Error setting up LiDAR socket: {}".format(e))
        
        # Radar socket setup
        if self.radar_flag:
            try:
                self.radar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket for RADAR
                self.radar_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Optimize for real-time data
                print("Radar TCP configured for {0}:{1}".format(self.host_ip, self.radar_port))
                
                try:
                    self.radar_socket.connect((self.host_ip, self.radar_port))  # Connect to RADAR server
                    print("Radar TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print("Radar TCP connection failed: {}. Will continue with local data only.".format(e))
            except Exception as e:
                print("Error setting up Radar socket: {}".format(e))
        
        # IMU socket setup
        if self.imu_flag:
            try:
                self.imu_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket for IMU
                self.imu_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Optimize for real-time data
                print("IMU TCP configured for {0}:{1}".format(self.host_ip, self.imu_port))
                
                try:
                    self.imu_socket.connect((self.host_ip, self.imu_port))  # Connect to IMU server
                    print("IMU TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print("IMU TCP connection failed: {}. Will continue with local data only.".format(e))
            except Exception as e:
                print("Error setting up IMU socket: {}".format(e))
        
        # Camera socket setup
        if self.camera_flag:
            try:
                self.camera_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket for camera
                self.camera_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
                self.camera_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Optimize for real-time data
                print("Camera TCP configured for {0}:{1}".format(self.host_ip, self.camera_port))
                
                try:
                    self.camera_socket.bind(('0.0.0.0', self.camera_port))  # Bind to all interfaces
                    self.camera_socket.listen(1)  # Listen for connections
                    self.camera_socket.settimeout(0.5)  # Non-blocking accept
                    print("Camera TCP server listening on port {0}".format(self.camera_port))
                    self.camera_client = None  # No client connected yet
                except Exception as e:
                    print("Error setting up camera server: {0}".format(e))
                    # Don't disable the sensor just because server setup failed
            except Exception as e:
                print("Error setting up Camera socket: {}".format(e))
                # Keep the flag enabled if possible
        
        # Waypoint socket setup
        if self.waypoint_flag:
            try:
                self.waypoint_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket for waypoint
                self.waypoint_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Optimize for real-time data
                print("Waypoint TCP configured for {0}:{1}".format(self.host_ip, self.waypoint_port))
                
                try:
                    self.waypoint_socket.connect((self.host_ip, self.waypoint_port))  # Connect to waypoint server
                    print("Waypoint TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print("Waypoint TCP connection failed: {}. Will continue with local data only.".format(e))
            except Exception as e:
                print("Error setting up Waypoint socket: {}".format(e))
        
        # Velocity socket setup
        if self.velocity_flag:
            try:
                self.velocity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.velocity_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print("Velocity TCP configured for {0}:{1}".format(self.host_ip, self.velocity_port))   
                
                try:
                    self.velocity_socket.connect((self.host_ip, self.velocity_port))
                    print("Velocity TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print("Velocity TCP connection failed: {}. Will continue with local data only.".format(e))
            except Exception as e:
                print("Error setting up Velocity socket: {}".format(e))
        
    def start_processing_threads(self):  # Initialize and start all sensor data processing threads
        # Create threads for each active sensor
        if self.lidar_flag:
            self.lidar_thread = threading.Thread(target=self.process_lidar_queue)  # Thread for processing LiDAR data
        if self.radar_flag:
            self.radar_thread = threading.Thread(target=self.process_radar_queue)  # Thread for processing RADAR data
        if self.imu_flag:
            self.imu_thread = threading.Thread(target=self.process_imu_queue)  # Thread for processing IMU data
        if self.camera_flag:
            self.camera_thread = threading.Thread(target=self.process_camera_queue)  # Thread for processing camera images
        if self.waypoint_flag:
            self.waypoint_thread = threading.Thread(target=self.process_waypoint_queue)  # Thread for processing navigation waypoints
        if self.velocity_flag:
            self.velocity_thread = threading.Thread(target=self.update_velocity)  # Thread for updating vehicle velocity

        # Set all threads as daemon threads (will terminate when main program ends)
        if self.lidar_flag:
            self.lidar_thread.daemon = True
        if self.radar_flag:
            self.radar_thread.daemon = True
        if self.imu_flag:
            self.imu_thread.daemon = True
        if self.camera_flag:
            self.camera_thread.daemon = True
        if self.waypoint_flag:
            self.waypoint_thread.daemon = True
        if self.velocity_flag:
            self.velocity_thread.daemon = True

        # Start all active sensor threads
        if self.lidar_flag:
            self.lidar_thread.start()
        if self.radar_flag:
            self.radar_thread.start()
        if self.imu_flag:
            self.imu_thread.start()
        if self.camera_flag:
            self.camera_thread.start()
        if self.waypoint_flag:
            self.waypoint_thread.start()
        if self.velocity_flag:
            self.velocity_thread.start()
    
    def process_lidar_queue(self):  # Process and transmit LiDAR point cloud data
        point_counter = 0  # Counter for monitoring point processing progress
        while self.running:
            try:
                if not self.lidar_queue.empty():
                    point_cloud = self.lidar_queue.get()  # Retrieve point cloud data from queue
                    for point in point_cloud:
                        if not self.running:
                            break
                        
                        # Print debug info periodically
                        point_counter += 1
                        if point_counter % 1000 == 0:  # Log every 1000th point for monitoring
                            # Handle different LiDAR point formats based on CARLA version
                            if hasattr(point, 'x'):
                                # Direct coordinate access for older CARLA versions
                                print("Sending LIDAR point #{0}: ({1:.2f}, {2:.2f}, {3:.2f})".format(
                                    point_counter, point.x, point.y, point.z))
                            elif hasattr(point, 'point'):
                                # Access through point attribute for newer CARLA versions
                                print("Sending LIDAR point #{0}: ({1:.2f}, {2:.2f}, {3:.2f})".format(
                                    point_counter, point.point.x, point.point.y, point.point.z))
                            else:
                                # Array/tuple access for alternative formats
                                print("Sending LIDAR point #{0}: ({1:.2f}, {2:.2f}, {3:.2f})".format(
                                    point_counter, point[0], point[1], point[2]))
                        
                        # Convert point data to network format
                        try:
                            # Handle different point formats and pack as network bytes
                            if hasattr(point, 'x'):
                                point_data = struct.pack('!fff', point.x, point.y, point.z)  # Pack coordinates as 3 floats
                            elif hasattr(point, 'point'):
                                point_data = struct.pack('!fff', point.point.x, point.point.y, point.point.z)  # Pack coordinates as 3 floats
                            else:
                                point_data = struct.pack('!fff', point[0], point[1], point[2])  # Pack array coordinates as 3 floats
                                
                            if self.lidar_flag:
                                self.lidar_socket.send(point_data)  # Transmit point data over TCP
                        except (AttributeError, IndexError, TypeError) as e:
                            print("Error processing LiDAR point: {0}".format(e))  # Log point processing errors
                            print("Point type: {0}, Point data: {1}".format(type(point), point))
                            continue  # Skip problematic points
                        except socket.error as e:
                            print("LiDAR socket error: {0}".format(e))  # Log network errors
                            break
                else:
                    time.sleep(0.001)  # Prevent CPU overuse when queue is empty
            except Exception as e:
                print("Error in LiDAR processing thread: {0}".format(e))  # Log general processing errors
                traceback.print_exc()
    
    def process_radar_queue(self):  # Process and transmit RADAR detection data
        point_counter = 0  # Counter for monitoring RADAR detections
        while self.running:
            try:
                if not self.radar_queue.empty():
                    radar_data = self.radar_queue.get()  # Retrieve RADAR data from queue
                    points = np.array([[det.altitude, det.azimuth, det.depth, det.velocity] 
                                    for det in radar_data], dtype=np.float32)  # Convert detections to numpy array
                    
                    # Periodic logging for monitoring
                    if point_counter % 50 == 0:
                        print("[RADAR DEBUG] Processing batch of {} radar points".format(len(points)))
                    
                    # Process and send data in batches for efficiency
                    if len(points) > 0 and self.radar_flag:
                        try:
                            # Send batch size first
                            num_points = struct.pack('!I', len(points))  # Pack point count as unsigned int
                            self.radar_socket.sendall(num_points)  # Send batch size
                            
                            # Prepare and send all points in one batch
                            batch_data = bytearray()
                            for point in points:
                                point_data = struct.pack('!ffff', point[0], point[1], point[2], point[3])  # Pack as 4 floats
                                batch_data.extend(point_data)  # Add to batch buffer
                            
                            self.radar_socket.sendall(batch_data)  # Send complete batch
                            point_counter += len(points)
                            
                            # Periodic progress logging
                            if point_counter % 50 == 0:
                                print("[RADAR DEBUG] Sent batch of {} points, total: {}".format(len(points), point_counter))
                        except socket.error as e:
                            print("[RADAR ERROR] Socket error in batch send: {}".format(e))  # Log network errors
                else:
                    time.sleep(0.001)  # Prevent CPU overuse when queue is empty
            except Exception as e:
                print("[RADAR ERROR] Processing error: {}".format(e))  # Log general processing errors
                import traceback
                traceback.print_exc()
    
    def process_imu_queue(self):  # Process and transmit IMU sensor data
        while self.running:
            try:
                if not self.imu_queue.empty():
                    imu_data = self.imu_queue.get()  # Retrieve IMU measurements from queue
                    # Pack all IMU measurements (acceleration, gyroscope, compass) into a single packet
                    data = struct.pack('fffffff', 
                                     imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z,  # 3D acceleration
                                     imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z,  # 3D angular velocity
                                     imu_data.compass)  # Magnetic heading
                    
                    try:
                        if self.imu_flag:
                            
                            self.imu_socket.sendall(data)  # Send IMU data over TCP
                    except socket.error as e:
                        print("IMU socket error: {0}".format(e))
                else:
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
            except Exception as e:
                print("Error in IMU processing thread: {0}".format(e))
    
    def process_camera_queue(self):  # Process and transmit camera image data
        frame_counter = 0  # Counter for monitoring frame processing
        while self.running:
            try:
                # Handle client connections for camera streaming
                if not hasattr(self, 'camera_client') or self.camera_client is None:
                    try:
                        self.camera_client, addr = self.camera_socket.accept()  # Wait for client connection
                        print("[CAMERA] Connected to client at {0}".format(addr))
                    except socket.timeout:
                        # No connection available, continue waiting
                        time.sleep(0.1)
                        continue
                    except Exception as e:
                        print("[CAMERA] Connection error: {0}".format(e))  # Log connection errors
                        time.sleep(0.5)
                        continue

                if not self.camera_queue.empty():
                    camera_data = self.camera_queue.get()  # Retrieve image from queue
                    
                    raw_data = camera_data.raw_data  # Extract raw image bytes
                    
                    # Periodic logging for monitoring
                    frame_counter += 1
                    if frame_counter % 10 == 0:  # Log every 10th frame
                        print("[CAMERA DEBUG] Sending camera frame #{0}, size: {1} bytes".format(frame_counter, len(raw_data)))
                    
                    try:
                        if self.camera_flag and self.camera_client:
                            size_header = struct.pack('!I', len(raw_data))  # Pack image size as unsigned int
                            self.camera_client.sendall(size_header)  # Send size header first
                            self.camera_client.sendall(raw_data)  # Send image data
                    except (BrokenPipeError, ConnectionResetError, socket.error) as e:
                        print("[CAMERA] Client disconnected: {0}".format(e))  # Log client disconnection
                        self.camera_client = None
                else:
                    time.sleep(0.001)  # Prevent CPU overuse when queue is empty
            except Exception as e:
                print("Error in Camera processing thread: {0}".format(e))  # Log general processing errors
                import traceback
                traceback.print_exc()
                time.sleep(0.5)  # Longer sleep after error to prevent rapid retries
    
    def update_velocity(self):  # Update and transmit vehicle velocity data
        """Update and publish vehicle velocity data"""
        reconnect_attempts = 0  # Counter for connection retry attempts
        last_reconnect_time = 0  # Timestamp of last reconnection attempt
        max_reconnect_attempts = 5  # Maximum number of reconnection attempts
        reconnect_cooldown = 5  # Seconds between reconnection attempts
        
        while self.running:
            try:
                if self.vehicle and self.vehicle.is_alive:
                    velocity = self.vehicle.get_velocity()  # Get current vehicle velocity
                    
                    # Update queue with latest velocity data
                    if self.velocity_queue.full():
                        try:
                            self.velocity_queue.get_nowait()  # Remove old velocity data
                        except Queue.Empty:
                            pass
                    self.velocity_queue.put(velocity)  # Add new velocity data
                    
                    # Process and transmit velocity data
                    if not self.velocity_queue.empty():
                        velocity_data = self.velocity_queue.get()  # Get velocity from queue
                        try:
                            if self.velocity_flag:
                                # Pack velocity components into network packet
                                data = struct.pack('!fff', 
                                                 velocity_data.x,  # X component of velocity
                                                 velocity_data.y,  # Y component of velocity
                                                 velocity_data.z)  # Z component of velocity
                                self.velocity_socket.sendall(data)  # Send velocity packet
                                reconnect_attempts = 0  # Reset reconnection counter on success
                        except (socket.error, AttributeError) as e:
                            print("Velocity socket error: {}".format(e))  # Log network errors
                            
                            # Handle connection loss with reconnection logic
                            current_time = time.time()
                            if (current_time - last_reconnect_time > reconnect_cooldown and 
                                reconnect_attempts < max_reconnect_attempts and 
                                self.velocity_flag):
                                
                                print("Attempting to reconnect velocity socket (attempt {}/{})...".format(
                                    reconnect_attempts + 1, max_reconnect_attempts))
                                
                                try:
                                    # Clean up existing socket
                                    if hasattr(self, 'velocity_socket'):
                                        try:
                                            self.velocity_socket.close()  # Close old socket
                                        except:
                                            pass
                                    
                                    # Create and configure new socket
                                    self.velocity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                                    self.velocity_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Optimize for real-time data
                                    self.velocity_socket.settimeout(2.0)  # Set connection timeout
                                    
                                    self.velocity_socket.connect((self.host_ip, self.velocity_port))  # Attempt reconnection
                                    print("Velocity socket reconnected successfully")
                                    
                                    self.velocity_socket.settimeout(None)  # Reset to blocking mode
                                except Exception as reconnect_error:
                                    print("Failed to reconnect velocity socket: {}".format(reconnect_error))  # Log reconnection failure
                                    reconnect_attempts += 1
                                
                                last_reconnect_time = current_time  # Update last attempt timestamp
                            
                time.sleep(0.05)  # 20Hz update rate
                
            except Exception as e:
                print("Error in velocity update: {}".format(e))  # Log general processing errors
                traceback.print_exc()
                time.sleep(1.0)  # Longer sleep after error
    
    def lidar_callback(self, point_cloud):  # Callback for handling new LiDAR data
        try:
            if self.lidar_queue.full():
                try:
                    self.lidar_queue.get(block=False)  # Remove oldest data if queue is full
                except Queue.Empty:
                    pass
            self.lidar_queue.put(point_cloud, block=False)  # Add new point cloud to queue
        except Exception as e:
            print("Error in LiDAR callback: {0}".format(e))  # Log callback errors
        
    def radar_callback(self, radar_data):  # Callback for handling new RADAR data
        try:
            if self.radar_queue.full():
                try:
                    self.radar_queue.get(block=False)  # Remove oldest data if queue is full
                except Queue.Empty:
                    pass
            self.radar_queue.put(radar_data, block=False)  # Add new radar data to queue
        except Exception as e:
            print("Error in Radar callback: {0}".format(e))  # Log callback errors
        
    def imu_callback(self, imu_data):  # Callback for handling new IMU data
        try:
            if self.imu_queue.full():
                try:
                    self.imu_queue.get(block=False)  # Remove oldest data if queue is full
                except Queue.Empty:
                    pass
            self.imu_queue.put(imu_data, block=False)  # Add new IMU data to queue
        except Exception as e:
            print("Error in IMU callback: {0}".format(e))  # Log callback errors
        
    def camera_callback(self, image):  # Callback for handling new camera images
        try:
            if self.camera_queue.full():
                try:
                    self.camera_queue.get(block=False)  # Remove oldest image if queue is full
                except Queue.Empty:
                    pass
            self.camera_queue.put(image, block=False)  # Add new image to queue
        except Exception as e:
            print("Error in Camera callback: {0}".format(e))  # Log callback errors
        
    def generate_waypoints_ahead(self, distance=100.0, lane_change=False):  # Generate future path waypoints
        """Generate waypoints ahead of the vehicle in coordinates relative to the vehicle"""
        if not self.vehicle or not self.waypoint_flag:
            return []
            
        # Get current vehicle pose
        vehicle_transform = self.vehicle.get_transform()  # Get vehicle position and orientation
        vehicle_location = vehicle_transform.location  # Extract vehicle position
        
        # Initialize waypoint generation
        waypoint = self.map.get_waypoint(vehicle_location)  # Find closest road waypoint
        
        # Setup waypoint containers
        absolute_waypoints = [waypoint]  # Waypoints in world coordinates
        relative_waypoints = []  # Waypoints in vehicle-relative coordinates
        distance_covered = 0.0  # Track total path length
        
        while distance_covered < distance:
            next_waypoints = waypoint.next(self.waypoint_distance)  # Get next waypoints along road
            
            if not next_waypoints:
                break
                
            waypoint = next_waypoints[0]  # Take first waypoint
            distance_covered += self.waypoint_distance
            
            # Optional lane change logic
            if lane_change and distance_covered > distance / 2 and random.random() > 0.8:  # Random lane changes after halfway
                if waypoint.get_right_lane():
                    waypoint = waypoint.get_right_lane()  # Switch to right lane if available
                elif waypoint.get_left_lane():
                    waypoint = waypoint.get_left_lane()  # Switch to left lane if right not available
            
            absolute_waypoints.append(waypoint)  # Add waypoint to world coordinate list
        
        # Transform waypoints from world to vehicle-relative coordinates
        for wp in absolute_waypoints:
            wp_location = wp.transform.location  # Get waypoint's world position
            
            # Calculate vector from vehicle to waypoint in world frame
            relative_vector = carla.Location(
                x=wp_location.x - vehicle_location.x,  # X displacement in world
                y=wp_location.y - vehicle_location.y,  # Y displacement in world
                z=wp_location.z - vehicle_location.z   # Z displacement in world
            )
            
            # Transform from world to vehicle-local coordinates
            yaw_rad = math.radians(vehicle_transform.rotation.yaw)  # Vehicle heading in radians
            cos_yaw = math.cos(yaw_rad)  # Precompute for efficiency
            sin_yaw = math.sin(yaw_rad)  # Precompute for efficiency
            
            # Apply 2D rotation matrix to transform coordinates
            local_x = cos_yaw * relative_vector.x + sin_yaw * relative_vector.y  # Forward distance in vehicle frame
            local_y = -sin_yaw * relative_vector.x + cos_yaw * relative_vector.y  # Lateral distance in vehicle frame
            local_z = relative_vector.z  # Vertical distance remains unchanged
            
            # Check for available lanes
            right_lane = wp.get_right_lane()  # Get adjacent right lane
            has_right_lane = 0  # Flag for valid right lane
            
            if right_lane is not None:
                # Verify right lane is drivable (not sidewalk, shoulder, or parking)
                if (right_lane.lane_type != carla.LaneType.Sidewalk and 
                    right_lane.lane_type != carla.LaneType.Shoulder and 
                    right_lane.lane_type != carla.LaneType.Parking):
                    has_right_lane = 1  # Mark as having valid right lane
            
            # Create waypoint dictionary with all relevant information
            relative_wp = {
                'x': local_x,  # Forward distance in vehicle frame
                'y': local_y,  # Lateral distance in vehicle frame
                'z': local_z,  # Vertical distance in vehicle frame
                'road_id': wp.road_id,  # Current road identifier
                'lane_id': wp.lane_id,  # Current lane identifier
                'lane_type': int(wp.lane_type),  # Type of current lane as integer value
                'has_right_lane': has_right_lane  # Whether lane change right is possible
            }
            
            relative_waypoints.append(relative_wp)  # Add to vehicle-relative waypoint list
        
        print("Generated {} waypoints over {:.1f} meters (relative to vehicle)".format(len(relative_waypoints), distance_covered))
        return relative_waypoints
        
    def process_waypoint_queue(self):  # Process and transmit waypoint data
        """Process waypoints from queue and update map"""
        waypoint_counter = 0  # Counter for monitoring waypoint processing
        while self.running:
            try:
                # Update waypoints periodically
                if self.waypoint_flag and (not self.waypoints or waypoint_counter % 10 == 0):
                    self.waypoints = self.generate_waypoints_ahead(distance=100.0)  # Generate new path
                    
                    # Update waypoint queue with new data
                    if self.waypoints:
                        if self.waypoint_queue.full():
                            try:
                                self.waypoint_queue.get_nowait()  # Remove old path
                            except Queue.Empty:
                                pass
                        self.waypoint_queue.put(self.waypoints)  # Add new path
                
                if not self.waypoint_queue.empty():
                    waypoints = self.waypoint_queue.get()  # Get waypoints for processing
                    
                    # Periodic logging for monitoring
                    waypoint_counter += 1
                    if waypoint_counter % 10 == 0:
                        print("[WAYPOINT DEBUG] Processing batch of {} waypoints".format(len(waypoints)))
                    
                    # Process and transmit waypoints
                    if len(waypoints) > 0 and self.waypoint_flag:
                        try:
                            # Send batch size header
                            num_waypoints = struct.pack('!I', len(waypoints))  # Pack count as unsigned int
                            self.waypoint_socket.sendall(num_waypoints)  # Send waypoint count
                            
                            # Prepare complete batch of waypoint data
                            batch_data = bytearray()
                            for waypoint in waypoints:
                                # Pack waypoint data: position, road info, and lane info
                                # Pack position and road info
                                position_data = struct.pack('!fffiiii', 
                                                       waypoint['x'],  # Forward distance
                                                       waypoint['y'],  # Lateral distance
                                                       waypoint['z'],  # Vertical distance
                                                       waypoint['road_id'],  # Road identifier
                                                       waypoint['lane_id'],  # Lane identifier
                                                       self.get_current_lane_type(),  # Lane type as integer
                                                       waypoint['has_right_lane'])  # Lane change possibility
                                batch_data.extend(position_data)  # Add to batch buffer
                            
                            self.waypoint_socket.sendall(batch_data)  # Send complete waypoint batch
                            
                            # Periodic progress logging
                            if waypoint_counter % 10 == 0:
                                print("[WAYPOINT DEBUG] Sent batch of {} relative waypoints".format(len(waypoints)))
                        except socket.error as e:
                            print("[WAYPOINT ERROR] Socket error in batch send: {}".format(e))  # Log network errors
                else:
                    time.sleep(0.1)  # Reduced update rate for waypoints
            except Exception as e:
                print("[WAYPOINT ERROR] Processing error: {}".format(e))  # Log processing errors
                import traceback
                traceback.print_exc()

    def generate_waypoints_periodically(self):  # Background thread for continuous path updates
        """Generate waypoints periodically in a separate thread"""
        while self.running and self.waypoint_flag:
            try:
                waypoints = self.generate_waypoints_ahead(distance=100.0)  # Generate 100m path ahead
                
                # Update waypoint queue
                if waypoints:
                    if self.waypoint_queue.full():
                        try:
                            self.waypoint_queue.get_nowait()  # Remove old path
                        except Queue.Empty:
                            pass
                    self.waypoint_queue.put(waypoints)  # Add new path
                    
                    # Debug logging of first waypoint
                    if len(waypoints) > 0:
                        first_wp = waypoints[0]
                        print("First relative waypoint: x={:.2f}, y={:.2f}, z={:.2f}".format(
                            first_wp['x'], first_wp['y'], first_wp['z']))
                    
                time.sleep(1.0)  # Update path every second
                
            except Exception as e:
                print("Error generating waypoints: {}".format(e))  # Log generation errors
                time.sleep(2.0)  # Longer delay after error

    def setup_sensors(self):  # Initialize all enabled vehicle sensors
        try:
            # Create only the sensors that are enabled
            if self.lidar_flag:
                self.setup_lidar()  # Initialize LiDAR sensor
            if self.radar_flag:
                self.setup_radar()  # Initialize RADAR sensor
            if self.imu_flag:
                self.setup_imu()  # Initialize IMU sensor
            if self.camera_flag:
                self.setup_camera()  # Initialize camera sensor
            if self.waypoint_flag:
                self.setup_waypoint()  # Initialize waypoint system
            print("Sensors setup complete")
        except Exception as e:
            print("Error in setup_sensors: {0}".format(e))  # Log setup errors
            raise

    def setup_lidar(self):  # Configure and spawn LiDAR sensor
        try:
            # Configure LiDAR sensor parameters
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')  # Get sensor blueprint
            lidar_bp.set_attribute('channels', '32')  # Number of vertical lasers
            lidar_bp.set_attribute('points_per_second', '100000')  # Scan density
            lidar_bp.set_attribute('rotation_frequency', '20')  # Rotation speed
            lidar_bp.set_attribute('range', '70.0')  # Maximum range
            lidar_bp.set_attribute('upper_fov', '10.0')  # Upward scan angle
            lidar_bp.set_attribute('lower_fov', '-2.0')  # Downward scan angle
            
            # Define sensor mounting position on vehicle
            lidar_transform = carla.Transform(
                carla.Location(x=1.5, z=2.0),  # Mount forward and above vehicle
                carla.Rotation(yaw=270)  # Orient sensor
            )
            
            # Create and attach sensor to vehicle
            self.lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)  # Spawn sensor
            self.actor_list.append(self.lidar)  # Track for cleanup
            self.lidar.listen(self.lidar_callback)  # Start data collection
            print("LiDAR sensor added at position: x=1.5m, z=2.0m")
            
        except Exception as e:
            print("Error in LiDAR setup: {0}".format(str(e)))  # Log setup errors
            raise
        
    def setup_radar(self):  # Configure and spawn RADAR sensor
        try:
            # Configure RADAR sensor parameters
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')  # Get sensor blueprint
            radar_bp.set_attribute('horizontal_fov', '15.0')  # Horizontal scan angle
            radar_bp.set_attribute('vertical_fov', '-15.0')   # Vertical scan angle
            radar_bp.set_attribute('points_per_second', '2000')  # Scan density
            radar_bp.set_attribute('range', '100.0')  # Maximum range
            
            # Mount next to the LiDAR with a slight horizontal offset
            radar_transform = carla.Transform(  # Define RADAR mounting position
                # Position radar at the same x (forward) position as LiDAR but offset to the right (y=0.5)
                carla.Location(x=1.5, y=0.5, z=2.0),  # Mount forward, right, and up from vehicle center
                carla.Rotation()  # Use vehicle's orientation
            )
            
            # Create and attach sensor to vehicle
            self.radar = self.world.spawn_actor(radar_bp, radar_transform, attach_to=self.vehicle)  # Spawn sensor
            self.actor_list.append(self.radar)  # Track for cleanup
            self.radar.listen(self.radar_callback)  # Start data collection
            print("Radar sensor added at position: x=1.5m, y=0.5m, z=2.0m")
            
        except Exception as e:
            print("Error in Radar setup: {0}".format(str(e)))  # Log setup errors
            raise

    def setup_imu(self):  # Configure and spawn IMU sensor
        try:
            # Configure IMU sensor parameters
            imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')  # Get sensor blueprint
            imu_bp.set_attribute('sensor_tick', '0.05')  # Set 20Hz update rate
            
            # Define sensor mounting position
            imu_transform = carla.Transform(
                carla.Location(x=0.0, z=0.0),  # Mount at vehicle's center of mass
                carla.Rotation()  # Use vehicle's orientation
            )
            
            # Create and attach sensor to vehicle
            self.imu = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)  # Spawn sensor
            self.actor_list.append(self.imu)  # Track for cleanup
            self.imu.listen(self.imu_callback)  # Start data collection
            print("IMU sensor added at position: x=0.0m, z=0.0m")
            
        except Exception as e:
            print("Error in IMU setup: {0}".format(str(e)))  # Log setup errors
            raise

    def setup_camera(self):  # Configure and spawn camera sensor
        try:
            # Configure camera sensor parameters
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')  # Get RGB camera blueprint
            camera_bp.set_attribute('image_size_x', '640')  # Set image width
            camera_bp.set_attribute('image_size_y', '480')  # Set image height
            camera_bp.set_attribute('fov', '90')  # Set horizontal field of view
            camera_bp.set_attribute('sensor_tick', '0.1')  # Set 10Hz capture rate
            
            # Define sensor mounting position
            camera_transform = carla.Transform(
                carla.Location(x=2.0, z=1.5),  # Mount forward and up from vehicle center
                carla.Rotation(pitch=-15.0)    # Tilt down for better road view
            )
            
            # Create and attach sensor to vehicle
            self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)  # Spawn sensor
            self.actor_list.append(self.camera)  # Track for cleanup
            self.camera.listen(self.camera_callback)  # Start data collection
            print("Camera sensor added at position: x=2.0m, z=1.5m, pitch=-15°")
            
        except Exception as e:
            print("Error in Camera setup: {0}".format(e))  # Log setup errors
            raise

    def setup_waypoint(self):  # Initialize waypoint generation system
        try:
            print("Setting up waypoint generation...")
            
            # Initialize waypoint generation (no physical sensor needed)
            
            # Start background waypoint generation if enabled
            if self.waypoint_flag:
                # Generate initial path
                try:
                    initial_waypoints = self.generate_waypoints_ahead(distance=100.0)  # Create initial path
                    if initial_waypoints:
                        self.waypoints = initial_waypoints  # Store current path
                        if self.waypoint_queue.full():
                            try:
                                self.waypoint_queue.get_nowait()  # Remove old path
                            except Queue.Empty:
                                pass
                        self.waypoint_queue.put(initial_waypoints)  # Queue new path
                        print("Initial waypoints generated: {} (relative to vehicle)".format(len(initial_waypoints)))
                        
                        # Verify first waypoint format
                        if len(initial_waypoints) > 0:
                            first_wp = initial_waypoints[0]
                            print("First relative waypoint: x={:.2f}, y={:.2f}, z={:.2f}".format(
                                first_wp['x'], first_wp['y'], first_wp['z']))
                        
                        # Start continuous path generation thread
                        self.waypoint_generation_thread = threading.Thread(target=self.generate_waypoints_periodically)  # Create updater thread
                        self.waypoint_generation_thread.daemon = True  # Set as background thread
                        self.waypoint_generation_thread.start()  # Begin updates
                        print("Waypoint generation thread started")
                except Exception as e:
                    print("Error generating initial waypoints: {}".format(e))  # Log generation errors
            
            print("Waypoint generation setup complete")
            
        except Exception as e:
            print("Error in Waypoint setup: {0}".format(e))  # Log setup errors
            raise

    def toggle_waypoint(self):  # Enable/disable waypoint generation
        """Toggle the waypoint sensor on/off"""
        self.waypoint_flag = not self.waypoint_flag  # Switch state
        print("Waypoint sensor toggled: {}".format(self.waypoint_flag))
        
        # Initialize network connection when enabled
        if self.waypoint_flag and not hasattr(self, 'waypoint_socket'):
            try:
                self.waypoint_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                self.waypoint_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Optimize for real-time data
                try:
                    self.waypoint_socket.connect((self.host_ip, self.waypoint_port))  # Connect to server
                    print("Waypoint TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print("Waypoint TCP connection failed: {}. Will continue with local data only.".format(e))
            except Exception as e:
                print("Error setting up waypoint socket: {}".format(e))  # Log network errors
        
        # Initialize path generation when enabled
        if self.waypoint_flag:
            try:
                # Generate initial path in vehicle-relative coordinates
                initial_waypoints = self.generate_waypoints_ahead(distance=100.0)  # Create initial path
                if initial_waypoints:
                    self.waypoints = initial_waypoints  # Store current path
                    if self.waypoint_queue.full():
                        try:
                            self.waypoint_queue.get_nowait()  # Remove old path
                        except Queue.Empty:
                            pass
                    self.waypoint_queue.put(initial_waypoints)  # Queue new path
                    print("Generated {} relative waypoints when toggling on".format(len(initial_waypoints)))
                    
                    # Verify waypoint format
                    if len(initial_waypoints) > 0:
                        first_wp = initial_waypoints[0]
                        print("First relative waypoint: x={:.2f}, y={:.2f}, z={:.2f}".format(
                            first_wp['x'], first_wp['y'], first_wp['z']))
            except Exception as e:
                print("Error generating waypoints when toggling: {}".format(e))  # Log generation errors
        
        # Cleanup network connection when disabled
        elif not self.waypoint_flag and hasattr(self, 'waypoint_socket'):
            try:
                self.waypoint_socket.shutdown(socket.SHUT_RDWR)  # Stop data transfer
                self.waypoint_socket.close()  # Close connection
                delattr(self, 'waypoint_socket')  # Remove socket reference
                print("Waypoint socket closed")
            except Exception as e:
                print("Error closing waypoint socket: {}".format(e))  # Log cleanup errors

    def toggle_lidar(self):  # Enable/disable LiDAR sensor
        """Toggle the LiDAR sensor on/off"""
        self.lidar_flag = not self.lidar_flag  # Switch state
        print("LiDAR sensor toggled: {}".format(self.lidar_flag))
        
        # Initialize systems when enabled
        if self.lidar_flag:
            # Setup network connection if needed
            if not hasattr(self, 'lidar_socket'):
                try:
                    self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                    self.lidar_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Optimize for real-time data
                    try:
                        self.lidar_socket.connect((self.host_ip, self.lidar_port))  # Connect to server
                        print("LiDAR TCP connected")
                    except (ConnectionRefusedError, socket.error) as e:
                        print("LiDAR TCP connection failed: {}. Will continue with local data only.".format(e))
                except Exception as e:
                    print("Error setting up LiDAR socket: {}".format(e))  # Log network errors
            
            # Create sensor if needed
            if not hasattr(self, 'lidar') or not self.lidar.is_alive:
                try:
                    self.setup_lidar()  # Initialize sensor
                    print("LiDAR sensor created")
                except Exception as e:
                    print("Error creating LiDAR sensor: {}".format(e))  # Log sensor errors
            
            # Start data processing if needed
            if not self.lidar_thread or not self.lidar_thread.is_alive():
                self.lidar_thread = threading.Thread(target=self.process_lidar_queue)  # Create processor thread
                self.lidar_thread.daemon = True  # Set as background thread
                self.lidar_thread.start()  # Begin processing
                print("LiDAR processing thread started")
        
        # Cleanup when disabled
        else:
            # Remove sensor if active
            if hasattr(self, 'lidar') and self.lidar.is_alive:
                try:
                    self.lidar.stop()  # Stop data collection
                    self.actor_list.remove(self.lidar)  # Remove from tracking
                    self.lidar.destroy()  # Delete sensor
                    delattr(self, 'lidar')  # Remove reference
                    print("LiDAR sensor destroyed")
                except Exception as e:
                    print("Error destroying LiDAR sensor: {}".format(e))  # Log cleanup errors

    def toggle_radar(self):  # Enable/disable RADAR sensor
        """Toggle the RADAR sensor on/off"""
        self.radar_flag = not self.radar_flag  # Switch state
        print("RADAR sensor toggled: {}".format(self.radar_flag))
        
        # Initialize systems when enabled
        if self.radar_flag:
            # Setup network connection if needed
            if not hasattr(self, 'radar_socket'):
                try:
                    self.radar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                    self.radar_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Optimize for real-time data
                    try:
                        self.radar_socket.connect((self.host_ip, self.radar_port))  # Connect to server
                        print("RADAR TCP connected")
                    except (ConnectionRefusedError, socket.error) as e:
                        print("RADAR TCP connection failed: {}. Will continue with local data only.".format(e))
                except Exception as e:
                    print("Error setting up RADAR socket: {}".format(e))  # Log network errors
            
            # Create sensor if needed
            if not hasattr(self, 'radar') or not self.radar.is_alive:
                try:
                    self.setup_radar()  # Initialize sensor
                    print("RADAR sensor created")
                except Exception as e:
                    print("Error creating RADAR sensor: {}".format(e))  # Log sensor errors
            
            # Start data processing if needed
            if not self.radar_thread or not self.radar_thread.is_alive():
                self.radar_thread = threading.Thread(target=self.process_radar_queue)  # Create processor thread
                self.radar_thread.daemon = True  # Set as background thread
                self.radar_thread.start()  # Begin processing
                print("RADAR processing thread started")
        
        # Cleanup when disabled
        else:
            # Remove sensor if active
            if hasattr(self, 'radar') and self.radar.is_alive:
                try:
                    self.radar.stop()  # Stop data collection
                    self.actor_list.remove(self.radar)  # Remove from tracking
                    self.radar.destroy()  # Delete sensor
                    delattr(self, 'radar')  # Remove reference
                    print("RADAR sensor destroyed")
                except Exception as e:
                    print("Error destroying RADAR sensor: {}".format(e))  # Log cleanup errors
            
            # Close network connection if exists
            if hasattr(self, 'radar_socket'):
                try:
                    self.radar_socket.shutdown(socket.SHUT_RDWR)  # Stop data transfer
                    self.radar_socket.close()  # Close connection
                    delattr(self, 'radar_socket')  # Remove reference
                    print("RADAR socket closed")
                except Exception as e:
                    print("Error closing RADAR socket: {}".format(e))  # Log cleanup errors

    def toggle_imu(self):  # Enable/disable IMU sensor
        """Toggle the IMU sensor on/off"""
        self.imu_flag = not self.imu_flag  # Switch state
        print("IMU sensor toggled: {}".format(self.imu_flag))
        
        # Initialize systems when enabled
        if self.imu_flag:
            # Setup network connection if needed
            if not hasattr(self, 'imu_socket'):
                try:
                    self.imu_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                    self.imu_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Optimize for real-time data
                    try:
                        self.imu_socket.connect((self.host_ip, self.imu_port))  # Connect to server
                        print("IMU TCP connected")
                    except (ConnectionRefusedError, socket.error) as e:
                        print("IMU TCP connection failed: {}. Will continue with local data only.".format(e))
                except Exception as e:
                    print("Error setting up IMU socket: {}".format(e))  # Log network errors
            
            # Create sensor if needed
            if not hasattr(self, 'imu') or not self.imu.is_alive:
                try:
                    self.setup_imu()  # Initialize sensor
                    print("IMU sensor created")
                except Exception as e:
                    print("Error creating IMU sensor: {}".format(e))  # Log sensor errors
            
            # Start data processing if needed
            if not self.imu_thread or not self.imu_thread.is_alive():
                self.imu_thread = threading.Thread(target=self.process_imu_queue)  # Create processor thread
                self.imu_thread.daemon = True  # Set as background thread
                self.imu_thread.start()  # Begin processing
                print("IMU processing thread started")
        
        # Cleanup when disabled
        else:
            # Remove sensor if active
            if hasattr(self, 'imu') and self.imu.is_alive:
                try:
                    self.imu.stop()  # Stop data collection
                    self.actor_list.remove(self.imu)  # Remove from tracking
                    self.imu.destroy()  # Delete sensor
                    delattr(self, 'imu')  # Remove reference
                    print("IMU sensor destroyed")
                except Exception as e:
                    print("Error destroying IMU sensor: {}".format(e))  # Log cleanup errors
            
            # Close network connection if exists
            if hasattr(self, 'imu_socket'):
                try:
                    self.imu_socket.shutdown(socket.SHUT_RDWR)  # Stop data transfer
                    self.imu_socket.close()  # Close connection
                    delattr(self, 'imu_socket')  # Remove reference
                    print("IMU socket closed")
                except Exception as e:
                    print("Error closing IMU socket: {}".format(e))  # Log cleanup errors

    def toggle_camera(self):  # Enable/disable camera sensor
        """Toggle the camera sensor on/off"""
        self.camera_flag = not self.camera_flag  # Switch state
        print("Camera sensor toggled: {}".format(self.camera_flag))
        
        # Initialize systems when enabled
        if self.camera_flag:
            # Setup network server if needed
            if not hasattr(self, 'camera_socket'):
                try:
                    self.camera_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                    self.camera_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Enable address reuse
                    self.camera_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Optimize for real-time data
                    try:
                        self.camera_socket.bind(('0.0.0.0', self.camera_port))  # Listen on all interfaces
                        self.camera_socket.listen(1)  # Accept single client
                        self.camera_socket.settimeout(0.5)  # Enable non-blocking mode
                        print("Camera TCP server listening on port {}".format(self.camera_port))
                        self.camera_client = None  # Initialize client state
                    except Exception as e:
                        print("Error setting up camera server: {}".format(e))  # Log server errors
                except Exception as e:
                    print("Error setting up camera socket: {}".format(e))  # Log socket errors
            
            # Create sensor if needed
            if not hasattr(self, 'camera') or not self.camera.is_alive:
                try:
                    self.setup_camera()  # Initialize sensor
                    print("Camera sensor created")
                except Exception as e:
                    print("Error creating camera sensor: {}".format(e))  # Log sensor errors
            
            # Start data processing if needed
            if not self.camera_thread or not self.camera_thread.is_alive():
                self.camera_thread = threading.Thread(target=self.process_camera_queue)  # Create processor thread
                self.camera_thread.daemon = True  # Set as background thread
                self.camera_thread.start()  # Begin processing
                print("Camera processing thread started")
        
        # Cleanup when disabled
        else:
            # Remove sensor if active
            if hasattr(self, 'camera') and self.camera.is_alive:
                try:
                    self.camera.stop()  # Stop data collection
                    self.actor_list.remove(self.camera)  # Remove from tracking
                    self.camera.destroy()  # Delete sensor
                    delattr(self, 'camera')  # Remove reference
                    print("Camera sensor destroyed")
                except Exception as e:
                    print("Error destroying camera sensor: {}".format(e))  # Log cleanup errors
            
            # Close network connections if exist
            if hasattr(self, 'camera_socket'):
                try:
                    if hasattr(self, 'camera_client') and self.camera_client:
                        self.camera_client.close()  # Close client connection
                    self.camera_socket.shutdown(socket.SHUT_RDWR)  # Shutdown camera socket
                    self.camera_socket.close()  # Close server socket
                    delattr(self, 'camera_socket')  # Remove reference
                    print("Camera socket closed")
                except Exception as e:
                    print("Error closing camera socket: {}".format(e))  # Log cleanup errors
    
    def toggle_velocity(self):  # Enable/disable velocity data streaming
        """Toggle velocity publishing on/off"""
        self.velocity_flag = not self.velocity_flag  # Switch state
        print("Velocity publishing toggled: {}".format(self.velocity_flag))
        
        # Initialize network connection when enabled
        if self.velocity_flag and not hasattr(self, 'velocity_socket'):
            try:
                self.velocity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                self.velocity_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Optimize for real-time data
                try:
                    self.velocity_socket.connect((self.host_ip, self.velocity_port))  # Connect to server
                    print("Velocity TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print("Velocity TCP connection failed: {}. Will continue with local data only.".format(e))
            except Exception as e:
                print("Error setting up velocity socket: {}".format(e))  # Log network errors
        
        # Cleanup network connection when disabled
        elif not self.velocity_flag and hasattr(self, 'velocity_socket'):
            try:
                self.velocity_socket.shutdown(socket.SHUT_RDWR)
                self.velocity_socket.close()
                delattr(self, 'velocity_socket')
                print("Velocity socket closed")
            except Exception as e:
                print("Error closing velocity socket: {}".format(e))
    
    def cleanup(self):
        print("Cleaning up sensors and sockets...")
        self.running = False  # Signal threads to stop
        
        # Wait for processing threads to finish
        if self.lidar_thread and self.lidar_thread.is_alive():
            self.lidar_thread.join(timeout=1.0)  # Wait for LiDAR thread to finish
        if self.radar_thread and self.radar_thread.is_alive():
            self.radar_thread.join(timeout=1.0)  # Wait for RADAR thread to finish
        if self.imu_thread and self.imu_thread.is_alive():
            self.imu_thread.join(timeout=1.0)  # Wait for IMU thread to finish
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=1.0)  # Wait for camera thread to finish
        if self.waypoint_thread and self.waypoint_thread.is_alive():
            self.waypoint_thread.join(timeout=1.0)  # Wait for waypoint thread to finish
        if self.velocity_thread and self.velocity_thread.is_alive():
            self.velocity_thread.join(timeout=1.0)  # Wait for velocity thread to finish
        
        # Clean up actors
        for actor in self.actor_list:
            if actor is not None and actor.is_alive:
                actor.destroy()  # Destroy all sensor actors
        
        # Close sockets
        if hasattr(self, 'lidar_socket'):
            try:
                if self.lidar_flag:
                    self.lidar_socket.shutdown(socket.SHUT_RDWR)  # Shutdown LiDAR socket
                    self.lidar_socket.close()  # Close LiDAR socket
            except Exception as e:
                print("Error closing LiDAR socket: {0}".format(e))
            
        if hasattr(self, 'radar_socket'):
            try:
                if self.radar_flag:
                    self.radar_socket.shutdown(socket.SHUT_RDWR)  # Shutdown RADAR socket
                    self.radar_socket.close()  # Close RADAR socket
            except Exception as e:
                print("Error closing Radar socket: {0}".format(e))
                
        if hasattr(self, 'imu_socket'):
            try:
                if self.imu_flag:
                    self.imu_socket.shutdown(socket.SHUT_RDWR)  # Shutdown IMU socket
                    self.imu_socket.close()  # Close IMU socket
            except Exception as e:
                print("Error closing IMU socket: {0}".format(e))
        
        if hasattr(self, 'camera_socket'):
            try:
                if self.camera_flag:
                    if hasattr(self, 'camera_client') and self.camera_client:
                        self.camera_client.close()  # Close camera client connection
                    self.camera_socket.shutdown(socket.SHUT_RDWR)  # Shutdown camera socket
                    self.camera_socket.close()  # Close camera socket
            except Exception as e:
                print("Error closing Camera socket: {0}".format(e))
                
        if hasattr(self, 'waypoint_socket'):
            try:
                if self.waypoint_flag:
                    self.waypoint_socket.shutdown(socket.SHUT_RDWR)  # Shutdown waypoint socket
                    self.waypoint_socket.close()  # Close waypoint socket
            except Exception as e:
                print("Error closing Waypoint socket: {0}".format(e))
                
        if hasattr(self, 'velocity_socket'):
            try:
                if self.velocity_flag:
                    self.velocity_socket.shutdown(socket.SHUT_RDWR)
                    self.velocity_socket.close()
            except Exception as e:
                print("Error closing Velocity socket: {0}".format(e))
                
        print("Sensor cleanup complete")

    def get_current_lane_type(self):
        """Get the current lane type of the vehicle as an integer"""
        if not self.vehicle or not self.vehicle.is_alive:
            return 0  # Return 0 for unknown
            
        try:
            # Get current vehicle location
            vehicle_location = self.vehicle.get_transform().location
            
            # Find the closest waypoint to the vehicle
            # Use include_junctions=True to get waypoints on junctions too
            waypoint = self.map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Any)
            
            if not waypoint:
                return 0  # Return 0 for no waypoint
            
            # Map lane types to human-readable strings for debug output only
            lane_type_mapping = {
                carla.LaneType.NONE: "None",
                carla.LaneType.Driving: "Driving",
                carla.LaneType.Stop: "Stop",
                carla.LaneType.Shoulder: "Shoulder",
                carla.LaneType.Bidirectional: "Bidirectional",
                carla.LaneType.Parking: "Parking",
                carla.LaneType.Restricted: "Restricted",
                carla.LaneType.Border: "Border",
                carla.LaneType.Sidewalk: "Sidewalk",
                carla.LaneType.Biking: "Biking",
                carla.LaneType.Tram: "Tram",
                carla.LaneType.Rail: "Rail"
            }
            
            # Get the lane type as integer
            lane_type = int(waypoint.lane_type)
            
            # Debug print to see both integer value and string representation
            print("Current lane type: {} ({})".format(lane_type, lane_type_mapping.get(waypoint.lane_type, 'Unknown')))
            
            # Return the integer value
            return lane_type
            
        except Exception as e:
            print("Error getting lane type: {}".format(e))
            traceback.print_exc()
            return 0  # Return 0 for error
            
    def has_right_lane(self):
        """Check if there's a lane to the right of the vehicle"""
        if not self.vehicle or not self.vehicle.is_alive:
            return False
            
        try:
            # Get current vehicle location
            vehicle_location = self.vehicle.get_transform().location
            
            # Find the closest waypoint to the vehicle
            waypoint = self.map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Any)
            
            if not waypoint:
                return False
                
            # Check if there's a lane to the right
            right_lane = waypoint.get_right_lane()
            
            # If there's no right lane, return False
            if right_lane is None:
                print("Has right lane: False (no lane)")
                return False
                
            # Check if the right lane is a sidewalk, shoulder, or parking lane
            if (right_lane.lane_type == carla.LaneType.Sidewalk or 
                right_lane.lane_type == carla.LaneType.Shoulder or 
                right_lane.lane_type == carla.LaneType.Parking):
                print("Has right lane: False (lane type: {})".format(right_lane.lane_type))
                return False
                
            # There is a valid right lane
            print("Has right lane: True (lane type: {})".format(right_lane.lane_type))
            return True
            
        except Exception as e:
            print("Error checking right lane: {}".format(e))
            traceback.print_exc()
            return False

    def get_lane_type_string(self, lane_type):
        """Convert lane type enum to string representation"""
        # Map lane types to human-readable strings
        lane_type_mapping = {
            carla.LaneType.NONE: "None",
            carla.LaneType.Driving: "Driving",
            carla.LaneType.Stop: "Stop",
            carla.LaneType.Shoulder: "Shoulder",
            carla.LaneType.Bidirectional: "Bidirectional",
            carla.LaneType.Parking: "Parking",
            carla.LaneType.Restricted: "Restricted",
            carla.LaneType.Border: "Border",
            carla.LaneType.Sidewalk: "Sidewalk",
            carla.LaneType.Biking: "Biking",
            carla.LaneType.Tram: "Tram",
            carla.LaneType.Rail: "Rail"
        }
        return lane_type_mapping.get(lane_type, "Unknown")

class TrafficManager:
    def __init__(self, world, ego_vehicle, client):
        self.world = world  # CARLA simulation world
        self.client = client  # CARLA client connection
        self.ego_vehicle = ego_vehicle  # Player-controlled vehicle
        self.traffic_vehicles = []  # List of AI vehicles
        self.roaming_mode = False  # Whether vehicles follow random paths
        self.actor_list = []  # All spawned actors for cleanup
        self.port = 8000  # Traffic manager network port
        
        # Initialize CARLA traffic management system
        try:
            print("Initializing CARLA traffic manager...")

            try:
                self.tm = self.client.get_trafficmanager(self.port)  # Get traffic manager instance
            except AttributeError:
                # Handle older CARLA versions
                self.tm = None
                print("Traffic manager not available in this CARLA version")
                
            # Configure traffic manager if available
            if self.tm:
                # Set up vehicle behavior parameters
                try:
                    self.tm.set_global_distance_to_leading_vehicle(2.5)  # Set following distance
                except AttributeError:
                    print("set_global_distance_to_leading_vehicle not available")  # Log unsupported feature
                    
                try:
                    self.tm.set_synchronous_mode(True)  # Enable synchronized updates
                except AttributeError:
                    print("set_synchronous_mode not available")  # Log unsupported feature
                    
                try:
                    self.tm.global_percentage_speed_difference(30.0)  # Set speed difference percentage
                except AttributeError:
                    print("global_percentage_speed_difference not available")  # Log unsupported feature
                
            print("Traffic manager initialized successfully")
        except Exception as e:
            print("Error initializing traffic manager: {}".format(e))  # Log initialization error
            self.tm = None  # Reset traffic manager on failure
            
    def spawn_traffic_vehicles(self, num_vehicles=3):  # Spawn AI vehicles in front of player
        """Spawn traffic vehicles in front of the ego vehicle"""
        try:
            print("Spawning {} traffic vehicles...".format(num_vehicles))
            
            # Get available spawn locations
            spawn_points = self.world.get_map().get_spawn_points()  # Get all spawn points
            if not spawn_points:
                print("No spawn points available")  # Log error if no spawn points found
                return
                
            # Calculate spawn positions relative to player
            ego_transform = self.ego_vehicle.get_transform()  # Get player position/rotation
            ego_location = ego_transform.location  # Extract position
            ego_forward_vector = ego_transform.get_forward_vector()  # Get forward direction
            
            # Get vehicle types from asset library
            blueprint_library = self.world.get_blueprint_library()  # Access vehicle blueprints
            
            # Filter for SUVs and large vehicles
            suv_blueprints = []
            for bp in blueprint_library.filter('vehicle.*'):
                # Only consider 4+ wheel vehicles
                if int(bp.get_attribute('number_of_wheels').as_int()) >= 4:
                    # Include SUVs, trucks, vans, and similar large vehicles
                    if any(tag in bp.id.lower() for tag in ['suv', 'offroad', 'truck', 'van', 'jeep', 'rubicon', 'patrol', 'cybertruck']):
                        suv_blueprints.append(bp)
            
            # Fallback to any 4+ wheel vehicle if no SUVs found
            if not suv_blueprints:
                print("No SUV blueprints found, falling back to 4+ wheel vehicles")
                suv_blueprints = [bp for bp in blueprint_library.filter('vehicle.*') 
                                if int(bp.get_attribute('number_of_wheels').as_int()) >= 4]
            
            # Create vehicles at increasing distances
            for i in range(num_vehicles):
                # Calculate distance for this vehicle
                spawn_distance = 20 + (i * 15)  # Space vehicles 15m apart, starting at 20m
                
                # Calculate spawn position in front of player
                spawn_location = carla.Location(
                    x=ego_location.x + ego_forward_vector.x * spawn_distance,  # Forward offset
                    y=ego_location.y + ego_forward_vector.y * spawn_distance,  # Lateral offset
                    z=ego_location.z + 0.5  # Height offset for ground clearance
                )
                
                # Find nearest valid spawn point
                closest_spawn_point = None
                min_distance = float('inf')
                for spawn_point in spawn_points:
                    dist = spawn_location.distance(spawn_point.location)  # Calculate distance to desired position
                    if dist < min_distance:
                        min_distance = dist
                        closest_spawn_point = spawn_point  # Update closest point
                
                if not closest_spawn_point:
                    print("Could not find a valid spawn point for vehicle {}".format(i+1))  # Log spawn point error
                    continue
                
                # Select random vehicle type
                vehicle_bp = random.choice(suv_blueprints)  # Choose random SUV blueprint
                
                # Create vehicle instance
                try:
                    vehicle = self.world.spawn_actor(vehicle_bp, closest_spawn_point)  # Spawn vehicle
                    if vehicle:
                        self.traffic_vehicles.append(vehicle)  # Track for management
                        print("Spawned {} at {}".format(vehicle.type_id, closest_spawn_point.location))
                        
                        # Enable basic AI control
                        vehicle.set_autopilot(True)  # Enable built-in autopilot
                        
                        # Configure advanced AI if available
                        if self.tm:
                            try:
                                # Set random speed variation
                                self.tm.vehicle_percentage_speed_difference(vehicle, random.uniform(-20, 10))  # Vary speed -20% to +10%
                            except AttributeError:
                                pass  # Skip if feature not available
                    else:
                        print("Failed to spawn vehicle {}".format(i+1))  # Log spawn failure
                except Exception as e:
                    print("Failed to spawn vehicle {}".format(i+1))
                    print("Error spawning vehicle {}: {}".format(i+1, e))  # Log spawn error
            
            print("Successfully spawned {} traffic vehicles".format(len(self.traffic_vehicles)))
            
        except Exception as e:
            print("Error in spawn_traffic_vehicles: {}".format(e))  # Log method error
    
    def spawn_random_traffic_vehicles(self, num_vehicles=3):  # Spawn AI vehicles at random locations
        try:
            print("Spawning {} random traffic vehicles...".format(num_vehicles))
            
            # Remove existing traffic
            for vehicle in self.traffic_vehicles:
                if vehicle and vehicle.is_alive:
                    vehicle.destroy()  # Remove vehicle from simulation
            self.traffic_vehicles = []  # Clear tracking list
            
            # Get available vehicle types
            blueprints = self.world.get_blueprint_library().filter('vehicle.*')  # Get all vehicle blueprints
            
            # Filter for SUVs and large vehicles
            suv_blueprints = []
            for bp in blueprints:
                # Only consider 4+ wheel vehicles
                if int(bp.get_attribute('number_of_wheels').as_int()) >= 4:
                    # Include SUVs, trucks, vans, and similar large vehicles
                    if any(tag in bp.id.lower() for tag in ['suv', 'offroad', 'truck', 'van', 'jeep', 'rubicon', 'patrol', 'cybertruck']):
                        suv_blueprints.append(bp)
            
            # Fallback to any 4+ wheel vehicle if no SUVs found
            if not suv_blueprints:
                print("No SUV blueprints found, falling back to 4+ wheel vehicles")
                suv_blueprints = [bp for bp in blueprints if int(bp.get_attribute('number_of_wheels').as_int()) >= 4]
            
            # Get available spawn locations
            spawn_points = self.world.get_map().get_spawn_points()  # Get all spawn points
            
            if not spawn_points:
                print("No spawn points available")  # Log error if no spawn points found
                return
                
            # Randomize spawn point order
            random.shuffle(spawn_points)  # Mix up spawn locations
            
            # Create vehicles at random points
            spawned_count = 0
            for i in range(num_vehicles):
                if i >= len(spawn_points):
                    break  # Stop if no more spawn points
                
                # Select random vehicle type
                blueprint = random.choice(suv_blueprints)  # Choose random SUV blueprint
                
                # Create vehicle instance
                try:
                    # Customize vehicle appearance
                    if blueprint.has_attribute('color'):
                        color = random.choice(blueprint.get_attribute('color').recommended_values)  # Choose random color
                        blueprint.set_attribute('color', color)  # Apply color
                    
                    # Set vehicle role
                    if blueprint.has_attribute('role_name'):
                        blueprint.set_attribute('role_name', 'traffic')  # Mark as traffic vehicle
                    
                    # Get spawn location
                    spawn_point = spawn_points[i]  # Use next available point
                    
                    # Create vehicle
                    vehicle = self.world.spawn_actor(blueprint, spawn_point)  # Spawn vehicle
                    
                    if vehicle:
                        # Track new vehicle
                        self.traffic_vehicles.append(vehicle)  # Add to traffic list
                        self.actor_list.append(vehicle)  # Add to cleanup list
                        spawned_count += 1
                        
                        # Enable basic AI control
                        vehicle.set_autopilot(True)  # Enable built-in autopilot
                        
                        # Configure advanced AI if available
                        if self.tm:
                            try:
                                # Set random speed variation
                                self.tm.vehicle_percentage_speed_difference(vehicle, random.uniform(-20, 10))  # Vary speed -20% to +10%
                            except AttributeError:
                                pass  # Skip if feature not available
                        
                        print("Spawned {} at {}".format(vehicle.type_id, spawn_point.location))
                
                except Exception as e:
                    print("Failed to spawn vehicle: {}".format(e))  # Log spawn error
            
            print("Successfully spawned {} random traffic vehicles".format(spawned_count))
            
        except Exception as e:
            print("Error in spawn_random_traffic_vehicles: {}".format(e))  # Log method error
    
    def set_vehicles_to_roam(self):  # Enable autonomous driving for all traffic
        try:
            # Verify traffic exists
            if not hasattr(self, 'traffic_vehicles') or len(self.traffic_vehicles) == 0:
                print("No traffic vehicles to control")  # Log error if no vehicles found
                return False
            
            # Toggle roaming mode state
            if not hasattr(self, 'roaming_mode'):
                self.roaming_mode = True
            else:
                self.roaming_mode = not self.roaming_mode  # Toggle roaming mode
            
            for vehicle in self.traffic_vehicles:
                if self.roaming_mode:
                    # Enable autopilot / roaming - basic autopilot should work in all versions
                    vehicle.set_autopilot(True)  # Enable autopilot
                    
                    # Try to configure TM settings only if available
                    if self.tm:
                        try:
                            
                            self.tm.vehicle_percentage_speed_difference(vehicle, random.uniform(-20, 10))  # Set random speed
                        except AttributeError:
                            # Handle case where method isn't available
                            pass
                else:
                    # Disable autopilot / roaming
                    vehicle.set_autopilot(False)  # Disable autopilot
                    
            return self.roaming_mode
        
        except Exception as e:
            print("Error setting vehicles to roam: {}".format(e))
            return False
            
    def remove_last_vehicle(self):
        if hasattr(self, 'traffic_vehicles') and len(self.traffic_vehicles) > 0:
            vehicle = self.traffic_vehicles.pop()  # Remove last vehicle from list
            if vehicle in self.actor_list:
                self.actor_list.remove(vehicle)  # Remove from actor list
            vehicle.destroy()  # Destroy vehicle
            print("Removed {}".format(vehicle.type_id))
            return True
        return False

    def cleanup(self):
        print("Cleaning up traffic...")
        
        if hasattr(self, 'traffic_vehicles'):
            for vehicle in self.traffic_vehicles:
                if vehicle and vehicle.is_alive:
                    vehicle.destroy()  # Destroy all traffic vehicles
        self.traffic_vehicles = []  # Clear traffic vehicles list

class CarlaControl:
    def __init__(self):
        print("\n=== Starting CarlaControl Initialization ===")
        try:
            print("Setting up Pygame...")
            self.setup_pygame()  # Initialize Pygame for GUI
            
            print("Setting up CARLA client...")
            # Control variables
            self.throttle = 0.0  # Throttle control value
            self.brake = 0.0  # Brake control value
            self.steer = 0.0  # Steering control value
            self.reverse = False  # Reverse gear state
            self.vehicle = None  # Player vehicle
            self.running = True  # Main loop control flag
            self.traffic_cars = []  # List to store traffic vehicles
            
            # Throttle control variables
            self.throttle_increment = 0.05  # Throttle increment per frame
            self.throttle_decrement = 0.03  # Throttle decrement per frame when released
            self.accelerating_forward = False  # Flag for W key pressed
            self.accelerating_reverse = False  # Flag for S key pressed
            
            # Initialize sensor manager and traffic manager references
            self.sensor_manager = None  # Will hold sensor manager instance
            self.traffic_manager = None  # Will hold traffic manager instance
            
            # TCP control server settings
            self.control_server_port = 12344  # Port for control server
            self.control_server_socket = None  # Server socket
            self.control_client_socket = None  # Client socket
            self.control_server_running = False  # Server running flag
            
            # Set up CARLA client and spawn vehicle
            self.setup_carla_client()  # Connect to CARLA
            
            # Set up TCP control server
            self.setup_control_server()  # Setup control server
            
            print("CarlaControl initialization complete")
            
        except Exception as e:
            print("ERROR in CarlaControl initialization: {0}".format(str(e)))
            self.cleanup()
            raise
            
    def setup_control_server(self):
        """Set up TCP server to receive control commands from external clients"""
        try:
            print("Setting up TCP control server on port {}...".format(self.control_server_port))
            self.control_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
            self.control_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
            self.control_server_socket.bind(('0.0.0.0', self.control_server_port))  # Bind to all interfaces
            self.control_server_socket.listen(1)  # Listen for connections
            self.control_server_running = True  # Set running flag
            
            # Start a thread to accept connections
            self.control_server_thread = threading.Thread(target=self.accept_control_connections)  # Create thread
            self.control_server_thread.daemon = True  # Set as daemon thread
            self.control_server_thread.start()  # Start thread
            print("TCP control server started successfully")
            
        except Exception as e:
            print("Error setting up TCP control server: {}".format(e))
            self.control_server_running = False
            
    def accept_control_connections(self):
        """Accept client connections for vehicle control"""
        while self.control_server_running:
            try:
                print("Waiting for control client connection...")
                # Set timeout to allow checking if server is still running
                self.control_server_socket.settimeout(1.0)  # Set timeout
                
                # Accept connection
                client_socket, client_address = self.control_server_socket.accept()  # Accept client connection
                print("Control client connected from {}".format(client_address))
                
                # Close previous client socket if exists
                if self.control_client_socket:
                    self.control_client_socket.close()
                    
                self.control_client_socket = client_socket
                
                # Start a thread to handle this client
                client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
                client_thread.daemon = True
                client_thread.start()
                
            except socket.timeout:
                # This is expected - just retry
                pass
            except Exception as e:
                if self.control_server_running:  # Only print error if we're still supposed to be running
                    print("Error accepting control connection: {}".format(e))
                    traceback.print_exc()
                time.sleep(1)  # Sleep to avoid tight loop
                
    def handle_client(self, client_socket):
        """Handle client connection and parse incoming control commands"""
        buffer = ""
        while self.control_server_running:
            try:
                data = client_socket.recv(1024)
                if not data:
                    print("Control client disconnected")
                    break
                    
                # Decode and add to buffer
                buffer += data.decode('utf-8')
                
                # Process complete commands (might receive multiple or partial commands)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.parse_control_command(line.strip())
                    
            except Exception as e:
                print("Error handling control client: {}".format(e))
                break
                
        # Clean up
        try:
            client_socket.close()
        except:
            pass
            
    def parse_control_command(self, command):
        """Parse the command string in format 'throttle,steering,brake,reverse'"""
        try:
            parts = command.split(',')
            if len(parts) >= 3:  # Accept at least 3 parts
                self.throttle = float(parts[0])
                self.steer = float(parts[1])
                self.brake = float(parts[2])
                
                # Check if reverse flag is included
                if len(parts) >= 4:
                    self.reverse = bool(int(parts[3]))
                
                # Clamp values to valid ranges
                self.throttle = max(0.0, min(1.0, self.throttle))
                self.steer = max(-1.0, min(1.0, self.steer))
                self.brake = max(0.0, min(1.0, self.brake))
                
                print("Received control: Throttle={:.2f}, Steering={:.2f}, Brake={:.2f}, Reverse={}".format(
                    self.throttle, self.steer, self.brake, self.reverse))
            else:
                print("Invalid command format: {}".format(command))
                
        except Exception as e:
            print("Error parsing control command '{}': {}".format(command, e))
            traceback.print_exc()

    def cleanup(self):
        print("\n=== Starting Cleanup ===")
        try:
            print("Setting running flag to False...")
            self.running = False  # Signal all threads to stop
            
            # Stop network control interface
            print("Stopping TCP control server...")
            self.control_server_running = False  # Signal server to stop
            
            # Close client connection if active
            if hasattr(self, 'control_client_socket') and self.control_client_socket:
                try:
                    self.control_client_socket.close()  # Close client socket
                except:
                    pass  # Ignore cleanup errors
                    
            # Close server socket if active
            if hasattr(self, 'control_server_socket') and self.control_server_socket:
                try:
                    self.control_server_socket.close()  # Close server socket
                except:
                    pass  # Ignore cleanup errors
            
            # Clean up sensor systems
            if hasattr(self, 'sensor_manager') and self.sensor_manager:
                print("Cleaning up sensor manager...")
                self.sensor_manager.cleanup()  # Remove all sensors
                
            # Clean up traffic systems
            if hasattr(self, 'traffic_manager') and self.traffic_manager:
                print("Cleaning up traffic vehicles...")
                self.traffic_manager.cleanup()  # Remove all AI vehicles
            
            # Remove player vehicle
            if hasattr(self, 'vehicle') and self.vehicle:
                print("Destroying vehicle...")
                self.vehicle.destroy()  # Remove from simulation
            
            # Reset simulation settings
            if hasattr(self, 'world'):
                print("Resetting world settings...")
                settings = self.world.get_settings()  # Get current settings
                settings.synchronous_mode = False  # Disable sync mode
                self.world.apply_settings(settings)  # Apply changes
            
            # Clean up display
            print("Quitting Pygame...")
            pygame.quit()  # Close display
            print("Cleanup complete")
            
        except Exception as e:
            print("ERROR during cleanup: {0}".format(str(e)))  # Log cleanup errors
            
    def setup_pygame(self):  # Initialize display and UI components
        """Initialize pygame and create the control panel window"""
        try:
            pygame.init()  # Initialize pygame
            pygame.font.init()  # Initialize font system
            
            # Configure display window
            self.display_width = 800  # Window width
            self.display_height = 600  # Window height
            self.display = pygame.display.set_mode((self.display_width, self.display_height))  # Create window
            pygame.display.set_caption("CARLA Control Panel")  # Set window title
            
            # Load text fonts
            self.font = pygame.font.SysFont('Arial', 20)  # Default text
            self.small_font = pygame.font.SysFont('Arial', 16)  # Small text
            self.large_font = pygame.font.SysFont('Arial', 24)  # Large text
            self.title_font = pygame.font.SysFont('Arial', 30, True)  # Section titles
            
            # Define UI colors
            self.BLACK = (0, 0, 0)  # Background
            self.WHITE = (255, 255, 255)  # Primary text
            self.GRAY = (100, 100, 100)  # Secondary elements
            self.LIGHT_GRAY = (200, 200, 200)  # Disabled elements
            self.RED = (255, 0, 0)  # Warnings/errors
            self.GREEN = (0, 255, 0)  # Success/active
            self.BLUE = (0, 0, 255)  # Highlights
            self.YELLOW = (255, 255, 0)  # Caution/status
            
            # UI state flags
            self.show_help = True  # Show help overlay
            
            print("Pygame setup complete")
            
        except Exception as e:
            print("ERROR in pygame setup: {0}".format(str(e)))  # Log setup errors
            raise
        
    def setup_fonts(self):  # Load text rendering fonts
        try:
            print("Loading fonts...")
            self.font = pygame.font.Font(None, 36)  # Default system font
            self.small_font = pygame.font.Font(None, 24)  # Smaller system font
            print("Fonts loaded successfully")
        except Exception as e:
            print("ERROR in font setup: {0}".format(str(e)))  # Log font errors
            raise
            
    def setup_carla_client(self):  # Connect to CARLA and initialize simulation
        try:
            print("Connecting to CARLA...")
            self.client = carla.Client('localhost', 2000)  # Connect to local server
            self.client.set_timeout(10.0)  # Set connection timeout
            
            # Get list of available maps
            available_maps = self.client.get_available_maps()  # Get all maps
            
            # Find requested map
            selected_map = None
            for map_path in available_maps:
                map_name = os.path.basename(map_path)  # Extract map name
                if TOWN_MAP in map_name:
                    selected_map = map_path  # Found requested map
                    break
            
            # Load appropriate map
            if selected_map:
                print("Loading selected map: {}...".format(TOWN_MAP))
                self.client.load_world(TOWN_MAP)  # Load requested map
            else:
                print("Warning: Map '{}' not found. Loading default map (Town01)...".format(TOWN_MAP))
                self.client.load_world('Town01')  # Load default map
            
            # Get world interface
            print("Getting CARLA world...")
            self.world = self.client.get_world()  # Get simulation world
            current_map = os.path.basename(self.world.get_map().name)  # Get active map name
            print("Connected to CARLA world ({})".format(current_map))
            
            # Configure environment
            self.apply_weather_preset(WEATHER_PRESET)  # Set weather conditions
            
            # Create player vehicle
            print("Spawning vehicle...")
            self.spawn_vehicle()  # Create and place vehicle
        except Exception as e:
            print("ERROR in CARLA client setup: {0}".format(str(e)))  # Log setup errors
            raise
        
    def spawn_vehicle(self):  # Create and place player vehicle
        try:
            # Remove existing vehicles
            print("Cleaning up existing vehicles...")
            vehicle_list = self.world.get_actors().filter('vehicle.*')  # Find all vehicles
            for vehicle in vehicle_list:
                vehicle.destroy()  # Remove each vehicle
                
            # Get valid spawn locations
            print("Getting spawn points...")
            spawn_points = self.world.get_map().get_spawn_points()  # Get all spawn points
            if not spawn_points:
                raise ValueError("No spawn points found!")  # Error if no spawns available
                
            # Select spawn location
            spawn_point = spawn_points[1]  # Use second spawn point
            spawn_point.location.z += 0.5  # Raise slightly for clearance
            
            # Get vehicle blueprints
            print("Setting up vehicle blueprint...")
            blueprint_library = self.world.get_blueprint_library()  # Get all blueprints
            
            # Define preferred vehicle models
            suv_models = [
                'vehicle.audi.etron',
                'vehicle.chevrolet.blazer',
                'vehicle.jeep.wrangler_rubicon',
                'vehicle.lincoln.mkz2017',
                'vehicle.mercedes.coupe',
                'vehicle.mercedes.sprinter',
                'vehicle.mini.cooper_s',
                'vehicle.nissan.patrol',
                'vehicle.tesla.cybertruck',
                'vehicle.toyota.prius',
                'vehicle.volkswagen.t2'
            ]
            
            # Try to find preferred vehicle
            vehicle_bp = None
            for model in suv_models:
                try:
                    bp = blueprint_library.find(model)  # Look for specific model
                    if bp:
                        vehicle_bp = bp  # Use first available model
                        print("Selected SUV model: {}".format(model))
                        break
                except:
                    continue  # Try next model
            
            # Fallback to generic SUV if preferred not found
            if not vehicle_bp:
                suv_blueprints = []
                for bp in blueprint_library.filter('vehicle'):
                    # Look for SUV-like vehicles
                    if any(tag in bp.id for tag in ['suv', 'offroad', 'truck']):
                        suv_blueprints.append(bp)
                
                if suv_blueprints:
                    vehicle_bp = random.choice(suv_blueprints)  # Pick random SUV
                    print("Selected generic SUV model: {}".format(vehicle_bp.id))
                else:
                    # Use default vehicle if no SUVs available
                    vehicle_bp = blueprint_library.find('vehicle.tesla.cybertruck')  # Use Cybertruck
                    print("No SUV models found, using fallback vehicle")
            
            # Create vehicle in simulation
            print("Spawning vehicle actor...")
            self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)  # Spawn vehicle
            if not self.vehicle:
                raise ValueError("Failed to spawn vehicle!")
                
            print("Vehicle spawned successfully")
            print("Initializing sensor manager...")
            self.sensor_manager = SensorManager(self.vehicle, self.world)
            
            if hasattr(self, 'sensor_manager'):
                print("Sensor manager created successfully")
                print("Sensor flags in manager - LIDAR: {}, RADAR: {}, IMU: {}, CAMERA: {}, WAYPOINT: {}, VELOCITY: {}".format(
                    self.sensor_manager.lidar_flag, self.sensor_manager.radar_flag, 
                    self.sensor_manager.imu_flag, self.sensor_manager.camera_flag,
                    self.sensor_manager.waypoint_flag, self.sensor_manager.velocity_flag))
            else:
                print("ERROR: Failed to create sensor manager!")
            
            # Set up traffic manager after main vehicle is spawned, but handle errors gracefully
            try:
                print("Setting up traffic manager...")
                self.traffic_manager = TrafficManager(self.world, self.vehicle, self.client)
                
                # Only try to spawn vehicles if the traffic manager was initialized
                if hasattr(self.traffic_manager, 'tm') and self.traffic_manager.tm is not None:
                    try:
                        self.traffic_manager.spawn_traffic_vehicles(3)  # Spawn 3 traffic vehicles
                        print("Successfully initialized traffic manager and spawned vehicles")
                    except Exception as e:
                        print("Error spawning initial traffic vehicles: {}".format(e))
                else:
                    print("Traffic manager initialized with limited functionality")
            except Exception as e:
                print("Error initializing traffic manager: {}".format(e))
                self.traffic_manager = None
            
            print("Vehicle setup complete")
            
        except Exception as e:
            print("ERROR in vehicle spawn: {}".format(str(e)))
            raise
            
    def apply_weather_preset(self, preset):  # Apply predefined weather conditions
        """Apply a predefined weather preset to the world"""
        try:
            # Dictionary of weather presets
            weather_presets = {  # Map preset names to CARLA weather parameters
                'ClearNoon': carla.WeatherParameters.ClearNoon,  # Clear sky with noon sun
                'CloudyNoon': carla.WeatherParameters.CloudyNoon,  # Cloudy sky with noon sun
                'WetNoon': carla.WeatherParameters.WetNoon,  # Wet roads with noon sun
                'WetCloudyNoon': carla.WeatherParameters.WetCloudyNoon,  # Wet roads with cloudy noon sky
                'MidRainyNoon': carla.WeatherParameters.MidRainyNoon,  # Medium rain with noon sun
                'HardRainNoon': carla.WeatherParameters.HardRainNoon,  # Heavy rain with noon sun
                'SoftRainNoon': carla.WeatherParameters.SoftRainNoon,  # Light rain with noon sun
                'ClearSunset': carla.WeatherParameters.ClearSunset,  # Clear sky with sunset sun
                'CloudySunset': carla.WeatherParameters.CloudySunset,  # Cloudy sky with sunset sun
                'WetSunset': carla.WeatherParameters.WetSunset,  # Wet roads with sunset sun
                'WetCloudySunset': carla.WeatherParameters.WetCloudySunset,  # Wet roads with cloudy sunset
                'MidRainSunset': carla.WeatherParameters.MidRainSunset,  # Medium rain with sunset sun
                'HardRainSunset': carla.WeatherParameters.HardRainSunset,  # Heavy rain with sunset sun
                'SoftRainSunset': carla.WeatherParameters.SoftRainSunset,  # Light rain with sunset sun
            }
            
            # Apply the selected weather preset or default to ClearNoon
            if preset in weather_presets:
                print("Applying weather preset: {}".format(preset))
                self.world.set_weather(weather_presets[preset])  # Set selected weather
            else:
                print("Warning: Weather preset '{}' not found. Using default (ClearNoon)...".format(preset))
                self.world.set_weather(carla.WeatherParameters.ClearNoon)  # Set default weather
                
        except Exception as e:
            print("Error applying weather preset: {}".format(e))
            print("Using default weather settings...")
            
    def process_input(self, event):  # Handle user input events
        """Process input events for vehicle control, sensors, and traffic management"""
        if event.type == pygame.QUIT:  # Handle window close button
            return True
        
        if event.type == pygame.KEYUP:  # Handle key release events
            if event.key == pygame.K_ESCAPE:  # ESC key exits program
                return True
            
            # Toggle sensors with number keys
            elif event.key == pygame.K_1:  # Toggle LiDAR
                if self.sensor_manager:
                    self.sensor_manager.toggle_lidar()
                    print("LIDAR toggled: {}".format(self.sensor_manager.lidar_flag))
            elif event.key == pygame.K_2:  # Toggle RADAR
                if self.sensor_manager:
                    self.sensor_manager.toggle_radar()
                    print("RADAR toggled: {}".format(self.sensor_manager.radar_flag))
            elif event.key == pygame.K_3:  # Toggle IMU
                if self.sensor_manager:
                    self.sensor_manager.toggle_imu()
                    print("IMU toggled: {}".format(self.sensor_manager.imu_flag))
            elif event.key == pygame.K_4:  # Toggle Camera
                if self.sensor_manager:
                    self.sensor_manager.toggle_camera()
                    print("Camera toggled: {}".format(self.sensor_manager.camera_flag))
            elif event.key == pygame.K_5:  # Toggle Waypoint
                if self.sensor_manager:
                    self.sensor_manager.toggle_waypoint()
                    print("Waypoint toggled: {}".format(self.sensor_manager.waypoint_flag))
            elif event.key == pygame.K_6:  # Toggle Velocity
                if self.sensor_manager:
                    self.sensor_manager.toggle_velocity()
                    print("Velocity toggled: {}".format(self.sensor_manager.velocity_flag))
            
            # Reset vehicle controls when key is released
            elif event.key == pygame.K_w or event.key == pygame.K_UP:  # Forward acceleration
                self.accelerating_forward = False  # Stop accelerating forward
                # Don't reset throttle immediately - it will gradually decrease
            elif event.key == pygame.K_s or event.key == pygame.K_DOWN:  # Reverse acceleration
                self.accelerating_reverse = False  # Stop accelerating reverse
                # Don't reset throttle immediately - it will gradually decrease
            elif event.key == pygame.K_a or event.key == pygame.K_LEFT or event.key == pygame.K_d or event.key == pygame.K_RIGHT:
                self.steer = 0.0  # Center steering
            
            # Toggle reverse with R key (kept for compatibility)
            elif event.key == pygame.K_r:  # Toggle reverse gear
                self.reverse = not self.reverse
                print("Reverse: {}".format(self.reverse))
            
            # Traffic management keys - handle gracefully if traffic_manager is None
            elif event.key == pygame.K_t:  # Add single traffic vehicle
                # Add a single traffic vehicle
                if self.traffic_manager:
                    try:
                        self.traffic_manager.spawn_traffic_vehicles(1)
                        print("Added 1 traffic vehicle")
                    except Exception as e:
                        print("Error adding traffic vehicle: {}".format(e))
                else:
                    print("Traffic manager not available")
            elif event.key == pygame.K_u:  # Add random traffic vehicles
                # Add random traffic vehicles
                if self.traffic_manager:
                    try:
                        self.traffic_manager.spawn_random_traffic_vehicles(3)
                        print("Added 3 random traffic vehicles")
                    except Exception as e:
                        print("Error adding random traffic vehicles: {}".format(e))
                else:
                    print("Traffic manager not available")
            elif event.key == pygame.K_y:  # Remove last traffic vehicle
                # Remove last traffic vehicle
                if self.traffic_manager:
                    try:
                        if self.traffic_manager.remove_last_vehicle():
                            print("Removed last traffic vehicle")
                        else:
                            print("No traffic vehicles to remove")
                    except Exception as e:
                        print("Error removing traffic vehicle: {}".format(e))
                else:
                    print("Traffic manager not available")
            elif event.key == pygame.K_m:  # Toggle roaming mode
                # Toggle roaming mode for traffic vehicles
                if self.traffic_manager:
                    try:
                        is_roaming = self.traffic_manager.set_vehicles_to_roam()
                        print("Traffic vehicles roaming mode: {}".format("ON" if is_roaming else "OFF"))
                    except Exception as e:
                        print("Error toggling vehicle roaming: {}".format(e))
                else:
                    print("Traffic manager not available")
            
            # Help toggle
            elif event.key == pygame.K_h:  # Toggle help display
                self.show_help = not self.show_help
                print("Help: {}".format(self.show_help))
        
        if event.type == pygame.KEYDOWN:  # Handle key press events
            # Vehicle control with WASD or arrow keys
            if event.key == pygame.K_w or event.key == pygame.K_UP:  # Forward
                self.accelerating_forward = True  # Start accelerating forward
                self.accelerating_reverse = False  # Stop accelerating reverse
                self.reverse = False  # Ensure reverse is off when accelerating forward
                self.brake = 0.0      # Ensure brake is off
            elif event.key == pygame.K_s or event.key == pygame.K_DOWN:  # Reverse
                self.accelerating_reverse = True  # Start accelerating reverse
                self.accelerating_forward = False  # Stop accelerating forward
                self.brake = 0.0      # Ensure brake is off
            elif event.key == pygame.K_a or event.key == pygame.K_LEFT:  # Steer left
                self.steer = max(-1.0, self.steer - 1.0)
            elif event.key == pygame.K_d or event.key == pygame.K_RIGHT:  # Steer right
                self.steer = min(1.0, self.steer + 1.0)
            # Add brake control with spacebar
            elif event.key == pygame.K_SPACE:  # Brake
                self.brake = min(1.0, self.brake + 1.0)
                self.throttle = 0.0   # Ensure throttle is off when braking
                self.accelerating_forward = False  # Stop accelerating
                self.accelerating_reverse = False  # Stop accelerating
        
        return False

    def update_throttle(self):
        """Update throttle based on acceleration flags"""
        # Handle transition between forward and reverse
        # If accelerating forward but currently in reverse (negative throttle)
        if self.accelerating_forward and self.throttle < 0:
            # First bring throttle to zero before accelerating forward
            self.throttle = min(0.0, self.throttle + self.throttle_decrement)
            # Only switch to forward acceleration once throttle reaches zero
            if self.throttle == 0:
                self.reverse = False
            return
        
        # If accelerating reverse but currently moving forward (positive throttle)
        if self.accelerating_reverse and self.throttle > 0:
            # First bring throttle to zero before accelerating in reverse
            self.throttle = max(0.0, self.throttle - self.throttle_decrement)
            return
        
        # Normal acceleration handling
        if self.accelerating_forward:
            # Gradually increase forward throttle
            self.throttle = min(1.0, self.throttle + self.throttle_increment)
            self.reverse = False
        elif self.accelerating_reverse:
            # Gradually increase reverse throttle (negative value)
            self.throttle = max(-1.0, self.throttle - self.throttle_increment)
            self.reverse = self.throttle < 0  # Set reverse based on throttle sign
        else:
            # Gradually decrease throttle when no acceleration key is pressed
            if self.throttle > 0:
                self.throttle = max(0.0, self.throttle - self.throttle_decrement)
            elif self.throttle < 0:
                self.throttle = min(0.0, self.throttle + self.throttle_decrement)
            
            # Update reverse flag based on throttle
            self.reverse = self.throttle < 0

    def update_spectator(self):
        """Update the spectator camera to follow the vehicle"""
        try:
            if not self.vehicle or not self.vehicle.is_alive:
                return
                
            # Get the vehicle's transform
            vehicle_transform = self.vehicle.get_transform()
            
            # Calculate a position behind and above the vehicle
            camera_offset = carla.Location(x=-5, z=3)  # 5 meters behind, 3 meters above
            camera_location = vehicle_transform.transform(camera_offset)
            
            # Create a transform for the spectator
            spectator_transform = carla.Transform(
                camera_location,
                carla.Rotation(pitch=-15, yaw=vehicle_transform.rotation.yaw)
            )
            
            # Apply the transform to the spectator
            spectator = self.world.get_spectator()
            spectator.set_transform(spectator_transform)
            
        except Exception as e:
            print("ERROR in update_spectator: {}".format(e))

    def draw_control_panel(self):
        """Draw the control panel with vehicle controls and sensor status"""
        try:
            # Clear screen with black background
            self.display.fill(self.BLACK)
            
            # Panel sections
            PANEL_WIDTH = self.display_width
            PANEL_HEIGHT = self.display_height
            
            # Draw title and border
            title = self.title_font.render("CARLA Sensor Control Panel", True, self.YELLOW)
            title_rect = title.get_rect(center=(PANEL_WIDTH // 2, 20))
            self.display.blit(title, title_rect)
            pygame.draw.line(self.display, self.YELLOW, (20, 40), (PANEL_WIDTH - 20, 40), 2)
            
            # Left panel - Sensor status
            left_panel_x = 30
            left_panel_y = 60
            
            # Draw sensor status section title
            sensor_title = self.large_font.render("Sensor Status", True, self.WHITE)
            self.display.blit(sensor_title, (left_panel_x, left_panel_y))
            left_panel_y += 40
            
            # Check if sensor manager exists
            if self.sensor_manager:
                # LIDAR status
                lidar_status = "ON" if self.sensor_manager.lidar_flag else "OFF"
                lidar_color = self.GREEN if self.sensor_manager.lidar_flag else self.RED
                lidar_text = self.font.render("LIDAR: {}".format(lidar_status), True, lidar_color)
                self.display.blit(lidar_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                # RADAR status
                radar_status = "ON" if self.sensor_manager.radar_flag else "OFF"
                radar_color = self.GREEN if self.sensor_manager.radar_flag else self.RED
                radar_text = self.font.render("RADAR: {}".format(radar_status), True, radar_color)
                self.display.blit(radar_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                # IMU status
                imu_status = "ON" if self.sensor_manager.imu_flag else "OFF"
                imu_color = self.GREEN if self.sensor_manager.imu_flag else self.RED
                imu_text = self.font.render("IMU: {}".format(imu_status), True, imu_color)
                self.display.blit(imu_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                # Camera status
                camera_status = "ON" if self.sensor_manager.camera_flag else "OFF"
                camera_color = self.GREEN if self.sensor_manager.camera_flag else self.RED
                camera_text = self.font.render("Camera: {}".format(camera_status), True, camera_color)
                self.display.blit(camera_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                # Waypoint status
                waypoint_status = "ON" if self.sensor_manager.waypoint_flag else "OFF"
                waypoint_color = self.GREEN if self.sensor_manager.waypoint_flag else self.RED
                waypoint_text = self.font.render("Waypoint: {}".format(waypoint_status), True, waypoint_color)
                self.display.blit(waypoint_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                # Velocity status
                velocity_status = "ON" if self.sensor_manager.velocity_flag else "OFF"
                velocity_color = self.GREEN if self.sensor_manager.velocity_flag else self.RED
                velocity_text = self.font.render("Velocity: {}".format(velocity_status), True, velocity_color)
                self.display.blit(velocity_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
            else:
                no_sensors_text = self.font.render("No sensors available", True, self.RED)
                self.display.blit(no_sensors_text, (left_panel_x, left_panel_y))
                left_panel_y += 50
            
            # Traffic vehicle information
            if hasattr(self, 'traffic_manager') and self.traffic_manager:
                traffic_title = self.large_font.render("Traffic Vehicles", True, self.WHITE)
                self.display.blit(traffic_title, (left_panel_x, left_panel_y))
                left_panel_y += 40
                
                vehicle_count = len(self.traffic_manager.traffic_vehicles) if hasattr(self.traffic_manager, 'traffic_vehicles') else 0
                traffic_text = self.font.render("Vehicles: {}".format(vehicle_count), True, self.WHITE)
                self.display.blit(traffic_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                roaming_status = "ON" if hasattr(self.traffic_manager, 'roaming_mode') and self.traffic_manager.roaming_mode else "OFF"
                roaming_color = self.GREEN if roaming_status == "ON" else self.GRAY
                roaming_text = self.font.render("Roaming Mode: {}".format(roaming_status), True, roaming_color)
                self.display.blit(roaming_text, (left_panel_x, left_panel_y))
            
            # Right panel - Vehicle controls
            right_panel_x = PANEL_WIDTH // 2 + 30
            right_panel_y = 60
            
            # Draw controls section title
            controls_title = self.large_font.render("Vehicle Controls", True, self.WHITE)
            self.display.blit(controls_title, (right_panel_x, right_panel_y))
            right_panel_y += 40
            
            # Vehicle speed display
            if self.vehicle and self.vehicle.is_alive:
                velocity = self.vehicle.get_velocity()
                speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # Convert to km/h
                speed_text = self.font.render("Speed: {:.1f} km/h".format(speed), True, self.YELLOW)
                self.display.blit(speed_text, (right_panel_x, right_panel_y))
                
                # Speed bar
                pygame.draw.rect(self.display, self.GRAY, (right_panel_x + 150, right_panel_y, 100, 20), 1)
                speed_percentage = min(speed / 120.0, 1.0)  # Assume 120 km/h is max for display
                speed_color = self.GREEN
                if speed > 60:
                    speed_color = self.YELLOW
                if speed > 100:
                    speed_color = self.RED
                pygame.draw.rect(self.display, speed_color, (right_panel_x + 150, right_panel_y, int(speed_percentage * 100), 20))
                
                # Remove velocity components display
                right_panel_y += 30
            
            # Throttle indicator
            throttle_text = self.font.render("Throttle: {:.2f}{}".format(
                abs(self.throttle), " (Reverse)" if self.reverse else ""), True, self.WHITE)
            self.display.blit(throttle_text, (right_panel_x, right_panel_y))
            pygame.draw.rect(self.display, self.GRAY, (right_panel_x + 150, right_panel_y, 100, 20), 1)
            pygame.draw.rect(self.display, self.GREEN if not self.reverse else self.BLUE, 
                            (right_panel_x + 150, right_panel_y, int(abs(self.throttle) * 100), 20))
            right_panel_y += 30
            
            # Brake indicator
            brake_text = self.font.render("Brake: {:.2f}".format(self.brake), True, self.WHITE)
            self.display.blit(brake_text, (right_panel_x, right_panel_y))
            pygame.draw.rect(self.display, self.GRAY, (right_panel_x + 150, right_panel_y, 100, 20), 1)
            pygame.draw.rect(self.display, self.RED, (right_panel_x + 150, right_panel_y, int(self.brake * 100), 20))
            right_panel_y += 30
            
            # Steering indicator
            steer_text = self.font.render("Steering: {:.2f}".format(self.steer), True, self.WHITE)
            self.display.blit(steer_text, (right_panel_x, right_panel_y))
            pygame.draw.rect(self.display, self.GRAY, (right_panel_x + 150, right_panel_y, 100, 20), 1)
            steer_center = right_panel_x + 150 + 50
            steer_pos = steer_center + int(self.steer * 50)
            pygame.draw.rect(self.display, self.BLUE, (steer_pos - 5, right_panel_y, 10, 20))
            pygame.draw.line(self.display, self.WHITE, (steer_center, right_panel_y), (steer_center, right_panel_y + 20), 1)
            right_panel_y += 30
            
            # Reverse indicator
            reverse_status = "ON" if self.reverse else "OFF"
            reverse_color = self.RED if self.reverse else self.GRAY
            reverse_text = self.font.render("Reverse: {}".format(reverse_status), True, reverse_color)
            self.display.blit(reverse_text, (right_panel_x, right_panel_y))
            right_panel_y += 30  # Reduced spacing to make room for lane type
            
            # Display current lane type above the help section
            if self.sensor_manager and self.vehicle and self.vehicle.is_alive:
                lane_type_int = self.sensor_manager.get_current_lane_type()
                lane_type_str = self.sensor_manager.get_lane_type_string(lane_type_int)
                lane_type_text = self.font.render("Lane Type: {} ({})".format(lane_type_int, lane_type_str), True, self.YELLOW)
                self.display.blit(lane_type_text, (right_panel_x, right_panel_y))
                right_panel_y += 30
                
                # Display whether there's a lane to the right
                has_right_lane = self.sensor_manager.has_right_lane()
                right_lane_status = "YES" if has_right_lane else "NO"
                right_lane_color = self.GREEN if has_right_lane else self.RED
                right_lane_text = self.font.render("Right Lane Available: {}".format(right_lane_status), True, right_lane_color)
                self.display.blit(right_lane_text, (right_panel_x, right_panel_y))
                right_panel_y += 30
            
            # Help section
            if self.show_help:
                help_y = PANEL_HEIGHT - 180
                help_title = self.large_font.render("Controls Help", True, self.WHITE)
                self.display.blit(help_title, (30, help_y))
                help_y += 30
                
                # Left column of help text
                help_texts_left = [
                    "W/Up: Accelerate",
                    "S/Down: Reverse",
                    "A/Left: Steer Left",
                    "D/Right: Steer Right",
                    "Space: Brake",
                    "ESC: Exit"
                ]
                
                # Right column of help text
                help_texts_right = [
                    "1-5: Toggle Sensors",
                    "6: Toggle Velocity",
                    "T: Add Traffic Vehicle",
                    "U: Add Random Traffic",
                    "Y: Remove Traffic Vehicle",
                    "M: Toggle Roaming Mode",
                    "H: Toggle Help"
                ]
                
                # Draw help text in two columns
                for i, text in enumerate(help_texts_left):
                    help_text = self.small_font.render(text, True, self.LIGHT_GRAY)
                    self.display.blit(help_text, (30, help_y + i * 20))
                    
                for i, text in enumerate(help_texts_right):
                    help_text = self.small_font.render(text, True, self.LIGHT_GRAY)
                    self.display.blit(help_text, (PANEL_WIDTH // 2 + 30, help_y + i * 20))
            
        except Exception as e:
            print("ERROR in draw_control_panel: {0}".format(str(e)))
            traceback.print_exc()

    def run(self):
        try:
            print("\n=== Starting Main Loop ===")
            print("Setting up synchronous mode...")
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05  # 20 FPS
            self.world.apply_settings(settings)
            
            print("Entering main loop...")
            while self.running:
                try:
                    # Process events
                    for event in pygame.event.get():
                        if self.process_input(event):  # Process keyboard/mouse input
                            self.running = False  # Exit if process_input returns True
                            break
                    
                    # Update throttle based on acceleration flags
                    self.update_throttle()
                    
                    # Tick the world
                    self.world.tick()  # Update CARLA world simulation
                    
                    # Update spectator camera
                    self.update_spectator()  # Update camera position
                    
                    # Apply control to vehicle
                    if self.vehicle and self.vehicle.is_alive:
                        # Use absolute value of throttle, reverse flag is already set based on throttle sign
                        control = carla.VehicleControl(
                            throttle=abs(self.throttle),  # Throttle control
                            steer=self.steer,  # Steering control
                            brake=self.brake,  # Brake control
                            hand_brake=False,  # No handbrake
                            reverse=self.reverse  # Reverse gear state
                        )
                        self.vehicle.apply_control(control)  # Apply control to vehicle
                    elif self.vehicle and not self.vehicle.is_alive:
                        print("Vehicle is not alive, respawning...")
                        self.spawn_vehicle()  # Respawn vehicle if destroyed
                    
                    # Update display
                    self.draw_control_panel()  # Draw GUI
                    pygame.display.flip()  # Update display
                    
                    # Cap the frame rate
                    pygame.time.Clock().tick(20)  # Limit to 20 FPS
                    
                except Exception as e:
                    print("ERROR in main loop iteration: {0}".format(str(e)))
                    traceback.print_exc()
            
            print("Main loop ended")
            
        except KeyboardInterrupt:
            print("KeyboardInterrupt received")
        except Exception as e:
            print("ERROR in run method: {0}".format(str(e)))
            traceback.print_exc()
        finally:
            print("Cleaning up...")
            self.cleanup()  # Clean up resources

def main():
    """
    Main entry point for the CARLA GUI Control application.
    Sets up the CARLA environment and starts the control interface.
    """
    try:
        carla_setup = CARLASetup()  # Initialize CARLA Python API
        control = CarlaControl()  # Create control interface
        control.run()  # Run main loop
    except Exception as e:
        print("Error:", str(e))
        sys.exit(1)  # Exit with error code

if __name__ == '__main__':
    main()  # Run main function when script is executed directly