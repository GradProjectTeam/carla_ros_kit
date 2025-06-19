#! /usr/bin/python3.7
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
import pygame
from pygame.locals import *
import threading
from queue import Queue
import traceback

# Version 6: Added traffic vehicles in front of the main vehicle
# The traffic vehicles have no sensors and use CARLA's autopilot system
# The main vehicle remains user-controlled with sensors

class CARLASetup:
    def __init__(self):
        print("Starting CARLA setup...")
        self.carla_path = '/home/shishtawy/Carla/CARLA_0.9.12/PythonAPI/carla/dist'
        self.setup_carla()
        
    def setup_carla(self):
        print("Looking for CARLA at:", self.carla_path)
        carla_eggs = glob.glob('{0}/carla-*{1}.{2}-{3}.egg'.format(
            self.carla_path,
            sys.version_info.major,
            sys.version_info.minor,
            "win-amd64" if os.name == "nt" else "linux-x86_64"
        ))
        sys.path.append(carla_eggs[0])
        print("Found CARLA egg:", carla_eggs[0])
        
        global carla
        import carla # type: ignore
        print("CARLA imported successfully")

class SensorManager:
    def __init__(self, vehicle, world):
        print("\n=== Initializing Sensor Manager ===")
        self.vehicle = vehicle
        self.world = world
        self.actor_list = []
        
        # Data queues with thread-safe implementation
        self.lidar_queue = Queue(maxsize=1)
        self.radar_queue = Queue(maxsize=1)
        self.imu_queue = Queue(maxsize=1)  # New IMU queue
        self.camera_queue = Queue(maxsize=1)  # New camera queue
        
        # TCP setup with different ports
        # self.host_ip = '192.168.1.2'
        self.host_ip = '127.0.0.1'
        self.lidar_port = 12349
        self.radar_port = 12347
        self.imu_port = 12341  # New IMU port
        self.camera_port = 12342  # New camera port

        # Sensor flags - set these to control which sensors are active
        self.lidar_flag = True
        self.radar_flag = True
        self.imu_flag = True
        self.camera_flag = False
        
        print(f"Initial sensor flags - LIDAR: {self.lidar_flag}, RADAR: {self.radar_flag}, IMU: {self.imu_flag}, CAMERA: {self.camera_flag}")
        
        # Thread control
        self.running = True
        self.lidar_thread = None
        self.radar_thread = None
        self.imu_thread = None  # New IMU thread
        self.camera_thread = None  # New camera thread
        
        # Setup separate sockets for each sensor
        self.setup_tcp_sockets()
        self.setup_sensors()
        
        # Start processing threads
        self.start_processing_threads()
        print("=== Sensor Manager Initialization Complete ===\n")
        
    def setup_tcp_sockets(self):
        # LiDAR socket
        if self.lidar_flag:
            try:
                self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.lidar_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print("LiDAR TCP configured for {0}:{1}".format(self.host_ip, self.lidar_port))
                
                # Try to connect but don't fail if connection fails
                try:
                    self.lidar_socket.connect((self.host_ip, self.lidar_port))
                    print("LiDAR TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print(f"LiDAR TCP connection failed: {e}. Will continue with local data only.")
                    # Don't disable the sensor just because connection failed
            except Exception as e:
                print(f"Error setting up LiDAR socket: {e}")
                # Keep the flag enabled - we'll just use local data
        
        # Radar socket
        if self.radar_flag:
            try:
                self.radar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.radar_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print("Radar TCP configured for {0}:{1}".format(self.host_ip, self.radar_port))
                
                # Try to connect but don't fail if connection fails
                try:
                    self.radar_socket.connect((self.host_ip, self.radar_port))
                    print("Radar TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print(f"Radar TCP connection failed: {e}. Will continue with local data only.")
                    # Don't disable the sensor just because connection failed
            except Exception as e:
                print(f"Error setting up Radar socket: {e}")
                # Keep the flag enabled - we'll just use local data
        
        # IMU socket
        if self.imu_flag:
            try:
                self.imu_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.imu_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print("IMU TCP configured for {0}:{1}".format(self.host_ip, self.imu_port))
                
                # Try to connect but don't fail if connection fails
                try:
                    self.imu_socket.connect((self.host_ip, self.imu_port))
                    print("IMU TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print(f"IMU TCP connection failed: {e}. Will continue with local data only.")
                    # Don't disable the sensor just because connection failed
            except Exception as e:
                print(f"Error setting up IMU socket: {e}")
                # Keep the flag enabled - we'll just use local data
        
        # Camera socket
        if self.camera_flag:
            try:
                self.camera_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.camera_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.camera_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print("Camera TCP configured for {0}:{1}".format(self.host_ip, self.camera_port))
                
                # Setup camera server to listen for connections
                try:
                    self.camera_socket.bind(('0.0.0.0', self.camera_port))
                    self.camera_socket.listen(1)
                    self.camera_socket.settimeout(0.5)  # Non-blocking accept
                    print("Camera TCP server listening on port {0}".format(self.camera_port))
                    self.camera_client = None
                except Exception as e:
                    print("Error setting up camera server: {0}".format(e))
                    # Don't disable the sensor just because server setup failed
            except Exception as e:
                print(f"Error setting up Camera socket: {e}")
                # Keep the flag enabled if possible
        
    def start_processing_threads(self):
        if self.lidar_flag:
            self.lidar_thread = threading.Thread(target=self.process_lidar_queue)
        if self.radar_flag:
            self.radar_thread = threading.Thread(target=self.process_radar_queue)
        if self.imu_flag:
            self.imu_thread = threading.Thread(target=self.process_imu_queue)
        if self.camera_flag:
            self.camera_thread = threading.Thread(target=self.process_camera_queue)
        
        if self.lidar_flag:
            self.lidar_thread.daemon = True
        if self.radar_flag:
            self.radar_thread.daemon = True
        if self.imu_flag:
            self.imu_thread.daemon = True
        if self.camera_flag:
            self.camera_thread.daemon = True
        
        if self.lidar_flag:
            self.lidar_thread.start()
        if self.radar_flag:
            self.radar_thread.start()
        if self.imu_flag:
            self.imu_thread.start()
        if self.camera_flag:
            self.camera_thread.start()
        
    def process_lidar_queue(self):
        point_counter = 0
        while self.running:
            try:
                if not self.lidar_queue.empty():
                    point_cloud = self.lidar_queue.get()
                    
                    # CARLA 0.9.12 LiDAR data structure:
                    # LiDAR data is now stored in raw_data as a numpy array
                    raw_data = np.frombuffer(point_cloud.raw_data, dtype=np.float32)
                    
                    # Reshape the array to get points (x, y, z, intensity)
                    # The data comes as [x, y, z, intensity, x, y, z, intensity, ...]
                    points = raw_data.reshape((-1, 4))
                    
                    for i in range(len(points)):
                        if not self.running:
                            break
                        
                        # Get x, y, z coordinates from the points array
                        x = points[i][0]
                        y = points[i][1]
                        z = points[i][2]
                        
                        # Print debug info every 1000 points
                        point_counter += 1
                        if point_counter % 1000 == 0:
                            print("Sending LIDAR point #{0}: ({1:.2f}, {2:.2f}, {3:.2f})".format(
                                point_counter, x, y, z))
                        
                        # Convert to network byte order (big-endian)
                        # Pack as float32 values in network byte order
                        point_data = struct.pack('!fff', x, y, z)
                        try:
                            if self.lidar_flag:
                                self.lidar_socket.send(point_data)
                        except socket.error as e:
                            print("LiDAR socket error: {0}".format(e))
                            break
                else:
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
            except Exception as e:
                print("Error in LiDAR processing thread: {0}".format(e))
                traceback.print_exc()
    
    def process_radar_queue(self):
        point_counter = 0  # Counter for radar points
        while self.running:
            try:
                if not self.radar_queue.empty():
                    radar_data = self.radar_queue.get()
                    points = np.array([[det.altitude, det.azimuth, det.depth, det.velocity] 
                                    for det in radar_data], dtype=np.float32)
                    
                    # Only log every 50 points batch
                    if point_counter % 50 == 0:
                        print("[RADAR DEBUG] Processing batch of {} radar points".format(len(points)))
                    
                    # Batch processing - send all points at once
                    if len(points) > 0 and self.radar_flag:
                        try:
                            # First send the number of points in the batch
                            num_points = struct.pack('!I', len(points))
                            self.radar_socket.sendall(num_points)
                            
                            # Then send all points data in one go
                            batch_data = bytearray()
                            for point in points:
                                # Pack each point
                                point_data = struct.pack('!ffff', point[0], point[1], point[2], point[3])
                                batch_data.extend(point_data)
                            
                            # Send the entire batch at once
                            self.radar_socket.sendall(batch_data)
                            point_counter += len(points)
                            
                            if point_counter % 50 == 0:
                                print("[RADAR DEBUG] Sent batch of {} points, total: {}".format(len(points), point_counter))
                        except socket.error as e:
                            print("[RADAR ERROR] Socket error in batch send: {}".format(e))
                else:
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
            except Exception as e:
                print("[RADAR ERROR] Processing error: {}".format(e))
                traceback.print_exc()
    
    def process_imu_queue(self):
        while self.running:
            try:
                if not self.imu_queue.empty():
                    imu_data = self.imu_queue.get()
                    # Pack IMU data: acceleration (3 floats) + gyroscope (3 floats) + compass (1 float)
                    data = struct.pack('fffffff', 
                                     imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z,
                                     imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z,
                                     imu_data.compass)
                    
                    try:
                        if self.imu_flag:
                            
                            self.imu_socket.sendall(data)
                    except socket.error as e:
                        print("IMU socket error: {0}".format(e))
                else:
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
            except Exception as e:
                print("Error in IMU processing thread: {0}".format(e))
    
    def process_camera_queue(self):
        frame_counter = 0
        while self.running:
            try:
                # Accept connections if no client is connected
                if not hasattr(self, 'camera_client') or self.camera_client is None:
                    try:
                        self.camera_client, addr = self.camera_socket.accept()
                        print("[CAMERA] Connected to client at {0}".format(addr))
                    except socket.timeout:
                        # No connection yet, that's fine
                        time.sleep(0.1)
                        continue
                    except Exception as e:
                        print("[CAMERA] Connection error: {0}".format(e))
                        time.sleep(0.5)
                        continue

                if not self.camera_queue.empty():
                    camera_data = self.camera_queue.get()
                    
                    # Get raw image data
                    raw_data = camera_data.raw_data
                    
                    # Log frame info periodically
                    frame_counter += 1
                    if frame_counter % 10 == 0:  # Log every 10 frames
                        print("[CAMERA DEBUG] Sending camera frame #{0}, size: {1} bytes".format(frame_counter, len(raw_data)))
                    
                    try:
                        if self.camera_flag and self.camera_client:
                            # Send image size first
                            size_header = struct.pack('!I', len(raw_data))
                            self.camera_client.sendall(size_header)
                            # Send image data
                            self.camera_client.sendall(raw_data)
                    except (BrokenPipeError, ConnectionResetError, socket.error) as e:
                        print("[CAMERA] Client disconnected: {0}".format(e))
                        self.camera_client = None
                else:
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
            except Exception as e:
                print("Error in Camera processing thread: {0}".format(e))
                traceback.print_exc()
                time.sleep(0.5)  # Sleep longer after an error
    
    def lidar_callback(self, point_cloud):
        try:
            if self.lidar_queue.full():
                try:
                    self.lidar_queue.get(block=False)  # Python 3.5 compatible
                except Queue.Empty:
                    pass
            self.lidar_queue.put(point_cloud, block=False)
        except Exception as e:
            print("Error in LiDAR callback: {0}".format(e))
        
    def radar_callback(self, radar_data):
        try:
            if self.radar_queue.full():
                try:
                    self.radar_queue.get(block=False)  # Python 3.5 compatible
                except Queue.Empty:
                    pass
            self.radar_queue.put(radar_data, block=False)
        except Exception as e:
            print("Error in Radar callback: {0}".format(e))
        
    def imu_callback(self, imu_data):
        try:
            if self.imu_queue.full():
                try:
                    self.imu_queue.get(block=False)  # Remove old data
                except Queue.Empty:
                    pass
            self.imu_queue.put(imu_data, block=False)
        except Exception as e:
            print("Error in IMU callback: {0}".format(e))
        
    def camera_callback(self, image):
        try:
            if self.camera_queue.full():
                try:
                    self.camera_queue.get(block=False)  # Remove old data
                except Queue.Empty:
                    pass
            self.camera_queue.put(image, block=False)
        except Exception as e:
            print("Error in Camera callback: {0}".format(e))
        
    def cleanup(self):
        print("Cleaning up sensors and sockets...")
        self.running = False  # Signal threads to stop
        
        # Wait for processing threads to finish
        if self.lidar_thread and self.lidar_thread.is_alive():
            self.lidar_thread.join(timeout=1.0)
        if self.radar_thread and self.radar_thread.is_alive():
            self.radar_thread.join(timeout=1.0)
        if self.imu_thread and self.imu_thread.is_alive():
            self.imu_thread.join(timeout=1.0)
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=1.0)
        
        # Clean up actors
        for actor in self.actor_list:
            if actor is not None and actor.is_alive:
                actor.destroy()
        
        # Close sockets
        if hasattr(self, 'lidar_socket'):
            try:
                if self.lidar_flag:
                    self.lidar_socket.shutdown(socket.SHUT_RDWR)
                    self.lidar_socket.close()
            except Exception as e:
                print("Error closing LiDAR socket: {0}".format(e))
            
        if hasattr(self, 'radar_socket'):
            try:
                if self.radar_flag:
                    self.radar_socket.shutdown(socket.SHUT_RDWR)
                    self.radar_socket.close()
            except Exception as e:
                print("Error closing Radar socket: {0}".format(e))
                
        if hasattr(self, 'imu_socket'):
            try:
                if self.imu_flag:
                    self.imu_socket.shutdown(socket.SHUT_RDWR)
                    self.imu_socket.close()
            except Exception as e:
                print("Error closing IMU socket: {0}".format(e))
        
        if hasattr(self, 'camera_socket'):
            try:
                if self.camera_flag:
                    if hasattr(self, 'camera_client') and self.camera_client:
                        self.camera_client.close()
                    self.camera_socket.shutdown(socket.SHUT_RDWR)
                    self.camera_socket.close()
            except Exception as e:
                print("Error closing Camera socket: {0}".format(e))
                
        print("Sensor cleanup complete")

    def setup_sensors(self):
        try:
            print("\n=== Starting Sensor Setup ===")
            if self.lidar_flag:
                print("Setting up LIDAR sensor...")
                self.setup_lidar()
            else:
                print("LIDAR sensor disabled, skipping setup")
                
            if self.radar_flag:
                print("Setting up RADAR sensor...")
                self.setup_radar()
            else:
                print("RADAR sensor disabled, skipping setup")
                
            if self.imu_flag:
                print("Setting up IMU sensor...")
                self.setup_imu()
            else:
                print("IMU sensor disabled, skipping setup")
                
            if self.camera_flag:
                print("Setting up camera sensor...")
                self.setup_camera()
            else:
                print("Camera sensor disabled, skipping setup")
                
            print("Sensors setup complete")
            print(f"Sensor flags after setup - LIDAR: {self.lidar_flag}, RADAR: {self.radar_flag}, IMU: {self.imu_flag}, CAMERA: {self.camera_flag}")
        except Exception as e:
            print("Error in setup_sensors: {0}".format(e))
            raise

    def setup_lidar(self):
        try:
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('channels', '32')
            lidar_bp.set_attribute('points_per_second', '100000')
            lidar_bp.set_attribute('rotation_frequency', '20')
            lidar_bp.set_attribute('range', '50.0')
            lidar_bp.set_attribute('upper_fov', '10.0')
            lidar_bp.set_attribute('lower_fov', '-5.0')    
            
            # Mount on top of the car, slightly forward
            lidar_transform = carla.Transform(
                carla.Location(x=1.5, z=2.0),  # x: forward, z: up (fixed height)
                carla.Rotation()  # Default rotation (0,0,0) will inherit car's rotation
            )
            
            self.lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
            self.actor_list.append(self.lidar)
            self.lidar.listen(self.lidar_callback)
            print("LiDAR sensor added")
            
            # Note: We don't try to connect here anymore, as it's handled in setup_tcp_sockets
            
        except Exception as e:
            print("Error in LiDAR setup: {0}".format(str(e)))
            # Don't raise the exception, just log it
            # We'll keep the sensor flag enabled
        
    def setup_radar(self):
        try:
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            radar_bp.set_attribute('horizontal_fov', '60.0')  # Increased from 30.0 for wider coverage
            radar_bp.set_attribute('vertical_fov', '-60.0')    # Increased from 10.0 for better height detection
            radar_bp.set_attribute('points_per_second', '2000')  # Increased from 1500 for better resolution
            radar_bp.set_attribute('range', '100.0')  # Increased from 50.0 for longer detection range
            
            # Mount next to the LiDAR with a slight horizontal offset
            radar_transform = carla.Transform(
                # Position radar at the same x (forward) position as LiDAR but offset to the right (y=0.5)
                carla.Location(x=1.5, y=0.5, z=2.0),  # Fixed height
                carla.Rotation()  # Default rotation (0,0,0) will inherit car's rotation
            )
            
            self.radar = self.world.spawn_actor(radar_bp, radar_transform, attach_to=self.vehicle)
            self.actor_list.append(self.radar)
            self.radar.listen(self.radar_callback)
            print("Radar sensor added")
            
            # Note: We don't try to connect here anymore, as it's handled in setup_tcp_sockets
            
        except Exception as e:
            print("Error in Radar setup: {0}".format(str(e)))
            # Don't raise the exception, just log it
            # We'll keep the sensor flag enabled

    def setup_imu(self):
        try:
            imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
            
            # Set IMU parameters
            imu_bp.set_attribute('sensor_tick', '0.05')  # 20Hz update rate
            
            # Mount at the center of the car
            imu_transform = carla.Transform(
                carla.Location(x=0.0, z=0.0),  # Center of the vehicle
                carla.Rotation()  # Default rotation
            )
            
            self.imu = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)
            self.actor_list.append(self.imu)
            self.imu.listen(self.imu_callback)
            print("IMU sensor added")
            
            # Note: We don't try to connect here anymore, as it's handled in setup_tcp_sockets
            
        except Exception as e:
            print("Error in IMU setup: {0}".format(str(e)))
            # Don't raise the exception, just log it
            # We'll keep the sensor flag enabled

    def setup_camera(self):
        try:
            # Create camera blueprint
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            
            # Set camera attributes for better image quality
            camera_bp.set_attribute('image_size_x', '640')
            camera_bp.set_attribute('image_size_y', '480')
            camera_bp.set_attribute('fov', '90')
            camera_bp.set_attribute('sensor_tick', '0.1')  # 10 FPS
            
            # Print camera configuration
            print("\nCamera Configuration:")
            print("- Resolution: {0}x{1}".format(
                camera_bp.get_attribute('image_size_x').as_int(),
                camera_bp.get_attribute('image_size_y').as_int()))
            print("- FOV: {0} degrees".format(camera_bp.get_attribute('fov').as_float()))
            print("- Update rate: {0} FPS".format(1.0/camera_bp.get_attribute('sensor_tick').as_float()))
            
            # Mount on front of the car, slightly elevated and tilted down for lane detection
            camera_transform = carla.Transform(
                carla.Location(x=2.0, z=1.5),  # Front of car, slightly elevated
                carla.Rotation(pitch=-15.0)    # Tilted down slightly
            )
            
            self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
            self.actor_list.append(self.camera)
            self.camera.listen(self.camera_callback)
            print("Camera sensor added at position: x=2.0m, z=1.5m, pitch=-15°")
            
        except Exception as e:
            print("Error in Camera setup: {0}".format(e))
            raise

class CarlaControl:
    def __init__(self):
        print("\n=== Starting CarlaControl Initialization ===")
        try:
            print("Setting up Pygame...")
            self.setup_pygame()
            
            print("Setting up CARLA client...")
            # Control variables
            self.throttle = 0.0
            self.brake = 0.0
            self.steer = 0.0
            self.reverse = False  # Add reverse state
            self.vehicle = None
            self.running = True
            self.traffic_cars = []  # List to store traffic vehicles
            
            # Initialize sensor manager and traffic manager references
            self.sensor_manager = None
            self.traffic_manager = None
            
            # Set up CARLA client and spawn vehicle
            self.setup_carla_client()
            
            print("CarlaControl initialization complete")
            
        except Exception as e:
            print("ERROR in CarlaControl initialization: {0}".format(str(e)))
            self.cleanup()
            raise
            
    def setup_pygame(self):
        """Initialize pygame and create the control panel window"""
        try:
            pygame.init()
            pygame.font.init()
            
            # Set up display
            self.display_width = 800
            self.display_height = 600
            self.display = pygame.display.set_mode((self.display_width, self.display_height))
            pygame.display.set_caption("CARLA Control Panel")
            
            # Set up fonts
            self.font = pygame.font.SysFont('Arial', 20)
            self.small_font = pygame.font.SysFont('Arial', 16)
            self.large_font = pygame.font.SysFont('Arial', 24)
            self.title_font = pygame.font.SysFont('Arial', 30, True)  # Bold font for titles
            
            # Define colors
            self.BLACK = (0, 0, 0)
            self.WHITE = (255, 255, 255)
            self.GRAY = (100, 100, 100)
            self.LIGHT_GRAY = (200, 200, 200)
            self.RED = (255, 0, 0)
            self.GREEN = (0, 255, 0)
            self.BLUE = (0, 0, 255)
            self.YELLOW = (255, 255, 0)
            
            # UI state
            self.show_help = True  # Show help text by default
            
            print("Pygame setup complete")
            
        except Exception as e:
            print("ERROR in pygame setup: {0}".format(str(e)))
            raise
        
    def setup_fonts(self):
        try:
            print("Loading fonts...")
            self.font = pygame.font.Font(None, 36)
            self.small_font = pygame.font.Font(None, 24)
            print("Fonts loaded successfully")
        except Exception as e:
            print("ERROR in font setup: {0}".format(str(e)))
            raise
            
    def setup_carla_client(self):
        try:
            print("Connecting to CARLA...")
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            print("Getting CARLA world...")
            self.world = self.client.get_world()
            print("Connected to CARLA world")
            print("Spawning vehicle...")
            self.spawn_vehicle()
        except Exception as e:
            print("ERROR in CARLA client setup: {0}".format(str(e)))
            raise
        
    def spawn_vehicle(self):
        try:
            print("Cleaning up existing vehicles...")
            vehicle_list = self.world.get_actors().filter('vehicle.*')
            for vehicle in vehicle_list:
                vehicle.destroy()
                
            print("Getting spawn points...")
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                raise ValueError("No spawn points found!")
                
            spawn_point = spawn_points[0]
            spawn_point.location.z += 0.5
            
            print("Setting up vehicle blueprint...")
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
            
            print("Spawning vehicle actor...")
            self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            if not self.vehicle:
                raise ValueError("Failed to spawn vehicle!")
                
            print("Vehicle spawned successfully")
            print("Initializing sensor manager...")
            self.sensor_manager = SensorManager(self.vehicle, self.world)
            
            if hasattr(self, 'sensor_manager'):
                print("Sensor manager created successfully")
                print(f"Sensor flags in manager - LIDAR: {self.sensor_manager.lidar_flag}, RADAR: {self.sensor_manager.radar_flag}, IMU: {self.sensor_manager.imu_flag}, CAMERA: {self.sensor_manager.camera_flag}")
            else:
                print("ERROR: Failed to create sensor manager!")
            
            # Set up traffic manager after main vehicle is spawned
            print("Setting up traffic manager...")
            self.traffic_manager = TrafficManager(self.world, self.vehicle, self.client)
            self.traffic_manager.spawn_traffic_vehicles(3)  # Spawn 3 traffic vehicles
            
            print("Vehicle setup complete")
            
        except Exception as e:
            print("ERROR in vehicle spawn: {0}".format(str(e)))
            raise
            
    def process_input(self, event):
        """Process input events"""
        if event.type == pygame.QUIT:
            return True
        
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
            
            # Toggle sensors with number keys
            elif event.key == pygame.K_1:
                if self.sensor_manager:
                    self.sensor_manager.toggle_lidar()
                    print(f"LIDAR toggled: {self.sensor_manager.lidar_flag}")
            elif event.key == pygame.K_2:
                if self.sensor_manager:
                    self.sensor_manager.toggle_radar()
                    print(f"RADAR toggled: {self.sensor_manager.radar_flag}")
            elif event.key == pygame.K_3:
                if self.sensor_manager:
                    self.sensor_manager.toggle_imu()
                    print(f"IMU toggled: {self.sensor_manager.imu_flag}")
            elif event.key == pygame.K_4:
                if self.sensor_manager:
                    self.sensor_manager.toggle_camera()
                    print(f"Camera toggled: {self.sensor_manager.camera_flag}")
            
            # Reset vehicle controls when key is released
            elif event.key == pygame.K_w or event.key == pygame.K_UP:
                self.throttle = 0.0
            elif event.key == pygame.K_s or event.key == pygame.K_DOWN:
                self.brake = 0.0
            elif event.key == pygame.K_a or event.key == pygame.K_LEFT or event.key == pygame.K_d or event.key == pygame.K_RIGHT:
                self.steer = 0.0
            
            # Toggle reverse with R key
            elif event.key == pygame.K_r:
                self.reverse = not self.reverse
                print("Reverse:", self.reverse)
            
            # Traffic management keys
            elif event.key == pygame.K_t:
                # Add a single traffic vehicle
                if self.traffic_manager:
                    self.traffic_manager.spawn_traffic_vehicles(1)
                    print("Added 1 traffic vehicle")
            elif event.key == pygame.K_u:
                # Add random traffic vehicles
                if self.traffic_manager:
                    self.traffic_manager.spawn_random_traffic_vehicles(3)
                    print("Added 3 random traffic vehicles")
            elif event.key == pygame.K_y:
                # Remove last traffic vehicle
                if self.traffic_manager and self.traffic_manager.remove_last_vehicle():
                    print("Removed last traffic vehicle")
            elif event.key == pygame.K_m:
                # Toggle roaming mode for traffic vehicles
                if self.traffic_manager:
                    is_roaming = self.traffic_manager.set_vehicles_to_roam()
                    print(f"Traffic vehicles roaming mode: {'ON' if is_roaming else 'OFF'}")
            
            # Help toggle
            elif event.key == pygame.K_h:
                self.show_help = not self.show_help
                print("Help:", self.show_help)
        
        if event.type == pygame.KEYDOWN:
            # Vehicle control with WASD or arrow keys
            if event.key == pygame.K_w or event.key == pygame.K_UP:
                self.throttle = min(1.0, self.throttle + 1.0)
            elif event.key == pygame.K_s or event.key == pygame.K_DOWN:
                self.brake = min(1.0, self.brake + 1.0)
            elif event.key == pygame.K_a or event.key == pygame.K_LEFT:
                self.steer = max(-1.0, self.steer - 1.0)
            elif event.key == pygame.K_d or event.key == pygame.K_RIGHT:
                self.steer = min(1.0, self.steer + 1.0)
        
        return False
            
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
            print(f"ERROR in update_spectator: {e}")

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
                lidar_text = self.font.render(f"LIDAR: {lidar_status}", True, lidar_color)
                self.display.blit(lidar_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                # RADAR status
                radar_status = "ON" if self.sensor_manager.radar_flag else "OFF"
                radar_color = self.GREEN if self.sensor_manager.radar_flag else self.RED
                radar_text = self.font.render(f"RADAR: {radar_status}", True, radar_color)
                self.display.blit(radar_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                # IMU status
                imu_status = "ON" if self.sensor_manager.imu_flag else "OFF"
                imu_color = self.GREEN if self.sensor_manager.imu_flag else self.RED
                imu_text = self.font.render(f"IMU: {imu_status}", True, imu_color)
                self.display.blit(imu_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                # Camera status
                camera_status = "ON" if self.sensor_manager.camera_flag else "OFF"
                camera_color = self.GREEN if self.sensor_manager.camera_flag else self.RED
                camera_text = self.font.render(f"Camera: {camera_status}", True, camera_color)
                self.display.blit(camera_text, (left_panel_x, left_panel_y))
                left_panel_y += 50
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
                traffic_text = self.font.render(f"Vehicles: {vehicle_count}", True, self.WHITE)
                self.display.blit(traffic_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                roaming_status = "ON" if hasattr(self.traffic_manager, 'roaming_mode') and self.traffic_manager.roaming_mode else "OFF"
                roaming_color = self.GREEN if roaming_status == "ON" else self.GRAY
                roaming_text = self.font.render(f"Roaming Mode: {roaming_status}", True, roaming_color)
                self.display.blit(roaming_text, (left_panel_x, left_panel_y))
            
            # Right panel - Vehicle controls
            right_panel_x = PANEL_WIDTH // 2 + 30
            right_panel_y = 60
            
            # Draw controls section title
            controls_title = self.large_font.render("Vehicle Controls", True, self.WHITE)
            self.display.blit(controls_title, (right_panel_x, right_panel_y))
            right_panel_y += 40
            
            # Throttle indicator
            throttle_text = self.font.render(f"Throttle: {self.throttle:.2f}", True, self.WHITE)
            self.display.blit(throttle_text, (right_panel_x, right_panel_y))
            pygame.draw.rect(self.display, self.GRAY, (right_panel_x + 150, right_panel_y, 100, 20), 1)
            pygame.draw.rect(self.display, self.GREEN, (right_panel_x + 150, right_panel_y, int(self.throttle * 100), 20))
            right_panel_y += 30
            
            # Brake indicator
            brake_text = self.font.render(f"Brake: {self.brake:.2f}", True, self.WHITE)
            self.display.blit(brake_text, (right_panel_x, right_panel_y))
            pygame.draw.rect(self.display, self.GRAY, (right_panel_x + 150, right_panel_y, 100, 20), 1)
            pygame.draw.rect(self.display, self.RED, (right_panel_x + 150, right_panel_y, int(self.brake * 100), 20))
            right_panel_y += 30
            
            # Steering indicator
            steer_text = self.font.render(f"Steering: {self.steer:.2f}", True, self.WHITE)
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
            reverse_text = self.font.render(f"Reverse: {reverse_status}", True, reverse_color)
            self.display.blit(reverse_text, (right_panel_x, right_panel_y))
            right_panel_y += 50
            
            # Help section
            if self.show_help:
                help_y = PANEL_HEIGHT - 180
                help_title = self.large_font.render("Controls Help", True, self.WHITE)
                self.display.blit(help_title, (30, help_y))
                help_y += 30
                
                # Left column of help text
                help_texts_left = [
                    "W/Up: Accelerate",
                    "S/Down: Brake",
                    "A/Left: Steer Left",
                    "D/Right: Steer Right",
                    "R: Toggle Reverse",
                    "ESC: Exit"
                ]
                
                # Right column of help text
                help_texts_right = [
                    "1-4: Toggle Sensors",
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
                        if self.process_input(event):
                            self.running = False
                            break
                    
                    # Tick the world
                    self.world.tick()
                    
                    # Update spectator camera
                    self.update_spectator()
                    
                    # Apply control to vehicle
                    if self.vehicle and self.vehicle.is_alive:
                        control = carla.VehicleControl(
                            throttle=self.throttle,
                            steer=self.steer,
                            brake=self.brake,
                            hand_brake=False,
                            reverse=self.reverse
                        )
                        self.vehicle.apply_control(control)
                    elif self.vehicle and not self.vehicle.is_alive:
                        print("Vehicle is not alive, respawning...")
                        self.spawn_vehicle()
                    
                    # Update display
                    self.draw_control_panel()
                    pygame.display.flip()
                    
                    # Cap the frame rate
                    pygame.time.Clock().tick(20)
                    
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
            self.cleanup()

    def cleanup(self):
        print("\n=== Starting Cleanup ===")
        try:
            print("Setting running flag to False...")
            self.running = False
            
            if hasattr(self, 'sensor_manager') and self.sensor_manager:
                print("Cleaning up sensor manager...")
                self.sensor_manager.cleanup()
                
            if hasattr(self, 'traffic_manager') and self.traffic_manager:
                print("Cleaning up traffic vehicles...")
                self.traffic_manager.cleanup()
            
            if hasattr(self, 'vehicle') and self.vehicle:
                print("Destroying vehicle...")
                self.vehicle.destroy()
            
            if hasattr(self, 'world'):
                print("Resetting world settings...")
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)
            
            print("Quitting Pygame...")
            pygame.quit()
            print("Cleanup complete")
            
        except Exception as e:
            print("ERROR during cleanup: {0}".format(str(e)))

class TrafficManager:
    def __init__(self, world, ego_vehicle, client):
        self.world = world
        self.client = client  # Get the client from the world
        self.ego_vehicle = ego_vehicle  # Main player vehicle
        self.traffic_vehicles = []  # List of spawned traffic vehicles
        self.roaming_mode = False  # Flag to track if vehicles are roaming
        
        # Get traffic manager from client
        try:
            print("Initializing CARLA traffic manager...")
            self.tm = self.client.get_trafficmanager(8000)
            self.tm.set_global_distance_to_leading_vehicle(2.5)
            self.tm.set_synchronous_mode(True)
            self.tm.global_percentage_speed_difference(30.0)
            print("Traffic manager initialized successfully")
        except Exception as e:
            print(f"Error initializing traffic manager: {e}")
            self.tm = None
            
    def spawn_traffic_vehicles(self, num_vehicles=3):
        """Spawn traffic vehicles in front of the ego vehicle"""
        if not self.tm:
            print("Traffic manager not available. Cannot spawn vehicles.")
            return
            
        try:
            print(f"Spawning {num_vehicles} traffic vehicles...")
            
            # Get spawn points
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                print("No spawn points available")
                return
                
            # Get ego vehicle transform
            ego_transform = self.ego_vehicle.get_transform()
            ego_location = ego_transform.location
            ego_forward_vector = ego_transform.get_forward_vector()
            
            # Get vehicle blueprints
            blueprint_library = self.world.get_blueprint_library()
            car_blueprints = [bp for bp in blueprint_library.filter('vehicle.*') 
                             if int(bp.get_attribute('number_of_wheels')) >= 4]  # Filter out bikes
            
            # Spawn vehicles in front of the ego vehicle
            for i in range(num_vehicles):
                # Calculate spawn distance (increasing for each vehicle)
                spawn_distance = 20 + (i * 15)  # 20m, 35m, 50m, etc.
                
                # Calculate spawn location in front of ego vehicle
                spawn_location = carla.Location(
                    x=ego_location.x + ego_forward_vector.x * spawn_distance,
                    y=ego_location.y + ego_forward_vector.y * spawn_distance,
                    z=ego_location.z + 0.5
                )
                
                # Find closest spawn point to desired location
                closest_spawn_point = None
                min_distance = float('inf')
                for spawn_point in spawn_points:
                    dist = spawn_location.distance(spawn_point.location)
                    if dist < min_distance:
                        min_distance = dist
                        closest_spawn_point = spawn_point
                
                if not closest_spawn_point:
                    print(f"Could not find a valid spawn point for vehicle {i+1}")
                    continue
                
                # Choose a random blueprint
                vehicle_bp = random.choice(car_blueprints)
                
                # Try to spawn the vehicle
                try:
                    vehicle = self.world.spawn_actor(vehicle_bp, closest_spawn_point)
                    if vehicle:
                        self.traffic_vehicles.append(vehicle)
                        print(f"Spawned {vehicle.type_id} at {closest_spawn_point.location}")
                        
                        # Set up autopilot with traffic manager
                        vehicle.set_autopilot(True, self.tm.get_port())
                        self.tm.auto_lane_change(vehicle, True)
                        self.tm.distance_to_leading_vehicle(vehicle, 5.0)
                        self.tm.vehicle_percentage_speed_difference(vehicle, random.uniform(-20, 10))
                        self.tm.ignore_vehicles_percentage(vehicle, 0)
                        self.tm.ignore_lights_percentage(vehicle, 0)
                        self.tm.ignore_signs_percentage(vehicle, 0)
                    else:
                        print(f"Failed to spawn vehicle {i+1}")
                except Exception as e:
                    print(f"Error spawning vehicle {i+1}: {e}")
            
            print(f"Successfully spawned {len(self.traffic_vehicles)} traffic vehicles")
            
        except Exception as e:
            print(f"Error in spawn_traffic_vehicles: {e}")
            
    def spawn_random_traffic_vehicles(self, num_vehicles=3):
        """Spawn traffic vehicles at random locations on the map"""
        if not self.tm:
            print("Traffic manager not available. Cannot spawn vehicles.")
            return
            
        try:
            print(f"Spawning {num_vehicles} random traffic vehicles...")
            
            # Get spawn points
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                print("No spawn points available")
                return
                
            # Get ego vehicle location
            ego_location = self.ego_vehicle.get_transform().location
            
            # Get vehicle blueprints (excluding bikes and motorcycles for stability)
            blueprint_library = self.world.get_blueprint_library()
            car_blueprints = [bp for bp in blueprint_library.filter('vehicle.*') 
                             if int(bp.get_attribute('number_of_wheels')) >= 4]
            
            # Filter spawn points that are at least 50m away from ego vehicle
            valid_spawn_points = []
            for sp in spawn_points:
                if sp.location.distance(ego_location) >= 50.0:
                    valid_spawn_points.append(sp)
            
            if not valid_spawn_points:
                print("No valid spawn points found (at least 50m from player)")
                return
                
            # Randomly select spawn points and spawn vehicles
            spawned_count = 0
            max_attempts = min(len(valid_spawn_points), num_vehicles * 3)  # Limit attempts
            
            for _ in range(max_attempts):
                if spawned_count >= num_vehicles:
                    break
                    
                # Choose a random spawn point and blueprint
                spawn_point = random.choice(valid_spawn_points)
                vehicle_bp = random.choice(car_blueprints)
                
                # Set autopilot attribute
                if vehicle_bp.has_attribute('role_name'):
                    vehicle_bp.set_attribute('role_name', 'autopilot')
                
                # Try to spawn the vehicle
                try:
                    vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
                    if vehicle:
                        self.traffic_vehicles.append(vehicle)
                        print(f"Spawned {vehicle.type_id} at {spawn_point.location}")
                        spawned_count += 1
                        
                        # Set up autopilot with traffic manager
                        if self.tm:
                            vehicle.set_autopilot(True, self.tm.get_port())
                            self.tm.auto_lane_change(vehicle, True)
                            self.tm.distance_to_leading_vehicle(vehicle, random.uniform(1.0, 3.0))
                            self.tm.vehicle_percentage_speed_difference(vehicle, random.uniform(-20, 10))
                            self.tm.ignore_vehicles_percentage(vehicle, 0)
                            self.tm.ignore_lights_percentage(vehicle, 0)
                            self.tm.ignore_signs_percentage(vehicle, 0)
                except Exception as e:
                    print(f"Failed to spawn vehicle: {e}")
            
            print(f"Successfully spawned {spawned_count} random traffic vehicles")
            
        except Exception as e:
            print(f"Error in spawn_random_traffic_vehicles: {e}")
    
    def set_vehicles_to_roam(self):
        """Toggle roaming mode for traffic vehicles"""
        if not self.tm:
            print("Traffic manager not available. Cannot set roaming mode.")
            return
            
        try:
            self.roaming_mode = not self.roaming_mode
            
            if self.roaming_mode:
                print("Setting traffic vehicles to roam freely")
                for vehicle in self.traffic_vehicles:
                    if vehicle.is_alive:
                        # Configure for free roaming
                        self.tm.auto_lane_change(vehicle, True)
                        self.tm.random_left_lanechange_percentage(vehicle, 10)
                        self.tm.random_right_lanechange_percentage(vehicle, 10)
                        self.tm.vehicle_percentage_speed_difference(vehicle, random.uniform(-30, 10))
                        self.tm.ignore_vehicles_percentage(vehicle, 0)
                        self.tm.ignore_lights_percentage(vehicle, 0)
                        self.tm.ignore_signs_percentage(vehicle, 0)
            else:
                print("Setting traffic vehicles to follow ego vehicle")
                for vehicle in self.traffic_vehicles:
                    if vehicle.is_alive:
                        # Configure for following behavior
                        self.tm.auto_lane_change(vehicle, False)
                        self.tm.vehicle_percentage_speed_difference(vehicle, random.uniform(-10, 5))
                        self.tm.ignore_vehicles_percentage(vehicle, 0)
                        self.tm.ignore_lights_percentage(vehicle, 0)
                        self.tm.ignore_signs_percentage(vehicle, 0)
                        
            return self.roaming_mode
            
        except Exception as e:
            print(f"Error setting vehicles to roam: {e}")
            return self.roaming_mode
            
    def remove_last_vehicle(self):
        """Remove the last spawned traffic vehicle"""
        if self.traffic_vehicles:
            vehicle = self.traffic_vehicles.pop()
            if vehicle.is_alive:
                vehicle.destroy()
                print(f"Removed {vehicle.type_id}")
                return True
        return False
        
    def cleanup(self):
        """Clean up all spawned traffic vehicles"""
        print("Cleaning up traffic vehicles...")
        for vehicle in self.traffic_vehicles:
            if vehicle and vehicle.is_alive:
                vehicle.destroy()
        self.traffic_vehicles = []

def main():
    try:
        carla_setup = CARLASetup()
        control = CarlaControl()
        control.run()
    except Exception as e:
        print("Error:", str(e))
        sys.exit(1)

if __name__ == '__main__':
    main()