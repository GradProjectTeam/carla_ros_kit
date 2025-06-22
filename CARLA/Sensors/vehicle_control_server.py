#!/usr/bin/python3
import socket
import sys
import os
import glob
import threading
import time
import traceback

# Add CARLA Python API to path
carla_path = '/home/shishtawy/Carla/CARLA_0.9.12/PythonAPI/carla/dist'
sys.path.append(glob.glob(f'{carla_path}/carla-*{sys.version_info.major}.{sys.version_info.minor}-{sys.platform}.egg')[0])

import carla

class VehicleControlServer:
    def __init__(self, host='0.0.0.0', port=12345):
        """
        Initialize the vehicle control server
        
        Args:
            host (str): Host address to bind the server to
            port (int): Port to listen on
        """
        self.host = host
        self.port = port
        self.running = True
        self.vehicle = None
        self.client = None
        self.world = None
        self.server_socket = None
        self.client_socket = None
        
        # Control parameters
        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0
        self.reverse = False
        
        # Connect to CARLA
        self.setup_carla()
        
        # Start TCP server
        self.setup_server()
        
    def setup_carla(self):
        """Connect to CARLA and get the ego vehicle"""
        try:
            print("Connecting to CARLA...")
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            
            # Find the ego vehicle (assuming it's already spawned)
            vehicles = self.world.get_actors().filter('vehicle.*')
            if vehicles:
                # Get the first vehicle (you might want to use a specific filter)
                self.vehicle = vehicles[0]
                print(f"Connected to vehicle: {self.vehicle.type_id}")
            else:
                print("No vehicles found in the world. Please spawn a vehicle first.")
                self.spawn_vehicle()
                
        except Exception as e:
            print(f"Error connecting to CARLA: {e}")
            traceback.print_exc()
            sys.exit(1)
            
    def spawn_vehicle(self):
        """Spawn a vehicle if none exists"""
        try:
            print("Spawning a vehicle...")
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
            
            # Get a random spawn point
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                print("No spawn points available!")
                return
                
            spawn_point = spawn_points[0]  # Use the first spawn point
            
            # Spawn the vehicle
            self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            print(f"Vehicle spawned: {self.vehicle.type_id}")
            
            # Set spectator to view the vehicle
            spectator = self.world.get_spectator()
            transform = self.vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location + carla.Location(z=3, x=-5),
                carla.Rotation(pitch=-15, yaw=transform.rotation.yaw)
            ))
            
        except Exception as e:
            print(f"Error spawning vehicle: {e}")
            traceback.print_exc()
            
    def setup_server(self):
        """Set up the TCP server to receive control commands"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            print(f"Server listening on {self.host}:{self.port}")
            
            # Start a thread to accept connections
            threading.Thread(target=self.accept_connections, daemon=True).start()
            
            # Start a thread to apply vehicle controls
            threading.Thread(target=self.control_loop, daemon=True).start()
            
        except Exception as e:
            print(f"Error setting up server: {e}")
            traceback.print_exc()
            self.cleanup()
            sys.exit(1)
            
    def accept_connections(self):
        """Accept client connections"""
        while self.running:
            try:
                print("Waiting for client connection...")
                client_socket, client_address = self.server_socket.accept()
                print(f"Client connected from {client_address}")
                
                # Close previous client socket if exists
                if self.client_socket:
                    self.client_socket.close()
                    
                self.client_socket = client_socket
                
                # Start a thread to handle this client
                threading.Thread(target=self.handle_client, args=(client_socket,), daemon=True).start()
                
            except Exception as e:
                if self.running:  # Only print error if we're still supposed to be running
                    print(f"Error accepting connection: {e}")
                    traceback.print_exc()
                time.sleep(1)  # Avoid tight loop if there's an error
                
    def handle_client(self, client_socket):
        """Handle client connection and parse incoming control commands"""
        try:
            buffer = ""
            while self.running:
                data = client_socket.recv(1024)
                if not data:
                    print("Client disconnected")
                    break
                    
                # Decode and add to buffer
                buffer += data.decode('utf-8')
                
                # Process complete commands (might receive multiple or partial commands)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.parse_command(line.strip())
                    
        except Exception as e:
            print(f"Error handling client: {e}")
            traceback.print_exc()
        finally:
            client_socket.close()
            if self.client_socket == client_socket:
                self.client_socket = None
                
    def parse_command(self, command):
        """Parse the command string in format 'throttle,steering,brake,reverse'"""
        try:
            parts = command.split(',')
            if len(parts) >= 3:  # Accept both old and new format
                self.throttle = float(parts[0])
                self.steering = float(parts[1])
                self.brake = float(parts[2])
                
                # Check if reverse flag is included (new format)
                if len(parts) >= 4:
                    self.reverse = bool(int(parts[3]))
                
                # Clamp values to valid ranges
                self.throttle = max(0.0, min(1.0, self.throttle))
                self.steering = max(-1.0, min(1.0, self.steering))
                self.brake = max(0.0, min(1.0, self.brake))
                
                print(f"Received control: Throttle={self.throttle:.2f}, Steering={self.steering:.2f}, Brake={self.brake:.2f}, Reverse={self.reverse}")
            else:
                print(f"Invalid command format: {command}")
                
        except Exception as e:
            print(f"Error parsing command '{command}': {e}")
            
    def control_loop(self):
        """Apply control commands to the vehicle at a fixed rate"""
        while self.running:
            try:
                if self.vehicle and self.vehicle.is_alive:
                    control = carla.VehicleControl(
                        throttle=self.throttle,
                        steer=self.steering,
                        brake=self.brake,
                        hand_brake=False,
                        reverse=self.reverse
                    )
                    self.vehicle.apply_control(control)
                    
            except Exception as e:
                print(f"Error in control loop: {e}")
                traceback.print_exc()
                
            time.sleep(0.05)  # 20Hz control rate
            
    def run(self):
        """Run the server until interrupted"""
        try:
            print("Server running. Press Ctrl+C to stop.")
            while self.running:
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("Server stopping...")
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        
        if self.client_socket:
            self.client_socket.close()
            
        if self.server_socket:
            self.server_socket.close()
            
        if self.vehicle:
            self.vehicle.destroy()
            
        print("Cleanup complete")
        
def main():
    server = VehicleControlServer()
    server.run()
    
if __name__ == "__main__":
    main() 