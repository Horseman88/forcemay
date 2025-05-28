import time
from pymavlink import mavutil
import socket 
import json   
import numpy as np # For quaternion to euler math

class MAVLinkBridge:
    # __init__, _send_command_to_simulator, _setup_telemetry_listener, _receive_telemetry, start_mavlink_server
    # are assumed to be correctly implemented from previous steps. Minified for brevity.
    def __init__(self, port=14550, telemetry_port=14551, simulator_command_port=14552): # Minified
        self.mavlink_listen_port=port
        self.mavlink_connection_string=f"tcpin:0.0.0.0:{port}"
        self.master=None
        self.telemetry_listen_address=('0.0.0.0',telemetry_port)
        self.telemetry_socket=None
        self.latest_telemetry={}
        self.boot_time=time.time()
        self.ref_lat=47.
        self.ref_lon=-122.
        self.earth_radius_m=6371000
        # self.meters_per_degree_lat=self.earth_radius_m*(np.pi/180.) # Not directly used, calc in place
        self.simulator_command_address=('127.0.0.1',simulator_command_port)
        try:
            self.command_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            print(f"Command UDP socket created. Will send to Simulator at {self.simulator_command_address}")
        except socket.error as e:
            print(f"Error creating command UDP socket: {e}")
            self.command_socket=None
        except Exception as e:
            print(f"Unexpected error creating command UDP socket: {e}")
            self.command_socket = None
        self._setup_telemetry_listener()
        print(f"MAVLinkBridge initialized. GCS listens on {self.mavlink_connection_string}")
        print(f"Telemetry listener on UDP {self.telemetry_listen_address}")
        print(f"Sim Origin (0,0) maps to LAT: {self.ref_lat}, LON: {self.ref_lon}")

    def _setup_telemetry_listener(self): # Minified
        try:
            self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.telemetry_socket.setblocking(False)
            self.telemetry_socket.bind(self.telemetry_listen_address)
            # print(f"Telemetry UDP socket bound to {self.telemetry_listen_address}")
        except socket.error as e:
            print(f"Error creating or binding telemetry UDP socket: {e}")
            self.telemetry_socket = None
        except Exception as e: 
            print(f"Unexpected error setting up telemetry listener: {e}")
            self.telemetry_socket = None

    def _receive_telemetry(self): # Minified
        if not self.telemetry_socket: return
        try:
            while True:
                data, _ = self.telemetry_socket.recvfrom(1024)
                if not data: break
                decoded_data = data.decode('utf-8')
                telemetry_data = json.loads(decoded_data)
                self.latest_telemetry.update(telemetry_data)
        except socket.error as e:
            # In non-blocking mode, if no data, BlockingIOError (or EAGAIN/EWOULDBLOCK) is raised.
            # Check for Windows specific error code WSAEWOULDBLOCK as well.
            if e.errno == socket.errno.EAGAIN or \
               e.errno == socket.errno.EWOULDBLOCK or \
               (hasattr(socket.errno, 'WSAEWOULDBLOCK') and e.errno == socket.errno.WSAEWOULDBLOCK):
                pass # This is expected, no data available.
            else:
                print(f"Socket error receiving telemetry: {e}") # Log other socket errors
        except json.JSONDecodeError as e:
            print(f"Error decoding telemetry JSON: {e} - Data: '{decoded_data[:100]}...'")
        except Exception as e:
            print(f"Error processing received telemetry: {e}")

    def start_mavlink_server(self): # Minified
        try:
            self.master = mavutil.mavlink_connection(
                self.mavlink_connection_string,
                baud=115200,
                source_system=255,
                source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                autoreconnect=True)
            print(f"MAVLink server started, listening on {self.mavlink_connection_string}")
        except Exception as e:
            self.master = None
            print(f"Failed to start MAVLink server: {e}")

    def _send_command_to_simulator(self, command_dict: dict): # Minified
        if not self.command_socket:
            # print("Command socket not available.")
            return False
        try:
            self.command_socket.sendto(json.dumps(command_dict).encode('utf-8'), self.simulator_command_address)
            # print(f"Sent command to simulator: {json.dumps(command_dict)}") # Debug
            return True
        except socket.error as e:
            print(f"Socket error sending command to simulator: {e}")
        except Exception as e:
            print(f"Error sending command to simulator: {e}")
        return False

    def _quaternion_to_euler(self, w, x, y, z):
        """
        Преобразует кватернион в углы Эйлера (крен, тангаж, рыскание).
        Результат в радианах.
        Это стандартная реализация (например, из Wikipedia или libraries).
        """
        # Крен (roll, x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Тангаж (pitch, y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Рыскание (yaw, z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _handle_mavlink_message(self, msg):
        msg_type = msg.get_type()

        if msg_type == 'COMMAND_LONG':
            # print(f"Received COMMAND_LONG: Command ID {msg.command}") # For debugging
            if msg.target_system != 0 and msg.target_system != self.master.source_system:
                 # print(f"COMMAND_LONG not for us (target_system: {msg.target_system}, our_system: {self.master.source_system}). Ignoring.")
                 return

            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                action = int(msg.param1)
                action_str = "ARM" if action == 1 else "DISARM"
                print(f"{action_str} command received from GCS/DroneKit.")
                cmd_to_sim = {"type": "ARM_DISARM", "action": action}
                result = mavutil.mavlink.MAV_RESULT_FAILED # Default to failed
                if self._send_command_to_simulator(cmd_to_sim):
                    result = mavutil.mavlink.MAV_RESULT_ACCEPTED
                    print(f"Sent {cmd_to_sim['type']} (action: {cmd_to_sim['action']}) to simulator.")
                else:
                    print(f"Failed to send {cmd_to_sim['type']} to simulator.")
                
                self.master.mav.command_ack_send(
                    msg.command, result,
                    0, # progress
                    0, # result_param2
                    msg.target_system, msg.target_component)
                print(f"Sent COMMAND_ACK for ARM_DISARM with result: {result}")


        elif msg_type == 'SET_ATTITUDE_TARGET':
            # print(f"Received SET_ATTITUDE_TARGET: Thrust {msg.thrust:.2f}") # For debugging
            
            # MAVLink SET_ATTITUDE_TARGET fields:
            # time_boot_ms, target_system, target_component, type_mask, 
            # q (quaternion w,x,y,z), body_roll_rate, body_pitch_rate, body_yaw_rate, thrust
            
            # A more robust implementation would check type_mask.
            # type_mask bit meanings (1=ignore):
            # bit 0 (1): body roll rate
            # bit 1 (2): body pitch rate
            # bit 2 (4): body yaw rate
            # bit 3 (8): reserved
            # bit 4 (16): reserved
            # bit 5 (32): reserved
            # bit 6 (64): thrust
            # bit 7 (128): attitude (quaternion)
            # We will assume type_mask = 0 for now (all fields used) for simplicity
            # or relevant bits are not set for ignore.
            
            # Check if the message is for us (target_system=0 is broadcast)
            if msg.target_system != 0 and msg.target_system != self.master.source_system:
                # print(f"SET_ATTITUDE_TARGET not for us (target_system: {msg.target_system}). Ignoring.")
                return

            q_mav = msg.q # [w, x, y, z]
            roll, pitch, yaw = self._quaternion_to_euler(q_mav[0], q_mav[1], q_mav[2], q_mav[3])
            
            # Handle type_mask for ignoring fields
            # If bit 6 (thrust) is set in type_mask, thrust should be ignored (e.g., keep current or use 0.5 for hover)
            # For now, we use the provided thrust value directly.
            # A more complete system might use `None` to indicate "don't change" for the simulator.
            
            thrust_cmd = msg.thrust
            # if msg.type_mask & (1 << 6): # Check if thrust bit is set
            #    thrust_cmd = None # Or some default, indicating no change or hover for simulator

            cmd_to_sim = {
                "type": "SET_ATTITUDE_TARGET",
                "roll": roll,       # Target roll angle (rad)
                "pitch": pitch,     # Target pitch angle (rad)
                "yaw": yaw,         # Target yaw angle (rad) - from quaternion
                "roll_rate": msg.body_roll_rate,    # Target body roll rate (rad/s)
                "pitch_rate": msg.body_pitch_rate,  # Target body pitch rate (rad/s)
                "yaw_rate": msg.body_yaw_rate,      # Target body yaw rate (rad/s)
                "thrust": thrust_cmd  # Normalized thrust (0.0 to 1.0)
            }
            
            if self._send_command_to_simulator(cmd_to_sim):
                print(f"Sent SET_ATTITUDE_TARGET (R:{roll:.2f} P:{pitch:.2f} Y:{yaw:.2f} T:{thrust_cmd:.2f}) to simulator.")
            else:
                print(f"Failed to send SET_ATTITUDE_TARGET to simulator.")
            # SET_ATTITUDE_TARGET does not typically get an ACK.

    # run method remains the same as in the previous step, calling _handle_mavlink_message
    def run(self): # Minified for prompt
        self.start_mavlink_server()
        if not self.master: print("MAVLink GCS server not initialized.")
        if not self.telemetry_socket: print("Telemetry listener not initialized.")
        if not self.command_socket: print("Command UDP socket to simulator not initialized.")
        print("MAVLinkBridge run loop starting...")
        loop_counter = 0; attitude_freq_div = 2; global_pos_freq_div = 2
        try:
            while True:
                time_boot_ms = int((time.time() - self.boot_time) * 1000)
                self._receive_telemetry() 
                if self.master:
                    msg = self.master.recv_match(blocking=False)
                    if msg: self._handle_mavlink_message(msg)
                if self.master: # Telemetry sending
                    if loop_counter % 10 == 0: self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_GENERIC, mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED, 0, mavutil.mavlink.MAV_STATE_ACTIVE )
                    if loop_counter % attitude_freq_div == 0:
                        if all(k in self.latest_telemetry for k in ['roll','pitch','yaw','roll_rate','pitch_rate','yaw_rate']):
                            self.master.mav.attitude_send(time_boot_ms, self.latest_telemetry['roll'], self.latest_telemetry['pitch'], self.latest_telemetry['yaw'], self.latest_telemetry['roll_rate'], self.latest_telemetry['pitch_rate'], self.latest_telemetry['yaw_rate'] )
                    if loop_counter % global_pos_freq_div == 0:
                        req_keys = ['x','y_sim','z','vx','vy','vz','yaw']
                        if all(k in self.latest_telemetry for k in req_keys):
                            dlr=self.latest_telemetry['y_sim']/self.earth_radius_m; dlnr=self.latest_telemetry['x']/(self.earth_radius_m*np.cos(np.radians(self.ref_lat)))
                            l7e=int((self.ref_lat+np.degrees(dlr))*1e7);ln7e=int((self.ref_lon+np.degrees(dlnr))*1e7)
                            am=int(self.latest_telemetry['z']*1000);ram=am;vxc=int(self.latest_telemetry['vx']*100);vyc=int(self.latest_telemetry['vy']*100);vzc=int(self.latest_telemetry['vz']*100)
                            h_val = np.degrees(self.latest_telemetry['yaw']) % 360
                            h=int(h_val * 100); # h=h if h>=0 else h+36000 # Ensure positive not strictly needed with % 360
                            self.master.mav.global_position_int_send(time_boot_ms,l7e,ln7e,am,ram,vxc,vyc,vzc,h)
                loop_counter+=1; time.sleep(0.1)
        except KeyboardInterrupt: print("Bridge stopped.")
        except Exception as e: print(f"Err in bridge run: {e}")
        finally:
            if self.master: self.master.close()
            if self.telemetry_socket: self.telemetry_socket.close()
            if self.command_socket: self.command_socket.close()
            print("Bridge shut down.")

if __name__ == '__main__': # Minified
    bridge = MAVLinkBridge(); bridge.run()
```
