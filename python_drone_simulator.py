import time
from pymavlink import mavutil, mavlink
import math # Added for math operations

class PythonDroneSimulator:
    def __init__(self, host='0.0.0.0', port=14550):
        # Drone state variables
        self.armed = False
        self.latitude = 47.397742  # Example initial latitude (Zurich)
        self.longitude = 8.545594  # Example initial longitude (Zurich)
        self.altitude = 0.0       # Relative altitude in meters
        self.roll = 0.0           # Radians
        self.pitch = 0.0          # Radians
        self.yaw = 0.0            # Radians (0 is North, positive East)
        self.vx = 0.0             # X velocity m/s (NED frame - North)
        self.vy = 0.0             # Y velocity m/s (NED frame - East)
        self.vz = 0.0             # Z velocity m/s (NED frame - Down)
        self.target_altitude = 0.0 # Target altitude for takeoff/waypoint
        self.is_landing = False    # Flag to indicate landing mode

        # Waypoint navigation attributes
        self.target_latitude = self.latitude
        self.target_longitude = self.longitude
        self.horizontal_speed = 2.0  # m/s
        self.waypoint_tolerance = 1.0 # meters, how close to get to a waypoint

        # Attributes for physics and telemetry
        self.last_update_time = time.time()
        self.update_rate = 10.0  # Hz, for physics and telemetry updates
        self.vertical_speed = 1.0  # m/s for takeoff/landing

        # MAVLink connection
        self.master = mavutil.mavlink_connection(f'udpin:{host}:{port}')
        print(f"MAVLink simulator listening on UDP port {port}")

        # System and component ID for the simulated drone
        self.system_id = 1
        self.component_id = mavlink.MAV_COMP_ID_AUTOPILOT1
        
        self.heartbeat_time = time.time()

    def send_heartbeat(self):
        mav_mode = mavlink.MAV_MODE_FLAG_GUIDED_INPUT | mavlink.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else mavlink.MAV_MODE_FLAG_GUIDED_INPUT
        mav_state = mavlink.MAV_STATE_ACTIVE if self.armed else mavlink.MAV_STATE_STANDBY
        
        self.master.mav.heartbeat_send(
            mavlink.MAV_TYPE_QUADROTOR,
            mavlink.MAV_AUTOPILOT_GENERIC,
            mav_mode,
            0,  # custom_mode
            mav_state
        )

    def update_physics_and_telemetry(self, dt):
        if not self.armed:
            if self.altitude > 0.01: 
                self.altitude -= self.vertical_speed * dt * 0.5 
                if self.altitude < 0: self.altitude = 0
            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0
            return

        # Vertical movement (takeoff/landing/waypoint altitude)
        if self.is_landing:
            self.altitude -= self.vertical_speed * dt
            self.vz = self.vertical_speed # Down is positive
            if self.altitude <= 0.05: 
                self.altitude = 0.0
                self.vz = 0.0
                self.armed = False 
                self.is_landing = False
                # self.target_altitude = 0.0 # Already set by LAND command handler
                print("Simulator: Landed and Disarmed.")
        elif self.altitude < self.target_altitude: # Ascending to target_altitude
            self.altitude += self.vertical_speed * dt
            self.vz = -self.vertical_speed 
            if self.altitude >= self.target_altitude:
                self.altitude = self.target_altitude
                self.vz = 0.0
                print(f"Simulator: Reached target altitude of {self.altitude:.1f}m")
        elif self.altitude > self.target_altitude: # Descending to target_altitude (e.g. waypoint alt change)
            self.altitude -= self.vertical_speed * dt
            self.vz = self.vertical_speed
            if self.altitude <= self.target_altitude:
                self.altitude = self.target_altitude
                self.vz = 0.0
        else: # Holding altitude
            self.vz = 0.0

        # Horizontal movement
        lat_diff_deg = self.target_latitude - self.latitude
        lon_diff_deg = self.target_longitude - self.longitude
        
        R_earth = 6371000 # Earth radius in meters
        
        # Convert lat/lon differences to radians for distance calculation
        dLat_rad = math.radians(lat_diff_deg)
        dLon_rad = math.radians(lon_diff_deg)
        lat1_rad = math.radians(self.latitude)
        
        # Simplified equirectangular approximation for distance
        x_dist_m = dLon_rad * math.cos(lat1_rad) * R_earth
        y_dist_m = dLat_rad * R_earth
        distance_to_target = math.sqrt(x_dist_m*x_dist_m + y_dist_m*y_dist_m)

        if distance_to_target > self.waypoint_tolerance:
            # Calculate bearing (angle from North, positive East)
            bearing_rad = math.atan2(lon_diff_deg, lat_diff_deg) 
            self.yaw = bearing_rad # Drone yaws towards waypoint

            self.vx = self.horizontal_speed * math.cos(bearing_rad) # North component
            self.vy = self.horizontal_speed * math.sin(bearing_rad) # East component

            # Update position based on velocity and dt
            meters_to_deg_lat = 1.0 / (R_earth * (math.pi/180.0)) # More precise
            meters_to_deg_lon = 1.0 / (R_earth * (math.pi/180.0) * math.cos(lat1_rad))
            
            self.latitude += (self.vx * dt) * meters_to_deg_lat
            self.longitude += (self.vy * dt) * meters_to_deg_lon
        else:
            self.vx = 0.0
            self.vy = 0.0
            # print(f"Simulator: Reached waypoint Lat: {self.target_latitude}, Lon: {self.target_longitude}")

        # Send GLOBAL_POSITION_INT
        self.master.mav.global_position_int_send(
            int(time.time() * 1e3), # time_boot_ms
            int(self.latitude * 1e7),  # lat (degE7)
            int(self.longitude * 1e7), # lon (degE7)
            int(self.altitude * 1000), # alt (mm AMSL - for sim, same as relative)
            int(self.altitude * 1000), # relative_alt (mm)
            int(self.vx * 100),       # vx (cm/s)
            int(self.vy * 100),       # vy (cm/s)
            int(self.vz * 100),       # vz (cm/s)
            int(math.degrees(self.yaw) * 100) % 36000 if -math.pi <= self.yaw <= math.pi else 0 # hdg (cdeg, 0..35999)
        )

        # Send ATTITUDE
        self.master.mav.attitude_send(
            int(time.time() * 1e3), # time_boot_ms
            self.roll,
            self.pitch,
            self.yaw, # yaw in radians
            0.0, # rollspeed
            0.0, # pitchspeed
            0.0  # yawspeed
        )

    def handle_mavlink_message(self, msg):
        msg_type = msg.get_type()
        if msg_type == "COMMAND_LONG":
            # print(f"Received COMMAND_LONG: command_id={msg.command}, params=({msg.param1}, {msg.param2}, ...)")
            if msg.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if msg.param1 == 1.0: # Arm
                    self.armed = True
                    print("Simulator: Armed")
                    ack_result = mavlink.MAV_RESULT_ACCEPTED
                elif msg.param1 == 0.0: # Disarm
                    self.armed = False
                    print("Simulator: Disarmed")
                    ack_result = mavlink.MAV_RESULT_ACCEPTED
                else:
                    ack_result = mavlink.MAV_RESULT_UNSUPPORTED
                self.master.mav.command_ack_send(msg.command, ack_result)

            elif msg.command == mavlink.MAV_CMD_NAV_TAKEOFF:
                if self.armed:
                    takeoff_alt = msg.param7 if msg.param7 > 0 else 10.0 
                    self.target_altitude = takeoff_alt
                    self.is_landing = False 
                    print(f"Simulator: Takeoff initiated to {self.target_altitude}m")
                    ack_result = mavlink.MAV_RESULT_ACCEPTED
                else:
                    print("Simulator: Takeoff rejected, not armed")
                    ack_result = mavlink.MAV_RESULT_TEMPORARILY_REJECTED 
                self.master.mav.command_ack_send(msg.command, ack_result)

            elif msg.command == mavlink.MAV_CMD_NAV_LAND:
                if self.armed:
                    print("Simulator: Land initiated")
                    self.is_landing = True
                    self.target_altitude = 0.0 
                    ack_result = mavlink.MAV_RESULT_ACCEPTED
                else:
                    print("Simulator: Land rejected, not armed")
                    ack_result = mavlink.MAV_RESULT_FAILED 
                self.master.mav.command_ack_send(msg.command, ack_result)
            
            elif msg.command == mavlink.MAV_CMD_NAV_WAYPOINT:
                if self.armed:
                    self.target_latitude = msg.param5 # x field in MISSION_ITEM, lat for COMMAND_LONG
                    self.target_longitude = msg.param6 # y field in MISSION_ITEM, lon for COMMAND_LONG
                    # self.target_altitude = msg.param7 # z field in MISSION_ITEM, alt for COMMAND_LONG
                    # For now, assume waypoint altitude is maintained or set by a separate command if needed.
                    # If param7 is used, ensure it's handled in update_physics_and_telemetry for vertical movement.
                    
                    print(f"Simulator: Received MAV_CMD_NAV_WAYPOINT to Lat: {self.target_latitude:.6f}, Lon: {self.target_longitude:.6f}, Alt: {msg.param7:.1f}m")
                    ack_result = mavlink.MAV_RESULT_ACCEPTED
                else:
                    print("Simulator: Waypoint command rejected, not armed.")
                    ack_result = mavlink.MAV_RESULT_TEMPORARILY_REJECTED
                self.master.mav.command_ack_send(msg.command, ack_result)

        elif msg_type == "HEARTBEAT":
            pass

    def run(self):
        print("Simulator running...")
        while True:
            current_time = time.time()

            if current_time - self.heartbeat_time > 1.0:
                self.send_heartbeat()
                self.heartbeat_time = current_time
            
            msg = self.master.recv_match(blocking=False)
            if msg:
                self.handle_mavlink_message(msg)

            if current_time - self.last_update_time > (1.0 / self.update_rate):
                dt = current_time - self.last_update_time
                if dt < 0: dt = 0 # Ensure dt is not negative if system time changes
                self.update_physics_and_telemetry(dt)
                self.last_update_time = current_time
            
            # Sleep to maintain approx update_rate, but ensure it's not too small
            sleep_duration = (1.0 / self.update_rate) - (time.time() - current_time)
            time.sleep(max(0.001, sleep_duration)) # Min sleep 1ms, or calculated duration

if __name__ == '__main__':
    simulator = PythonDroneSimulator()
    simulator.run()
