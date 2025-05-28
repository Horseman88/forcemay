import time
from dronekit import connect, VehicleMode, APIException, DroneKitConnectionError, DroneKitTimeoutError
from pymavlink import mavutil # Needed for ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE

class DroneController:
    def __init__(self, connection_string="tcp:127.0.0.1:14550"):
        """
        Инициализация контроллера дрона.
        :param connection_string: Строка подключения к MAVLink (например, к MAVLink мосту).
        """
        self.connection_string = connection_string
        self.vehicle = None
        # print(f"DroneController initialized. Will attempt to connect to: {self.connection_string}") # Can be verbose

    def connect(self):
        """
        Устанавливает соединение с дроном (MAVLink мостом).
        """
        print(f"Connecting to vehicle on: {self.connection_string}...")
        try:
            # source_system=2 to identify as a GCS/companion computer
            self.vehicle = connect(self.connection_string, wait_ready=True, timeout=60, source_system=2)
            
            print("Successfully connected to vehicle!")
            print("Vehicle attributes:")
            print(f"  APM version: {self.vehicle.version}")
            print(f"  Mode: {self.vehicle.mode.name}")
            print(f"  Armed: {self.vehicle.armed}")
            print(f"  Global Location: {self.vehicle.location.global_frame}")
            print(f"  Attitude: {self.vehicle.attitude}") 
            print(f"  GPS Info: {self.vehicle.gps_0}") 
            print(f"  Battery: {self.vehicle.battery}")
            print(f"  Heartbeat last heard: {self.vehicle.last_heartbeat:.2f}s ago")
            
            return True

        except DroneKitConnectionError as e:
            print(f"Connection Error: {e}")
        except DroneKitTimeoutError:
            print("Connection timed out. Check if MAVLink bridge is running and reachable.")
        except APIException as e:
            print(f"DroneKit API Exception: {e}")
        except Exception as e:
            print(f"An unexpected error occurred during connection: {e}")
        
        self.vehicle = None 
        return False

    def display_telemetry(self, duration_sec=3): # Shortened default for testing
        """
        Отображает основную телеметрию с дрона в течение заданного времени.
        :param duration_sec: Продолжительность отображения телеметрии в секундах.
        """
        if not self.vehicle:
            print("Vehicle not connected. Cannot display telemetry.")
            return

        print("\n--- Displaying Telemetry ---")
        print(f"Will display for {duration_sec} seconds. Press Ctrl+C to stop earlier.")
        
        start_time = time.time()
        try:
            while (time.time() - start_time) < duration_sec:
                print("\nVehicle Telemetry Update:")
                print(f"  Mode: {self.vehicle.mode.name}")
                print(f"  Armed: {self.vehicle.armed}")
                
                if self.vehicle.attitude:
                    print(f"  Attitude: Roll={self.vehicle.attitude.roll:.2f}, Pitch={self.vehicle.attitude.pitch:.2f}, Yaw={self.vehicle.attitude.yaw:.2f}")
                else:
                    print("  Attitude: N/A")

                if self.vehicle.location.global_frame and self.vehicle.location.global_frame.lat is not None:
                    loc_glob = self.vehicle.location.global_frame
                    print(f"  Global Loc: Lat={loc_glob.lat:.7f}, Lon={loc_glob.lon:.7f}, Alt={loc_glob.alt:.2f}m")
                else:
                    print("  Global Location: N/A")
                
                if self.vehicle.location.global_relative_frame and self.vehicle.location.global_relative_frame.alt is not None:
                    loc_rel = self.vehicle.location.global_relative_frame
                    print(f"  Rel Alt: {loc_rel.alt:.2f}m")
                else:
                    print("  Relative Altitude: N/A")

                if self.vehicle.velocity: 
                    print(f"  Velocity (m/s): vx={self.vehicle.velocity[0]:.2f}, vy={self.vehicle.velocity[1]:.2f}, vz={self.vehicle.velocity[2]:.2f}")
                else:
                    print("  Velocity: N/A")
                
                if self.vehicle.gps_0:
                    print(f"  GPS Fix: {self.vehicle.gps_0.fix_type}, Satellites: {self.vehicle.gps_0.satellites_visible}")
                else:
                    print("  GPS Info: N/A")

                if self.vehicle.battery:
                    print(f"  Battery: V={self.vehicle.battery.voltage:.2f}V, Lvl={self.vehicle.battery.level}%") 
                else:
                    print("  Battery: N/A")
                
                print(f"  Last Heartbeat: {self.vehicle.last_heartbeat:.2f}s ago")
                
                time.sleep(1) 
        except KeyboardInterrupt:
            print("\nTelemetry display stopped by user.")
        except Exception as e:
            print(f"An error occurred during telemetry display: {e}")
        
        print("--- End of Telemetry Display ---")

    def arm(self, timeout_sec=10):
        if not self.vehicle:
            print("Vehicle not connected. Cannot arm.")
            return False
        if self.vehicle.armed:
            print("Vehicle is already armed.")
            return True
        print("Attempting to arm vehicle...")
        # Optional: Set mode to GUIDED if not already
        # if self.vehicle.mode.name != "GUIDED":
        #     print("Setting mode to GUIDED...")
        #     self.vehicle.mode = VehicleMode("GUIDED")
        #     # Wait for mode change confirmation if necessary, depends on DroneKit version and AP behavior
        #     time.sleep(1) # Simple wait
        try:
            self.vehicle.armed = True
            start_time = time.time()
            while not self.vehicle.armed:
                if time.time() - start_time > timeout_sec:
                    print("Timeout waiting for vehicle to arm.")
                    return False
                print(f"Waiting for arming... Current mode: {self.vehicle.mode.name}, Armed: {self.vehicle.armed}")
                time.sleep(0.5)
            print("Vehicle ARMED successfully!")
            return True
        except Exception as e:
            print(f"An error occurred while trying to arm: {e}")
            return False

    def disarm(self, timeout_sec=10):
        if not self.vehicle:
            print("Vehicle not connected. Cannot disarm.")
            return False
        if not self.vehicle.armed:
            print("Vehicle is already disarmed.")
            return True
        print("Attempting to disarm vehicle...")
        try:
            self.vehicle.armed = False
            start_time = time.time()
            while self.vehicle.armed:
                if time.time() - start_time > timeout_sec:
                    print("Timeout waiting for vehicle to disarm.")
                    return False
                print("Waiting for disarming...")
                time.sleep(0.5)
            print("Vehicle DISARMED successfully!")
            return True
        except Exception as e:
            print(f"An error occurred while trying to disarm: {e}")
            return False

    def set_attitude_target(self, roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0, 
                            thrust=0.5, duration_sec=0, target_attitude_q=None):
        """
        Отправляет команду SET_ATTITUDE_TARGET для управления ориентацией и тягой.
        :param roll_rate: Угловая скорость крена (rad/s). Положительное значение - крен вправо.
        :param pitch_rate: Угловая скорость тангажа (rad/s). Положительное значение - нос вверх.
        :param yaw_rate: Угловая скорость рыскания (rad/s). Положительное значение - по часовой стрелке (вправо).
        :param thrust: Нормализованная тяга (0.0 до 1.0). 0.5 - обычно для удержания высоты.
        :param duration_sec: Продолжительность отправки команды (сек). Если 0, отправляет один раз.
        :param target_attitude_q: Кватернион [w,x,y,z] для целевой ориентации (опционально).
        """
        if not self.vehicle:
            print("Vehicle not connected. Cannot set attitude target.")
            return

        if not self.vehicle.armed:
            print("Vehicle not armed. Arm vehicle before setting attitude target.")
            return

        # print(f"Setting attitude target: RollRate={roll_rate:.2f}, PitchRate={pitch_rate:.2f}, YawRate={yaw_rate:.2f}, Thrust={thrust:.2f} for {duration_sec}s")
        
        current_type_mask = 0 
        
        if target_attitude_q is None:
            # Ignore attitude if quaternion is not provided; control rates and thrust.
            current_type_mask = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE # Value is 128
            q_to_send = [1, 0, 0, 0] # Neutral quaternion when attitude is ignored
        else:
            q_to_send = target_attitude_q
        
        clamped_thrust = max(0.0, min(1.0, thrust))

        start_time_loop = time.time()
        loop_count = 0
        try:
            while True:
                msg = self.vehicle.message_factory.set_attitude_target_encode(
                    0,  # time_boot_ms (0 if not used)
                    self.vehicle.target_system,  # target_system
                    self.vehicle.target_component, # target_component
                    current_type_mask,  # type_mask
                    q_to_send,  # q: quaternion [w,x,y,z]
                    roll_rate,  # body_roll_rate (rad/s)
                    pitch_rate,  # body_pitch_rate (rad/s)
                    yaw_rate,  # body_yaw_rate (rad/s)
                    clamped_thrust  # thrust (0 to 1)
                )
                self.vehicle.send_mavlink(msg)
                # print(f"Sent SET_ATTITUDE_TARGET: Mask={current_type_mask}, Loop={loop_count+1}") # Debug

                loop_count += 1
                if duration_sec == 0: # Send only once if duration is 0
                    break 
                
                if time.time() - start_time_loop >= duration_sec and loop_count > 0 : # Ensure at least one send if duration > 0
                    break
                
                time.sleep(0.1) # Send at approximately 10Hz
            
            if duration_sec > 0:
                print(f"Finished sending attitude targets for {duration_sec}s.")

        except Exception as e:
            print(f"Error sending SET_ATTITUDE_TARGET: {e}")

    # Placeholder for future methods
    def takeoff(self, altitude): print(f"Placeholder: Takeoff to {altitude}m") 
    def goto(self, lat, lon, alt): print(f"Placeholder: Goto {lat},{lon} at {alt}m")
    def land(self): print("Placeholder: Land")


# ==================================================
# Основной блок для запуска контроллера напрямую
# ==================================================
if __name__ == '__main__':
    connection_string_default = "tcp:127.0.0.1:14550"
    print(f"--- DroneKit Controller Standalone Test ---")
    print(f"Attempting to connect to MAVLink Bridge at: {connection_string_default}")
    
    controller = DroneController(connection_string=connection_string_default)
    
    try:
        if controller.connect():
            print("Connection successful.")
            # controller.display_telemetry(duration_sec=2) # Optional: initial telemetry check

            print("\n--- Attempting to Arm ---")
            if controller.arm():
                print("Vehicle is ARMED. Current mode: %s" % controller.vehicle.mode.name)
                # controller.display_telemetry(duration_sec=2) # Check armed telemetry

                # Test Case 1: Gentle Yaw
                hover_thrust_test = 0.74 
                print(f"\n--- Test Case 1: Commanding Yaw Rate (0.3 rad/s) with thrust ({hover_thrust_test}) for 3s ---")
                controller.set_attitude_target(yaw_rate=0.3, thrust=hover_thrust_test, duration_sec=3)
                print("Waiting for yaw command to complete (4s)...")
                time.sleep(4) 
                controller.display_telemetry(duration_sec=1)


                # Test Case 2: Gentle Roll
                print(f"\n--- Test Case 2: Commanding Roll Rate (0.2 rad/s) with thrust ({hover_thrust_test}) for 3s ---")
                controller.set_attitude_target(roll_rate=0.2, thrust=hover_thrust_test, duration_sec=3)
                print("Waiting for roll command to complete (4s)...")
                time.sleep(4)
                controller.display_telemetry(duration_sec=1)

                # Test Case 3: Stabilize (zero rates, then lower thrust to descend/land)
                print(f"\n--- Test Case 3: Commanding Zero Rates with thrust ({hover_thrust_test}) for 2s (stabilize) ---")
                controller.set_attitude_target(roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0, thrust=hover_thrust_test, duration_sec=2)
                print("Waiting for stabilization command to complete (3s)...")
                time.sleep(3)
                controller.display_telemetry(duration_sec=1)

                print(f"\n--- Test Case 4: Lowering thrust to 0.2 for 3s (descend) ---")
                controller.set_attitude_target(roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0, thrust=0.2, duration_sec=3)
                print("Waiting for descent command to complete (4s)...")
                time.sleep(4)
                controller.display_telemetry(duration_sec=1)


                print("\n--- Attempting to Disarm ---")
                controller.disarm()
                # controller.display_telemetry(duration_sec=1) # Check disarmed state
            else:
                print("Failed to arm the vehicle.")
        else:
            print("Failed to connect to the vehicle. Please ensure the MAVLink bridge (or SITL) is running.")

    except APIException as api_e:
        print(f"DroneKit API Error: {api_e}")
    except DroneKitConnectionError as conn_e: # More specific connection error
        print(f"DroneKit Connection Error: {conn_e}")
    except Exception as e:
        print(f"An error occurred in the main execution block: {e}")
    finally:
        if controller.vehicle:
            print("Closing vehicle connection...")
            controller.vehicle.close()
            print("Vehicle connection closed.")
        print("--- DroneKit Controller Test Finished ---")
```
