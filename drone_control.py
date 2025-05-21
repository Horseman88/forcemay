import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil # Needed for command message definitions

def connect_vehicle(connection_string):
    """
    Connects to the vehicle and returns the vehicle object.
    Handles potential connection failures.
    """
    print(f"Connecting to vehicle on: {connection_string}")
    try:
        vehicle = connect(connection_string, wait_ready=True, timeout=60)
        print("Vehicle connected!")
        return vehicle
    except APIException as e:
        print(f"APIException: Failed to connect: {e}")
        return None
    except ConnectionRefusedError:
        print("Connection refused. Make sure SITL or vehicle is running.")
        return None
    except TimeoutError:
        print("Timeout connecting to vehicle. Make sure SITL or vehicle is running and accessible.")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during connection: {e}")
        return None

def arm_and_takeoff(vehicle, target_altitude):
    """
    Arms the vehicle and takes off to a target altitude.
    """
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {target_altitude}m")
    vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing MAVLink commands
    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def land_vehicle(vehicle):
    """
    Sets the vehicle mode to LAND and prints a status message.
    """
    print("Setting mode to LAND")
    vehicle.mode = VehicleMode("LAND")
    print("Landing...")
    while vehicle.armed: # Wait until disarmed
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}m, Status: {vehicle.system_status.state}")
        time.sleep(1)
    print("Vehicle landed and disarmed.")


def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and duration.
    This uses the SET_POSITION_TARGET_LOCAL_NED MAVLink message.
    velocity_x: North (m/s)
    velocity_y: East (m/s)
    velocity_z: Down (m/s) - Positive for down, negative for up.
    duration: Time (s) to maintain velocity.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not used)

    print(f"Sending NED velocity: Vx={velocity_x}, Vy={velocity_y}, Vz={velocity_z} for {duration}s")
    for _ in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    # Send a message with zero velocity to stop
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(stop_msg)
    print("Movement finished, stopping vehicle.")


if __name__ == "__main__":
    connection_string = 'tcp:127.0.0.1:5760' # Default SITL connection string
    # Example usage:
    # dronekit-sitl copter --home=YOUR_LAT,YOUR_LON,ALT,DIRECTION
    # e.g. dronekit-sitl copter --home=-35.363261,149.165230,584,353
    # You may need to run the above in a separate terminal if SITL isn't already running

    vehicle = connect_vehicle(connection_string)

    if vehicle:
        try:
            # Ensure vehicle is disarmed if it was previously armed (e.g. from a crashed script)
            if vehicle.armed:
                print("Vehicle is already armed. Disarming first for safety.")
                vehicle.armed = False
                while vehicle.armed:
                    time.sleep(1)
                print("Vehicle disarmed.")

            target_alt = 5 # Target altitude in meters
            arm_and_takeoff(vehicle, target_alt)

            print("Demonstrating movement: Moving NORTH for 3 seconds")
            send_ned_velocity(vehicle, 2, 0, 0, 3) # Move North at 2 m/s for 3 seconds
            time.sleep(2) # Pause

            print("Demonstrating movement: Moving EAST for 3 seconds")
            send_ned_velocity(vehicle, 0, 2, 0, 3) # Move East at 2 m/s for 3 seconds
            time.sleep(2) # Pause
            
            print("Demonstrating movement: Moving UP for 2 seconds (VZ is negative for UP)")
            send_ned_velocity(vehicle, 0, 0, -1, 2) # Move UP at 1m/s for 2 seconds
            time.sleep(2)

            land_vehicle(vehicle)

        except Exception as e:
            print(f"An error occurred in the main execution: {e}")
            # Attempt to land if an error occurs mid-flight
            if vehicle.armed:
                print("Error occurred while armed. Attempting to land.")
                land_vehicle(vehicle)
        finally:
            print("Closing vehicle connection.")
            vehicle.close()
            print("Script finished.")
    else:
        print("Could not connect to vehicle. Exiting script.")
