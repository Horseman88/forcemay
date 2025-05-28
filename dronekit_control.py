import time
from dronekit import connect, VehicleMode, APIException, Command, LocationGlobalRelative
import math

def get_distance_metres(location1, location2):
    # ... (implementation as previously defined)
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def connect_to_simulator(connection_string="udp:127.0.0.1:14550"):
    # ... (implementation as previously defined)
    print(f"Connecting to vehicle on: {connection_string}")
    try:
        vehicle = connect(connection_string, wait_ready=True, timeout=60) 
        print("Connected to vehicle.")
        print(f"  Vehicle type: {vehicle.vehicle_type}")
        print(f"  Firmware version: {vehicle.version}")
        print(f"  System status: {vehicle.system_status.state}")
        print(f"  Mode: {vehicle.mode.name}")
        print(f"  Armed: {vehicle.armed}")
        print(f"  Altitude: {vehicle.location.global_relative_frame.alt}")
        return vehicle
    except APIException as e:
        print(f"APIException: Failed to connect: {e}")
        return None
    except Exception as e:
        print(f"Exception: Failed to connect: {e}")
        return None

def arm_vehicle(vehicle):
    # ... (implementation as previously defined)
    if not vehicle: return False
    print("Arming vehicle...")
    try:
        vehicle.mode = VehicleMode("GUIDED")
        while not vehicle.mode.name == 'GUIDED':
            print("  Waiting for GUIDED mode...")
            time.sleep(0.5)
        
        vehicle.armed = True
        while not vehicle.armed:
            print("  Waiting for arming...")
            time.sleep(0.5)
        print("Vehicle ARMED")
        return True
    except Exception as e:
        print(f"Error arming vehicle: {e}")
        return False

def takeoff_vehicle(vehicle, target_altitude):
    # ... (implementation as previously defined)
    if not vehicle or not vehicle.armed:
        print("Vehicle not available or not armed for takeoff.")
        return False
    print(f"Taking off to {target_altitude}m...")
    try:
        vehicle.simple_takeoff(target_altitude)
        while True:
            current_altitude = vehicle.location.global_relative_frame.alt
            print(f"  Altitude: {current_altitude:.2f}m")
            if current_altitude >= target_altitude * 0.95:
                print("Reached target altitude.")
                break
            if current_altitude > target_altitude * 1.5: 
                print("Overshot target altitude, something might be wrong.")
                break
            time.sleep(1)
        return True
    except Exception as e:
        print(f"Error during takeoff: {e}")
        return False

def land_vehicle(vehicle):
    # ... (implementation as previously defined)
    if not vehicle:
        print("Vehicle not available for landing.")
        return False
    print("Landing vehicle...")
    try:
        vehicle.mode = VehicleMode("LAND")
        while vehicle.armed: 
            current_altitude = vehicle.location.global_relative_frame.alt
            print(f"  Descending... Altitude: {current_altitude:.2f}m")
            if current_altitude < 0.1: 
                 print("  Assumed landed or close to ground.")
                 break
            time.sleep(1)
        print("Vehicle LANDED (or assumed landed and disarmed by simulator)")
        return True
    except Exception as e:
        print(f"Error during landing: {e}")
        return False
        
def goto_position(vehicle, target_lat, target_lon, target_alt):
    # ... (implementation as previously defined)
    if not vehicle or not vehicle.armed:
        print("Vehicle not available or not armed for goto.")
        return False

    target_location = LocationGlobalRelative(target_lat, target_lon, target_alt)
    print(f"Commanding GOTO: Lat={target_lat}, Lon={target_lon}, Alt={target_alt}m")
    
    try:
        vehicle.simple_goto(target_location)
        
        # start_time = time.time() # For timeout
        # MAX_GOTO_TIME = 60 # seconds
        
        while True:
            current_location = vehicle.location.global_relative_frame
            if not current_location or current_location.lat is None or current_location.lon is None:
                print("  Waiting for valid location data...")
                time.sleep(1)
                continue

            distance = get_distance_metres(current_location, target_location)
            print(f"  Distance to target: {distance:.2f}m, "
                  f"Current Alt: {current_location.alt:.2f}m")

            if distance < 2.0: # Tolerance of 2 meters
                print("Reached target waypoint.")
                break
            
            # if time.time() - start_time > MAX_GOTO_TIME:
            #     print("GOTO command timed out.")
            #     return False
            time.sleep(1)
        return True
    except Exception as e:
        print(f"Error during GOTO: {e}")
        return False

if __name__ == '__main__':
    sim_vehicle = connect_to_simulator()

    if sim_vehicle:
        print("\n--- Starting mission ---")
        
        if arm_vehicle(sim_vehicle):
            time.sleep(2) 

            if takeoff_vehicle(sim_vehicle, 5.0): 
                print("\n--- Hovering for a bit after takeoff ---")
                time.sleep(3)

                print("\n--- Commanding GOTO to a nearby location ---")
                target_lat = 47.397742 + 0.0001 
                target_lon = 8.545594 + 0.0001 
                
                if goto_position(sim_vehicle, target_lat, target_lon, 5.0):
                    print("\n--- Hovering at new location ---")
                    time.sleep(5)
                else:
                    print("GOTO command failed.")

                land_vehicle(sim_vehicle)
            else:
                print("Takeoff failed.")
        else:
            print("Arming failed.")

        print("\n--- Mission complete ---")
        sim_vehicle.close()
        print("Vehicle connection closed.")
    else:
        print("Could not connect to simulator. Ensure it is running.")
