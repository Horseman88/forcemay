import time
from dronekit import connect, APIException

# Connection string for SITL
connection_string = 'tcp:127.0.0.1:5760'
vehicle = None  # Initialize vehicle to None

print(f"Connecting to vehicle on: {connection_string}")
try:
    # Connect to the Vehicle
    vehicle = connect(connection_string, wait_ready=True, timeout=60) # Increased timeout

    # Print vehicle attributes
    print(f"GPS: {vehicle.gps_0}")
    print(f"Battery: {vehicle.battery}")
    print(f"Last Heartbeat: {vehicle.last_heartbeat}")
    print(f"Is Armable?: {vehicle.is_armable}")
    print(f"System status: {vehicle.system_status.state}")
    print(f"Mode: {vehicle.mode.name}")

except APIException as e:
    print(f"APIException: {e}")
except ConnectionRefusedError:
    print("Connection refused. Make sure SITL is running.")
except TimeoutError: # dronekit.APIException can wrap a TimeoutError for wait_ready
    print("Timeout connecting to vehicle. Make sure SITL is running and accessible.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    # Close vehicle object before exiting script
    if vehicle:
        print("Closing vehicle connection.")
        vehicle.close()

print("Script finished.")
