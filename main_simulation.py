import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import time
import math # For trigonometric functions in rotation

# From our existing scripts
import drone_control
from dronekit import APIException, VehicleMode, LocationGlobalRelative
# from pymavlink.mavutil import mavlink # Not strictly needed if using dronekit's location math

# --- Helper function for distance calculation ---
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal or LocationGlobalRelative objects.
    This function is a simplified version, good for short distances.
    """
    if aLocation1 is None or aLocation2 is None or \
       not hasattr(aLocation1, 'lat') or not hasattr(aLocation1, 'lon') or \
       not hasattr(aLocation2, 'lat') or not hasattr(aLocation2, 'lon'):
        # print("Warning: Invalid location objects for distance calculation.")
        return float('inf') 

    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# --- 3D Model Definition (Simple Cube) ---
# Vertices for a cube centered at origin
vertices_cube = (
    (0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (-0.5, 0.5, -0.5), (-0.5, -0.5, -0.5),
    (0.5, -0.5, 0.5), (0.5, 0.5, 0.5), (-0.5, -0.5, 0.5), (-0.5, 0.5, 0.5)
)
edges_cube = (
    (0,1), (0,3), (0,4), (2,1), (2,3), (2,7),
    (6,3), (6,4), (6,7), (5,1), (5,4), (5,7)
)
surfaces_cube = (
    (0,1,2,3), (3,2,7,6), (6,7,5,4),
    (4,5,1,0), (1,5,7,2), (4,0,3,6)
)
colors_cube = (
    (1,0,0), (0,1,0), (0,0,1), (1,1,0), (1,0,1), (0,1,1)
)

def draw_model(model_vertices, model_surfaces, model_colors):
    glBegin(GL_QUADS)
    for i, surface in enumerate(model_surfaces):
        glColor3fv(model_colors[i % len(model_colors)])
        for vertex_index in surface:
            glVertex3fv(model_vertices[vertex_index])
    glEnd()

    # Draw edges for clarity (optional, can be slow)
    # glColor3fv((0,0,0)) # Black color for edges
    # glBegin(GL_LINES)
    # for edge in model_edges:
    #   for vertex_index in edge:
    #       glVertex3fv(model_vertices[vertex_index])
    # glEnd()

def draw_ground_grid(size=50, step=2):
    """Draws a grid on the XZ plane."""
    glLineWidth(1.0)
    glColor3f(0.5, 0.5, 0.5) # Grey color for grid
    glBegin(GL_LINES)
    for i in range(-size, size + 1, step):
        # Lines along X-axis
        glVertex3f(i, 0, -size)
        glVertex3f(i, 0, size)
        # Lines along Z-axis
        glVertex3f(-size, 0, i)
        glVertex3f(size, 0, i)
    glEnd()


def main():
    # --- Initialization ---
    pygame.init()
    display_width = 1000
    display_height = 750
    display = (display_width, display_height)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Drone Simulation - SITL Integration")

    # Setup 3D perspective
    gluPerspective(45, (display_width / display_height), 0.1, 100.0) # Increased far clipping plane
    # Camera initial position: looking somewhat down from a distance
    glTranslatef(0.0, -5.0, -20) # X, Y (Up/Down), Z (Forward/Backward)
    glRotatef(-30, 1, 0, 0)       # Pitch camera down slightly

    glEnable(GL_DEPTH_TEST) # Important for correct 3D rendering

    # DroneKit Connection
    connection_string = 'tcp:127.0.0.1:5760'
    print(f"Attempting to connect to vehicle on: {connection_string}")
    vehicle = drone_control.connect_vehicle(connection_string)

    if not vehicle:
        print("Failed to connect to vehicle. Exiting.")
        pygame.quit()
        return

    # Simulation state variables
    running = True
    target_altitude = 5.0 # meters
    movement_speed = 1.0  # m/s for keyboard controls
    movement_duration = 1 # seconds for keyboard controls
    
    # Drone telemetry variables (initialize to safe defaults)
    drone_pos_north = 0
    drone_pos_east = 0
    drone_pos_down = 0 # Will be negative for altitude
    drone_pitch = 0
    drone_roll = 0
    drone_yaw = 0

    # --- Path Following Variables ---
    waypoints_global = [] # List of LocationGlobalRelative objects for the path
    # For visualization, we'll use NED coordinates relative to home for drawing
    waypoints_ned_for_drawing = [] 
    current_waypoint_index = -1 
    path_following_active = False
    WAYPOINT_REACH_THRESHOLD_METERS = 2.5 # How close to get to a waypoint (ground distance)
    WAYPOINT_ALTITUDE_THRESHOLD_METERS = 0.5 # How close to get to waypoint altitude

    # --- Main Simulation Loop ---
    try:
        # Wait for home location to be set by SITL before defining waypoints
        if vehicle: # Only proceed if vehicle object exists
            print("Waiting for home location to be set by SITL...")
            while running and vehicle and not vehicle.home_location:
                time.sleep(0.1) 
                for event_wait in pygame.event.get(): # Process quit events while waiting
                    if event_wait.type == pygame.QUIT or \
                       (event_wait.type == pygame.KEYDOWN and event_wait.key == pygame.K_ESCAPE):
                        running = False
                        break
            
            if running and vehicle and vehicle.home_location:
                print(f"Home location set: Lat {vehicle.home_location.lat}, Lon {vehicle.home_location.lon}, Alt {vehicle.home_location.alt}")
                
                # Define waypoints as NED offsets from home, then convert to LocationGlobalRelative
                # (North, East, Altitude_above_home)
                # Example: A square pattern 10m North, then 10m East, then 10m South, then 10m West
                wp_alt_agl = target_altitude # Altitude above ground level (AGL) for waypoints

                # These are NED offsets from home (N, E, D - but we use AGL for alt)
                # D (down) will be -wp_alt_agl
                path_definition_ned = [
                    (10, 0, wp_alt_agl),   # 10m North, 0m East, at target_altitude
                    (10, 10, wp_alt_agl),  # 10m North, 10m East
                    (0, 10, wp_alt_agl),   # 0m North, 10m East (effectively 10m East of home)
                    (0, 0, wp_alt_agl),    # Back to home location (0m N, 0m E) at altitude
                ]
                
                # Store NED for drawing (X_gl=E, Y_gl=Up=-D, Z_gl=-N)
                # So, for drawing: (E_ned, Alt_agl, -N_ned)
                waypoints_ned_for_drawing = [(p[1], p[2], -p[0]) for p in path_definition_ned]

                # Convert NED waypoints to LocationGlobalRelative
                home_lat = vehicle.home_location.lat
                home_lon = vehicle.home_location.lon
                # home_alt = vehicle.home_location.alt # This is AMSL altitude

                for ned_point in path_definition_ned:
                    north_m, east_m, alt_m_agl = ned_point
                    # Create LocationGlobal (absolute lat/lon, absolute AMSL alt) first
                    # then convert to LocationGlobalRelative if needed, or use directly if simple_goto handles it.
                    # DroneKit's simple_goto expects LocationGlobal or LocationGlobalRelative.
                    # For LocationGlobalRelative, 'alt' is AGL.
                    
                    # Calculate global lat/lon for the waypoint
                    # Approximation:
                    dlat = north_m / 111319.5  # meters to degrees latitude
                    dlon = east_m / (111319.5 * math.cos(math.radians(home_lat))) # meters to degrees longitude
                    
                    wp_global_lat = home_lat + dlat
                    wp_global_lon = home_lon + dlon
                    # For LocationGlobalRelative, alt is AGL.
                    waypoints_global.append(LocationGlobalRelative(wp_global_lat, wp_global_lon, alt_m_agl))

                if waypoints_global:
                    print(f"Defined {len(waypoints_global)} waypoints globally.")
                    # print("NED for drawing:", waypoints_ned_for_drawing)
                else:
                    print("No waypoints were defined.")
            elif not vehicle:
                print("Vehicle connection lost before home location could be set.")
                running = False
            elif not running:
                print("Exiting before home location set.")


        while running:
            # current_time = time.time() # Not used yet, but can be for timeouts
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    
                    # Process drone commands only if vehicle is connected
                    if vehicle:
                        # Path following mode takes precedence for some keys
                        if path_following_active:
                            if event.key == pygame.K_l: # Land command interrupts path following
                                print("LAND command received. Interrupting path following and landing.")
                                path_following_active = False
                                current_waypoint_index = -1
                                drone_control.land_vehicle(vehicle)
                            elif event.key == pygame.K_c: # Cancel path following and hover
                                print("CANCEL command received. Stopping path following and hovering (GUIDED).")
                                path_following_active = False
                                current_waypoint_index = -1
                                if vehicle.mode.name != "GUIDED":
                                    vehicle.mode = VehicleMode("GUIDED")
                                print("Switched to GUIDED mode, holding position.")
                            # Other keys are ignored during active path following
                        
                        else: # Not path following, process manual controls
                            if event.key == pygame.K_t:
                                print("Attempting arm and takeoff...")
                                if vehicle.mode.name == "GUIDED" or vehicle.mode.name == "STABILIZE" or vehicle.mode.name == "LOITER": # common modes for takeoff
                                    drone_control.arm_and_takeoff(vehicle, target_altitude)
                                else:
                                    print(f"Cannot takeoff in mode {vehicle.mode.name}. Switch to GUIDED, STABILIZE or LOITER first.")
                            elif event.key == pygame.K_l:
                                print("Attempting to land...")
                                drone_control.land_vehicle(vehicle)
                            elif event.key == pygame.K_p: # Start Path Following
                                if not waypoints_global:
                                    print("No waypoints defined. Cannot start path following.")
                                elif not vehicle.armed:
                                    print("Vehicle not armed. Arm and takeoff first (T).")
                                elif vehicle.location.global_relative_frame is None or \
                                     vehicle.location.global_relative_frame.alt < target_altitude * 0.8:
                                     alt_actual = vehicle.location.global_relative_frame.alt if vehicle.location.global_relative_frame else -1.0
                                     print(f"Vehicle too low (alt: {alt_actual:.1f}m) or location not available. Takeoff to ~{target_altitude:.1f}m first.")
                                else:
                                    print("Starting path following ('P' pressed)...")
                                    path_following_active = True
                                    current_waypoint_index = 0
                                    target_wp = waypoints_global[current_waypoint_index]
                                    print(f"Moving to waypoint {current_waypoint_index + 1}/{len(waypoints_global)}: Lat {target_wp.lat:.6f}, Lon {target_wp.lon:.6f}, Alt {target_wp.alt:.1f}m AGL")
                                    if vehicle.mode.name != "GUIDED":
                                        print("Switching to GUIDED mode for path following.")
                                        vehicle.mode = VehicleMode("GUIDED")
                                        time.sleep(0.5) # Give mode change time
                                    if vehicle.mode.name == "GUIDED": # Check if mode change was successful
                                        vehicle.simple_goto(target_wp) # groundspeed can be set here too
                                    else:
                                        print(f"Failed to switch to GUIDED mode. Current mode: {vehicle.mode.name}. Aborting path following.")
                                        path_following_active = False
                                        current_waypoint_index = -1

                            # Manual NED Movement (only if not path following)
                            elif event.key == pygame.K_UP: 
                                print("Manual: Moving North...")
                                drone_control.send_ned_velocity(vehicle, movement_speed, 0, 0, movement_duration)
                            elif event.key == pygame.K_DOWN: 
                                print("Manual: Moving South...")
                                drone_control.send_ned_velocity(vehicle, -movement_speed, 0, 0, movement_duration)
                            elif event.key == pygame.K_LEFT: 
                                print("Manual: Moving West...")
                                drone_control.send_ned_velocity(vehicle, 0, -movement_speed, 0, movement_duration)
                            elif event.key == pygame.K_RIGHT: 
                                print("Manual: Moving East...")
                                drone_control.send_ned_velocity(vehicle, 0, movement_speed, 0, movement_duration)
                            elif event.key == pygame.K_w: 
                                print("Manual: Moving Up...")
                                drone_control.send_ned_velocity(vehicle, 0, 0, -movement_speed, movement_duration)
                            elif event.key == pygame.K_s: 
                                print("Manual: Moving Down...")
                                drone_control.send_ned_velocity(vehicle, 0, 0, movement_speed, movement_duration)
            
            # --- Path Following Logic ---
            if vehicle and path_following_active and current_waypoint_index >= 0:
                if vehicle.mode.name != "GUIDED":
                    print("Warning: Vehicle no longer in GUIDED mode during path following. Path aborted.")
                    path_following_active = False
                    current_waypoint_index = -1
                else:
                    target_wp_global = waypoints_global[current_waypoint_index]
                    current_loc_global = vehicle.location.global_relative_frame 
                    
                    if current_loc_global and hasattr(current_loc_global, 'lat') and hasattr(current_loc_global, 'lon') and hasattr(current_loc_global, 'alt'):
                        dist_to_target_ground = get_distance_metres(current_loc_global, target_wp_global)
                        alt_diff = abs(current_loc_global.alt - target_wp_global.alt)
                        
                        # print(f"To WP {current_waypoint_index+1}: Dist {dist_to_target_ground:.1f}m, AltDiff {alt_diff:.1f}m, CurAlt {current_loc_global.alt:.1f}m")

                        if dist_to_target_ground <= WAYPOINT_REACH_THRESHOLD_METERS and alt_diff <= WAYPOINT_ALTITUDE_THRESHOLD_METERS:
                            print(f"Reached waypoint {current_waypoint_index + 1}.")
                            current_waypoint_index += 1
                            if current_waypoint_index < len(waypoints_global):
                                next_wp = waypoints_global[current_waypoint_index]
                                print(f"Moving to waypoint {current_waypoint_index + 1}/{len(waypoints_global)}: Lat {next_wp.lat:.6f}, Lon {next_wp.lon:.6f}, Alt {next_wp.alt:.1f}m AGL")
                                vehicle.simple_goto(next_wp)
                            else:
                                print("All waypoints reached. Path complete. Hovering in GUIDED mode.")
                                path_following_active = False
                                current_waypoint_index = -1
                                # Optional: Land after path completion
                                # print("Path complete. Landing now.")
                                # drone_control.land_vehicle(vehicle)
                    else:
                        # print("Waiting for valid location data for path following...")
                        time.sleep(0.1) # Brief pause if location data is temporarily unavailable

            # --- Telemetry Update ---
            if vehicle and vehicle.location.local_frame and vehicle.attitude and vehicle.location.global_relative_frame:
                drone_pos_north = vehicle.location.local_frame.north or 0
                drone_pos_east = vehicle.location.local_frame.east or 0
                drone_pos_down = vehicle.location.local_frame.down or 0 
                
                drone_pitch = vehicle.attitude.pitch or 0 
                drone_roll = vehicle.attitude.roll or 0   
                drone_yaw = vehicle.attitude.yaw or 0     
            
            # --- Rendering ---
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            
            # Static Ground Grid at Y_gl=0
            glPushMatrix()
            draw_ground_grid()
            glPopMatrix()

            # Draw Waypoints Path and Markers (using waypoints_ned_for_drawing)
            if waypoints_ned_for_drawing:
                glPushMatrix()
                # Path lines
                glColor3f(0.9, 0.9, 0.2) # Yellow for path lines
                glLineWidth(2.0)
                glBegin(GL_LINE_STRIP)
                for p_ned_gl in waypoints_ned_for_drawing: # These are already (E_gl, Alt_gl, N_gl_inverted)
                    glVertex3fv(p_ned_gl)
                glEnd()

                # Waypoint markers (spheres)
                for i, p_ned_gl in enumerate(waypoints_ned_for_drawing):
                    glPushMatrix()
                    glTranslatef(p_ned_gl[0], p_ned_gl[1], p_ned_gl[2])
                    quad = gluNewQuadric()
                    if path_following_active and i == current_waypoint_index :
                         glColor3f(0.0, 1.0, 0.0) # Green for current target waypoint
                    elif path_following_active and i < current_waypoint_index and current_waypoint_index != -1: # Check index not -1 for completed paths
                         glColor3f(0.4, 0.4, 0.4) # Darker Grey for visited waypoints
                    else: # Pending or path not active
                        glColor3f(1.0, 0.5, 0.0) # Orange for pending waypoints
                    gluSphere(quad, 0.3, 12, 12) 
                    gluDeleteQuadric(quad)
                    glPopMatrix()
                glPopMatrix()
            
            # Drone Model Rendering
            glPushMatrix() 
            glTranslatef(drone_pos_east, -drone_pos_down, -drone_pos_north)
            glRotatef(math.degrees(drone_yaw), 0, 1, 0)    
            glRotatef(math.degrees(drone_pitch), 1, 0, 0)  
            glRotatef(math.degrees(drone_roll), 0, 0, 1)   

            draw_model(vertices_cube, surfaces_cube, colors_cube) 
            glPopMatrix() # Restore matrix state

            pygame.display.flip()
            pygame.time.wait(20) # Approx 50 FPS

    except APIException as api_e:
        print(f"DroneKit API Error during simulation: {api_e}")
    except Exception as e:
        print(f"An unexpected error occurred in the simulation loop: {e}")
    finally:
        # --- Cleanup ---
        if vehicle:
            # If still armed, attempt to land before closing
            if vehicle.armed:
                print("Simulation ending, vehicle still armed. Attempting to land...")
                drone_control.land_vehicle(vehicle)
            print("Closing vehicle connection.")
            vehicle.close()
        pygame.quit()
        print("Simulation finished.")

if __name__ == '__main__':
    # --- Start SITL if not already running (Example for Copter) ---
    # You would typically run this in a separate terminal:
    # dronekit-sitl copter --home=-35.363261,149.165230,584,353
    # Or for plane:
    # dronekit-sitl plane --home=-35.363261,149.165230,584,353
    print("Ensure SITL is running in a separate terminal.")
    print("Example: dronekit-sitl copter --home=-35.363261,149.165230,584,353")
    main()
