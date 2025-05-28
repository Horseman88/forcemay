from panda3d.core import AmbientLight, DirectionalLight, NodePath, PandaNode, InputDevice
from panda3d.core import Plane, Vec3, Vec4, Point3
from panda3d.core import GeomVertexFormat, GeomVertexData
from panda3d.core import Geom, GeomTriangles, GeomVertexWriter
from panda3d.core import TransparencyAttrib 
from panda3d.core import CardMaker 
from direct.showbase.ShowBase import ShowBase
import numpy as np
import socket # For UDP communication
import json   # For formatting/parsing telemetry and commands
import sys # For sys.exit() in userExit
import time # For command timeout

from .physics_engine import DronePhysics 

class DroneSimulator3D(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        
        self.physics_drone = DronePhysics() # Full physics setup
        self.setup_scene() 
        self.setup_drone()
        
        # Keyboard control inputs
        self.current_thrust_factor = 0.73575 
        self.accept("o", self.increase_thrust); self.accept("p", self.decrease_thrust)
        self.desired_roll_input = 0.0; self.desired_pitch_input = 0.0; self.desired_yaw_input = 0.0
        self.roll_input_strength = 0.2; self.pitch_input_strength = 0.2; self.yaw_input_strength = 0.3
        self.accept("a", self.set_roll_input, [-1.0]); self.accept("a-up", self.set_roll_input, [0.0])
        self.accept("d", self.set_roll_input, [1.0]); self.accept("d-up", self.set_roll_input, [0.0])
        self.accept("w",self.set_pitch_input,[1.]); self.accept("w-up",self.set_pitch_input,[0.])
        self.accept("s",self.set_pitch_input,[-1.]); self.accept("s-up",self.set_pitch_input,[0.])
        self.accept("q",self.set_yaw_input,[1.]); self.accept("q-up",self.set_yaw_input,[0.])
        self.accept("e",self.set_yaw_input,[-1.]); self.accept("e-up",self.set_yaw_input,[0.])
        print("Controls: Thrust:o/p, Roll:a/d, Pitch:w/s, Yaw:q/e")

        # UDP Socket for sending telemetry to MAVLink Bridge
        self.bridge_udp_telemetry_egress_address = ('127.0.0.1', 14551) 
        try:
            self.udp_telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # print(f"UDP Telemetry Egress socket created. Will send to {self.bridge_udp_telemetry_egress_address}")
        except socket.error as e:
            print(f"Error creating UDP telemetry Egress socket: {e}")
            self.udp_telemetry_socket = None
        except Exception as e:
            print(f"Unexpected error creating Telemetry Egress socket: {e}")
            self.udp_telemetry_socket = None

        # UDP Socket for receiving commands from MAVLink Bridge
        self.bridge_udp_command_ingress_address = ('0.0.0.0', 14552) # Listen on all interfaces, port 14552
        self.command_socket = None
        self.active_mavlink_command = None # Changed to None, new command overwrites
        try:
            self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.command_socket.setblocking(False) # Non-blocking
            self.command_socket.bind(self.bridge_udp_command_ingress_address)
            print(f"UDP Command Ingress socket listening on {self.bridge_udp_command_ingress_address}")
        except socket.error as e:
            print(f"Error creating or binding UDP command Ingress socket: {e}")
            self.command_socket = None
        except Exception as e:
            print(f"Unexpected error setting up command Ingress socket: {e}")
            self.command_socket = None
        
        self.is_armed = False # Initial armed state
        self.last_attitude_target_time = 0 # For timing out MAVLink commands if desired

        # Register cleanup function for Panda3D's exit mechanism
        self.accept('escape', self.userExit) # Default Panda3D exit key

        self.taskMgr.add(self.update_simulation_task, "update_simulation_task")

    def _process_incoming_commands(self):
        if not self.command_socket:
            return

        new_command_received = False
        json_string = "" # Initialize to prevent reference before assignment in except block
        try:
            while True: # Read all available packets
                data, addr = self.command_socket.recvfrom(1024) # Buffer 1KB
                if not data:
                    break 
                
                json_string = data.decode('utf-8')
                # print(f"Sim received command string: {json_string}") # Debug
                command_data = json.loads(json_string)
                
                # Store the latest command, overwriting previous of same type or any type
                self.active_mavlink_command = command_data 
                new_command_received = True
                # print(f"Sim updated active_mavlink_command: {self.active_mavlink_command}") # Debug

                # Handle immediate state changes like ARM/DISARM
                if self.active_mavlink_command and self.active_mavlink_command.get("type") == "ARM_DISARM":
                    action = self.active_mavlink_command.get("action")
                    if action == 1 and not self.is_armed:
                        self.is_armed = True
                        print("SIM: Drone ARMED via MAVLink command.")
                    elif action == 0 and self.is_armed:
                        self.is_armed = False
                        print("SIM: Drone DISARMED via MAVLink command.")
                    # If ARM_DISARM is received, perhaps clear other commands like SET_ATTITUDE_TARGET?
                    # For now, let SET_ATTITUDE_TARGET persist until overwritten or timed out.
                    
        except socket.error as e:
            # In non-blocking mode, if no data, BlockingIOError (or EAGAIN/EWOULDBLOCK) is raised.
            # Check for Windows specific error code WSAEWOULDBLOCK as well.
            if e.errno == socket.errno.EAGAIN or \
               e.errno == socket.errno.EWOULDBLOCK or \
               (hasattr(socket.errno, 'WSAEWOULDBLOCK') and e.errno == socket.errno.WSAEWOULDBLOCK):
                pass # This is expected, no data available.
            else:
                print(f"Sim socket error receiving commands: {e}") # Log other socket errors
        except json.JSONDecodeError as e:
            print(f"Sim error decoding command JSON: {e} - Data: '{json_string[:100]}...'")
        except Exception as e:
            print(f"Sim error processing received commands: {e}")
        
        if new_command_received and self.active_mavlink_command and self.active_mavlink_command.get("type") == "SET_ATTITUDE_TARGET":
            self.last_attitude_target_time = time.time()


    def update_simulation_task(self, task):
        dt = globalClock.getDt()
        self._process_incoming_commands()

        # Determine control inputs (MAVLink priority)
        base_thrust_cmd = 0.0
        roll_effect = 0.0
        pitch_effect = 0.0
        yaw_effect = 0.0
        
        # Command timeout (e.g., if no new SET_ATTITUDE_TARGET for 0.5s, revert to keyboard)
        CMD_TIMEOUT_SEC = 0.5 
        mavlink_attitude_active = False

        if self.active_mavlink_command and self.active_mavlink_command.get("type") == "SET_ATTITUDE_TARGET":
            # Check if the command is recent enough
            if (time.time() - self.last_attitude_target_time) < CMD_TIMEOUT_SEC:
                mavlink_attitude_active = True
                cmd = self.active_mavlink_command
                # For SET_ATTITUDE_TARGET, thrust is direct. Rates are body rates.
                base_thrust_cmd = cmd.get("thrust", 0.0) 
                # The physics engine's mixer expects effects that produce corresponding moments.
                # If cmd roll_rate is positive (roll right), this should lead to positive tau_x.
                # Mixer: roll_effect is positive for roll right.
                roll_effect = cmd.get("roll_rate", 0.0) # Direct rate command
                pitch_effect = cmd.get("pitch_rate", 0.0) # Direct rate command
                
                # MAVLink body_yaw_rate: positive is CW. 
                # Our mixer: yaw_effect positive for Yaw Left (CCW). So, invert.
                yaw_effect = -cmd.get("yaw_rate", 0.0)

                # print(f"Sim MAVLink CMD: T={base_thrust_cmd:.2f} RR={roll_effect:.2f} PR={pitch_effect:.2f} YR={yaw_effect:.2f}") # Debug
            else:
                # print("SET_ATTITUDE_TARGET timed out, reverting to keyboard/manual.")
                self.active_mavlink_command = None # Clear stale command
                # This ensures that if MAVLink stops, keyboard can take over if armed.

        if not mavlink_attitude_active: # Revert to keyboard if no active MAVLink attitude command
            base_thrust_cmd = np.clip(self.current_thrust_factor, 0.0, 1.0)
            roll_effect = self.desired_roll_input * self.roll_input_strength
            pitch_effect = self.desired_pitch_input * self.pitch_input_strength
            yaw_effect = self.desired_yaw_input * self.yaw_input_strength
            # print(f"Sim Keyboard CMD: T={base_thrust_cmd:.2f} RR={roll_effect:.2f} PR={pitch_effect:.2f} YR={yaw_effect:.2f}") # Debug


        # Apply arming state
        if not self.is_armed:
            motor_speeds = np.zeros(4)
            # Optional: if disarmed, reset active_mavlink_command to prevent re-arming with old thrust value
            # self.active_mavlink_command = None 
        else:
            # Calculate motor speeds based on the determined effects and base thrust
            motor_speeds = np.zeros(4)
            # Mixer for X-quad:
            # Motor 0 (FR): Thrust - Roll + Pitch - Yaw
            # Motor 1 (FL): Thrust + Roll + Pitch + Yaw
            # Motor 2 (RL): Thrust + Roll - Pitch - Yaw
            # Motor 3 (RR): Thrust - Roll - Pitch + Yaw
            # Effects:
            # roll_effect: +ve rolls right (increases M1, M2; decreases M0, M3)
            # pitch_effect: +ve pitches nose up (increases M0, M1; decreases M2, M3)
            # yaw_effect: +ve yaws left/CCW (increases M1, M3; decreases M0, M2)

            motor_speeds[0] = base_thrust_cmd - roll_effect + pitch_effect - yaw_effect # FR
            motor_speeds[1] = base_thrust_cmd + roll_effect + pitch_effect + yaw_effect # FL
            motor_speeds[2] = base_thrust_cmd + roll_effect - pitch_effect - yaw_effect # RL (Corrected from prompt's M2 = T+R-P-Y)
            motor_speeds[3] = base_thrust_cmd - roll_effect - pitch_effect + yaw_effect # RR (Corrected from prompt's M3 = T-R-P+Y)
            
            motor_speeds = np.clip(motor_speeds, 0.0, 1.0) # Ensure motor speeds are valid

        # The rest is the same: physics update, telemetry sending, 3D model update
        if hasattr(self, 'physics_drone') and self.physics_drone is not None:
            self.physics_drone.update_physics(dt, motor_speeds) # Pass final motor_speeds
            
            telemetry_dict = {
                "roll": self.physics_drone.attitude[0],     # Радианы
                "pitch": self.physics_drone.attitude[1],    # Радианы
                "yaw": self.physics_drone.attitude[2],      # Радианы
                "x": self.physics_drone.position[0],        # Метры
                "y_sim": self.physics_drone.position[1],    # Метры (y_sim)
                "z": self.physics_drone.position[2],        # Метры (высота)
                "vx": self.physics_drone.velocity[0],       # м/с
                "vy": self.physics_drone.velocity[1],       # м/с
                "vz": self.physics_drone.velocity[2],       # м/с
                "roll_rate": self.physics_drone.angular_velocity[0],  # рад/с
                "pitch_rate": self.physics_drone.angular_velocity[1], # рад/с
                "yaw_rate": self.physics_drone.angular_velocity[2]    # рад/с
            }
            self.send_telemetry(telemetry_dict)
            
            new_pos = self.physics_drone.position
            new_att_rad = self.physics_drone.attitude
            if hasattr(self, 'drone_model') and self.drone_model:
                self.drone_model.setPos(new_pos[0], new_pos[1], new_pos[2])
                h_deg = np.degrees(new_att_rad[2])
                p_deg = np.degrees(new_att_rad[1])
                r_deg = np.degrees(new_att_rad[0])
                self.drone_model.setHpr(h_deg, p_deg, r_deg)
        return task.cont

    def cleanup_sockets(self): 
        print("Cleaning up simulator sockets...")
        if hasattr(self, 'udp_telemetry_socket') and self.udp_telemetry_socket:
            self.udp_telemetry_socket.close()
            self.udp_telemetry_socket = None
            print("UDP Telemetry Egress socket closed.")
        if hasattr(self, 'command_socket') and self.command_socket:
            self.command_socket.close()
            self.command_socket = None
            print("UDP Command Ingress socket closed.")
            
    def userExit(self): 
        """Panda3D specific exit handler."""
        self.cleanup_sockets()
        print("Panda3D simulator userExit called. Exiting.")
        if hasattr(self, 'destroy'): # Ensure destroy is callable (ShowBase method)
             self.destroy() 
        sys.exit() 

    def send_telemetry(self, telemetry_data: dict):
        if not self.udp_telemetry_socket:
            return
        try:
            json_data = json.dumps(telemetry_data)
            byte_data = json_data.encode('utf-8')
            self.udp_telemetry_socket.sendto(byte_data, self.bridge_udp_telemetry_egress_address)
        except socket.error as e:
            print(f"Socket error sending telemetry: {e}")
        except Exception as e:
            print(f"Error sending telemetry: {e}")

    def set_roll_input(self,v:float): self.desired_roll_input=v
    def set_pitch_input(self,v:float): self.desired_pitch_input=v
    def set_yaw_input(self,v:float): self.desired_yaw_input=v
    def increase_thrust(self): self.current_thrust_factor=min(1.,self.current_thrust_factor+.02)
    def decrease_thrust(self): self.current_thrust_factor=max(0.,self.current_thrust_factor-.02)
    
    def setup_scene(self): 
        ground_plane = Plane(Vec3(0,0,1),Point3(0,0,0));gd=GeomVertexData('g',GeomVertexFormat.getV3n3c4(),Geom.UHStatic);v=GeomVertexWriter(gd,'vertex');n=GeomVertexWriter(gd,'normal');c=GeomVertexWriter(gd,'color');s=50;v.addData3f(-s,-s,0);v.addData3f(s,-s,0);v.addData3f(s,s,0);v.addData3f(-s,s,0);
        for _ in range(4):n.addData3f(0,0,1);c.addData4f(.2,.6,.2,1);
        t1=GeomTriangles(Geom.UHStatic);t1.addVertices(0,1,2);t1.closePrimitive();t2=GeomTriangles(Geom.UHStatic);t2.addVertices(0,2,3);t2.closePrimitive();gg=Geom(gd);gg.addPrimitive(t1);gg.addPrimitive(t2);
        self.ground=self.render.attachNewNode(NodePath("ground_node"));self.ground.attachNewNode(gg);self.ground.setPos(0,0,0);
        al=AmbientLight('al');al.setColor(Vec4(.4,.4,.4,1));self.alnp=self.render.attachNewNode(al);self.render.setLight(self.alnp);
        dl=DirectionalLight('dl');dl.setColor(Vec4(.8,.8,.7,1));dl.setDirection(Vec3(-5,-5,-5));self.dlnp=self.render.attachNewNode(dl);self.render.setLight(self.dlnp);
        self.disableMouse();self.camera.setPos(0,-60,25);self.camera.lookAt(0,0,5);
    
    def setup_drone(self): 
        try:
            self.drone_model = self.loader.loadModel("models/box") 
        except Exception: 
            self.drone_model = None

        if self.drone_model is None or self.drone_model.isEmpty():
            cm=CardMaker("drone_cube_maker");cm.setFrame(-0.5,.5,-0.5,.5,-0.5,.5);self.drone_model=NodePath(PandaNode("drone_cube_node"));
            faces_props = [ 
                {'name': 'top', 'pos': (0, 0, 0.5), 'hpr': (0, -90, 0)},
                {'name': 'bottom', 'pos': (0, 0, -0.5), 'hpr': (0, 90, 0)},
                {'name': 'right', 'pos': (0.5, 0, 0), 'hpr': (90, 0, 0)},
                {'name': 'left', 'pos': (-0.5, 0, 0), 'hpr': (-90, 0, 0)},
                {'name': 'front', 'pos': (0, 0.5, 0), 'hpr': (0, 0, 0)},
                {'name': 'back', 'pos': (0, -0.5, 0), 'hpr': (180, 0, 0)}
            ]
            for props in faces_props:
                face = cm.generate()
                np_face = self.drone_model.attachNewNode(props['name']) 
                np_face.attachNewNode(face) 
                np_face.setPosHpr(props['pos'][0], props['pos'][1], props['pos'][2], props['hpr'][0], props['hpr'][1], props['hpr'][2])
        
        self.drone_model.reparentTo(self.render);self.drone_model.setColor(.8,.2,.2,1);self.drone_model.setScale(1.0, 1.0, 0.3); 
        if hasattr(self, 'physics_drone') and self.physics_drone:
            ipos=self.physics_drone.position;self.drone_model.setPos(ipos[0],ipos[1],ipos[2]);
            iatt=self.physics_drone.attitude;h,p,r=np.degrees(iatt[2]),np.degrees(iatt[1]),np.degrees(iatt[0]);self.drone_model.setHpr(h,p,r);
        else: self.drone_model.setPos(0,0,5);

if __name__ == '__main__':
    app = DroneSimulator3D()
    try:
        app.run()
    except SystemExit: 
        print("Caught SystemExit, Panda3D app closed.")
    except Exception as e:
        print(f"An unhandled exception occurred in app.run(): {e}")
        if hasattr(app, 'cleanup_sockets') and callable(app.cleanup_sockets):
            app.cleanup_sockets()
```
