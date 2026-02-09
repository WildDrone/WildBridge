"""
WildBridge MAVLink Proxy for QGroundControl

This script bridges WildBridge DJI drone telemetry to QGroundControl via MAVLink protocol.
It allows you to visualize DJI drone data in QGroundControl and send basic commands.

Features:
- Real-time telemetry display in QGC (position, attitude, battery, GPS)
- Map visualization with drone position and home point
- Basic command support (takeoff, land, RTL)
- Waypoint mission support

Usage:
    python mavlink_proxy.py --drone-ip 192.168.1.100 --qgc-port 14550

Requirements:
    pip install pymavlink

Author: Edouard Rolland
Project: WildDrone / WildBridge
License: MIT
"""

import argparse
import socket
import threading
import time
import math
import struct
from datetime import datetime

# MAVLink library
try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mavlink
except ImportError:
    print("ERROR: pymavlink not installed. Install with: pip install pymavlink")
    exit(1)

import sys
import os

# Add parent path to import djiInterface
sys.path.insert(0, os.path.dirname(__file__))
from djiInterface import DJIInterface


class MAVLinkProxy:
    """
    Proxy that bridges WildBridge telemetry to QGroundControl via MAVLink.
    """
    
    # MAVLink system and component IDs
    SYSTEM_ID = 1
    COMPONENT_ID = 1
    
    # MAVLink message rates (Hz)
    HEARTBEAT_RATE = 1.0
    POSITION_RATE = 10.0
    ATTITUDE_RATE = 20.0
    STATUS_RATE = 2.0
    
    # Flight mode mapping: DJI to MAVLink custom mode
    # Using PX4 mode values for compatibility
    DJI_TO_MAV_MODE = {
        'MANUAL': 0,
        'GPS': 3,          # POSCTL
        'ATTI': 2,         # ALTCTL  
        'GO_HOME': 5,      # RTL
        'AUTO_LANDING': 6, # LAND
        'AUTO_TAKEOFF': 2, # TAKEOFF
        'VIRTUAL_STICK': 6,# OFFBOARD
        'WAYPOINT': 4,     # MISSION
        'UNKNOWN': 0,
    }
    
    def __init__(self, drone_ip: str, qgc_host: str = "127.0.0.1", qgc_port: int = 14550, use_dji_native: bool = True):
        """
        Initialize the MAVLink proxy.
        
        Args:
            drone_ip: IP address of the WildBridge RC
            qgc_host: Host for QGroundControl UDP connection
            qgc_port: Port for QGroundControl UDP connection (default 14550)
            use_dji_native: Use DJI native waypoint missions (default True)
        """
        self.drone_ip = drone_ip
        self.qgc_host = qgc_host
        self.qgc_port = qgc_port
        
        # Initialize DJI interface
        if drone_ip:
            print(f"Connecting to WildBridge at {drone_ip}...")
        else:
            print("Connecting to WildBridge (auto-discovery)...")
            
        self.dji = DJIInterface(drone_ip)
        
        # Update drone_ip if discovered
        if not self.drone_ip and self.dji.IP_RC:
            self.drone_ip = self.dji.IP_RC
            print(f"Connected to WildBridge at {self.drone_ip}")
        
        # UDP socket for QGC communication
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.bind(('0.0.0.0', qgc_port + 1))  # Listen on next port for commands
        self.udp_socket.setblocking(False)
        
        # QGC address (will be updated when we receive first message from QGC)
        self.qgc_address = (qgc_host, qgc_port)
        
        # MAVLink connection for encoding messages
        self.mav = mavlink.MAVLink(None, srcSystem=self.SYSTEM_ID, srcComponent=self.COMPONENT_ID)
        
        # State tracking
        self.running = False
        self.armed = False
        self.mode = "UNKNOWN"
        self.boot_time = time.time()
        self.last_telemetry = {}
        
        # Mission state
        self.mission_items = []  # List of waypoints [(lat, lon, alt), ...]
        self.mission_count = 0
        self.expected_mission_count = 0
        self.current_mission_seq = 0
        self.mission_running = False
        self.use_dji_native = use_dji_native  # Use DJI native waypoint mission
        
        # Home position (set on first GPS fix)
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        
        # Threads
        self.telemetry_thread = None
        self.command_thread = None
        self.heartbeat_thread = None
        
        print(f"MAVLink proxy initialized - QGC endpoint: {qgc_host}:{qgc_port}")
    
    def start(self):
        """Start the MAVLink proxy."""
        print("Starting WildBridge telemetry stream...")
        self.dji.startTelemetryStream()
        
        self.running = True
        
        # Start threads
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.command_thread = threading.Thread(target=self._command_loop, daemon=True)
        
        self.heartbeat_thread.start()
        self.telemetry_thread.start()
        self.command_thread.start()
        
        print(f"\n{'='*60}")
        print("WildBridge MAVLink Proxy Running")
        print(f"{'='*60}")
        print(f"Drone IP:     {self.drone_ip}")
        print(f"QGC Address:  {self.qgc_host}:{self.qgc_port}")
        print(f"\nOpen QGroundControl and connect via UDP on port {self.qgc_port}")
        print("Press Ctrl+C to stop")
        print(f"{'='*60}\n")
    
    def stop(self):
        """Stop the MAVLink proxy."""
        print("\nStopping MAVLink proxy...")
        self.running = False
        self.dji.stopTelemetryStream()
        self.udp_socket.close()
    
    def _send_mavlink(self, msg):
        """Send a MAVLink message to QGC."""
        try:
            buf = msg.pack(self.mav)
            self.udp_socket.sendto(buf, self.qgc_address)
        except Exception as e:
            pass  # Ignore send errors
    
    def _get_boot_time_ms(self) -> int:
        """Get time since boot in milliseconds."""
        return int((time.time() - self.boot_time) * 1000)
    
    def _heartbeat_loop(self):
        """Send HEARTBEAT messages at regular intervals."""
        while self.running:
            try:
                telemetry = self.dji.getTelemetry()
                dji_mode = telemetry.get('flightMode', 'UNKNOWN') if telemetry else 'UNKNOWN'
                
                # Determine base mode flags
                base_mode = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                if self.armed:
                    base_mode |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                
                # Check if GPS is active
                sat_count = telemetry.get('satelliteCount', 0) if telemetry else 0
                if sat_count > 5:
                    base_mode |= mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
                    base_mode |= mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
                
                # Custom mode (PX4-style)
                custom_mode = self.DJI_TO_MAV_MODE.get(dji_mode, 0)
                
                # Create HEARTBEAT message
                msg = self.mav.heartbeat_encode(
                    type=mavlink.MAV_TYPE_QUADROTOR,
                    autopilot=mavlink.MAV_AUTOPILOT_PX4,
                    base_mode=base_mode,
                    custom_mode=custom_mode,
                    system_status=mavlink.MAV_STATE_ACTIVE if self.armed else mavlink.MAV_STATE_STANDBY
                )
                self._send_mavlink(msg)
                
            except Exception as e:
                print(f"Heartbeat error: {e}")
            
            time.sleep(1.0 / self.HEARTBEAT_RATE)
    
    def _telemetry_loop(self):
        """Convert WildBridge telemetry to MAVLink messages."""
        last_position_time = 0
        last_attitude_time = 0
        last_status_time = 0
        
        while self.running:
            try:
                telemetry = self.dji.getTelemetry()
                if not telemetry:
                    time.sleep(0.05)
                    continue
                
                self.last_telemetry = telemetry
                current_time = time.time()
                
                # Update armed state from flight mode
                dji_mode = telemetry.get('flightMode', 'UNKNOWN')
                self.armed = dji_mode not in ['UNKNOWN', 'MANUAL', '']
                self.mode = dji_mode
                self.manual_override = telemetry.get('isManualOverrideActive', False)
                
                # Set home position on first GPS fix
                location = telemetry.get('location', {})
                home_loc = telemetry.get('homeLocation', {})
                if home_loc and self.home_lat is None:
                    self.home_lat = home_loc.get('latitude', 0)
                    self.home_lon = home_loc.get('longitude', 0)
                    self.home_alt = location.get('altitude', 0)
                    print(f"Home position set: {self.home_lat:.6f}, {self.home_lon:.6f}")
                
                # Send position at POSITION_RATE
                if current_time - last_position_time >= 1.0 / self.POSITION_RATE:
                    self._send_position(telemetry)
                    self._send_gps_raw(telemetry)
                    last_position_time = current_time
                
                # Send attitude at ATTITUDE_RATE
                if current_time - last_attitude_time >= 1.0 / self.ATTITUDE_RATE:
                    self._send_attitude(telemetry)
                    last_attitude_time = current_time
                
                # Send status at STATUS_RATE
                if current_time - last_status_time >= 1.0 / self.STATUS_RATE:
                    self._send_sys_status(telemetry)
                    self._send_vfr_hud(telemetry)
                    self._send_battery_status(telemetry)
                    if self.home_lat is not None:
                        self._send_home_position()
                    last_status_time = current_time
                
            except Exception as e:
                print(f"Telemetry error: {e}")
            
            time.sleep(0.01)
    
    def _send_position(self, telemetry: dict):
        """Send GLOBAL_POSITION_INT message."""
        location = telemetry.get('location', {})
        speed = telemetry.get('speed', {})
        heading = telemetry.get('heading', 0)
        
        lat = int(location.get('latitude', 0) * 1e7)
        lon = int(location.get('longitude', 0) * 1e7)
        alt = int(location.get('altitude', 0) * 1000)  # mm
        relative_alt = alt  # mm above home
        
        vx = int(speed.get('x', 0) * 100)  # cm/s
        vy = int(speed.get('y', 0) * 100)
        vz = int(-speed.get('z', 0) * 100)  # NED convention (down is positive)
        
        hdg = int(heading * 100) % 36000  # cdeg
        
        msg = self.mav.global_position_int_encode(
            time_boot_ms=self._get_boot_time_ms(),
            lat=lat,
            lon=lon,
            alt=alt,
            relative_alt=relative_alt,
            vx=vx,
            vy=vy,
            vz=vz,
            hdg=hdg
        )
        self._send_mavlink(msg)
    
    def _send_gps_raw(self, telemetry: dict):
        """Send GPS_RAW_INT message."""
        location = telemetry.get('location', {})
        sat_count = telemetry.get('satelliteCount', 0)
        
        lat = int(location.get('latitude', 0) * 1e7)
        lon = int(location.get('longitude', 0) * 1e7)
        alt = int(location.get('altitude', 0) * 1000)  # mm
        
        # GPS fix type
        fix_type = 3 if sat_count >= 6 else (2 if sat_count >= 4 else 0)
        
        msg = self.mav.gps_raw_int_encode(
            time_usec=int(time.time() * 1e6),
            fix_type=fix_type,
            lat=lat,
            lon=lon,
            alt=alt,
            eph=100,  # HDOP * 100
            epv=100,  # VDOP * 100
            vel=0,    # Ground speed
            cog=0,    # Course over ground
            satellites_visible=int(sat_count)
        )
        self._send_mavlink(msg)
    
    def _send_attitude(self, telemetry: dict):
        """Send ATTITUDE message."""
        attitude = telemetry.get('attitude', {})
        
        roll = math.radians(attitude.get('roll', 0))
        pitch = math.radians(attitude.get('pitch', 0))
        yaw = math.radians(attitude.get('yaw', 0))
        
        msg = self.mav.attitude_encode(
            time_boot_ms=self._get_boot_time_ms(),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            rollspeed=0,
            pitchspeed=0,
            yawspeed=0
        )
        self._send_mavlink(msg)
    
    def _send_sys_status(self, telemetry: dict):
        """Send SYS_STATUS message."""
        battery_level = telemetry.get('batteryLevel', 0)
        
        # Sensors present and healthy
        sensors = (
            mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO |
            mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
            mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG |
            mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
            mavlink.MAV_SYS_STATUS_SENSOR_GPS
        )
        
        msg = self.mav.sys_status_encode(
            onboard_control_sensors_present=sensors,
            onboard_control_sensors_enabled=sensors,
            onboard_control_sensors_health=sensors,
            load=0,
            voltage_battery=0,  # Not available from DJI
            current_battery=-1,
            battery_remaining=int(battery_level),
            drop_rate_comm=0,
            errors_comm=0,
            errors_count1=0,
            errors_count2=0,
            errors_count3=0,
            errors_count4=0
        )
        self._send_mavlink(msg)
    
    def _send_vfr_hud(self, telemetry: dict):
        """Send VFR_HUD message (heads-up display data)."""
        location = telemetry.get('location', {})
        speed = telemetry.get('speed', {})
        heading = telemetry.get('heading', 0)
        
        # Calculate ground speed
        vx = speed.get('x', 0)
        vy = speed.get('y', 0)
        groundspeed = math.sqrt(vx**2 + vy**2)
        
        msg = self.mav.vfr_hud_encode(
            airspeed=groundspeed,
            groundspeed=groundspeed,
            heading=int(heading) % 360,
            throttle=50,  # Unknown
            alt=location.get('altitude', 0),
            climb=-speed.get('z', 0)  # Positive = climbing
        )
        self._send_mavlink(msg)
    
    def _send_battery_status(self, telemetry: dict):
        """Send BATTERY_STATUS message."""
        battery_level = telemetry.get('batteryLevel', 0)
        remaining_time = telemetry.get('remainingFlightTime', 0)
        
        msg = self.mav.battery_status_encode(
            id=0,
            battery_function=mavlink.MAV_BATTERY_FUNCTION_ALL,
            type=mavlink.MAV_BATTERY_TYPE_LIPO,
            temperature=32767,  # Unknown
            voltages=[0] * 10,  # Unknown
            current_battery=-1,
            current_consumed=-1,
            energy_consumed=-1,
            battery_remaining=int(battery_level),
            time_remaining=int(remaining_time),
            charge_state=mavlink.MAV_BATTERY_CHARGE_STATE_OK
        )
        self._send_mavlink(msg)
    
    def _send_home_position(self):
        """Send HOME_POSITION message."""
        if self.home_lat is None:
            return
        
        msg = self.mav.home_position_encode(
            latitude=int(self.home_lat * 1e7),
            longitude=int(self.home_lon * 1e7),
            altitude=int((self.home_alt or 0) * 1000),
            x=0, y=0, z=0,
            q=[1, 0, 0, 0],  # Identity quaternion
            approach_x=0, approach_y=0, approach_z=0
        )
        self._send_mavlink(msg)
    
    def _command_loop(self):
        """Receive and process MAVLink commands from QGC."""
        mav_conn = mavutil.mavlink_connection(
            f'udpin:0.0.0.0:{self.qgc_port + 1}',
            source_system=self.SYSTEM_ID,
            source_component=self.COMPONENT_ID
        )
        
        while self.running:
            try:
                msg = mav_conn.recv_match(blocking=True, timeout=0.1)
                if msg is None:
                    continue
                
                msg_type = msg.get_type()
                
                # Update QGC address from incoming messages
                if hasattr(mav_conn, 'last_address'):
                    self.qgc_address = mav_conn.last_address
                
                # Handle different message types
                if msg_type == 'COMMAND_LONG':
                    self._handle_command_long(msg)
                elif msg_type == 'SET_MODE':
                    self._handle_set_mode(msg)
                elif msg_type == 'MISSION_ITEM':
                    self._handle_mission_item(msg)
                elif msg_type == 'MISSION_ITEM_INT':
                    self._handle_mission_item_int(msg)
                elif msg_type == 'MISSION_COUNT':
                    self._handle_mission_count(msg)
                elif msg_type == 'MISSION_REQUEST_LIST':
                    self._handle_mission_request_list(msg)
                elif msg_type == 'MISSION_REQUEST':
                    self._handle_mission_request(msg)
                elif msg_type == 'MISSION_REQUEST_INT':
                    self._handle_mission_request(msg)
                elif msg_type == 'MISSION_ACK':
                    self._handle_mission_ack(msg)
                elif msg_type == 'MISSION_CLEAR_ALL':
                    self._handle_mission_clear(msg)
                elif msg_type == 'PARAM_REQUEST_LIST':
                    self._handle_param_request(msg)
                    
            except Exception as e:
                if self.running:
                    pass  # Ignore timeout errors
    
    def _handle_command_long(self, msg):
        """Handle COMMAND_LONG messages from QGC."""
        command = msg.command
        
        print(f"Received command: {command}")
        
        # MAV_CMD_NAV_TAKEOFF
        if command == mavlink.MAV_CMD_NAV_TAKEOFF:
            print("Executing TAKEOFF command")
            self.dji.requestSendTakeOff()
            self._send_command_ack(command, mavlink.MAV_RESULT_ACCEPTED)
        
        # MAV_CMD_NAV_LAND
        elif command == mavlink.MAV_CMD_NAV_LAND:
            print("Executing LAND command")
            self.dji.requestSendLand()
            self._send_command_ack(command, mavlink.MAV_RESULT_ACCEPTED)
        
        # MAV_CMD_NAV_RETURN_TO_LAUNCH
        elif command == mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            print("Executing RTL command")
            self.dji.requestSendRTH()
            self._send_command_ack(command, mavlink.MAV_RESULT_ACCEPTED)
        
        # MAV_CMD_COMPONENT_ARM_DISARM
        elif command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            arm = msg.param1 == 1
            if arm:
                print("ARM requested - DJI drones arm on takeoff")
                self._send_command_ack(command, mavlink.MAV_RESULT_ACCEPTED)
            else:
                print("DISARM requested - Landing")
                self.dji.requestSendLand()
                self._send_command_ack(command, mavlink.MAV_RESULT_ACCEPTED)
        
        # MAV_CMD_DO_SET_MODE
        elif command == mavlink.MAV_CMD_DO_SET_MODE:
            mode = int(msg.param2)
            print(f"Set mode to: {mode}")
            # Enable virtual stick for offboard control
            if mode == 6:  # OFFBOARD
                self.dji.requestSendEnableVirtualStick()
            self._send_command_ack(command, mavlink.MAV_RESULT_ACCEPTED)
        
        # MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
        elif command == mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            self._send_autopilot_version()
            self._send_command_ack(command, mavlink.MAV_RESULT_ACCEPTED)
        
        # MAV_CMD_MISSION_START
        elif command == mavlink.MAV_CMD_MISSION_START:
            print("Starting mission...")
            self._start_mission()
            self._send_command_ack(command, mavlink.MAV_RESULT_ACCEPTED)
        
        # MAV_CMD_DO_PAUSE_CONTINUE (pause/resume mission)
        elif command == 193:  # MAV_CMD_DO_PAUSE_CONTINUE
            if msg.param1 == 0:
                print("Pausing mission...")
                self._pause_mission()
            else:
                print("Resuming mission...")
                self._resume_mission()
            self._send_command_ack(command, mavlink.MAV_RESULT_ACCEPTED)
        
        # MAV_CMD_NAV_WAYPOINT (go to specific waypoint)
        elif command == mavlink.MAV_CMD_NAV_WAYPOINT:
            lat = msg.param5
            lon = msg.param6
            alt = msg.param7
            print(f"Direct waypoint command: {lat}, {lon}, {alt}")
            yaw = self.last_telemetry.get('heading', 0)
            self.dji.requestSendGoToWPwithPID(lat, lon, alt, yaw)
            self._send_command_ack(command, mavlink.MAV_RESULT_ACCEPTED)
        
        else:
            print(f"Unknown command: {command}")
            self._send_command_ack(command, mavlink.MAV_RESULT_UNSUPPORTED)
    
    def _handle_set_mode(self, msg):
        """Handle SET_MODE messages."""
        mode = msg.custom_mode
        print(f"Set mode request: {mode}")
        
        # Mode 4 = AUTO/Mission
        if mode == 4:
            print("Auto mode - starting mission")
            self._start_mission()
        # Mode 5 = RTL
        elif mode == 5:
            self.dji.requestSendRTH()
        # Mode 6 = OFFBOARD
        elif mode == 6:
            self.dji.requestSendEnableVirtualStick()
    
    def _handle_mission_count(self, msg):
        """Handle MISSION_COUNT messages - QGC is uploading a mission."""
        count = msg.count
        print(f"Mission upload started - expecting {count} items")
        
        self.expected_mission_count = count
        self.mission_items = []
        self.mission_count = 0
        
        if count > 0:
            # Request first mission item
            self._request_mission_item(0)
        else:
            # Empty mission
            self._send_mission_ack(mavlink.MAV_MISSION_ACCEPTED)
    
    def _handle_mission_item(self, msg):
        """Handle MISSION_ITEM messages (waypoints from QGC)."""
        seq = msg.seq
        command = msg.command
        lat = msg.x
        lon = msg.y
        alt = msg.z
        
        print(f"Received mission item {seq}: cmd={command}, lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}")
        
        # Store waypoint (only NAV_WAYPOINT commands)
        if command == mavlink.MAV_CMD_NAV_WAYPOINT:
            self.mission_items.append((lat, lon, alt))
            print(f"  -> Added waypoint {len(self.mission_items)}: ({lat:.6f}, {lon:.6f}, {alt:.1f})")
        elif command == mavlink.MAV_CMD_NAV_TAKEOFF:
            print(f"  -> Takeoff command at alt={alt}")
            # Store as first waypoint with current position
            if self.last_telemetry:
                loc = self.last_telemetry.get('location', {})
                self.mission_items.append((loc.get('latitude', lat), loc.get('longitude', lon), alt))
        elif command == mavlink.MAV_CMD_NAV_LAND:
            print(f"  -> Land command")
            # Land will be handled at end of mission
        elif command == mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            print(f"  -> RTL command")
        else:
            print(f"  -> Skipping command type {command}")
        
        self.mission_count = seq + 1
        
        # Request next item or finalize
        if seq + 1 < self.expected_mission_count:
            self._request_mission_item(seq + 1)
        else:
            print(f"\nMission upload complete: {len(self.mission_items)} waypoints")
            for i, (lat, lon, alt) in enumerate(self.mission_items):
                print(f"  WP{i+1}: ({lat:.6f}, {lon:.6f}, {alt:.1f})")
            self._send_mission_ack(mavlink.MAV_MISSION_ACCEPTED)
    
    def _handle_mission_item_int(self, msg):
        """Handle MISSION_ITEM_INT messages (integer lat/lon format)."""
        seq = msg.seq
        command = msg.command
        lat = msg.x / 1e7  # Convert from int to degrees
        lon = msg.y / 1e7
        alt = msg.z
        
        print(f"Received mission item INT {seq}: cmd={command}, lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}")
        
        # Store waypoint
        if command == mavlink.MAV_CMD_NAV_WAYPOINT:
            self.mission_items.append((lat, lon, alt))
            print(f"  -> Added waypoint {len(self.mission_items)}")
        elif command == mavlink.MAV_CMD_NAV_TAKEOFF:
            if self.last_telemetry:
                loc = self.last_telemetry.get('location', {})
                self.mission_items.append((loc.get('latitude', lat), loc.get('longitude', lon), alt))
        
        self.mission_count = seq + 1
        
        # Request next item or finalize
        if seq + 1 < self.expected_mission_count:
            self._request_mission_item_int(seq + 1)
        else:
            print(f"\nMission upload complete: {len(self.mission_items)} waypoints")
            self._send_mission_ack(mavlink.MAV_MISSION_ACCEPTED)
    
    def _handle_mission_request_list(self, msg):
        """Handle MISSION_REQUEST_LIST - QGC wants to download current mission."""
        print(f"Mission request list - sending {len(self.mission_items)} items")
        
        msg = self.mav.mission_count_encode(
            target_system=msg.target_system if hasattr(msg, 'target_system') else 255,
            target_component=msg.target_component if hasattr(msg, 'target_component') else 0,
            count=len(self.mission_items)
        )
        self._send_mavlink(msg)
    
    def _handle_mission_request(self, msg):
        """Handle MISSION_REQUEST - QGC wants a specific mission item."""
        seq = msg.seq
        print(f"Mission request for item {seq}")
        
        if seq < len(self.mission_items):
            lat, lon, alt = self.mission_items[seq]
            
            msg = self.mav.mission_item_encode(
                target_system=255,
                target_component=0,
                seq=seq,
                frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=mavlink.MAV_CMD_NAV_WAYPOINT,
                current=1 if seq == self.current_mission_seq else 0,
                autocontinue=1,
                param1=0, param2=0, param3=0, param4=0,
                x=lat, y=lon, z=alt
            )
            self._send_mavlink(msg)
    
    def _handle_mission_ack(self, msg):
        """Handle MISSION_ACK from QGC."""
        result = msg.type
        print(f"Mission ACK received: {result}")
    
    def _handle_mission_clear(self, msg):
        """Handle MISSION_CLEAR_ALL - clear current mission."""
        print("Clearing mission")
        self.mission_items = []
        self.mission_count = 0
        self.current_mission_seq = 0
        self.mission_running = False
        self._send_mission_ack(mavlink.MAV_MISSION_ACCEPTED)
    
    def _request_mission_item(self, seq: int):
        """Request a mission item from QGC."""
        msg = self.mav.mission_request_encode(
            target_system=255,
            target_component=0,
            seq=seq
        )
        self._send_mavlink(msg)
    
    def _request_mission_item_int(self, seq: int):
        """Request a mission item (INT format) from QGC."""
        msg = self.mav.mission_request_int_encode(
            target_system=255,
            target_component=0,
            seq=seq
        )
        self._send_mavlink(msg)
    
    def _send_mission_ack(self, result: int):
        """Send MISSION_ACK message."""
        msg = self.mav.mission_ack_encode(
            target_system=255,
            target_component=0,
            type=result
        )
        self._send_mavlink(msg)
    
    def _send_mission_current(self, seq: int):
        """Send MISSION_CURRENT message to indicate active waypoint."""
        msg = self.mav.mission_current_encode(seq=seq)
        self._send_mavlink(msg)
    
    def _send_mission_item_reached(self, seq: int):
        """Send MISSION_ITEM_REACHED message."""
        msg = self.mav.mission_item_reached_encode(seq=seq)
        self._send_mavlink(msg)
    
    def _start_mission(self):
        """Start executing the uploaded mission."""
        if not self.mission_items:
            print("No mission items to execute!")
            return
        
        print(f"\n{'='*60}")
        print(f"STARTING MISSION - {len(self.mission_items)} waypoints")
        print(f"{'='*60}")
        
        for i, (lat, lon, alt) in enumerate(self.mission_items):
            print(f"  WP{i+1}: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
        
        self.mission_running = True
        self.current_mission_seq = 0
        self._send_mission_current(0)
        
        if self.use_dji_native and len(self.mission_items) >= 2:
            # Use DJI native waypoint mission for better performance
            print("\nUsing DJI Native Waypoint Mission")
            self.dji.requestSendNavigateTrajectoryDJINative(self.mission_items)
        else:
            # Use Virtual Stick PID navigation for single waypoint or fallback
            print("\nUsing Virtual Stick PID Navigation")
            self.dji.requestSendEnableVirtualStick()
            
            if len(self.mission_items) > 1:
                # Navigate trajectory
                self.dji.requestSendNavigateTrajectory(self.mission_items, finalYaw=0)
            elif len(self.mission_items) == 1:
                # Single waypoint
                lat, lon, alt = self.mission_items[0]
                yaw = self.last_telemetry.get('heading', 0)
                self.dji.requestSendGoToWPwithPID(lat, lon, alt, yaw)
    
    def _pause_mission(self):
        """Pause the current mission."""
        print("Mission paused")
        self.mission_running = False
        # Stop by sending zero stick commands (hover)
        self.dji.requestSendStick(0, 0, 0, 0)
    
    def _resume_mission(self):
        """Resume the current mission."""
        if self.mission_items:
            print(f"Resuming mission from waypoint {self.current_mission_seq}")
            self.mission_running = True
            # Re-execute remaining waypoints
            remaining = self.mission_items[self.current_mission_seq:]
            if remaining:
                if self.use_dji_native and len(remaining) >= 2:
                    self.dji.requestSendNavigateTrajectoryDJINative(remaining)
                else:
                    lat, lon, alt = remaining[0]
                    yaw = self.last_telemetry.get('heading', 0)
                    self.dji.requestSendGoToWPwithPID(lat, lon, alt, yaw)


def main():
    parser = argparse.ArgumentParser(
        description='WildBridge MAVLink Proxy for QGroundControl',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python mavlink_proxy.py --drone-ip 192.168.1.100
  python mavlink_proxy.py --drone-ip 10.0.0.5 --qgc-port 14550

In QGroundControl:
  1. Go to Application Settings > Comm Links
  2. Add new UDP connection on port 14550
  3. Connect to see drone telemetry
        """
    )
    
    parser.add_argument('--drone-ip', '-d', required=False, default="",
                        help='IP address of WildBridge RC (optional, will auto-discover if not provided)')
    parser.add_argument('--qgc-host', default='127.0.0.1',
                        help='QGroundControl host (default: 127.0.0.1)')
    parser.add_argument('--qgc-port', '-p', type=int, default=14550,
                        help='QGroundControl UDP port (default: 14550)')
    parser.add_argument('--no-native-mission', action='store_true',
                        help='Disable DJI native waypoint missions, use Virtual Stick PID instead')
    
    args = parser.parse_args()
    
    proxy = MAVLinkProxy(
        drone_ip=args.drone_ip,
        qgc_host=args.qgc_host,
        qgc_port=args.qgc_port,
        use_dji_native=not args.no_native_mission
    )
    
    try:
        proxy.start()
        
        # Keep running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        pass
    finally:
        proxy.stop()


if __name__ == '__main__':
    main()
