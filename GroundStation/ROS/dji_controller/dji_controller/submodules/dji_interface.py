"""
WildBridge - DJI Interface Module

A Python interface for controlling DJI drones through HTTP requests and TCP sockets,
providing seamless integration for drone operations, telemetry retrieval, and video streaming.

Authors:  Edouard G.A. Rolland, Kilian Meier
Project: WildDrone
Institution: University of Bristol, University of Southern Denmark (SDU)
License: MIT

For more information, visit: https://github.com/WildDrone/WildBridge
"""

import cv2
import requests
import ast
import json
import os
import re
import socket
import threading
import time
from datetime import datetime

# Discovery Configuration
DISCOVERY_PORT = 30000
DISCOVERY_MSG = b"DISCOVER_WILDBRIDGE"
DISCOVERY_RESPONSE_PREFIX = "WILDBRIDGE_HERE:"

def get_local_ips():
    """Get all local IP addresses for subnet detection."""
    ip_list = []
    try:
        hostname = socket.gethostname()
        for ip in socket.gethostbyname_ex(hostname)[2]:
            if not ip.startswith("127."):
                ip_list.append(ip)
    except:
        pass
    
    # Fallback method
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        if ip not in ip_list and not ip.startswith("127."):
            ip_list.append(ip)
    except:
        pass
    
    return ip_list

def scan_subnet_for_drones(local_ips, timeout=0.1, verbose=True):
    """
    Scan subnet for WildBridge drones using direct UDP probing.
    Returns list of tuples [(drone_ip, drone_name), ...]
    """
    found_drones = {}
    if verbose:
        print("Scanning subnet for WildBridge drones...")
    
    for local_ip in local_ips:
        parts = local_ip.split('.')
        subnet = f"{parts[0]}.{parts[1]}.{parts[2]}"
        
        # Try common IP ranges
        ranges = list(range(1, 51)) + list(range(100, 121)) + list(range(150, 171)) + list(range(200, 221))
        
        for i in ranges:
            ip = f"{subnet}.{i}"
            if ip == local_ip:
                continue
            
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(timeout)
                sock.sendto(DISCOVERY_MSG, (ip, DISCOVERY_PORT))
                
                try:
                    data, addr = sock.recvfrom(1024)
                    message = data.decode('utf-8')
                    if message.startswith(DISCOVERY_RESPONSE_PREFIX):
                        parts = message.split(':')
                        drone_ip = parts[1] if len(parts) > 1 else addr[0]
                        drone_name = parts[2] if len(parts) > 2 else "UNKNOWN"
                        if verbose:
                            print(f"Found WildBridge drone at {drone_ip} (Name: {drone_name})")
                        found_drones[drone_ip] = drone_name
                except socket.timeout:
                    pass
                
                sock.close()
            except:
                pass
    
    return list(found_drones.items())

def discover_all_drones(timeout=5.0, verbose=True):
    """
    Discover all WildBridge drones on the network.
    Returns list of tuples [(drone_ip, drone_name), ...]
    """
    found_drones = {}
    
    # Try broadcast first
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(timeout)
    
    try:
        sock.sendto(DISCOVERY_MSG, ('<broadcast>', DISCOVERY_PORT))
        if verbose:
            print(f"Broadcasting discovery message on port {DISCOVERY_PORT}...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                data, addr = sock.recvfrom(1024)
                message = data.decode('utf-8')
                if message.startswith(DISCOVERY_RESPONSE_PREFIX):
                    parts = message.split(':')
                    drone_ip = parts[1] if len(parts) > 1 else None
                    drone_name = parts[2] if len(parts) > 2 else "UNKNOWN"
                    if drone_ip and drone_ip not in found_drones:
                        if verbose:
                            print(f"Found WildBridge drone at {drone_ip} (Name: {drone_name})")
                        found_drones[drone_ip] = drone_name
            except socket.timeout:
                continue
    except Exception as e:
        if verbose:
            print(f"Broadcast discovery failed: {e}")
    finally:
        sock.close()
    
    if not found_drones:
        if verbose:
            print("Broadcast found no drones, scanning subnet...")
        local_ips = get_local_ips()
        if local_ips:
            subnet_drones = scan_subnet_for_drones(local_ips, timeout=0.1, verbose=verbose)
            for ip, name in subnet_drones:
                found_drones[ip] = name
    
    return list(found_drones.items())

def discover_drone(timeout=5.0, verbose=True):
    """
    Discover a single WildBridge drone.
    Returns tuple (drone_ip, drone_name) or (None, None).
    """
    drones = discover_all_drones(timeout, verbose)
    if drones:
        return drones[0]
    return None, None

# Aircraft state endpoint suffixes
# GETTER
EP_BASE = "/"
EP_SPEED = "/aircraft/speed"
EP_HEADING = "/aircraft/heading"
EP_ATTITUDE = "/aircraft/attitude"
EP_LOCATION = "/aircraft/location"
EP_GIMBAL_ATTITUDE = "/aircraft/gimbalAttitude"
EP_ALL_STATES = "/aircraft/allStates"
EP_STICK_VALUES = "/aircraft/rcStickValues"
EP_YAW_REACHED = "/status/yawReached"
EP_ALTITUDE_REACHED = "/status/altitudeReached"
EP_WP_REACHED = "/status/waypointReached"
EP_HOME_LOCATION = "/home/location"
EP_CAMERA_IS_RECORDING = "/status/camera/isRecording"
EP_CAPTURE_THERMAL_IMAGE = "/send/captureThermalImage"
EP_CAPTURE_TEMPERATURE = "/send/captureTemperature"  # temperature-only read, no shutter

# SETTER
# HTTP POST Command Endpoints (port 8080)
EP_STICK = "/send/stick"  # expects a formatted string: "<leftX>,<leftY>,<rightX>,<rightY>"
EP_ZOOM = "/send/camera/zoom"
EP_GIMBAL_SET_PITCH = "/send/gimbal/pitch"
EP_GIMBAL_SET_YAW = "/send/gimbal/yaw"  # !!! This is the yaw joint angle !!!
EP_GIMBAL_SET_REL_PITCH = "/send/gimbal/rel_pitch"
EP_GIMBAL_SET_REL_YAW = "/send/gimbal/rel_yaw"
EP_TAKEOFF = "/send/takeoff"
EP_LAND = "/send/land"
EP_RTH = "/send/RTH"
EP_ENABLE_VIRTUAL_STICK = "/send/enableVirtualStick"
EP_ABORT_MISSION = "/send/abortMission"
EP_ABORT_ALL = "/send/abortAll"
EP_GOTO_YAW = "/send/gotoYaw"
EP_GOTO_WP_NOSE_FORWARD = "/send/gotoWaypointNoseForward"
EP_GOTO_ALTITUDE = "/send/gotoAltitude"
EP_CAMERA_START_RECORDING = "/send/camera/startRecording"
EP_CAMERA_STOP_RECORDING = "/send/camera/stopRecording"
EP_GOTO_TRAJECTORY_DJI_NATIVE = "/send/navigateTrajectoryDJINative"
EP_ABORT_DJI_NATIVE_MISSION = "/send/abort/DJIMission"
EP_SET_RTH_ALTITUDE = "/send/setRTHAltitude"
EP_DEACTIVATE_MANUAL_OVERRIDE = "/send/deactivateManualOverride"
EP_GET_MANUAL_OVERRIDE = "/get/isManualOverrideActive"
EP_LRF_MEASURE = "/send/lrf/measure"
EP_LIST_MEDIA = "/send/listMedia"
EP_DOWNLOAD_MEDIA_BY_NAME = "/send/downloadMediaByName"

# The bridge identifies the three co-aligned lenses of one H20T shutter by these
# exact names (no aliases). One shutter exposes all of them at once; the lens(es)
# you pass select which are downloaded.
LENS_KEYS = ("thermal", "wide", "zoom")


def canonical_lenses(lenses):
    """Normalize a lens selection into an ordered, de-duplicated list of lens names.

    Accepts a single name ("wide"), a list/tuple, or a comma/space-separated string
    ("thermal,zoom"). The only valid names are the members of LENS_KEYS — no aliases;
    matching is case-insensitive. Output order follows LENS_KEYS. Raises ValueError on
    an unknown lens name.
    """
    if isinstance(lenses, str):
        lenses = [part for part in lenses.replace(",", " ").split() if part]
    requested = set()
    for name in lenses:
        key = str(name).strip().lower()
        if key not in LENS_KEYS:
            raise ValueError(f"unknown lens {name!r}; valid: {', '.join(LENS_KEYS)}")
        requested.add(key)
    return [k for k in LENS_KEYS if k in requested]


# PID Tuning
EP_GOTO_WP_HOLD_HEADING = "/send/gotoWaypointHoldHeading"
EP_PAYLOAD_DROP = "/send/drop"

# Thermal Image handling
SAVE_SUCCESS = "T_IMG_SAVE_SUCCESS"
SAVE_FAILURE = "T_IMG_SAVE_FAILURE"
CAP_FAILURE = "T_IMG_CAP_FAILURE"


def get_config(ip_address):
    """
    Query drone configuration via HTTP GET /config endpoint.
    Returns dict with droneName, ipAddress, ports, or None if failed.
    """
    try:
        response = requests.get(f"http://{ip_address}:8080/config", timeout=2.0)
        if response.status_code == 200:
            return json.loads(response.text)
    except Exception as e:
        print(f"Failed to get config from {ip_address}: {e}")
    return None


class DJIInterface:
    """
    Interface for DJI drone control via HTTP commands (port 8080) and 
    TCP telemetry socket (port 8081).
    """
    
    def __init__(self, IP_RC=""):
        if not IP_RC:
            print("No IP provided, attempting to discover drone...")
            discovered_ip, _ = discover_drone()
            if discovered_ip:
                self.IP_RC = discovered_ip
            else:
                print("Drone discovery failed.")
                self.IP_RC = ""
        else:
            self.IP_RC = IP_RC

        self.baseCommandUrl = f"http://{self.IP_RC}:8080"
        self.telemetryPort = 8081
        self.videoSource = f"rtsp://aaa:aaa@{self.IP_RC}:8554/streaming/live/1"
        
        # Telemetry state (updated via TCP socket)
        self._telemetry = {}
        self._telemetry_lock = threading.Lock()
        self._telemetry_socket = None
        self._telemetry_thread = None
        self._running = False

    def getVideoSource(self):
        if self.IP_RC == "":
            return ""
        return self.videoSource

    # ==================== Telemetry (TCP Socket on port 8081) ====================
    
    def startTelemetryStream(self):
        """
        Start receiving telemetry data via TCP socket connection.
        The drone sends JSON telemetry data continuously.
        """
        if self._running:
            return
        
        self._running = True
        self._telemetry_thread = threading.Thread(target=self._telemetry_receiver, daemon=True)
        self._telemetry_thread.start()
    
    def stopTelemetryStream(self):
        """Stop the telemetry stream and close the socket."""
        self._running = False
        if self._telemetry_socket:
            try:
                self._telemetry_socket.close()
            except:
                pass
        if self._telemetry_thread:
            self._telemetry_thread.join(timeout=2)
    
    def _telemetry_receiver(self):
        """Background thread that receives telemetry data from TCP socket."""
        buffer = ""
        while self._running:
            try:
                # Connect to telemetry server
                self._telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._telemetry_socket.settimeout(5.0)
                self._telemetry_socket.connect((self.IP_RC, self.telemetryPort))
                
                while self._running:
                    data = self._telemetry_socket.recv(4096)
                    if not data:
                        break
                    
                    buffer += data.decode('utf-8')
                    
                    # Process complete JSON objects (separated by newlines)
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            try:
                                telemetry = json.loads(line)
                                with self._telemetry_lock:
                                    self._telemetry = telemetry
                                    self._telemetry["timestamp"] = datetime.now().strftime(
                                        "%Y-%m-%d_%H-%M-%S.%f")
                            except json.JSONDecodeError:
                                pass
                                
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Telemetry connection error: {e}")
                import time
                time.sleep(1)  # Wait before reconnecting
            finally:
                if self._telemetry_socket:
                    try:
                        self._telemetry_socket.close()
                    except:
                        pass
    
    def getTelemetry(self):
        """
        Get the latest telemetry data.
        Returns a dictionary with all telemetry fields from the drone.
        """
        with self._telemetry_lock:
            return self._telemetry.copy()
    
    def requestAllStates(self, verbose=False):
        """
        Get all aircraft states from telemetry.
        Note: You must call startTelemetryStream() first.
        """
        telemetry = self.getTelemetry()
        if verbose and telemetry:
            print("Telemetry:", json.dumps(telemetry, indent=2))
        return telemetry
    
    # Telemetry field accessors
    def getSpeed(self):
        """Get aircraft velocity (x, y, z)."""
        return self.getTelemetry().get("speed", {})
    
    def getHeading(self):
        """Get compass heading in degrees."""
        return self.getTelemetry().get("heading", 0.0)
    
    def getAttitude(self):
        """Get aircraft attitude (pitch, roll, yaw)."""
        return self.getTelemetry().get("attitude", {})
    
    def getLocation(self):
        """Get aircraft 3D location (latitude, longitude, altitude)."""
        return self.getTelemetry().get("location", {})
    
    def getGimbalAttitude(self):
        """Get gimbal attitude (pitch, roll, yaw)."""
        return self.getTelemetry().get("gimbalAttitude", {})
    
    def getGimbalJointAttitude(self):
        """Get gimbal joint attitude (pitch, roll, yaw)."""
        return self.getTelemetry().get("gimbalJointAttitude", {})
    
    def getZoomFocalLength(self):
        """Get camera zoom focal length."""
        return self.getTelemetry().get("zoomFl", -1)
    
    def getHybridFocalLength(self):
        """Get camera hybrid focal length."""
        return self.getTelemetry().get("hybridFl", -1)
    
    def getOpticalFocalLength(self):
        """Get camera optical focal length."""
        return self.getTelemetry().get("opticalFl", -1)
    
    def getZoomRatio(self):
        """Get camera zoom ratio."""
        return self.getTelemetry().get("zoomRatio", 1.0)
    
    def getBatteryLevel(self):
        """Get battery level percentage."""
        return self.getTelemetry().get("batteryLevel", -1)
    
    def getSatelliteCount(self):
        """Get GPS satellite count."""
        return self.getTelemetry().get("satelliteCount", -1)

    def isReadyToTakeoff(self):
        """Whether the drone is ready to take off / arm (derived on the aircraft side)."""
        return self.getTelemetry().get("readyToTakeoff", False)

    def getTakeoffBlockReason(self):
        """Reason the drone cannot take off: FCMotorStartFailureError name, 'NONE', or 'UNKNOWN'."""
        return self.getTelemetry().get("takeoffBlockReason", "UNKNOWN")

    def getHomeLocation(self):
        """Get home location (latitude, longitude)."""
        return self.getTelemetry().get("homeLocation", {})
    
    def getDistanceToHome(self):
        """Get distance to home in meters."""
        return self.getTelemetry().get("distanceToHome", 0.0)
    
    def getLRFTarget(self):
        """Get the last LRF-locked target position (latitude, longitude, altitude)."""
        return self.getTelemetry().get("lrfTarget")

    def getWaypointSeq(self):
        """Id of the waypoint the streamed 'waypointReached' currently refers to.

        Mirrors DroneController._waypointSeq, incremented by the app for every
        requestSendGoToWaypointNoseForward. Returns -1 if telemetry hasn't reported it yet.
        """
        return self.getTelemetry().get("waypointSeq", -1)

    def isWaypointReached(self, seq=None):
        """Check if a commanded waypoint has been reached."""
        telemetry = self.getTelemetry()
        reached = telemetry.get("waypointReached", False)
        if seq is None:
            return reached
        return reached and telemetry.get("waypointSeq", -1) == seq

    def isIntermediaryWaypointReached(self):
        """Check if an intermediary waypoint has been reached."""
        return self.getTelemetry().get("intermediaryWaypointReached", False)

    def getYawSeq(self):
        """Id of the gotoYaw command the streamed 'yawReached' refers to (-1 if unknown)."""
        return self.getTelemetry().get("yawSeq", -1)

    def isYawReached(self, seq=None):
        """Check if a commanded yaw has been reached.

        'yawReached' latches until the next command, so pass the seq returned by
        requestSendGotoYaw to require the streamed status belongs to that command.
        seq=None keeps the legacy (race-prone) raw-flag read.
        """
        telemetry = self.getTelemetry()
        reached = telemetry.get("yawReached", False)
        if seq is None:
            return reached
        return reached and telemetry.get("yawSeq", -1) == seq

    def getAltitudeSeq(self):
        """Id of the gotoAltitude command the streamed 'altitudeReached' refers to (-1 if unknown)."""
        return self.getTelemetry().get("altitudeSeq", -1)

    def isAltitudeReached(self, seq=None):
        """Check if a commanded altitude has been reached.

        Pass the seq returned by requestSendGotoAltitude to avoid the stale-latch race.
        seq=None keeps the legacy (race-prone) raw-flag read.
        """
        telemetry = self.getTelemetry()
        reached = telemetry.get("altitudeReached", False)
        if seq is None:
            return reached
        return reached and telemetry.get("altitudeSeq", -1) == seq

    def isCameraRecording(self):
        """Check if the camera is currently recording."""
        return self.getTelemetry().get("isRecording", False)
    
    def isHomeSet(self):
        """Check if the home location has been set."""
        return self.getTelemetry().get("homeSet", False)
    
    def getRemainingFlightTime(self):
        """Get remaining flight time in minutes."""
        return self.getTelemetry().get("remainingFlightTime", 0)
    
    def getTimeNeededToGoHome(self):
        """Get time needed to return home in seconds."""
        return self.getTelemetry().get("timeNeededToGoHome", 0)
    
    def getTimeNeededToLand(self):
        """Get time needed to land in seconds."""
        return self.getTelemetry().get("timeNeededToLand", 0)
    
    def getTotalTime(self):
        """Get total time needed (go home + land) in seconds."""
        return self.getTelemetry().get("totalTime", 0)
    
    def getMaxRadiusCanFlyAndGoHome(self):
        """Get maximum radius the drone can fly and still return home."""
        return self.getTelemetry().get("maxRadiusCanFlyAndGoHome", 0)
    
    def getRemainingCharge(self):
        """Get remaining battery charge percentage."""
        return self.getTelemetry().get("remainingCharge", 0)
    
    def getBatteryNeededToLand(self):
        """Get battery percentage needed to land."""
        return self.getTelemetry().get("batteryNeededToLand", 0)
    
    def getBatteryNeededToGoHome(self):
        """Get battery percentage needed to return home."""
        return self.getTelemetry().get("batteryNeededToGoHome", 0)
    
    def getSeriousLowBatteryThreshold(self):
        """Get serious low battery warning threshold percentage."""
        return self.getTelemetry().get("seriousLowBatteryThreshold", 0)
    
    def getLowBatteryThreshold(self):
        """Get low battery warning threshold percentage."""
        return self.getTelemetry().get("lowBatteryThreshold", 0)
    
    def getFlightMode(self):
        """Get the current flight mode (e.g., 'MANUAL', 'GPS', 'GO_HOME', etc.)."""
        return self.getTelemetry().get("flightMode", "UNKNOWN")

    def isManualOverrideActive(self):
        """Check if manual override is active (pilot took RC control).
        
        When True, autonomous HTTP commands are being rejected by the app.
        The pilot must deactivate manual override before autonomous commands work again.
        """
        return self.getTelemetry().get("isManualOverrideActive", False)

    # ==================== Commands (HTTP POST on port 8080) ====================

    def requestSend(self, endPoint, data, verbose=False):
        """Send a POST request to the drone."""
        if self.IP_RC == "":
            print(f"No IP_RC provided, returning empty string for request at {endPoint}")
            return ""
        try:
            response = requests.post(self.baseCommandUrl + endPoint, str(data), timeout=5)
            if verbose:
                print("EP : " + endPoint + "\t" + str(response.content, encoding="utf-8"))
            return response.content.decode('utf-8')
        except requests.exceptions.RequestException as e:
            print(f"Request error at {endPoint}: {e}")
            return ""

    def requestSendStick(self, leftX=0, leftY=0, rightX=0, rightY=0):
        """Send virtual stick commands. Values should be in [-1, 1]."""
        # Saturate values such that they are in [-1;1]
        s = 0.3
        leftX = max(-s, min(s, leftX))
        leftY = max(-s, min(s, leftY))
        rightX = max(-s, min(s, rightX))
        rightY = max(-s, min(s, rightY))
        rep = self.requestSend(
            EP_STICK, f"{leftX:.4f},{leftY:.4f},{rightX:.4f},{rightY:.4f}")
        return rep

    def requestSendGimbalPitch(self, pitch=0):
        """Set gimbal pitch angle."""
        return self.requestSend(EP_GIMBAL_SET_PITCH, f"0,{pitch},0")

    def requestSendGimbalYaw(self, yaw=0):
        """Set gimbal yaw angle."""
        return self.requestSend(EP_GIMBAL_SET_YAW, f"0,0,{yaw}")

    def requestSendGimbalRelPitch(self, rel_pitch=0):
        """Adjust gimbal pitch by a relative angle."""
        return self.requestSend(EP_GIMBAL_SET_REL_PITCH, f"0,{rel_pitch},0")

    def requestSendGimbalRelYaw(self, rel_yaw=0):
        """Adjust gimbal yaw by a relative angle."""
        return self.requestSend(EP_GIMBAL_SET_REL_YAW, f"0,0,{rel_yaw}")

    def requestSendZoomRatio(self, zoomRatio=1):
        """Set camera zoom ratio."""
        return self.requestSend(EP_ZOOM, zoomRatio)

    def requestSendTakeOff(self):
        """Command the drone to take off."""
        return self.requestSend(EP_TAKEOFF, "")

    def requestSendLand(self):
        """Command the drone to land."""
        return self.requestSend(EP_LAND, "")

    def requestSendRTH(self):
        """Command the drone to return to home.
        
        Note: This first aborts any active mission and disables virtual stick
        to prevent conflicts with RTH. Virtual stick mode can interfere with
        RTH causing erratic behavior.
        """
        # CRITICAL: Disable virtual stick before RTH to prevent conflicts
        self.requestAbortMission()
        return self.requestSend(EP_RTH, "")

    def requestSendGoToWaypointNoseForward(self, latitude, longitude, altitude, yaw, speed: float = 20.0):
        """Navigate to a waypoint with PID control (nose-follows-path, final-heading).

        CONTRACT: during travel the drone faces its direction of motion — the bridge forces the
        travel heading to bearing(current->waypoint). The `yaw` argument is the FINAL arrival
        heading: once the drone reaches the waypoint it rotates in place to `yaw` (Phase 3), and
        only then is the waypoint reported reached. If you instead need the nose pointed at `yaw`
        *while* translating, use requestSendGoToWaypointHoldHeading, which projects the to-waypoint
        vector into the body frame.

        Args:
            latitude: Target latitude
            longitude: Target longitude
            altitude: Target altitude
            yaw: Final arrival heading (deg). Drone rotates to this in place after reaching the WP;
                 it does NOT set the travel heading (that is auto = bearing to waypoint).
            speed: Max speed in m/s (default 20.0)

        Returns:
            int: the sequence id the app assigned to this request (parsed from the
                 "WAYPOINT_ACCEPTED seq=<n> ..." response). Pass it to
                 isWaypointReached(seq) to avoid the stale-latch race.
            None: if the command was rejected or the response had no seq.
        """
        response = self.requestSend(EP_GOTO_WP_NOSE_FORWARD, f"{latitude},{longitude},{altitude},{yaw},{speed}")
        return self._parseSeq(response)

    @staticmethod
    def _parseSeq(response):
        """Extract the integer seq from an '<X>_ACCEPTED seq=<n> ...' response, else None."""
        match = re.search(r"seq=(\d+)", str(response))
        return int(match.group(1)) if match else None

    def requestSendGoToWaypointHoldHeading(self, latitude, longitude, altitude, yaw, speed: float = 5.0):
        """Navigate to a waypoint holding a fixed heading for the whole flight.

        CONTRACT: the nose stays on `yaw` from start to arrival — the drone crabs sideways or
        diagonally instead of turning to face where it is going. Use this when the payload must
        keep looking at one bearing while repositioning. Tighter arrival tolerance than
        requestSendGoToWaypointNoseForward, which turns the nose along the leg instead.

        Args:
            latitude, longitude, altitude: Target position
            yaw: Heading (deg) held for the entire flight, not just on arrival
            speed: Max speed in m/s (default 5.0)

        Returns:
            int: the seq id parsed from "WAYPOINT_ACCEPTED seq=<n> ...", or None if rejected.
        """
        response = self.requestSend(EP_GOTO_WP_HOLD_HEADING, f"{latitude},{longitude},{altitude},{yaw},{speed}")
        return self._parseSeq(response)

    def requestCapture(self):
        """Trigger ONE H20T shutter (no image download). Returns the capture descriptor.
        Returns:
            dict {"thermal": fn|None, "wide": fn|None, "zoom": fn|None} on success (fn is the
            on-camera filename, None if that lens was not stored), else False.

        Download any returned filename with downloadByName().
        For the thermal max temperature (no shutter), use requestCaptureTemperature().
        """

        if self.IP_RC == "":
            print("No IP_RC provided, cannot capture image")
            return False
        # Trip the shutter. The bridge returns a JSON descriptor naming the on-camera filename
        # of each lens the H20T stored (no image yet).
        try:
            # Generous timeout: the very first capture after connect can be cold (the bridge builds
            # the full SD-card list once), so allow well past the server's internal resolution cap.
            response = requests.post(
                self.baseCommandUrl + EP_CAPTURE_THERMAL_IMAGE, data="", timeout=60)
        except requests.exceptions.RequestException as e:
            print(f"Error capturing image: {e}")
            return False
        try:
            info = response.json()
        except ValueError:
            print(f"Capture returned non-JSON: HTTP {response.status_code}, "
                  f"body={response.text[:200]!r}")
            return False
        if info.get("error") or not info.get("thermal"):
            print(f"Capture failed: {info}")
            return False
        return info

    def requestCaptureTemperature(self):
        """Read the highest temperature (deg C) on the thermal feed. No shutter, no download.

        Returns the bridge's raw JSON response body, e.g. '{"thermalMaxTemp":21.5}'
        (thermalMaxTemp is null if no radiometric value was available).
        """
        return self.requestSend(EP_CAPTURE_TEMPERATURE, "")

    def listMedia(self):
        """List every file on the camera's SD card (robust path — source of truth, not the
        bounded recent-capture cache).

        Returns:
            list of dicts {"name": str, "index": int, "size": int, "type": str} on success,
            else False.
        """
        if self.IP_RC == "":
            print("No IP_RC provided, cannot list media")
            return False
        try:
            response = requests.post(
                self.baseCommandUrl + EP_LIST_MEDIA, data="", timeout=30)
        except requests.exceptions.RequestException as e:
            print(f"Error listing media: {e}")
            return False
        try:
            info = response.json()
        except ValueError:
            print(f"listMedia returned non-JSON: HTTP {response.status_code}, "
                  f"body={response.text[:200]!r}")
            return False
        return info.get("files", [])

    def downloadByName(self, file_name, save_path=None, out_dir="."):
        """Download ANY file from the SD card by its on-camera filename. Works for any file the
        camera ever wrote, regardless of how many captures happened since — no dependence on the
        bounded recent-capture cache.

        Args:
            file_name: the on-camera filename (e.g. from listMedia() or a capture descriptor).
            save_path: full output path; defaults to out_dir/file_name.
            out_dir: directory used when save_path is not given (created if missing).

        Returns:
            the saved path, or None on failure.
        """
        if self.IP_RC == "":
            print("No IP_RC provided, cannot download image")
            return None
        if not file_name:
            print("Download error: no file_name")
            return None
        if save_path is None:
            os.makedirs(out_dir, exist_ok=True)
            save_path = os.path.join(out_dir, file_name)
        try:
            response = requests.post(
                self.baseCommandUrl + EP_DOWNLOAD_MEDIA_BY_NAME,
                data=file_name, timeout=120)
        except requests.exceptions.RequestException as e:
            print(f"{file_name}: download error: {e}")
            return None
        content_type = response.headers.get("Content-Type", "")
        if response.status_code != 200 or not content_type.startswith("image/"):
            print(f"{file_name}: download failed (HTTP {response.status_code}, "
                  f"Content-Type={content_type!r}, body={response.text[:200]!r})")
            return None
        with open(save_path, "wb") as f:
            f.write(response.content)
        print(f"{file_name} saved to: {save_path} ({len(response.content)} bytes)")
        return save_path

    def requestLRFMeasure(self):
        """Fire the H20T laser range finder once and return its reading."""
        response = self.requestSend(EP_LRF_MEASURE, "")
        if not response:
            return {"distance": None, "target": None, "state": None}
        try:
            print(response)
            return json.loads(response)
        except ValueError:
            print(f"LRF: could not parse response: {response!r}")
            return {"distance": None, "target": None, "state": None}

    def requestSendNavigateTrajectoryDJINative(self, waypoints, speed: float = 10.0):
        """
        Send waypoints to be executed using DJI's native waypoint mission system.
        :param waypoints: A list of triples (latitude, longitude, altitude) for each waypoint.
        :param speed: Flight speed in m/s (default 10.0)
        :return: The response from the server.
        """
        if not waypoints:
            raise ValueError("No waypoints provided")
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints for DJI native mission")

        # Build the message format: "speed;lat,lon,alt;lat,lon,alt;..."
        segments = [str(speed)]
        for lat, lon, alt in waypoints:
            segments.append(f"{lat},{lon},{alt}")

        message = ";".join(segments)
        return self.requestSend(EP_GOTO_TRAJECTORY_DJI_NATIVE, message)
    
    def requestAbortDJINativeMission(self):
        """Abort the current DJI native waypoint mission."""
        return self.requestSend(EP_ABORT_DJI_NATIVE_MISSION, "")

    def requestAbortMission(self):
        """Abort the current mission and disable virtual stick."""
        return self.requestSend(EP_ABORT_MISSION, "")

    def requestAbortAll(self):
        """Cancel any active PID control loop (controlLoopEnabled=false, activeLoopIsWaypoint=false).

        Use before a tuning command to guarantee a COLD start so new distanceKp/maxHorizontalAccel
        are re-captured — without this, a running loop would just hot-swap the target and keep the
        gains it already has.
        """
        return self.requestSend(EP_ABORT_ALL, "")

    def requestSendEnableVirtualStick(self):
        """Enable virtual stick control mode."""
        return self.requestSend(EP_ENABLE_VIRTUAL_STICK, "")

    def requestSendGotoYaw(self, yaw):
        """Rotate to a specific yaw angle.

        Returns the int seq the app assigned (for isYawReached(seq)), or None if rejected.
        """
        self.requestSendEnableVirtualStick()
        return self._parseSeq(self.requestSend(EP_GOTO_YAW, f"{yaw}"))

    def requestSendGotoAltitude(self, altitude):
        """Navigate to a specific altitude.

        Returns the int seq the app assigned (for isAltitudeReached(seq)), or None if rejected.
        """
        self.requestSendEnableVirtualStick()
        return self._parseSeq(self.requestSend(EP_GOTO_ALTITUDE, f"{altitude}"))

    def requestCameraStartRecording(self):
        """Start camera recording."""
        return self.requestSend(EP_CAMERA_START_RECORDING, "")

    def requestCameraStopRecording(self):
        """Stop camera recording."""
        return self.requestSend(EP_CAMERA_STOP_RECORDING, "")
    
    def requestSetRTHAltitude(self, altitude):
        """Set the return-to-home altitude in meters."""
        return self.requestSend(EP_SET_RTH_ALTITUDE, str(altitude))

    def requestDeactivateManualOverride(self):
        """Deactivate manual override latch so autonomous commands are accepted again.
        
        This should be called after the pilot has finished manual control
        and wants to allow autonomous commands to work again.
        """
        return self.requestSend(EP_DEACTIVATE_MANUAL_OVERRIDE, "")

    # ==================== Deprecated methods (kept for backward compatibility) ====================
    
    def requestSticks(self):
        """Deprecated: RC stick values are now available via getTelemetry()."""
        print("Warning: requestSticks() is deprecated. Use getTelemetry() instead.")
        return ""

    def requestWaypointStatus(self):
        """Deprecated: Use isWaypointReached() instead."""
        return str(self.isWaypointReached()).lower()

    def requestIntermediaryWaypointStatus(self):
        """Deprecated: Use isIntermediaryWaypointReached() instead."""
        return str(self.isIntermediaryWaypointReached()).lower()

    def requestYawStatus(self):
        """Deprecated: Use isYawReached() instead."""
        return str(self.isYawReached()).lower()

    def requestAltitudeStatus(self):
        """Deprecated: Use isAltitudeReached() instead."""
        return str(self.isAltitudeReached()).lower()

    def requestHomePosition(self):
        """Deprecated: Use getHomeLocation() instead."""
        return self.getHomeLocation()

    def requestCameraIsRecording(self):
        """Deprecated: Use isCameraRecording() instead."""
        return self.isCameraRecording()

    def requestDrop(self):
        """Drop the payload."""
        return self.requestSend(EP_PAYLOAD_DROP, "")


def _selftest():
    """Offline checks for the pure logic added here (no drone required)."""
    assert canonical_lenses("thermal,zoom") == ["thermal", "zoom"]
    assert canonical_lenses(["ZOOM", "wide", "zoom"]) == ["wide", "zoom"]
    try:
        canonical_lenses("rgb")
        raise AssertionError("expected ValueError for unknown lens")
    except ValueError:
        pass
    assert DJIInterface._parseSeq("WAYPOINT_ACCEPTED seq=42 lat=1") == 42
    assert DJIInterface._parseSeq("REJECTED") is None
    print("selftest OK")


if __name__ == '__main__':
    import time
    import sys
    
    IP_RC = "10.102.252.30"  # REPLACE WITH YOUR RC IP

    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        _selftest()
        sys.exit(0)

    if len(sys.argv) > 1:
        IP_RC = sys.argv[1]

    print(f"Connecting to {IP_RC}...")
    dji = DJIInterface(IP_RC)

    # Start telemetry stream (TCP socket on port 8081)
    print("Starting telemetry stream...")
    dji.startTelemetryStream()
    
    # Wait for initial connection
    time.sleep(1)
    
    print("\n" + "="*60)
    print("TCP Telemetry Socket Test - Press Ctrl+C to stop")
    print("="*60 + "\n")
    
    try:
        while True:
            telemetry = dji.getTelemetry()
            
            if telemetry:
                # Clear screen effect by printing separator
                print("-" * 60)
                print(f"[{telemetry.get('timestamp', 'N/A')}]")
                print(f"  Battery:     {dji.getBatteryLevel()}%")
                print(f"  Satellites:  {dji.getSatelliteCount()}")
                print(f"  Heading:     {dji.getHeading():.1f}°")
                print(f"  Location:    {dji.getLocation()}")
                print(f"  Altitude:    {dji.getLocation().get('altitude', 'N/A')} m")
                print(f"  Speed:       {dji.getSpeed()}")
                print(f"  Attitude:    {dji.getAttitude()}")
                print(f"  Gimbal:      {dji.getGimbalAttitude()}")
                print(f"  Home Set:    {dji.isHomeSet()}")
                print(f"  Home Loc:    {dji.getHomeLocation()}")
                print(f"  Dist Home:   {dji.getDistanceToHome():.1f} m")
                print(f"  Recording:   {dji.isCameraRecording()}")
                print(f"  WP Reached:  {dji.isWaypointReached()}")
                print(f"  Yaw Reached: {dji.isYawReached()}")
                print(f"  Alt Reached: {dji.isAltitudeReached()}")
                print(f"  Flight Time: {dji.getRemainingFlightTime()} s remaining")
                print(f"  Total Time:  {dji.getTotalTime()} s")
                print(f"  Time to RTH: {dji.getTimeNeededToGoHome()} s")
                print(f"  Time to Land:{dji.getTimeNeededToLand()} s")
                print(f"  Max Radius:  {dji.getMaxRadiusCanFlyAndGoHome()} m")
                print(f"  --- Battery Thresholds ---")
                print(f"  Remaining:   {dji.getRemainingCharge()}%")
                print(f"  Need Land:   {dji.getBatteryNeededToLand()}%")
                print(f"  Need RTH:    {dji.getBatteryNeededToGoHome()}%")
                print(f"  Low Batt:    {dji.getLowBatteryThreshold()}%")
                print(f"  Serious Low: {dji.getSeriousLowBatteryThreshold()}%")
                print(f"  Flight Mode: {dji.getFlightMode()}")
            else:
                print("Waiting for telemetry data...")
            
            time.sleep(0.1)  # Update every 500ms
            
    except KeyboardInterrupt:
        print("\n\nStopping telemetry stream...")
        dji.stopTelemetryStream()
        print("Done.")