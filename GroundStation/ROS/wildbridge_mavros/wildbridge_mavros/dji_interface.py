"""
WildBridge - DJI Interface Module

A Python interface for controlling DJI drones through HTTP requests and TCP sockets,
providing seamless integration for drone operations, telemetry retrieval, and video streaming.

Authors: Edouard G.A. Rolland, Kilian Meier 
Project: WildDrone
Institution: University of Bristol, University of Southern Denmark (SDU)
License: MIT

For more information, visit: https://github.com/WildDrone/WildBridge
"""

import cv2
import requests
import ast
import json
import socket
import threading
import time
import asyncio
from datetime import datetime
from pathlib import Path
from enum import Enum
from concurrent.futures import ThreadPoolExecutor, as_completed

# Discovery Configuration
DISCOVERY_PORT = 30000
DISCOVERY_MSG = b"DISCOVER_WILDBRIDGE"
DISCOVERY_RESPONSE_PREFIX = "WILDBRIDGE_HERE:"
MULTICAST_GROUP = "239.255.42.99"  # Multicast address for drone discovery
MULTICAST_PORT = 30001

# mDNS/Zeroconf Configuration
MDNS_SERVICE_TYPE = "_wildbridge._tcp.local."
MDNS_SERVICE_NAME = "WildBridge Drone"

# Cache configuration
CACHE_DIR = Path.home() / ".wildbridge"
CACHE_FILE = CACHE_DIR / "drones_cache.json"
CACHE_MAX_AGE = 3600  # 1 hour

# Optional: Try to import zeroconf for mDNS support
try:
    from zeroconf import ServiceBrowser, ServiceListener, Zeroconf, ServiceInfo
    ZEROCONF_AVAILABLE = True
except ImportError:
    ZEROCONF_AVAILABLE = False
    print("Note: zeroconf library not installed. Install with: pip install zeroconf")

class DroneState(Enum):
    """Drone connection states for health monitoring."""
    DISCOVERING = 1
    CONNECTED = 2
    DISCONNECTED = 3
    RECONNECTING = 4
    FAILED = 5


class DiscoveryCache:
    """Persistent cache for discovered drones."""
    
    def __init__(self, cache_file=CACHE_FILE):
        self.cache_file = Path(cache_file)
        self.cache_file.parent.mkdir(parents=True, exist_ok=True)
        self.drones = self.load()
    
    def load(self):
        """Load cache from disk."""
        if not self.cache_file.exists():
            return {}
        try:
            with open(self.cache_file, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Failed to load drone cache: {e}")
            return {}
    
    def save(self):
        """Save cache to disk."""
        try:
            with open(self.cache_file, 'w') as f:
                json.dump(self.drones, f, indent=2)
        except Exception as e:
            print(f"Failed to save drone cache: {e}")
    
    def update(self, ip, name, metadata=None):
        """Update cache with new or existing drone."""
        self.drones[ip] = {
            "name": name,
            "last_seen": time.time(),
            "metadata": metadata or {}
        }
        self.save()
    
    def get_recent(self, max_age=CACHE_MAX_AGE):
        """Get recently seen drones (within max_age seconds)."""
        cutoff = time.time() - max_age
        recent = {}
        for ip, data in self.drones.items():
            if data.get("last_seen", 0) > cutoff:
                recent[ip] = data
        return recent
    
    def get_all(self):
        """Get all cached drones."""
        return self.drones.copy()
    
    def remove(self, ip):
        """Remove a drone from cache."""
        if ip in self.drones:
            del self.drones[ip]
            self.save()
            return True
        return False
    
    def clear(self):
        """Clear all cache."""
        self.drones = {}
        self.save()


class DroneHealthMonitor:
    """Monitor drone health with automatic reconnection."""
    
    def __init__(self, check_interval=5.0):
        self.check_interval = check_interval
        self.monitors = {}
        self._running = False
    
    def start_monitoring(self, ip, name, on_disconnect=None, on_reconnect=None):
        """Start monitoring a drone."""
        if ip in self.monitors:
            return
        
        monitor_info = {
            "ip": ip,
            "name": name,
            "state": DroneState.CONNECTED,
            "consecutive_failures": 0,
            "on_disconnect": on_disconnect,
            "on_reconnect": on_reconnect,
            "thread": None
        }
        
        self.monitors[ip] = monitor_info
        
        # Start monitoring thread
        thread = threading.Thread(
            target=self._monitor_drone,
            args=(ip,),
            daemon=True
        )
        monitor_info["thread"] = thread
        thread.start()
    
    def stop_monitoring(self, ip):
        """Stop monitoring a drone."""
        if ip in self.monitors:
            del self.monitors[ip]
    
    def stop_all(self):
        """Stop all monitoring."""
        self.monitors.clear()
    
    def _monitor_drone(self, ip):
        """Background monitoring thread."""
        while ip in self.monitors:
            monitor = self.monitors[ip]
            
            try:
                # Try to ping the config endpoint
                response = requests.get(
                    f"http://{ip}:8080/config",
                    timeout=2.0
                )
                
                if response.status_code == 200:
                    # Drone is healthy
                    if monitor["state"] != DroneState.CONNECTED:
                        # Reconnected!
                        monitor["state"] = DroneState.CONNECTED
                        monitor["consecutive_failures"] = 0
                        if monitor["on_reconnect"]:
                            monitor["on_reconnect"](ip, monitor["name"])
                else:
                    self._handle_failure(ip, monitor)
                    
            except Exception as e:
                self._handle_failure(ip, monitor)
            
            time.sleep(self.check_interval)
    
    def _handle_failure(self, ip, monitor):
        """Handle drone connection failure."""
        monitor["consecutive_failures"] += 1
        
        if monitor["consecutive_failures"] == 1:
            # First failure, mark as disconnected
            monitor["state"] = DroneState.DISCONNECTED
            if monitor["on_disconnect"]:
                monitor["on_disconnect"](ip, monitor["name"])
        
        if monitor["consecutive_failures"] > 5:
            monitor["state"] = DroneState.FAILED
    
    def get_state(self, ip):
        """Get current state of a drone."""
        if ip in self.monitors:
            return self.monitors[ip]["state"]
        return None


def get_local_ips():
    """Get all local IP addresses and network information."""
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


def get_broadcast_addresses():
    """Get all possible broadcast addresses for local networks."""
    broadcast_addrs = ['255.255.255.255']  # Global broadcast
    
    local_ips = get_local_ips()
    for ip in local_ips:
        parts = ip.split('.')
        # Assume /24 subnet for simplicity (most common)
        subnet_broadcast = f"{parts[0]}.{parts[1]}.{parts[2]}.255"
        if subnet_broadcast not in broadcast_addrs:
            broadcast_addrs.append(subnet_broadcast)
        
        # Also try /16 subnet broadcast
        class_b_broadcast = f"{parts[0]}.{parts[1]}.255.255"
        if class_b_broadcast not in broadcast_addrs:
            broadcast_addrs.append(class_b_broadcast)
    
    return broadcast_addrs


def discover_via_multicast(timeout=3.0, verbose=True):
    """
    Discover drones using IP multicast - works better across VLANs and larger networks.
    Returns list of tuples [(drone_ip, drone_name), ...]
    """
    found_drones = {}
    
    try:
        # Create multicast socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Set multicast TTL
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        
        # Bind to receive responses
        sock.bind(('', MULTICAST_PORT))
        
        # Join multicast group
        mreq = socket.inet_aton(MULTICAST_GROUP) + socket.inet_aton('0.0.0.0')
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        
        sock.settimeout(timeout)
        
        # Send multicast discovery
        sock.sendto(DISCOVERY_MSG, (MULTICAST_GROUP, MULTICAST_PORT))
        
        if verbose:
            print(f"Sent multicast discovery to {MULTICAST_GROUP}:{MULTICAST_PORT}")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                data, addr = sock.recvfrom(1024)
                message = data.decode('utf-8')
                if message.startswith(DISCOVERY_RESPONSE_PREFIX):
                    parts = message.split(':')
                    drone_ip = parts[1] if len(parts) > 1 else addr[0]
                    drone_name = parts[2] if len(parts) > 2 else "UNKNOWN"
                    if drone_ip and drone_ip not in found_drones:
                        if verbose:
                            print(f"✓ Found drone via multicast: {drone_ip} (Name: {drone_name})")
                        found_drones[drone_ip] = drone_name
            except socket.timeout:
                break
        
        sock.close()
        
    except Exception as e:
        if verbose:
            print(f"Multicast discovery failed: {e}")
    
    return list(found_drones.items())


class WildBridgeServiceListener:
    """Listener for mDNS/Zeroconf service discovery."""
    
    def __init__(self, verbose=True):
        self.found_drones = {}
        self.verbose = verbose
    
    def add_service(self, zc, type_, name):
        """Called when a service is discovered."""
        info = zc.get_service_info(type_, name)
        if info:
            # Extract IP address
            if info.addresses:
                ip = socket.inet_ntoa(info.addresses[0])
                # Get drone name from service properties or name
                drone_name = info.properties.get(b'name', b'').decode('utf-8')
                if not drone_name:
                    # Extract from service name (e.g., "cacatua._wildbridge._tcp.local.")
                    drone_name = name.split('.')[0]
                
                if ip not in self.found_drones:
                    self.found_drones[ip] = drone_name
                    if self.verbose:
                        print(f"  ✓ mDNS: Found {drone_name} at {ip}")
    
    def remove_service(self, zc, type_, name):
        """Called when a service is removed."""
        pass
    
    def update_service(self, zc, type_, name):
        """Called when a service is updated."""
        self.add_service(zc, type_, name)


def discover_via_mdns(timeout=5.0, verbose=True):
    """
    Discover drones using mDNS/Zeroconf - the industry standard for local service discovery.
    Works across subnets (with mDNS repeaters), more reliable than UDP broadcast.
    
    Returns list of tuples [(drone_ip, drone_name), ...]
    """
    if not ZEROCONF_AVAILABLE:
        if verbose:
            print("  ⚠ mDNS not available (install: pip install zeroconf)")
        return []
    
    found_drones = {}
    
    try:
        zeroconf = Zeroconf()
        listener = WildBridgeServiceListener(verbose=verbose)
        
        # Browse for WildBridge services
        browser = ServiceBrowser(zeroconf, MDNS_SERVICE_TYPE, listener)
        
        if verbose:
            print(f"  Browsing for {MDNS_SERVICE_TYPE}...")
        
        # Wait for discovery
        time.sleep(timeout)
        
        # Get results
        found_drones = listener.found_drones.copy()
        
        # Cleanup
        browser.cancel()
        zeroconf.close()
        
    except Exception as e:
        if verbose:
            print(f"  ✗ mDNS discovery failed: {e}")
    
    return list(found_drones.items())


def discover_via_broadcast_enhanced(timeout=3.0, verbose=True):
    """
    Enhanced broadcast discovery trying multiple broadcast addresses and socket configurations.
    Returns list of tuples [(drone_ip, drone_name), ...]
    """
    found_drones = {}
    broadcast_addrs = get_broadcast_addresses()
    
    if verbose:
        print(f"Trying broadcast discovery on {len(broadcast_addrs)} address(es)...")
    
    for broadcast_addr in broadcast_addrs:
        try:
            # Create a new socket for each broadcast address
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Bind to a specific port to receive responses
            try:
                sock.bind(('', DISCOVERY_PORT + 1))
            except:
                sock.bind(('', 0))  # Let OS choose port
            
            sock.settimeout(timeout)
            
            # Send broadcast
            sock.sendto(DISCOVERY_MSG, (broadcast_addr, DISCOVERY_PORT))
            
            if verbose:
                print(f"  → Broadcast to {broadcast_addr}:{DISCOVERY_PORT}")
            
            start_time = time.time()
            while time.time() - start_time < timeout:
                try:
                    data, addr = sock.recvfrom(1024)
                    message = data.decode('utf-8')
                    if message.startswith(DISCOVERY_RESPONSE_PREFIX):
                        parts = message.split(':')
                        drone_ip = parts[1] if len(parts) > 1 else addr[0]
                        drone_name = parts[2] if len(parts) > 2 else "UNKNOWN"
                        if drone_ip and drone_ip not in found_drones:
                            if verbose:
                                print(f"  ✓ Response from {drone_ip} (Name: {drone_name})")
                            found_drones[drone_ip] = drone_name
                except socket.timeout:
                    break
            
            sock.close()
            
        except Exception as e:
            if verbose:
                print(f"  ✗ Broadcast to {broadcast_addr} failed: {e}")
    
    return list(found_drones.items())


def probe_single_ip(ip, timeout=1.5):
    """
    Probe a single IP for WildBridge drone.
    Returns (ip, name) tuple or None.
    """
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
                sock.close()
                return (drone_ip, drone_name)
        except socket.timeout:
            pass
        
        sock.close()
    except:
        pass
    
    return None


def scan_subnet_for_drones_parallel(local_ips, timeout=1.5, verbose=True, max_workers=50):
    """
    Scan subnet for WildBridge drones using parallel UDP probing.
    Much faster than sequential scanning.
    Returns list of tuples [(drone_ip, drone_name), ...]
    """
    found_drones = {}
    
    if verbose:
        print("Scanning subnet for WildBridge drones (parallel mode)...")
    
    # Build list of IPs to scan
    ips_to_scan = []
    for local_ip in local_ips:
        parts = local_ip.split('.')
        subnet = f"{parts[0]}.{parts[1]}.{parts[2]}"
        
        # Try common IP ranges
        ranges = list(range(1, 51)) + list(range(100, 121)) + list(range(150, 171)) + list(range(200, 221))
        
        for i in ranges:
            ip = f"{subnet}.{i}"
            if ip != local_ip:
                ips_to_scan.append(ip)
    
    # Probe all IPs in parallel
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_ip = {executor.submit(probe_single_ip, ip, timeout): ip for ip in ips_to_scan}
        
        for future in as_completed(future_to_ip):
            result = future.result()
            if result:
                drone_ip, drone_name = result
                if drone_ip not in found_drones:
                    found_drones[drone_ip] = drone_name
                    if verbose:
                        print(f"✓ Found WildBridge drone at {drone_ip} (Name: {drone_name})")
    
    return list(found_drones.items())


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

def discover_all_drones(timeout=5.0, verbose=True, use_cache=True, max_retries=2):
    """
    Discover all WildBridge drones on the network with enhanced robustness.
    
    Discovery methods (in order of preference):
    1. Cache verification (fastest - instant if drone was seen recently)
    2. mDNS/Zeroconf (most reliable, industry standard)
    3. Targeted subnet scan (always runs to find drones not in cache/mDNS)
    
    Args:
        timeout: Timeout for each discovery method
        verbose: Print discovery progress
        use_cache: Try cached drones first
        max_retries: Number of retry attempts (unused, kept for API compatibility)
    
    Returns:
        List of tuples [(drone_ip, drone_name), ...]
    """
    found_drones = {}
    failed_cache_entries = {}  # Track {old_ip: name} for drones that failed verification
    cache = DiscoveryCache() if use_cache else None
    
    def verify_drone_http(ip, name, source=""):
        """Verify drone is running by checking HTTP endpoint."""
        try:
            response = requests.get(f"http://{ip}:8080/config", timeout=1.0)
            if response.status_code == 200:
                return True
        except:
            pass
        return False
    
    # Method 1: Try cached drones first for faster startup
    if cache:
        recent = cache.get_recent()
        if recent:
            if verbose:
                print(f"[Cache] Found {len(recent)} drone(s) in cache, verifying...")
            for ip, data in recent.items():
                name = data.get("name", "UNKNOWN")
                if verify_drone_http(ip, name):
                    found_drones[ip] = name
                    if verbose:
                        print(f"  ✓ Verified {ip} (Name: {name})")
                else:
                    # Track failed entries with their names so we can find them with new IPs
                    failed_cache_entries[ip] = name
                    if verbose:
                        print(f"  ✗ {ip} ({name}) not responding - will search for new IP")
    
    if found_drones and verbose:
        print(f"[Cache] Verified {len(found_drones)} cached drone(s), continuing to search for more...")
    
    # Method 2: mDNS/Zeroconf discovery (PREFERRED - industry standard)
    if ZEROCONF_AVAILABLE:
        if verbose:
            print("[mDNS] Trying mDNS/Zeroconf discovery...")
        
        mdns_drones = discover_via_mdns(timeout=min(timeout, 5.0), verbose=verbose)
        new_from_mdns = 0
        for ip, name in mdns_drones:
            if ip not in found_drones:
                # Verify the drone is actually running
                if verify_drone_http(ip, name):
                    found_drones[ip] = name
                    new_from_mdns += 1
                    if cache:
                        cache.update(ip, name)
                    # Check if this drone was in cache with different IP
                    for old_ip, old_name in list(failed_cache_entries.items()):
                        if old_name == name:
                            if verbose:
                                print(f"  ↳ {name} IP changed: {old_ip} → {ip}")
                            cache.remove(old_ip)
                            del failed_cache_entries[old_ip]
                            break
                    if verbose:
                        print(f"  ✓ Verified {ip} (Name: {name})")
                else:
                    if verbose:
                        print(f"  ✗ {ip} mDNS registered but app not running")
        
        if verbose:
            if new_from_mdns > 0:
                print(f"[mDNS] ✓ Found {new_from_mdns} verified drone(s) (total: {len(found_drones)})")
            else:
                print(f"[mDNS] No new active drones found")
    
    # Method 3: Targeted subnet scan (ALWAYS run to find drones with changed IPs)
    # This ensures we find drones even if cache is stale or mDNS isn't working
    if verbose:
        print("[Subnet Scan] Scanning local subnets for drones...")
    
    local_ips = get_local_ips()
    if local_ips:
        if verbose:
            print(f"  Local IPs detected: {', '.join(local_ips)}")
        
        # Scan subnets to find any drones we might have missed
        subnet_drones = scan_subnet_for_drones_parallel(
            local_ips, 
            timeout=1.5,  # Allow time for slower drones to respond
            verbose=verbose,
            max_workers=50
        )
        new_from_scan = 0
        for ip, name in subnet_drones:
            if ip not in found_drones:
                # Verify the drone is actually running
                if verify_drone_http(ip, name):
                    found_drones[ip] = name
                    new_from_scan += 1
                    if cache:
                        cache.update(ip, name)
                    # Check if this drone was in cache with different IP (IP changed via DHCP)
                    for old_ip, old_name in list(failed_cache_entries.items()):
                        if old_name == name:
                            if verbose:
                                print(f"  ↳ {name} IP changed: {old_ip} → {ip}")
                            cache.remove(old_ip)
                            del failed_cache_entries[old_ip]
                            break
                    if verbose:
                        print(f"  ✓ Verified {ip} (Name: {name})")
                else:
                    if verbose:
                        print(f"  ✗ {ip} responded to UDP but app not running")
        
        if verbose:
            if new_from_scan > 0:
                print(f"[Subnet Scan] ✓ Found {new_from_scan} new drone(s) (total: {len(found_drones)})")
            else:
                print(f"[Subnet Scan] No additional drones found")
    
    # Clean up remaining stale cache entries that weren't found with any method
    if cache and failed_cache_entries:
        for ip, name in failed_cache_entries.items():
            cache.remove(ip)
            if verbose:
                print(f"[Cache] Removed stale entry for {name} at {ip} (not found on network)")
    
    if verbose:
        if found_drones:
            print(f"✓ Discovery complete: Found {len(found_drones)} active drone(s)")
        else:
            print("✗ No active drones found with any method")
            print("\nTroubleshooting tips:")
            print("  1. Ensure drone WildBridge app is running")
            print("  2. Check firewall allows UDP port 30000-30001 and TCP port 8080")
            print("  3. Verify you're on the same network as the drone")
            print("  4. Try: sudo iptables -L to check for blocking rules")
    
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

# HTTP POST Command Endpoints (port 8080)
EP_STICK = "/send/stick"  # expects a formatted string: "<leftX>,<leftY>,<rightX>,<rightY>"
EP_ZOOM = "/send/camera/zoom"
EP_GIMBAL_SET_PITCH = "/send/gimbal/pitch"
EP_GIMBAL_SET_YAW = "/send/gimbal/yaw"  # !!! This is the yaw joint angle !!!
EP_TAKEOFF = "/send/takeoff"
EP_LAND = "/send/land"
EP_RTH = "/send/RTH"
EP_ENABLE_VIRTUAL_STICK = "/send/enableVirtualStick"
EP_ABORT_MISSION = "/send/abortMission"
EP_ABORT_ALL = "/send/abortAll"
EP_GOTO_WP = "/send/gotoWP"
EP_GOTO_YAW = "/send/gotoYaw"
EP_GOTO_WP_PID = "/send/gotoWPwithPID"
EP_GOTO_TRAJECTORY = "/send/navigateTrajectory"
EP_GOTO_ALTITUDE = "/send/gotoAltitude"
EP_CAMERA_START_RECORDING = "/send/camera/startRecording"
EP_CAMERA_STOP_RECORDING = "/send/camera/stopRecording"
EP_GOTO_TRAJECTORY_DJI_NATIVE = "/send/navigateTrajectoryDJINative"
EP_ABORT_DJI_NATIVE_MISSION = "/send/abort/DJIMission"
EP_SET_RTH_ALTITUDE = "/send/setRTHAltitude"

# PID Tuning
EP_TUNING = "/send/gotoWPwithPIDtuning"


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
            discovered_ip, drone_name = discover_drone()
            if discovered_ip:
                self.IP_RC = discovered_ip
                self.drone_name = drone_name
                
                # If discovery didn't provide name, query config endpoint
                if self.drone_name == "UNKNOWN":
                    config = get_config(self.IP_RC)
                    if config and "droneName" in config:
                        self.drone_name = config["droneName"]
                        print(f"Retrieved drone name from config: {self.drone_name}")
            else:
                print("Drone discovery failed.")
                self.IP_RC = ""
                self.drone_name = "UNKNOWN"
        else:
            self.IP_RC = IP_RC
            # Query config endpoint to get drone name
            config = get_config(self.IP_RC)
            if config and "droneName" in config:
                self.drone_name = config["droneName"]
                print(f"Retrieved drone name from config: {self.drone_name}")
            else:
                self.drone_name = "UNKNOWN"

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
    
    def getHomeLocation(self):
        """Get home location (latitude, longitude)."""
        return self.getTelemetry().get("homeLocation", {})
    
    def getDistanceToHome(self):
        """Get distance to home in meters."""
        return self.getTelemetry().get("distanceToHome", 0.0)
    
    def isWaypointReached(self):
        """Check if the current waypoint has been reached."""
        return self.getTelemetry().get("waypointReached", False)
    
    def isIntermediaryWaypointReached(self):
        """Check if an intermediary waypoint has been reached."""
        return self.getTelemetry().get("intermediaryWaypointReached", False)
    
    def isYawReached(self):
        """Check if the target yaw has been reached."""
        return self.getTelemetry().get("yawReached", False)
    
    def isAltitudeReached(self):
        """Check if the target altitude has been reached."""
        return self.getTelemetry().get("altitudeReached", False)
    
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

    def requestSendGoToWP(self, latitude, longitude, altitude):
        """Navigate to a waypoint."""
        return self.requestSend(EP_GOTO_WP, f"{latitude},{longitude},{altitude}")

    def requestSendGoToWPwithPID(self, latitude, longitude, altitude, yaw, speed: float = 5.0):
        """Navigate to a waypoint with PID control.
        
        Args:
            latitude: Target latitude
            longitude: Target longitude
            altitude: Target altitude
            yaw: Target yaw angle
            speed: Max speed in m/s (default 5.0)
        """
        return self.requestSend(EP_GOTO_WP_PID, f"{latitude},{longitude},{altitude},{yaw},{speed}")
    
    def requestSendGoToWPwithPIDtuning(self, latitude, longitude, altitude, yaw, kp_pos, ki_pos, kd_pos, kp_yaw, ki_yaw, kd_yaw):
        """Navigate to a waypoint with custom PID tuning parameters."""
        return self.requestSend(EP_TUNING, f"{latitude},{longitude},{altitude},{yaw},{kp_pos},{ki_pos},{kd_pos},{kp_yaw},{ki_yaw},{kd_yaw}")

    def requestSendNavigateTrajectory(self, waypoints, finalYaw):
        """
        Navigate through a series of waypoints.
        :param waypoints: A list of triples (latitude, longitude, altitude) for each waypoint.
        :param finalYaw: The final yaw angle at the last waypoint.
        :return: The response from the server.
        """
        self.requestSendEnableVirtualStick()
        if not waypoints:
            raise ValueError("No waypoints provided")

        # Build the message
        # All waypoints except the last: "lat,lon,alt"
        # Last waypoint: "lat,lon,alt,yaw"
        segments = []
        for i, (lat, lon, alt) in enumerate(waypoints):
            if i < len(waypoints) - 1:
                # Intermediary waypoint: lat,lon,alt
                segments.append(f"{lat},{lon},{alt}")
            else:
                # Last waypoint: lat,lon,alt,yaw
                segments.append(f"{lat},{lon},{alt},{finalYaw}")

        message = ";".join(segments)
        return self.requestSend(EP_GOTO_TRAJECTORY, message)
    
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
        """Abort ALL missions (PID control loops + DJI native waypoint missions).
        This is the most comprehensive abort - use when you want to stop everything."""
        return self.requestSend(EP_ABORT_ALL, "")

    def requestSendEnableVirtualStick(self):
        """Enable virtual stick control mode."""
        return self.requestSend(EP_ENABLE_VIRTUAL_STICK, "")

    def requestSendGotoYaw(self, yaw):
        """Rotate to a specific yaw angle."""
        self.requestSendEnableVirtualStick()
        return self.requestSend(EP_GOTO_YAW, f"{yaw}")

    def requestSendGotoAltitude(self, altitude):
        """Navigate to a specific altitude."""
        self.requestSendEnableVirtualStick()
        return self.requestSend(EP_GOTO_ALTITUDE, f"{altitude}")

    def requestCameraStartRecording(self):
        """Start camera recording."""
        return self.requestSend(EP_CAMERA_START_RECORDING, "")

    def requestCameraStopRecording(self):
        """Stop camera recording."""
        return self.requestSend(EP_CAMERA_STOP_RECORDING, "")
    
    def requestSetRTHAltitude(self, altitude):
        """Set the return-to-home altitude in meters."""
        return self.requestSend(EP_SET_RTH_ALTITUDE, str(altitude))

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

if __name__ == '__main__':
    import time
    import sys
    
    IP_RC = "10.102.252.30"  # REPLACE WITH YOUR RC IP
    
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