"""
WildBridge MAVROS Bridge

A MAVROS-compatible ROS 2 node that bridges WildBridge DJI drone control
to standard MAVROS topics and services.

This allows applications built for PX4/ArduPilot drones to control DJI drones
through WildBridge with minimal changes.

Author: Edouard Rolland
Project: WildDrone
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Standard ROS messages
from std_msgs.msg import Header, Bool, Float64, UInt32, String
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point
from sensor_msgs.msg import NavSatFix, NavSatStatus, BatteryState, Imu
from nav_msgs.msg import Odometry

# MAVROS messages (we'll use standard messages where mavros_msgs isn't available)
# If mavros_msgs is installed, these can be swapped
try:
    from mavros_msgs.msg import State, ExtendedState, StatusText, HomePosition
    from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, CommandLong
    MAVROS_MSGS_AVAILABLE = True
except ImportError:
    MAVROS_MSGS_AVAILABLE = False

import sys
import os
import math
import threading
from datetime import datetime

# Add parent path to import djiInterface
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'Python'))
try:
    from djiInterface import DJIInterface
except ImportError:
    # Fallback: try from submodules (same as dji_controller)
    from wildbridge_mavros.dji_interface import DJIInterface


class WildBridgeMavrosNode(Node):
    """
    MAVROS-compatible ROS 2 node for WildBridge DJI drone control.
    
    Publishes MAVROS-style topics and provides MAVROS-compatible services
    while using WildBridge's TCP/HTTP interface under the hood.
    """
    
    # Flight mode mapping: DJI modes to MAVROS/PX4-style modes
    DJI_TO_MAVROS_MODE = {
        'MANUAL': 'MANUAL',
        'GPS': 'POSCTL',
        'ATTI': 'ALTCTL',
        'GO_HOME': 'AUTO.RTL',
        'AUTO_LANDING': 'AUTO.LAND',
        'AUTO_TAKEOFF': 'AUTO.TAKEOFF',
        'VIRTUAL_STICK': 'OFFBOARD',
        'WAYPOINT': 'AUTO.MISSION',
        'UNKNOWN': 'UNKNOWN',
    }
    
    MAVROS_TO_DJI_MODE = {v: k for k, v in DJI_TO_MAVROS_MODE.items()}
    
    def __init__(self):
        super().__init__('wildbridge_mavros')
        
        self.get_logger().info("Initializing WildBridge MAVROS Bridge...")
        
        # Parameters
        self.declare_parameter('drone_ip', '')  # Empty default for auto-discovery
        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 1)
        self.declare_parameter('telemetry_rate', 20.0)  # Hz
        
        self.drone_ip = self.get_parameter('drone_ip').get_parameter_value().string_value
        self.telemetry_rate = self.get_parameter('telemetry_rate').get_parameter_value().double_value
        
        if not self.drone_ip:
            self.get_logger().info("No drone_ip provided, attempting auto-discovery...")
        else:
            self.get_logger().info(f"Connecting to drone at {self.drone_ip}...")

        # Initialize DJI interface
        self.dji = DJIInterface(self.drone_ip)
        
        # Update drone_ip if discovered
        if not self.drone_ip and self.dji.IP_RC:
            self.drone_ip = self.dji.IP_RC
            drone_name = getattr(self.dji, 'drone_name', 'UNKNOWN')
            self.get_logger().info(f"Discovered and connected to drone at {self.drone_ip} (Name: {drone_name})")
        elif not self.drone_ip:
            self.get_logger().warn("Failed to discover drone. Connection may fail.")
        
        # State tracking
        self.armed = False
        self.connected = False
        self.guided = False
        self.mode = "UNKNOWN"
        self.last_telemetry = {}
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Create publishers (MAVROS-compatible topics)
        self._create_publishers()
        
        # Create subscribers (MAVROS-compatible setpoints)
        self._create_subscribers()
        
        # Create services (MAVROS-compatible)
        self._create_services()
        
        # Start telemetry stream
        self.dji.startTelemetryStream()
        self.connected = True
        
        # Timer for publishing telemetry
        timer_period = 1.0 / self.telemetry_rate
        self.telemetry_timer = self.create_timer(timer_period, self._publish_telemetry)
        
        self.get_logger().info(f"WildBridge MAVROS Bridge initialized - Connected to {self.drone_ip}")
    
    def _create_publishers(self):
        """Create MAVROS-compatible publishers."""
        
        # State information
        self.state_pub = self.create_publisher(
            Bool, 'mavros/state/connected', self.state_qos)
        self.armed_pub = self.create_publisher(
            Bool, 'mavros/state/armed', self.state_qos)
        self.mode_pub = self.create_publisher(
            String, 'mavros/state/mode', self.state_qos)
        
        # Position/Pose
        self.local_position_pub = self.create_publisher(
            PoseStamped, 'mavros/local_position/pose', self.sensor_qos)
        self.global_position_pub = self.create_publisher(
            NavSatFix, 'mavros/global_position/global', self.sensor_qos)
        self.home_position_pub = self.create_publisher(
            NavSatFix, 'mavros/home_position/home', self.state_qos)
        
        # Velocity
        self.local_velocity_pub = self.create_publisher(
            TwistStamped, 'mavros/local_position/velocity_local', self.sensor_qos)
        
        # Attitude
        self.imu_pub = self.create_publisher(
            Imu, 'mavros/imu/data', self.sensor_qos)
        
        # Battery
        self.battery_pub = self.create_publisher(
            BatteryState, 'mavros/battery', self.sensor_qos)
        
        # GPS
        self.gps_satellites_pub = self.create_publisher(
            UInt32, 'mavros/global_position/satellites', self.sensor_qos)
        
        # Heading
        self.heading_pub = self.create_publisher(
            Float64, 'mavros/global_position/compass_hdg', self.sensor_qos)
        
        # Relative altitude
        self.rel_alt_pub = self.create_publisher(
            Float64, 'mavros/global_position/rel_alt', self.sensor_qos)
        
        # WildBridge-specific status
        self.waypoint_reached_pub = self.create_publisher(
            Bool, 'wildbridge/waypoint_reached', 10)
        self.distance_to_home_pub = self.create_publisher(
            Float64, 'wildbridge/distance_to_home', 10)
        self.flight_time_remaining_pub = self.create_publisher(
            Float64, 'wildbridge/flight_time_remaining', 10)
        
        self.get_logger().info("Publishers created")
    
    def _create_subscribers(self):
        """Create MAVROS-compatible subscribers for setpoints."""
        
        # Position setpoint (local frame)
        self.create_subscription(
            PoseStamped,
            'mavros/setpoint_position/local',
            self._setpoint_position_callback,
            10
        )
        
        # Global position setpoint
        self.create_subscription(
            NavSatFix,
            'mavros/setpoint_position/global',
            self._setpoint_global_callback,
            10
        )
        
        # Velocity setpoint
        self.create_subscription(
            TwistStamped,
            'mavros/setpoint_velocity/cmd_vel',
            self._setpoint_velocity_callback,
            10
        )
        
        # Raw attitude setpoint (for gimbal control)
        self.create_subscription(
            PoseStamped,
            'mavros/setpoint_attitude/attitude',
            self._setpoint_attitude_callback,
            10
        )
        
        self.get_logger().info("Subscribers created")
    
    def _create_services(self):
        """Create MAVROS-compatible services."""
        
        # Arming service
        self.arming_srv = self.create_service(
            SetBool, 'mavros/cmd/arming', self._arming_callback)
        
        # Takeoff service
        self.takeoff_srv = self.create_service(
            Trigger, 'mavros/cmd/takeoff', self._takeoff_callback)
        
        # Land service  
        self.land_srv = self.create_service(
            Trigger, 'mavros/cmd/land', self._land_callback)
        
        # RTL (Return to Launch) service
        self.rtl_srv = self.create_service(
            Trigger, 'mavros/cmd/rtl', self._rtl_callback)
        
        # Set mode service (simplified - uses Trigger with mode as parameter)
        self.set_offboard_srv = self.create_service(
            Trigger, 'mavros/set_mode/offboard', self._set_offboard_callback)
        
        # WildBridge-specific services
        self.enable_vs_srv = self.create_service(
            Trigger, 'wildbridge/enable_virtual_stick', self._enable_virtual_stick_callback)
        self.abort_mission_srv = self.create_service(
            Trigger, 'wildbridge/abort_mission', self._abort_mission_callback)
        
        self.get_logger().info("Services created")
    
    # ==================== Telemetry Publishing ====================
    
    def _publish_telemetry(self):
        """Publish telemetry data to MAVROS-compatible topics."""
        
        telemetry = self.dji.getTelemetry()
        if not telemetry:
            return
        
        self.last_telemetry = telemetry
        now = self.get_clock().now().to_msg()
        
        # Update state
        dji_mode = telemetry.get('flightMode', 'UNKNOWN')
        self.mode = self.DJI_TO_MAVROS_MODE.get(dji_mode, dji_mode)
        
        # Publish connected state
        connected_msg = Bool()
        connected_msg.data = self.connected
        self.state_pub.publish(connected_msg)
        
        # Publish armed state (inferred from flight mode)
        armed_msg = Bool()
        self.armed = dji_mode not in ['UNKNOWN', 'MANUAL']
        armed_msg.data = self.armed
        self.armed_pub.publish(armed_msg)
        
        # Publish mode
        from std_msgs.msg import String
        mode_msg = String()
        mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)
        
        # Publish global position (GPS)
        location = telemetry.get('location', {})
        if location:
            gps_msg = NavSatFix()
            gps_msg.header.stamp = now
            gps_msg.header.frame_id = 'earth'
            gps_msg.latitude = float(location.get('latitude', 0.0))
            gps_msg.longitude = float(location.get('longitude', 0.0))
            gps_msg.altitude = float(location.get('altitude', 0.0))
            gps_msg.status.status = NavSatStatus.STATUS_FIX
            gps_msg.status.service = NavSatStatus.SERVICE_GPS
            self.global_position_pub.publish(gps_msg)
            
            # Relative altitude
            alt_msg = Float64()
            alt_msg.data = float(location.get('altitude', 0.0))
            self.rel_alt_pub.publish(alt_msg)
        
        # Publish home position
        home_location = telemetry.get('homeLocation', {})
        if home_location:
            home_msg = NavSatFix()
            home_msg.header.stamp = now
            home_msg.header.frame_id = 'earth'
            home_msg.latitude = float(home_location.get('latitude', 0.0))
            home_msg.longitude = float(home_location.get('longitude', 0.0))
            home_msg.altitude = 0.0  # Home altitude typically at ground level
            self.home_position_pub.publish(home_msg)
        
        # Publish local position (simplified - relative to home)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        
        # Convert GPS to local coordinates (simplified: meters from home)
        if location and home_location:
            lat_diff = float(location.get('latitude', 0.0)) - float(home_location.get('latitude', 0.0))
            lon_diff = float(location.get('longitude', 0.0)) - float(home_location.get('longitude', 0.0))
            # Approximate conversion to meters (at equator 1 degree ≈ 111km)
            pose_msg.pose.position.x = float(lon_diff * 111000 * math.cos(math.radians(float(location.get('latitude', 0.0)))))
            pose_msg.pose.position.y = float(lat_diff * 111000)
            pose_msg.pose.position.z = float(location.get('altitude', 0.0))
        
        # Orientation from attitude
        attitude = telemetry.get('attitude', {})
        if attitude:
            # Convert euler to quaternion
            roll = math.radians(float(attitude.get('roll', 0.0)))
            pitch = math.radians(float(attitude.get('pitch', 0.0)))
            yaw = math.radians(float(attitude.get('yaw', 0.0)))
            q = self._euler_to_quaternion(roll, pitch, yaw)
            pose_msg.pose.orientation = q
        
        self.local_position_pub.publish(pose_msg)
        
        # Publish velocity
        speed = telemetry.get('speed', {})
        if speed:
            vel_msg = TwistStamped()
            vel_msg.header.stamp = now
            vel_msg.header.frame_id = 'base_link'
            vel_msg.twist.linear.x = float(speed.get('x', 0.0))
            vel_msg.twist.linear.y = float(speed.get('y', 0.0))
            vel_msg.twist.linear.z = float(speed.get('z', 0.0))
            self.local_velocity_pub.publish(vel_msg)
        
        # Publish IMU data
        if attitude:
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = 'base_link'
            imu_msg.orientation = pose_msg.pose.orientation
            self.imu_pub.publish(imu_msg)
        
        # Publish battery state
        battery_level = telemetry.get('batteryLevel', -1)
        if battery_level >= 0:
            battery_msg = BatteryState()
            battery_msg.header.stamp = now
            battery_msg.percentage = float(battery_level) / 100.0
            battery_msg.voltage = 0.0  # Not available from DJI
            battery_msg.present = True
            self.battery_pub.publish(battery_msg)
        
        # Publish satellite count
        sat_count = telemetry.get('satelliteCount', 0)
        if sat_count is not None:
            sat_msg = UInt32()
            sat_val = int(sat_count) if sat_count >= 0 else 0
            sat_msg.data = max(0, min(sat_val, 4294967295))  # Clamp to uint32 range
            self.gps_satellites_pub.publish(sat_msg)
        
        # Publish heading
        heading = telemetry.get('heading', 0.0)
        hdg_msg = Float64()
        hdg_msg.data = float(heading)
        self.heading_pub.publish(hdg_msg)
        
        # WildBridge-specific
        wp_reached_msg = Bool()
        wp_reached_msg.data = telemetry.get('waypointReached', False)
        self.waypoint_reached_pub.publish(wp_reached_msg)
        
        dist_home_msg = Float64()
        dist_home_msg.data = float(telemetry.get('distanceToHome', 0.0))
        self.distance_to_home_pub.publish(dist_home_msg)
        
        flight_time_msg = Float64()
        flight_time_msg.data = float(telemetry.get('remainingFlightTime', 0))
        self.flight_time_remaining_pub.publish(flight_time_msg)
    
    # ==================== Setpoint Callbacks ====================
    
    def _setpoint_position_callback(self, msg: PoseStamped):
        """Handle local position setpoint (converted to GPS waypoint)."""
        
        # Convert local position back to GPS coordinates
        home = self.last_telemetry.get('homeLocation', {})
        if not home:
            self.get_logger().warn("Cannot process local setpoint - no home position")
            return
        
        home_lat = home.get('latitude', 0.0)
        home_lon = home.get('longitude', 0.0)
        
        # Convert meters back to GPS (inverse of publish calculation)
        lat = home_lat + (msg.pose.position.y / 111000)
        lon = home_lon + (msg.pose.position.x / (111000 * math.cos(math.radians(home_lat))))
        alt = msg.pose.position.z
        
        # Extract yaw from quaternion
        yaw = self._quaternion_to_yaw(msg.pose.orientation)
        yaw_deg = math.degrees(yaw)
        
        self.get_logger().info(f"Setpoint position: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}, yaw={yaw_deg:.1f}")
        self.dji.requestSendGoToWPwithPID(lat, lon, alt, yaw_deg)
    
    def _setpoint_global_callback(self, msg: NavSatFix):
        """Handle global position setpoint (direct GPS coordinates)."""
        
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        
        # Use current heading as yaw
        current_yaw = self.last_telemetry.get('heading', 0.0)
        
        self.get_logger().info(f"Global setpoint: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}")
        self.dji.requestSendGoToWPwithPID(lat, lon, alt, current_yaw)
    
    def _setpoint_velocity_callback(self, msg: TwistStamped):
        """Handle velocity setpoint (mapped to virtual stick)."""
        
        # Map velocities to stick inputs (-1 to 1)
        # Assuming max velocity of ~10 m/s
        max_vel = 10.0
        
        leftX = msg.twist.angular.z / math.pi  # Yaw rate
        leftY = msg.twist.linear.z / max_vel    # Vertical velocity
        rightX = msg.twist.linear.y / max_vel   # Lateral (roll)
        rightY = msg.twist.linear.x / max_vel   # Forward (pitch)
        
        # Clamp to [-1, 1]
        leftX = max(-1.0, min(1.0, leftX))
        leftY = max(-1.0, min(1.0, leftY))
        rightX = max(-1.0, min(1.0, rightX))
        rightY = max(-1.0, min(1.0, rightY))
        
        self.dji.requestSendStick(leftX, leftY, rightX, rightY)
    
    def _setpoint_attitude_callback(self, msg: PoseStamped):
        """Handle attitude setpoint (used for gimbal control)."""
        
        # Extract euler angles from quaternion
        roll, pitch, yaw = self._quaternion_to_euler(msg.pose.orientation)
        
        # Use pitch for gimbal
        pitch_deg = math.degrees(pitch)
        self.dji.requestSendGimbalPitch(pitch_deg)
    
    # ==================== Service Callbacks ====================
    
    def _arming_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Handle arming request (DJI drones auto-arm on takeoff)."""
        
        if request.data:
            self.get_logger().info("Arm request received - DJI drones arm automatically on takeoff")
            response.success = True
            response.message = "DJI drones arm automatically. Use takeoff service to arm and takeoff."
        else:
            self.get_logger().info("Disarm request - landing drone")
            self.dji.requestSendLand()
            response.success = True
            response.message = "Land command sent"
        
        return response
    
    def _takeoff_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Handle takeoff request."""
        
        self.get_logger().info("Takeoff requested")
        result = self.dji.requestSendTakeOff()
        
        response.success = True if result else False
        response.message = "Takeoff command sent" if result else "Takeoff failed"
        
        return response
    
    def _land_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Handle land request."""
        
        self.get_logger().info("Land requested")
        result = self.dji.requestSendLand()
        
        response.success = True if result else False
        response.message = "Land command sent" if result else "Land failed"
        
        return response
    
    def _rtl_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Handle Return to Launch request."""
        
        self.get_logger().info("RTL requested")
        result = self.dji.requestSendRTH()
        
        response.success = True if result else False
        response.message = "RTL command sent" if result else "RTL failed"
        
        return response
    
    def _set_offboard_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Enable offboard (virtual stick) mode."""
        
        self.get_logger().info("Offboard mode requested - enabling virtual stick")
        result = self.dji.requestSendEnableVirtualStick()
        
        response.success = True if result else False
        response.message = "Virtual stick enabled" if result else "Failed to enable virtual stick"
        
        return response
    
    def _enable_virtual_stick_callback(self, request: Trigger.Request, response: Trigger.Response):
        """WildBridge-specific: Enable virtual stick mode."""
        
        self.get_logger().info("Enabling virtual stick")
        result = self.dji.requestSendEnableVirtualStick()
        
        response.success = True if result else False
        response.message = "Virtual stick enabled" if result else "Failed"
        
        return response
    
    def _abort_mission_callback(self, request: Trigger.Request, response: Trigger.Response):
        """WildBridge-specific: Abort current mission."""
        
        self.get_logger().info("Aborting mission")
        result = self.dji.requestAbortMission()
        
        response.success = True if result else False
        response.message = "Mission aborted" if result else "Failed to abort"
        
        return response
    
    # ==================== Helper Functions ====================
    
    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert euler angles (radians) to quaternion."""
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q
    
    def _quaternion_to_euler(self, q: Quaternion) -> tuple:
        """Convert quaternion to euler angles (roll, pitch, yaw) in radians."""
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def _quaternion_to_yaw(self, q: Quaternion) -> float:
        """Extract yaw angle from quaternion."""
        
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info("Shutting down WildBridge MAVROS Bridge...")
        self.dji.stopTelemetryStream()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = WildBridgeMavrosNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
