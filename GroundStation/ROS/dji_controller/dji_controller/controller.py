"""
Author: Edouard Rolland
Project: WildDrone
Contact: edr@mmmi.sdu.dk

This file was written as part of the WildDrone project and implements a ROS 2 node for controlling a DJI drone 
via the WildBridge app. The node handles both command reception and telemetry publishing.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Empty, String, Float64MultiArray, Float64, Int32, Bool
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from datetime import datetime
from requests.exceptions import RequestException

from dji_controller.submodules.dji_interface import *
from concurrent.futures import ThreadPoolExecutor, TimeoutError
import numpy as np


class DjiNode(Node):
    def __init__(self):
        super().__init__('DjiNode')
        self.get_logger().info("Node Initialisation")

        # Retrieve the drone's IP address from the parameter server
        self.declare_parameter('ip_rc', '')  # Default IP (empty for auto-discovery)
        self.ip_rc = self.get_parameter(
            'ip_rc').get_parameter_value().string_value

        # Initialize the DJI drone interface
        self.dji_interface = DJIInterface(self.ip_rc)
        
        # Update IP if discovered and set the ROS2 parameter so other nodes can query it
        if not self.ip_rc and self.dji_interface.IP_RC:
            self.ip_rc = self.dji_interface.IP_RC
            # Update the ROS2 parameter so bridge can query it
            self.set_parameters([Parameter('ip_rc', Parameter.Type.STRING, self.ip_rc)])
            self.get_logger().info(f"Discovered drone at {self.ip_rc}, updated ip_rc parameter")

        # Verify the connection to the drone
        if not self.verify_connection():
            self.get_logger().error(
                f"Unable to connect to the drone at IP: {self.ip_rc}. Shutting down node.")
            self.get_logger().info("Connection Failure")
            rclpy.shutdown()  # Shut down ROS
            return

        # Start the telemetry stream (TCP socket on port 8081)
        self.dji_interface.startTelemetryStream()

        # Subscribers for drone commands with Empty messages
        self.create_subscription(
            Empty, 'command/takeoff', self.takeoff_callback, 10)
        self.create_subscription(Empty, 'command/land', self.land_callback, 10)
        self.create_subscription(Empty, 'command/rth', self.rth_callback, 10)
        self.create_subscription(
            Empty, 'command/abort_mission', self.abort_mission_callback, 10)
        self.create_subscription(
            Empty, 'command/abort_all', self.abort_all_callback, 10)
        self.create_subscription(
            Empty, 'command/enable_virtual_stick', self.enable_virtual_stick_callback, 10)
        self.create_subscription(
            Empty, 'command/abort_dji_native_mission', self.abort_dji_native_mission_callback, 10)

        # Subscribers for drone commands with specific messages
        self.create_subscription(
            Float64MultiArray, 'command/goto_waypoint', self.goto_waypoint_callback, 10)
        self.create_subscription(
            Float64MultiArray, 'command/goto_waypoint_pid_tuning', self.goto_waypoint_pid_tuning_callback, 10)

        self.create_subscription(
            String, 'command/goto_trajectory', self.goto_trajectory_callback, 10)
        self.create_subscription(
            String, 'command/goto_trajectory_dji_native', self.goto_trajectory_dji_native_callback, 10)

        self.create_subscription(
            Float64, 'command/goto_yaw', self.goto_yaw_callback, 10)
        self.create_subscription(
            Float64, 'command/goto_altitude', self.goto_altitude_callback, 10)
        self.create_subscription(
            Float64, 'command/gimbal_pitch', self.gimbal_pitch_callback, 10)
        self.create_subscription(
            Float64, 'command/gimbal_yaw', self.gimbal_yaw_callback, 10)
        self.create_subscription(
            Float64, 'command/zoom_ratio', self.zoom_ratio_callback, 10)
        self.create_subscription(
            Float64, 'command/set_rth_altitude', self.set_rth_altitude_callback, 10)
        
        # Virtual stick control subscriber (leftX, leftY, rightX, rightY)
        self.create_subscription(
            Float64MultiArray, 'command/stick', self.stick_callback, 10)

        # Subscribers for camera commands
        self.create_subscription(
            Empty, 'command/camera/start_recording', self.start_recording_callback, 10)
        self.create_subscription(
            Empty, 'command/camera/stop_recording', self.stop_recording_callback, 10)

        # Publishers for telemetry
        self.speed_pub = self.create_publisher(Float64, 'speed', 10)
        self.speed_vector_pub = self.create_publisher(Vector3, 'speed_vector', 10)
        self.heading_pub = self.create_publisher(Float64, 'heading', 10)
        self.attitude_pub = self.create_publisher(String, 'attitude', 10)
        self.location_pub = self.create_publisher(NavSatFix, 'location', 10)
        self.gimbal_attitude_pub = self.create_publisher(
            String, 'gimbal_attitude', 10)
        self.gimbal_joint_attitude_pub = self.create_publisher(
            String, 'gimbal_joint_attitude', 10)
        self.zoom_fl_pub = self.create_publisher(Float64, 'zoom_fl', 10)
        self.hybrid_fl_pub = self.create_publisher(Float64, 'hybrid_fl', 10)
        self.optical_fl_pub = self.create_publisher(Float64, 'optical_fl', 10)
        self.zoom_ratio_pub = self.create_publisher(Float64, 'zoom_ratio', 10)
        self.battery_level_pub = self.create_publisher(
            Float64, 'battery_level', 10)
        self.satellite_count_pub = self.create_publisher(
            Int32, 'satellite_count', 10)

        self.gimbal_yaw_pub = self.create_publisher(Float64, 'gimbal_yaw', 10)
        self.gimbal_pitch_pub = self.create_publisher(
            Float64, 'gimbal_pitch', 10)

        # Mission status publishers
        self.waypoint_reached_pub = self.create_publisher(
            Bool, 'waypoint_reached', 10)
        self.intermediary_waypoint_reached_pub = self.create_publisher(
            Bool, 'intermediary_waypoint_reached', 10)
        self.altitude_reached_pub = self.create_publisher(
            Bool, 'altitude_reached', 10)
        self.yaw_reached_pub = self.create_publisher(
            Bool, 'yaw_reached', 10)

        # Home location publishers
        self.home_location_pub = self.create_publisher(
            NavSatFix, 'home_location', 10)
        self.home_set_pub = self.create_publisher(
            Bool, 'home_set', 10)
        self.distance_to_home_pub = self.create_publisher(
            Float64, 'distance_to_home', 10)

        # Flight time publishers
        self.remaining_flight_time_pub = self.create_publisher(
            Float64, 'remaining_flight_time', 10)
        self.time_needed_to_go_home_pub = self.create_publisher(
            Float64, 'time_needed_to_go_home', 10)
        self.time_needed_to_land_pub = self.create_publisher(
            Float64, 'time_needed_to_land', 10)
        self.time_to_landing_spot_pub = self.create_publisher(
            Float64, 'time_to_landing_spot', 10)
        self.max_radius_can_fly_and_go_home_pub = self.create_publisher(
            Float64, 'max_radius_can_fly_and_go_home', 10)
        
        # Battery needed publishers
        self.battery_needed_to_go_home_pub = self.create_publisher(
            Float64, 'battery_needed_to_go_home', 10)
        self.battery_needed_to_land_pub = self.create_publisher(
            Float64, 'battery_needed_to_land', 10)

        # Camera Publisher
        self.camera_is_recording_pub = self.create_publisher(
            Bool, 'camera/is_recording', 10)

        # Timer to publish telemetry at regular intervals
        # Publish every 1/20 second (50ms)
        self.create_timer(0.05, self.publish_states)

        self.get_logger().info(
            f"DroneNode initialized and connected to IP: {self.ip_rc}")

    ##############################
    # Connection Verification    #
    ##############################

    def verify_connection(self):
        """Verify the connection to the drone by sending a test request."""
        timeout_duration = 5  # Timeout in seconds

        def connection_attempt():
            try:
                # Try to get config to verify connection (cleaner than probing /)
                config = get_config(self.ip_rc)
                if config:
                    self.get_logger().info(f"Connection verified. Drone config: {config}")
                    return True
                
                # Fallback to old method if config fails but maybe server is up
                response = self.dji_interface.requestSend("/", "", verbose=False)
                if response:
                    self.get_logger().info("Connection verified (via fallback probe).")
                    return True
                return False
            except RequestException as e:
                self.get_logger().error(f"Connection failed: {e}")
                return False
            except Exception as e:
                self.get_logger().error(f"Connection failed with unexpected error: {e}")
                return False

        with ThreadPoolExecutor(max_workers=1) as executor:
            future = executor.submit(connection_attempt)
            try:
                return future.result(timeout=timeout_duration)
            except TimeoutError:
                self.get_logger().error(
                    f"Connection to {self.ip_rc} timed out after {timeout_duration} seconds.")
                return False

    ################################
    # Callbacks for drone commands #
    ################################

    def takeoff_callback(self, msg):
        self.get_logger().info("Received takeoff command.")
        self.dji_interface.requestSendTakeOff()

    def land_callback(self, msg):
        self.get_logger().info("Received land command.")
        self.dji_interface.requestSendLand()

    def rth_callback(self, msg):
        self.get_logger().info("Received return to home command.")
        self.dji_interface.requestSendRTH()

    def abort_mission_callback(self, msg):
        self.get_logger().info("Received abort mission command.")
        self.dji_interface.requestAbortMission()

    def abort_all_callback(self, msg):
        self.get_logger().info("Received abort ALL command - stopping all missions.")
        self.dji_interface.requestAbortAll()

    def enable_virtual_stick_callback(self, msg):
        self.get_logger().info("Received enable virtual stick command.")
        self.dji_interface.requestSendEnableVirtualStick()

    def abort_dji_native_mission_callback(self, msg):
        self.get_logger().info("Received abort DJI native mission command.")
        self.dji_interface.requestAbortDJINativeMission()

    def goto_waypoint_callback(self, msg: Float64MultiArray):
        """Navigate to waypoint with PID control.
        Expected: [lat, lon, alt, yaw] or [lat, lon, alt, yaw, speed]
        """
        self.get_logger().info("Received goto waypoint command.")
        data = msg.data
        if len(data) >= 4:
            latitude, longitude, altitude, yaw = data[:4]
            speed = data[4] if len(data) >= 5 else 5.0  # Default 5 m/s
            self.get_logger().info(
                f'Received: lat={latitude}, lon={longitude}, alt={altitude}, yaw={yaw}, speed={speed}')
        else:
            self.get_logger().warning('Received an array with fewer than 4 elements.')
            return

        self.dji_interface.requestSendGoToWPwithPID(
            latitude, longitude, altitude, yaw, speed)

    def goto_waypoint_pid_tuning_callback(self, msg: Float64MultiArray):
        """Navigate to waypoint with custom PID tuning parameters.
        Expected: [lat, lon, alt, yaw, kp_pos, ki_pos, kd_pos, kp_yaw, ki_yaw, kd_yaw]
        """
        self.get_logger().info("Received goto waypoint with PID tuning command.")
        data = msg.data
        if len(data) >= 10:
            lat, lon, alt, yaw, kp_pos, ki_pos, kd_pos, kp_yaw, ki_yaw, kd_yaw = data[:10]
            self.get_logger().info(
                f'Waypoint: ({lat}, {lon}, {alt}), Yaw: {yaw}, PID_pos: ({kp_pos}, {ki_pos}, {kd_pos}), PID_yaw: ({kp_yaw}, {ki_yaw}, {kd_yaw})')
            self.dji_interface.requestSendGoToWPwithPIDtuning(
                lat, lon, alt, yaw, kp_pos, ki_pos, kd_pos, kp_yaw, ki_yaw, kd_yaw)
        else:
            self.get_logger().warning('Received an array with fewer than 10 elements for PID tuning.')

    def goto_trajectory_callback(self, msg: String):
        """Navigate through a trajectory.
        Expected format: list of (lat, lon, alt) tuples with optional final yaw.
        Example: "[(lat1,lon1,alt1), (lat2,lon2,alt2), ...], finalYaw" or
                 "[(lat1,lon1,alt1), (lat2,lon2,alt2), ...]"
        """
        self.get_logger().info("Received goto trajectory command.")
        data = ast.literal_eval(msg.data)
        
        # Handle both formats: just waypoints list, or (waypoints, finalYaw) tuple
        if isinstance(data, tuple) and len(data) == 2:
            waypoints, finalYaw = data
        else:
            waypoints = data
            finalYaw = 0.0  # Default yaw if not provided
        
        self.get_logger().info(f"Received waypoints: {waypoints}, finalYaw: {finalYaw}")
        self.dji_interface.requestSendNavigateTrajectory(waypoints, finalYaw)

    def goto_trajectory_dji_native_callback(self, msg: String):
        """Navigate using DJI's native waypoint mission system.
        Expected format: "(speed, [(lat, lon, alt), (lat, lon, alt), ...])"
        or legacy format: "[(lat, lon, alt), (lat, lon, alt), ...]"
        """
        self.get_logger().info("Received DJI native trajectory command.")
        data = ast.literal_eval(msg.data)
        
        # Support both formats: (speed, waypoints) tuple or just waypoints list
        if isinstance(data, tuple) and len(data) == 2:
            speed, waypoints = data
        else:
            # Legacy format: just waypoints, use default speed
            waypoints = data
            speed = 10.0
        
        self.get_logger().info(f"Received DJI native waypoints: {waypoints}, speed: {speed} m/s")
        self.dji_interface.requestSendNavigateTrajectoryDJINative(waypoints, speed)

    def goto_yaw_callback(self, msg):
        self.get_logger().info("Received goto yaw command.")
        self.dji_interface.requestSendGotoYaw(msg.data)

    def goto_altitude_callback(self, msg):
        self.get_logger().info("Received goto altitude command.")
        self.dji_interface.requestSendGotoAltitude(msg.data)

    def gimbal_pitch_callback(self, msg):
        self.get_logger().info("Received gimbal pitch command.")
        self.dji_interface.requestSendGimbalPitch(msg.data)

    def gimbal_yaw_callback(self, msg):
        self.get_logger().info("Received gimbal yaw command.")
        self.dji_interface.requestSendGimbalYaw(msg.data)

    def zoom_ratio_callback(self, msg):
        self.get_logger().info("Received zoom ratio command.")
        self.dji_interface.requestSendZoomRatio(msg.data)

    def set_rth_altitude_callback(self, msg):
        self.get_logger().info("Received set RTH altitude command.")
        self.dji_interface.requestSetRTHAltitude(msg.data)

    def stick_callback(self, msg: Float64MultiArray):
        """Virtual stick control. Expected: [leftX, leftY, rightX, rightY] in range [-1, 1]."""
        data = msg.data
        if len(data) >= 4:
            leftX, leftY, rightX, rightY = data[:4]
            self.dji_interface.requestSendStick(leftX, leftY, rightX, rightY)
        else:
            self.get_logger().warning('Stick command requires 4 values: leftX, leftY, rightX, rightY')

    def start_recording_callback(self, msg):
        self.get_logger().info("Received start recording command.")
        response = self.dji_interface.requestCameraStartRecording()
        if response:
            self.get_logger().info("Camera recording started successfully.")
        else:
            self.get_logger().error("Failed to start camera recording.")

    def stop_recording_callback(self, msg):
        self.get_logger().info("Received stop recording command.")
        response = self.dji_interface.requestCameraStopRecording()
        if response:
            self.get_logger().info("Camera recording stopped successfully.")
        else:
            self.get_logger().error("Failed to stop camera recording.")

    ##############################
    # Telemetry Publishers       #
    ##############################

    def publish_states(self):
        try:
            # Get telemetry from TCP socket stream
            telemetry = self.dji_interface.getTelemetry()
            
            if not telemetry:
                return  # No telemetry data available yet
            
            # Speed (scalar and vector)
            speed_data = telemetry.get('speed', {})
            speed_x = float(speed_data.get('x', 0.0))
            speed_y = float(speed_data.get('y', 0.0))
            speed_z = float(speed_data.get('z', 0.0))
            speed = np.sqrt(speed_x**2 + speed_y**2 + speed_z**2)
            
            self.speed_pub.publish(Float64(data=speed))
            self.speed_vector_pub.publish(Vector3(x=speed_x, y=speed_y, z=speed_z))
            
            # Heading
            self.heading_pub.publish(Float64(data=float(telemetry.get('heading', 0.0))))
            
            # Attitude
            self.attitude_pub.publish(String(data=str(telemetry.get('attitude', {}))))
            
            # Location
            location = telemetry.get('location', {})
            self.location_pub.publish(NavSatFix(
                latitude=float(location.get('latitude', 0.0)),
                longitude=float(location.get('longitude', 0.0)),
                altitude=float(location.get('altitude', 0.0))
            ))
            
            # Gimbal
            gimbal_attitude = telemetry.get('gimbalAttitude', {})
            self.gimbal_attitude_pub.publish(String(data=str(gimbal_attitude)))
            self.gimbal_joint_attitude_pub.publish(
                String(data=str(telemetry.get('gimbalJointAttitude', {}))))
            self.gimbal_yaw_pub.publish(
                Float64(data=float(gimbal_attitude.get('yaw', 0.0))))
            self.gimbal_pitch_pub.publish(
                Float64(data=float(gimbal_attitude.get('pitch', 0.0))))
            
            # Camera zoom
            self.zoom_fl_pub.publish(Float64(data=float(telemetry.get('zoomFl', -1))))
            self.hybrid_fl_pub.publish(Float64(data=float(telemetry.get('hybridFl', -1))))
            self.optical_fl_pub.publish(Float64(data=float(telemetry.get('opticalFl', -1))))
            self.zoom_ratio_pub.publish(Float64(data=float(telemetry.get('zoomRatio', 1.0))))
            
            # Battery and satellites
            self.battery_level_pub.publish(
                Float64(data=float(telemetry.get('batteryLevel', -1))))
            self.satellite_count_pub.publish(
                Int32(data=int(telemetry.get('satelliteCount', -1))))
            
            # Mission status (using new telemetry-based methods)
            self.waypoint_reached_pub.publish(
                Bool(data=telemetry.get('waypointReached', False)))
            self.intermediary_waypoint_reached_pub.publish(
                Bool(data=telemetry.get('intermediaryWaypointReached', False)))
            self.altitude_reached_pub.publish(
                Bool(data=telemetry.get('altitudeReached', False)))
            self.yaw_reached_pub.publish(
                Bool(data=telemetry.get('yawReached', False)))
            
            # Home location
            home_location = telemetry.get('homeLocation', {})
            self.home_location_pub.publish(NavSatFix(
                latitude=float(home_location.get('latitude', 0.0)),
                longitude=float(home_location.get('longitude', 0.0)),
                altitude=0.0  # Home location typically doesn't include altitude
            ))
            self.home_set_pub.publish(Bool(data=telemetry.get('homeSet', False)))
            self.distance_to_home_pub.publish(
                Float64(data=float(telemetry.get('distanceToHome', 0.0))))
            
            # Flight time information
            self.remaining_flight_time_pub.publish(
                Float64(data=float(telemetry.get('remainingFlightTime', 0))))
            self.time_needed_to_go_home_pub.publish(
                Float64(data=float(telemetry.get('timeNeededToGoHome', 0))))
            self.time_needed_to_land_pub.publish(
                Float64(data=float(telemetry.get('timeNeededToLand', 0))))
            self.time_to_landing_spot_pub.publish(
                Float64(data=float(telemetry.get('totalTime', 0))))
            self.max_radius_can_fly_and_go_home_pub.publish(
                Float64(data=float(telemetry.get('maxRadiusCanFlyAndGoHome', 0))))
            
            # Battery needed information
            self.battery_needed_to_go_home_pub.publish(
                Float64(data=float(telemetry.get('batteryNeededToGoHome', 0))))
            self.battery_needed_to_land_pub.publish(
                Float64(data=float(telemetry.get('batteryNeededToLand', 0))))
            
            # Camera recording status
            self.camera_is_recording_pub.publish(
                Bool(data=telemetry.get('isRecording', False)))

        except Exception as e:
            self.get_logger().error(f"Error while publishing states: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DjiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
