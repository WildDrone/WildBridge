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
        self.create_subscription(
            Empty, 'command/deactivate_manual_override', self.deactivate_manual_override_callback, 10)

        # Subscribers for drone commands with specific messages
        self.create_subscription(
            Float64MultiArray, 'command/goto_waypoint_nose_forward', self.goto_waypoint_nose_forward_callback, 10)

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

        self.create_subscription(
            Float64, 'command/gimbal_rel_pitch', self.gimbal_rel_pitch_callback, 10)
        self.create_subscription(
            Float64, 'command/gimbal_rel_yaw', self.gimbal_rel_yaw_callback, 10)

        self.create_subscription(
            Float64MultiArray, 'command/goto_waypoint_hold_heading', self.goto_waypoint_hold_heading_callback, 10)

        # Subscribers for camera commands
        self.create_subscription(
            Empty, 'command/camera/start_recording', self.start_recording_callback, 10)
        self.create_subscription(
            Empty, 'command/camera/stop_recording', self.stop_recording_callback, 10)
        self.create_subscription(
            Empty, 'command/camera/capture', self.capture_callback, 10)
        self.create_subscription(
            Empty, 'command/camera/capture_temperature', self.capture_temperature_callback, 10)
        self.create_subscription(
            Empty, 'command/camera/list_media', self.list_media_callback, 10)
        self.create_subscription(
            String, 'command/camera/download_media', self.download_media_callback, 10)

        # Payload / LRF commands
        self.create_subscription(
            Empty, 'command/lrf/measure', self.lrf_measure_callback, 10)
        self.create_subscription(
            Empty, 'command/drop', self.drop_callback, 10)

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

        # Flight mode publisher
        self.flight_mode_pub = self.create_publisher(
            String, 'flight_mode', 10)

        # Manual override publisher
        self.manual_override_pub = self.create_publisher(
            Bool, 'manual_override_active', 10)

        # Command sequence ids streamed back with the *_reached flags. Compare against the
        # seq acked on command_ack/* to know which command a reached flag belongs to.
        self.waypoint_seq_pub = self.create_publisher(Int32, 'waypoint_seq', 10)
        self.yaw_seq_pub = self.create_publisher(Int32, 'yaw_seq', 10)
        self.altitude_seq_pub = self.create_publisher(Int32, 'altitude_seq', 10)

        # Seq the app assigned to the command we just sent (-1 if it was rejected)
        self.waypoint_ack_pub = self.create_publisher(Int32, 'command_ack/waypoint_seq', 10)
        self.yaw_ack_pub = self.create_publisher(Int32, 'command_ack/yaw_seq', 10)
        self.altitude_ack_pub = self.create_publisher(Int32, 'command_ack/altitude_seq', 10)

        # LRF / thermal / media result publishers
        self.lrf_target_pub = self.create_publisher(NavSatFix, 'lrf/target', 10)
        self.lrf_measurement_pub = self.create_publisher(String, 'lrf/measurement', 10)
        self.thermal_max_temp_pub = self.create_publisher(
            Float64, 'camera/thermal_max_temp', 10)
        self.capture_result_pub = self.create_publisher(
            String, 'camera/capture_result', 10)
        self.media_list_pub = self.create_publisher(String, 'camera/media_list', 10)
        self.download_result_pub = self.create_publisher(
            String, 'camera/download_result', 10)

        # Takeoff readiness
        self.ready_to_takeoff_pub = self.create_publisher(Bool, 'ready_to_takeoff', 10)
        self.takeoff_block_reason_pub = self.create_publisher(
            String, 'takeoff_block_reason', 10)

        # Capture / listMedia / download block for up to 2 minutes on the HTTP call, so they run
        # off the executor thread — otherwise they stall the 20 Hz telemetry timer.
        # ponytail: single worker serializes them, which is what the camera wants anyway.
        self.blocking_calls = ThreadPoolExecutor(max_workers=1)

        # Directory downloaded media is written to
        self.declare_parameter('media_dir', 'media')
        self.media_dir = self.get_parameter(
            'media_dir').get_parameter_value().string_value

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

    def deactivate_manual_override_callback(self, msg):
        self.get_logger().info("Received deactivate manual override command.")
        self.dji_interface.requestDeactivateManualOverride()

    def goto_waypoint_nose_forward_callback(self, msg: Float64MultiArray):
        """Navigate to waypoint nose-first: the drone turns to face the leg, flies forward, then
        rotates in place to `yaw` on arrival (so `yaw` is the FINAL heading, not the travel one).
        Expected: [lat, lon, alt, yaw] or [lat, lon, alt, yaw, speed]
        """
        self.get_logger().info("Received goto waypoint (nose forward) command.")
        data = msg.data
        if len(data) >= 4:
            latitude, longitude, altitude, yaw = data[:4]
            speed = data[4] if len(data) >= 5 else 5.0  # Default 5 m/s
            self.get_logger().info(
                f'Received: lat={latitude}, lon={longitude}, alt={altitude}, yaw={yaw}, speed={speed}')
        else:
            self.get_logger().warning('Received an array with fewer than 4 elements.')
            return

        seq = self.dji_interface.requestSendGoToWaypointNoseForward(
            latitude, longitude, altitude, yaw, speed)
        self.waypoint_ack_pub.publish(Int32(data=int(seq) if seq is not None else -1))

    def goto_waypoint_hold_heading_callback(self, msg: Float64MultiArray):
        """Navigate to waypoint holding `yaw` for the whole flight (body-frame projection), so the
        drone crabs sideways instead of turning to face where it is going.
        Expected: [lat, lon, alt, yaw] or [lat, lon, alt, yaw, speed]
        """
        self.get_logger().info("Received goto waypoint (hold heading) command.")
        data = msg.data
        if len(data) < 4:
            self.get_logger().warning('Received an array with fewer than 4 elements.')
            return
        latitude, longitude, altitude, yaw = data[:4]
        speed = data[4] if len(data) >= 5 else 5.0  # Default 5 m/s
        self.get_logger().info(
            f'HoldHeading: lat={latitude}, lon={longitude}, alt={altitude}, yaw={yaw}, speed={speed}')
        self.dji_interface.requestSendGoToWaypointHoldHeading(
            latitude, longitude, altitude, yaw, speed)

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

            # Flight mode
            self.flight_mode_pub.publish(
                String(data=telemetry.get('flightMode', 'UNKNOWN')))

            # Manual override state
            self.manual_override_pub.publish(
                Bool(data=telemetry.get('isManualOverrideActive', False)))

            # Command sequence ids the *_reached flags above refer to
            self.waypoint_seq_pub.publish(
                Int32(data=int(telemetry.get('waypointSeq', -1))))
            self.yaw_seq_pub.publish(Int32(data=int(telemetry.get('yawSeq', -1))))
            self.altitude_seq_pub.publish(
                Int32(data=int(telemetry.get('altitudeSeq', -1))))

            # Takeoff readiness
            self.ready_to_takeoff_pub.publish(
                Bool(data=telemetry.get('readyToTakeoff', False)))
            self.takeoff_block_reason_pub.publish(
                String(data=telemetry.get('takeoffBlockReason', 'UNKNOWN')))

            # Last LRF-locked target (only published once the LRF has locked something)
            lrf_target = telemetry.get('lrfTarget')
            if lrf_target:
                self.lrf_target_pub.publish(NavSatFix(
                    latitude=float(lrf_target.get('latitude', 0.0)),
                    longitude=float(lrf_target.get('longitude', 0.0)),
                    altitude=float(lrf_target.get('altitude', 0.0))
                ))

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
