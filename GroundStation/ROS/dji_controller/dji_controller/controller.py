"""
Author: Edouard Rolland
Project: WildDrone
Contact: edr@mmmi.sdu.dk

This file was written as part of the WildDrone project and implements a ROS 2 node for controlling a DJI drone 
via the WildBridge app. The node handles both command reception and telemetry publishing.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String, Float64MultiArray, Float64, Int32, Bool
from sensor_msgs.msg import NavSatFix
from requests.exceptions import RequestException

from dji_controller.submodules.dji_interface import *
from concurrent.futures import ThreadPoolExecutor, TimeoutError
import numpy as np


class DjiNode(Node):
    def __init__(self):
        super().__init__('DjiNode')
        self.get_logger().info("Node Initialisation")

        # Retrieve the drone's IP address from the parameter server
        self.declare_parameter('ip_rc', '192.168.1.2')  # Default IP
        self.ip_rc = self.get_parameter(
            'ip_rc').get_parameter_value().string_value

        # Initialize the DJI drone interface
        self.dji_interface = DJIInterfaceLite(self.ip_rc)

        # Verify the connection to the drone
        if not self.verify_connection():
            self.get_logger().error(
                f"Unable to connect to the drone at IP: {self.ip_rc}. Shutting down node.")
            self.get_logger().info("Connection Failure")
            rclpy.shutdown()  # Shut down ROS
            return

        # Subscribers for drone commands with Empty messages
        self.create_subscription(
            Empty, 'command/takeoff', self.takeoff_callback, 10)
        self.create_subscription(Empty, 'command/land', self.land_callback, 10)
        self.create_subscription(Empty, 'command/rth', self.rth_callback, 10)
        self.create_subscription(
            Empty, 'command/abort_mission', self.abort_mission_callback, 10)
        # Abort DJI native waypoint mission (different backend than generic mission)
        self.create_subscription(
            Empty, 'command/abort_trajectory', self.abort_dji_native_mission_callback, 10)

        # Subscribers for drone commands with specific messages
        self.create_subscription(
            Float64MultiArray, 'command/goto_waypoint', self.goto_waypoint_callback, 10)

        # Trajectory: expect Float64MultiArray flattened as [lat,lon,alt, lat,lon,alt, ...]
        self.create_subscription(
            Float64MultiArray, 'command/goto_trajectory', self.goto_trajectory_callback, 10)

        self.create_subscription(
            Float64, 'command/goto_yaw', self.goto_yaw_callback, 10)
        self.create_subscription(
            Float64, 'command/goto_altitude', self.goto_altitude_callback, 10)
        self.create_subscription(
            Float64, 'command/gimbal_pitch', self.gimbal_pitch_callback, 10)
        self.create_subscription(
            Float64, 'command/gimbal_yaw', self.gimbal_yaw_callback, 10)

        # Subscribers for camera commands

        self.create_subscription(
            Empty, 'command/camera/start_recording', self.start_recording_callback, 10)
        self.create_subscription(
            Empty, 'command/camera/stop_recording', self.stop_recording_callback, 10)

        # Publishers for telemetry
        self.speed_pub = self.create_publisher(Float64, 'speed', 10)
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

        self.waypoint_reached_pub = self.create_publisher(
            Bool, 'waypoint_reached', 10)
        self.altitude_reached_pub = self.create_publisher(
            Bool, 'altitude_reached', 10)

        # Camera Publisher
        self.camera_is_recording_pub = self.create_publisher(
            Bool, 'camera/is_recording', 10)

        # Timer to publish telemetry at regular intervals
        # Publish every 1/10 second
        self.create_timer(0.1, self.publish_states)

        self.create_timer(1.0, self.publish_camera_status)

        self.get_logger().info(
            f"DroneNode initialized and connected to IP: {self.ip_rc}")

    ##############################
    # Connection Verification    #
    ##############################

    def verify_connection(self):
        """Verify the connection to the drone by sending a request to EP_BASE."""
        timeout_duration = 5  # Timeout in seconds

        def connection_attempt():
            try:
                response = self.dji_interface.requestGet("/", verbose=True)
                self.get_logger().info(f"Connection successful: {response}")
                return True
            except RequestException as e:
                self.get_logger().error(f"Connection failed: {e}")
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

    def abort_dji_native_mission_callback(self, msg):
        self.get_logger().info("Received abort DJI native mission command.")
        self.dji_interface.requestAbortDJINativeMission()

    def goto_waypoint_callback(self, msg: Float64MultiArray):
        self.get_logger().info("Received goto waypoint command.")
        data = msg.data
        if len(data) >= 4:  # Check if there are at least 4 elements
            latitude, longitude, altitude, yaw = data[:4]
            self.get_logger().info(
                f'Received: {latitude}, {longitude}, {altitude}, {yaw}')
        else:
            self.get_logger().warning('Received an array with fewer than 4 elements.')

        self.dji_interface.requestSendGoToWPwithPID(
            latitude, longitude, altitude, yaw)

    def goto_trajectory_callback(self, msg: Float64MultiArray):
        self.get_logger().info("Received goto trajectory command.")
        data = msg.data
        # Each waypoint is a group of 3 values: [lat, lon, alt]
        if len(data) % 3 != 0:
            self.get_logger().warning("Received trajectory array length is not a multiple of 3.")
        waypoints_triples = [data[i:i+3] for i in range(0, len(data), 3)]
        self.get_logger().info(f"Received waypoints: {waypoints_triples}")

        self.dji_interface.requestSendNavigateTrajectoryDJINative(
            waypoints_triples)

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
        self.dji_interface.requestSendGimbalPitch(msg.data)

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
            states = self.dji_interface.requestAllStates()

            speed = np.sqrt(float(states['speed']['x'])**2 + float(
                states['speed']['y'])**2 + float(states['speed']['z'])**2)

            self.speed_pub.publish(Float64(data=speed))
            self.heading_pub.publish(Float64(data=float(states['heading'])))
            self.attitude_pub.publish(String(data=str(states['attitude'])))

            self.location_pub.publish(NavSatFix(
                latitude=float(states['location'].get('latitude', 0.0)),
                longitude=float(states['location'].get('longitude', 0.0)),
                altitude=float(states['location'].get('altitude', 0.0))
            ))

            self.gimbal_attitude_pub.publish(
                String(data=str(states['gimbalAttitude'])))
            self.gimbal_joint_attitude_pub.publish(
                String(data=str(states['gimbalJointAttitude'])))
            self.zoom_fl_pub.publish(Float64(data=float(states['zoomFl'])))
            self.hybrid_fl_pub.publish(Float64(data=float(states['hybridFl'])))
            self.optical_fl_pub.publish(
                Float64(data=float(states['opticalFl'])))
            self.zoom_ratio_pub.publish(
                Float64(data=float(states['zoomRatio'])))
            self.battery_level_pub.publish(
                Float64(data=float(states['batteryLevel'])))
            self.satellite_count_pub.publish(
                Int32(data=int(states['satelliteCount'])))

            self.gimbal_yaw_pub.publish(
                Float64(data=float(states['gimbalAttitude'].get('yaw', 0.0))))
            self.gimbal_pitch_pub.publish(
                Float64(data=float(states['gimbalAttitude'].get('pitch', 0.0))))

            # Mission Status publishers

            wpStatus = self.dji_interface.requestWaypointStatus()
            altitudeStatus = self.dji_interface.requestAltitudeStatus()

            wpStatus = wpStatus == "true"
            altitudeStatus = altitudeStatus == "true"

            self.waypoint_reached_pub.publish(Bool(data=wpStatus))
            self.altitude_reached_pub.publish(Bool(data=altitudeStatus))

        except Exception as e:
            self.get_logger().error(f"Error while publishing states: {e}")

    def publish_camera_status(self):
        try:
            is_recording = self.dji_interface.requestCameraIsRecording()
            self.camera_is_recording_pub.publish(Bool(data=is_recording))
            # self.get_logger().info(f"Camera is_recording: {is_recording}")

        except Exception as e:
            self.get_logger().error(
                f"Error while publishing camera status: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DjiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
