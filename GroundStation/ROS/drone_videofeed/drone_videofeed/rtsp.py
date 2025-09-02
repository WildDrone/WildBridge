import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from collections import deque
import time
import threading

class RtspNode(Node):
    def __init__(self):
        super().__init__('RtspNode')
        self.declare_parameter('ip_rc', '192.168.8.14')  # Default IP
        self.ip_rc = self.get_parameter('ip_rc').get_parameter_value().string_value

        # Generate a unique HTTP port based on the IP address
        port_base = 8000
        try:
            ip_parts = list(map(int, self.ip_rc.split('.')))
            if len(ip_parts) != 4:
                raise ValueError("Invalid IP address format.")
        except ValueError as ve:
            self.get_logger().error(f"Invalid IP address provided: {ve}")
            raise

        self.http_port = port_base + (ip_parts[2] * 256 + ip_parts[3]) % 1000

        # Log initialization details
        self.get_logger().info(f"Node initialized with IP: {self.ip_rc}")
        self.get_logger().info(f"HTTP server accessible at: http://localhost:{self.http_port}/video_feed")

        # Connection attempt counter
        self.number_connection_attempts = 0

        # RTSP stream parameters
        self.rtsp_url = f"rtsp://aaa:aaa@{self.ip_rc}:8554/streaming/live/1"
        self.retry_delay = 1  # Seconds between retry attempts
        self.cap = None
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_buffer = deque(maxlen=1)  # Store only the latest frame

        # ROS publisher for video frames
        self.publisher = self.create_publisher(Image, "video_frames", 10)

        # Start the frame publishing thread
        self.publisher_thread = threading.Thread(target=self.publish_frames, daemon=True)
        self.publisher_thread.start()

    def connect_to_stream(self):
        """
        Connect to the RTSP stream with retry attempts.
        """
        while rclpy.ok():
            self.cap = cv2.VideoCapture(self.rtsp_url)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer size
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920 // 2)  # Reduced resolution
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080 // 2)

            if self.cap.isOpened():
                self.get_logger().info("RTSP stream opened successfully.")
                self.number_connection_attempts = 0
                return
            else:
                self.get_logger().error(f"Failed to open RTSP stream. Retrying in {self.retry_delay} seconds...")
                self.number_connection_attempts += 1
                self.cap.release()
                time.sleep(self.retry_delay)

                if self.number_connection_attempts > 5:
                    self.get_logger().error("Failed to connect after multiple attempts. Stopping.")
                    raise ConnectionError("Unable to connect to RTSP stream after multiple attempts.")

    def publish_frames(self):
        """
        Read frames from the RTSP stream, publish them as ROS Image messages, and display them in an OpenCV window.
        """
        try:
            self.connect_to_stream()
        except ConnectionError as ce:
            self.get_logger().error(f"Connection error: {ce}")
            return

        cv2.namedWindow("RTSP Video", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("RTSP Video", 640, 480)

        while rclpy.ok():
            if not self.cap.isOpened():
                self.get_logger().warn("RTSP stream is not open. Attempting to reconnect...")
                try:
                    self.connect_to_stream()
                except ConnectionError:
                    self.get_logger().error("Reconnection attempts failed. Exiting publish_frames.")
                    break

            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to read frame from RTSP stream. Attempting to reconnect...")
                self.cap.release()
                time.sleep(self.retry_delay)
                continue

            # Store the latest frame in the buffer
            self.frame_buffer.append(frame)

            # Publish the frame as a ROS message
            try:
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher.publish(image_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish frame: {e}")

            # Display the frame in the OpenCV window
            cv2.imshow("RTSP Video", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("OpenCV window close requested by user.")
                break

            time.sleep(0.01)  # High frequency for lower latency

        # Cleanup after the loop
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = RtspNode()

    try:
        rclpy.spin(node)  # Use spin to handle ROS callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Stopping.")
    except Exception as e:
        node.get_logger().error(f"An unexpected exception occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
