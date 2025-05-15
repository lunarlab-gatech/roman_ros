import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from diagnostic_updater import Updater, DiagnosticStatusWrapper
import cv_bridge

import cv2
import numpy as np
import time

class LowLatencyViewer(Node):
    def __init__(self):
        super().__init__('low_latency_image_viewer')

        self.sub = self.create_subscription(
            Image,
            '/input/image_raw',
            self.image_callback,
            qos_profile=10
        )

        # Declare and get ROS parameters
        self.declare_parameter('robot', 'Drone1')
        self.declare_parameter('expected_fps', 25.0)
        self.declare_parameter('bag_play_rate', 1.0)

        self.robot_name = self.get_parameter("robot").value
        self.expected_fps = self.get_parameter("expected_fps").value
        self.bag_play_rate = self.get_parameter("bag_play_rate").value

        self.window_name = "Annotated Image - " + str(self.robot_name)

        self.updater = Updater(self)
        self.updater.setHardwareID(self.get_fully_qualified_name())
        self.updater.add("ImageView", self.diagnostic_callback)
        self.updater_timer = self.create_timer(1 * self.bag_play_rate / 20, self.updater.update)

        self.recieved_times = []
        self.start_time = 0
        self.bridge = cv_bridge.CvBridge()

        # Plot a blank image
        self.blank_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.imshow(self.window_name, self.blank_image)
        cv2.waitKey(1)

    def image_callback(self, msg: Image):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if image is not None:
            # Reduce the image size by half
            image = cv2.resize(image, (image.shape[1] // 2, image.shape[0] // 2), interpolation=cv2.INTER_LINEAR)
            cv2.imshow(self.window_name, image)
            key = cv2.waitKey(1)
        else:
            self.get_logger().warn("Failed to decode compressed image")

        self.recieved_times.append(self.get_clock().now())

    def diagnostic_callback(self, stat: DiagnosticStatusWrapper):
        # Remove all entries in self.recieved_times that are older than 1 second
        time_float = self.get_clock().now().nanoseconds / 1e9
        self.recieved_times = [t for t in self.recieved_times if (t.nanoseconds / 1e9) > time_float - 1]

        # Calculate Display FPS
        while self.start_time == 0:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            time.sleep(0.1)

        window_size = np.min([1.0, (self.get_clock().now().nanoseconds / 1e9) - self.start_time])
        fps = len(self.recieved_times) / window_size

        # Based on the FPS, set the status
        if fps < self.expected_fps * 0.4:
            stat.summary(DiagnosticStatusWrapper.ERROR, "Display FPS below 40% of the expected range")
        elif fps < self.expected_fps * 0.8:
            stat.summary(DiagnosticStatusWrapper.WARN, "Display FPS below 80% of the expected range")
        else:
            stat.summary(DiagnosticStatusWrapper.OK, "Display FPS within expected range")
        stat.add("FPS", str(fps))
        stat.add("Expected FPS", str(self.expected_fps))
        stat.add("Window Size", str(window_size))
        return stat
    
def main(args=None):
    rclpy.init(args=args)
    node = LowLatencyViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()