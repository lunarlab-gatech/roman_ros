import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ResizeCompressNode(Node):
    def __init__(self):
        super().__init__('resize_compress_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/input/image_raw',  # Change this to match your image topic
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            CompressedImage,
            '/output/image/compressed',
            10
        )

    def image_callback(self, msg):
        try:
            # Convert to OpenCV image
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize image
            resized = cv2.resize(cv_img, (256, 144), interpolation=cv2.INTER_LINEAR)

            # Encode as JPEG
            success, encoded_image = cv2.imencode('.jpg', resized)
            if not success:
                self.get_logger().error('Could not encode image')
                return

            # Fill CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = encoded_image.tobytes()

            # Publish compressed image
            self.publisher.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

def main():
    rclpy.init()
    node = ResizeCompressNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
