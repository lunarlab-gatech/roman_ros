#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class DepthDebugger(Node):
    def __init__(self):
        super().__init__('depth_debugger')
        self.bridge = CvBridge()
        self.latest_depth = None

        # subscribe to depth perspective topic
        self.subscription = self.create_subscription(
            Image,
            '/hercules_node/Drone1/front_center_DepthPerspective/image',
            self.image_callback,
            10
        )

        # main display window
        cv2.namedWindow('Depth Perspective', cv2.WINDOW_NORMAL)
        # popup window for clicked depth
        cv2.namedWindow('Clicked Depth', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Clicked Depth', 300, 100)

        # register mouse callback on the main window
        cv2.setMouseCallback('Depth Perspective', DepthDebugger._on_mouse, self)

        # maximum depth (in meters) for clipping/display
        self.max_depth = 35.0

    @staticmethod
    def _on_mouse(event, x, y, flags, param):
        """
        Mouse callback: on left click, display the depth in a second window.
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            self = param
            if self.latest_depth is None:
                return

            h, w = self.latest_depth.shape
            if not (0 <= y < h and 0 <= x < w):
                return

            d = self.latest_depth[y, x]
            text = f"{d:.2f} m"

            # create a blank image and draw the depth text
            disp = np.zeros((100, 300, 3), dtype=np.uint8)
            cv2.putText(
                disp, text, (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 2.0,
                (255, 255, 255), thickness=3, lineType=cv2.LINE_AA
            )

            # show it
            cv2.imshow('Clicked Depth', disp)
            cv2.waitKey(1)  # refresh the popup

    # FOR COLORMAP JET DEPTH IMAGE
    # def image_callback(self, msg: Image):
    #     try:
    #         # Convert ROS2 Image (32FC1) → NumPy float32 array (meters)
    #         depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
    #         self.latest_depth = depth_image

    #         h, w = depth_image.shape
    #         cy, cx = h // 2, w // 2
    #         center_depth = depth_image[cy, cx]
    #         self.get_logger().info(f'Depth at center ({cy}, {cx}): {center_depth:.2f} m')

    #         # Log a small neighborhood
    #         for dy in (-1, 0, 1):
    #             vals = depth_image[cy + dy, cx - 2:cx + 3]
    #             formatted = ', '.join(f"{v:.2f}" for v in vals)
    #             self.get_logger().info(f'Row {cy + dy}: {formatted}')

    #         # Clip → normalize → 8-bit for display
    #         clipped = np.clip(depth_image, 0.0, self.max_depth)
    #         vis = (clipped / self.max_depth * 255.0).astype(np.uint8)
    #         vis_color = cv2.applyColorMap(vis, cv2.COLORMAP_JET)

    #         cv2.imshow('Depth Perspective', vis_color)
    #         cv2.waitKey(1)

    #     except Exception as e:
    #         self.get_logger().error(f'Failed to process depth image: {e}')


    # FOR GREYSCALE DEPTH IMAGE
    def image_callback(self, msg: Image):
        try:
            # Convert ROS2 Image (32FC1) → NumPy float32 array (meters)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.latest_depth = depth_image

            # Log center pixel depth
            h, w = depth_image.shape
            cy, cx = h // 2, w // 2
            center_depth = depth_image[cy, cx]
            self.get_logger().info(f'Depth at center ({cy}, {cx}): {center_depth:.2f} m')

            # Log a small 3×5 neighborhood around center
            for dy in (-1, 0, 1):
                vals = depth_image[cy + dy, cx - 2:cx + 3]
                formatted = ', '.join(f"{v:.2f}" for v in vals)
                self.get_logger().info(f'Row {cy + dy}: {formatted}')

            # Clip to [0, max_depth], normalize to [0,255], convert to uint8
            clipped = np.clip(depth_image, 0.0, self.max_depth)
            vis = (clipped / self.max_depth * 255.0).astype(np.uint8)

            # Display as plain greyscale
            cv2.imshow('Depth Perspective', vis)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Failed to process depth image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()