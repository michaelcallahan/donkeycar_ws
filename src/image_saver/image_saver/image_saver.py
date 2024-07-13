#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.image_counter = 0
        self.output_dir = os.path.join(os.getcwd(), 'saved_images')
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info('ImageSaver node has been started')

    def listener_callback(self, msg):
        self.get_logger().info('Received an image')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            print(cv_image)
            image_path = os.path.join(self.output_dir, f'image_{self.image_counter:04d}.png')
            cv2.imwrite(image_path, cv_image)
            self.get_logger().info(f'Saved image {self.image_counter:04d}.png')
            self.image_counter += 1
        except Exception as e:
            self.get_logger().error(f'Failed to save image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
