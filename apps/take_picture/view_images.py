#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np




class MinimalSubscriber(Node):

    def __init__(self, ):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(CompressedImage, 'freertos_picture_publisher', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning 
        self.bridge = CvBridge()
    def listener_callback(self, image_message):
        self.get_logger().info('recieved an image')
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(image_message,'8UC3')        
    	#recieve image and co nvert to cv2 image
        cv2.imshow('esp32_image', self.cv_image)
        cv2.waitKey(3)

        


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()
