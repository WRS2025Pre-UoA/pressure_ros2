import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # 画像データを受信
from std_msgs.msg import Float64  # 結果を返す
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
from pathlib import Path
from pressure import func_main

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'image_topic', self.image_callback, 10)
        self.image_publisher = self.create_publisher(Image, 'pressure_result_image', 10)
        self.value_publisher = self.create_publisher(Float64, 'pressure_result_value', 10)
        # self.result_publisher = self.create_publisher(String, 'result_topic', 10)
        self.bridge = CvBridge()

        self.clicks = []
        
    def image_callback(self, msg):
        try:

            # 画像リセット
            self.clicks = []
            
            # ROS 画像メッセージを OpenCV 画像に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            #画像の処理
            result_img,result_Pa = func_main.m1(cv_image,self.clicks)

            # 結果のサイズを計算し、テキストメッセージとしてパブリッシュ
            # result_msg = String()
            # result_msg.data = 'Complete'
            # self.result_publisher.publish(result_msg)
            if result_Pa != None:
                result_value = Float64()
                result_value.data = result_Pa
                self.value_publisher.publish(result_value)
                ros_image = self.bridge.cv2_to_imgmsg(result_img, 'bgr8')
                self.image_publisher.publish(ros_image)

        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
