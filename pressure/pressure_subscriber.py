import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # 画像データを受信
from std_msgs.msg import String  # 結果を返す
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
from pathlib import Path
import sys
sys.path.append(str(Path(__file__).resolve().parent))
import func_main

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'image_topic', self.image_callback, 10)
        # self.result_publisher = self.create_publisher(String, 'result_topic', 10)
        self.bridge = CvBridge()


    def image_callback(self, msg):
        try:

            # ROS 画像メッセージを OpenCV 画像に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            #画像の処理
            result_img,result_Pa = func_main.m1(cv_image)

            # 結果のサイズを計算し、テキストメッセージとしてパブリッシュ
            # result_msg = String()
            # result_msg.data = 'Complete'
            # self.result_publisher.publish(result_msg)

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
