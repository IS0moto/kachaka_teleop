import os
import cv2
import numpy as np
from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CompressedImage2Dataset(Node):
    def __init__(self):
        super().__init__('compressedImage2dataset')

        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.subscription = self.create_subscription(
            Image,
            '/Head/camera/rgb/image_raw',
            self.cb_image,
            qos_profile
        )
        self.count = 0
        self.bride = CvBridge()
        self.image_folder = 'images'
        self.date = datetime.now().strftime("%Y%m%d%H%M%S")
        if not os.path.exists(os.path.join(self.image_folder, self.date)):
            os.makedirs(os.path.join(self.image_folder, self.date))
    
    def cb_image(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_ANYCOLOR)
        image_np = self.bride.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.count < 10:
            image_name = f'image_0000{self.count}.png'
        elif self.count < 100:
            image_name = f'image_000{self.count}.png'
        elif self.count < 1000:
            image_name = f'image_00{self.count}.png'
        elif self.count < 10000:
            image_name = f'image_0{self.count}.png'
        else:
            image_name = f'image_{self.count}.png'
        image_path = os.path.join(self.image_folder, self.date, image_name)
        cv2.imwrite(image_path, image_np)

        self.count += 1

def main(args=None):
    rclpy.init(args=args)

    ci2d = CompressedImage2Dataset()
    rclpy.spin(ci2d)

    ci2d.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()