import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageToVideoNode(Node):
    def __init__(self):
        super().__init__('image_to_video_node')
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.subscription = self.create_subscription(Image, 'EncodeImage', self.image_callback, qos_profile)
        self.subscription  # prevent unused variable warning
        self.cv_bridge = CvBridge()
        self.frames = []
        self.output_filename = 'output_video.mp4'
        self.video_writer = None

    def image_callback(self, msg):
        # cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        self.frames.append(cv_image)

    def save_video(self):
        if not self.frames:
            return

        first_frame = self.frames[0]
        height, width, _ = first_frame.shape
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 'mp4v' is a common codec for MP4
        self.video_writer = cv2.VideoWriter(self.output_filename, fourcc, 10, (width, height))

        for frame in self.frames:
            self.video_writer.write(frame)

        if self.video_writer is not None:
            self.video_writer.release()

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_to_video_node = ImageToVideoNode()
    try:
        rclpy.spin(image_to_video_node)
    except KeyboardInterrupt:
        pass
    finally:
        image_to_video_node.save_video()
        image_to_video_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
