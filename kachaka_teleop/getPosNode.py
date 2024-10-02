import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry



class GetPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("get_pos_node")

        qos_profile = rclpy.qos.qos_profile_sensor_data
        # Set Subscriber
        self._lidar_subscriber = self.create_subscription(
            Odometry, "/kachaka/odometry/odometry", self.callback, qos_profile
        )

        # Set Action client

        self.get_logger().info('Initialized')


    def callback(self, msg: Odometry) -> None:
        print(msg.pose.pose.position)
        

def main(args = None):
    rclpy.init(args = args)
    teleop = GetPoseNode()
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
