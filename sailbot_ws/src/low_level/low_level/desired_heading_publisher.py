import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int64
import json


class DesiredHeading(Node):

    def __init__(self):
        super().__init__('Desired_Heading')
        self.publisher_ = self.create_publisher(Int64, 'd_heading', 10)
        self.subscription = self.create_subscription(String, '/airmar')
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.current_heading = 0

    def timer_callback(self):
        msg = String()
        print("Current Heading: ", self.current_heading)
        d_heading = input("Desired Heading: ")
        msg.data = int(d_heading)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

    def listener_callback(self, msg):
        msg_dict = json.loads(msg)
        if "magnetic-sensor-heading" in msg_dict:
            self.current_heading = float(msg_dict["magnetic-sensor-heading"])


def main(args=None):
    rclpy.init(args=args)

    d_heading_publisher = DesiredHeading()

    rclpy.spin(d_heading_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    d_heading_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()