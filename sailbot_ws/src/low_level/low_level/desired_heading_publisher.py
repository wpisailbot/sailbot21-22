import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DesiredHeading(Node):

    def __init__(self):
        super().__init__('Desired_Heading')
        self.publisher_ = self.create_publisher(String, 'd_heading', 10)
        timer_period = 1000  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        d_heading = input("Desired Heading: ")
        msg.data = d_heading
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


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