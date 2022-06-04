import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('Rudder_Test')
        self.publisher_ = self.create_publisher(String, 'pwm_control', 10)
        timer_period = 15  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def make_json_string(self, json_msg):
        json_str = json.dumps(json_msg)
        message = String()
        message.data = json_str
        return message

    def timer_callback(self):
        # msg = String()
        angle = [25, 70, 115]  # port limit, middle ground, starboard limit
        # msg.data = 'Hello World: %d' % self.i
        rudder_json = {"channel": "8", "angle": angle[self.i]}
        self.i += 1
        self.i %= 3
        rudder_string = self.make_json_string(rudder_json)
        self.publisher_.publish(rudder_string)
        self.get_logger().info('Publishing: "%s"' % rudder_string)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()