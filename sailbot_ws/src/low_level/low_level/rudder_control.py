import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int64
import json


class RudderController(Node):
    middle_pos = 70
    port_pos = 115
    starboard_pos = 25

    def __init__(self):
        super().__init__('Rudder_Controller')
        self.publisher_ = self.create_publisher(String, 'pwm_control', 10)
        self.airmar_subscription = self.create_subscription(String, 'airmar_data', self.airmar_callback, 15)
        self.d_heading_subscription = self.create_subscription(Int64, 'd_heading', self.d_heading_callback, 15)
        # timer_period = 1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_heading = 0
        self.d_heading = 0

    def make_json_string(self, json_msg):
        json_str = json.dumps(json_msg)
        message = String()
        message.data = json_str
        return message

    def airmar_callback(self, msg):
        msg_dict = json.loads(msg.data)
        if "magnetic-sensor-heading" in msg_dict:
            self.current_heading = float(msg_dict["magnetic-sensor-heading"])

        direction_diff = self.current_heading - self.d_heading
        desired_rudder_angle = 0
        if direction_diff > 10:
            desired_rudder_angle = RudderController.port_pos
        elif direction_diff < -10:
            desired_rudder_angle = RudderController.starboard_pos
        else:
            desired_rudder_angle = RudderController.middle_pos

        rudder_json = {"channel": "8", "angle": desired_rudder_angle}
        rudder_string = self.make_json_string(rudder_json)
        self.publisher_.publish(rudder_string)
        self.get_logger().info('Publishing: "%s"' % rudder_string)

    def d_heading_callback(self, msg):
        self.d_heading = msg.data
        print("Desired Heading: ", self.d_heading)
        print("Current Heading: ", self.current_heading)


def main(args=None):
    rclpy.init(args=args)

    rudder_cont_node = RudderController()

    rclpy.spin(rudder_cont_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rudder_cont_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
