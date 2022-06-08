import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
import json


class TrimTabStatePub(Node):

    def __init__(self):
        super().__init__('TrimTabStatePub')
        self.trim_tab_control_publisher_ = self.create_publisher(Int8, 'tt_control', 10)
        self.subscription = self.create_subscription(String, 'airmar_data', self.listener_callback, 15)
        timer = 30  # seconds between publish
        self.input_timer = self.create_timer(timer, self.timer_callback)
        self.lastWinds = []
        self.best_state = 4  # default to min lift

    def listener_callback(self, msg):
        """
        Just update internval variables
        :param msg: air mar message
        """
        msg_dict = json.loads(msg.data)
        if "wind-angle-relative" in msg_dict:
            self.find_trim_tab_state(float(msg_dict["wind-angle-relative"]))

    def timer_callback(self):
        """
        We only want to publish the message at regular intervals
        """
        msg = Int8()
        msg.data = self.best_state
        self.trim_tab_control_publisher_.publish(msg)

    def median(self, lst):
        """
        Get the median of a given list
        :param lst: list of numbers
        :return: median of list
        """
        n = len(lst)
        s = sorted(lst)
        return (sum(s[n//2-1:n//2+1])/2.0, s[n//2])[n % 2] if n else None

    def update_winds(self, relative_wind):
        """
        Instead of going off of the last wind direction detected, we keep a list of the previous 10 and go off of the
        median of that list
        :param relative_wind: new relative wind direction to add to the list
        :return: median angle of the last 10 wind directions
        """
        # First add wind to running list
        self.lastWinds.append(float(relative_wind))
        # keep the list at only 10
        if len(self.lastWinds) > 10:
            self.lastWinds.pop(0)
        # Now find best trim tab state
        smooth_angle = self.median(self.lastWinds)
        return smooth_angle

    def find_trim_tab_state(self, relative_wind):
        """
        Get the best trim tab state based on relative wind angle. We limit the states to send to only 2, 3, and 4 because
        those states do not take into account the wind vane on the sail, which has a tendency to move the trim tab
        erratically, especially when travelling upwind. The other states are a legacy of previous design, feel free to
        experiment.
        :param relative_wind: most recently read relative wind angle
        """
        smooth_angle = self.update_winds(relative_wind)
        self.get_logger().info(str(smooth_angle))
        if 45.0 <= smooth_angle < 135:
            # Max lift port
            self.best_state = 3
        elif 135 <= smooth_angle < 180:
            # Max drag port
            self.best_state = 2
        elif 180 <= smooth_angle < 225:
            # Max drag starboard
            self.best_state = 3
        elif 225 <= smooth_angle < 315:
            # Max lift starboard
            self.best_state = 2
        else:
            # In irons, min lift
            self.best_state = 4


def main(args=None):
    rclpy.init(args=args)

    tt_state_publisher = TrimTabStatePub()

    rclpy.spin(tt_state_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tt_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
