import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from geometry_msgs.msg import Point
import geopy.distance


class TargetFinder(Node):
    """
    Quick and dirty version of target generation, should be improved on
    """
    def __init__(self):
        super().__init__('target_finder')
        self.publisher_ = self.create_publisher(Point, 'target_position', 10)
        self.airmar_subscription = self.create_subscription(String, 'airmar_data', self.airmar_callback, 15)
        self.current_pos = Point()
        self.current_pos.x, self.current_pos.y = 0.0, 0.0
        self.acceptable_range = 5  # in ft
        # this is a bad way of doing it
        # fill these in
        point1 = Point()
        point1.x, point1.y = 42.271013, -71.755552


        point2 = Point()
        point2.x, point2.y = 0.0, 0.0

        point3 = Point()
        point3.x, point3.y = 0.0, 0.0

        self.points = [point1, point2, point3]
        self.current_target = 0

    def airmar_callback(self, msg):
        msg_dict = json.loads(msg.data)
        if "Latitude" in msg_dict:
            self.current_pos.x = float(msg_dict["Latitude"])
            self.current_pos.y = float(msg_dict["Longitude"])

            # check if we're near current target, update target if we are
            if self.calc_coord_dist(self.current_pos, self.points[self.current_target]) <= self.acceptable_range:
                self.get_logger().info("at target {}".format(self.current_target))
                self.current_target += 1
                self.current_target %= len(self.points)  # loop back to start of list

            self.publisher_.publish(self.points[self.current_target])

    def calc_coord_dist(self, point1: Point, point2: Point):
        # calc dist between gps coords
        # trust me you want to pass this off to a module
        p1 = (point1.x, point1.y)
        p2 = (point2.x, point2.y)
        return geopy.distance.distance(p1, p2).ft

def main(args=None):
    rclpy.init(args=args)

    target_publisher = TargetFinder()

    rclpy.spin(target_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    target_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
