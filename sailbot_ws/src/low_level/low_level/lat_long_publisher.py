import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class TargetLocation(Node):

    def __init__(self):
        super().__init__('Target_Location')
        self.publisher = self.create_publisher(Point, 'target_position', 10)
        instant_refresh = 1
        self.input_timer = self.create_timer(instant_refresh, self.input_callback)

    def input_callback(self):
        msg = Point()
        lat = input("Desired Latitude: ")
        long = input("Desired Longitude: ")
        msg.x = float(lat)
        msg.y = float(long)
        self.publisher.publish(msg)
        self.get_logger().info("Lat: {}, Long: {}".format(lat, long))


def main(args=None):
    rclpy.init(args=args)

    target_location_publisher = TargetLocation()

    rclpy.spin(target_location_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    target_location_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
