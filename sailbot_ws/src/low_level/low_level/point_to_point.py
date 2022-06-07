import math
import numpy as np
import rclpy
from rclpy.node import Node
import json

from std_msgs.msg import String
from std_msgs.msg import Int64
from geometry_msgs.msg import Point


class HeadingPublisher(Node):

    def __init__(self):
        super().__init__('heading_publisher')
        self.publisher_ = self.create_publisher(Int64, 'd_heading', 10)
        self.airmar_subscription = self.create_subscription(String, 'airmar_data', self.airmar_callback, 15)
        self.target_subscrption = self.create_subscription(Point, "target_position", self.target_callback, 15)
        # set how often heading is recalculated
        timer_period = 30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # internal variables
        self.wind_angle = 0.0
        self.target_pos = Point()  # x = Latitude, y = Longitude
        self.target_pos.x, self.target_pos.y = 0.0, 0.0
        self.current_pos = Point()  # ditto
        self.current_pos.x, self.current_pos.y = 0.0, 0.0
        self.num_headings = 20  # how many possible headings to calculate, more = more precise but takes longer to calc
        self.beating_factor = 1  # used to prevent tacking back and forth upwind, higher = less likely to tack
        self.current_target_heading = 0
        self.current_best_projection = 0

    def target_callback(self, msg):
        """
        On new target published, update internal variable
        :param msg: message from target generation node
        """
        self.target_pos = msg

    def airmar_callback(self, msg):
        """
        On airmar reading, set internal variables
        :param msg: message from airmar_reader node
        """
        msg_dict = json.loads(msg.data)
        if "Latitude" in msg_dict:
            self.current_pos.x = float(msg_dict["Latitude"])
            self.current_pos.y = float(msg_dict["Longitude"])
        elif "trueWind" in msg_dict:
            self.wind_angle = float(msg_dict["trueWind"]["direction"])

    def calc_target_angle(self):
        """
        calculate the heading straight to the target
        equation breakdown at: https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
        :return: direct heading to the target from current position
        """
        theta_a, l_a = math.radians(self.current_pos.x), math.radians(self.current_pos.y)
        theta_b, l_b = math.radians(self.target_pos.x), math.radians(self.target_pos.y)
        delta_l = l_b - l_a

        X = math.cos(theta_b) * delta_l
        Y = math.cos(theta_a) * math.sin(theta_b) - math.sin(theta_a) * math.cos(theta_b) * math.cos(delta_l)

        return math.radians(math.atan2(X, Y))

    @staticmethod
    def polar_to_cart(angle, mag):
        """
        convert polar coordinates to cartesian
        :param angle: angular component
        :param mag: radial component
        :return: x, y cartesian positions
        """
        angle = math.radians(angle)
        x = mag * math.cos(angle)
        y = mag * math.sin(angle)
        return x, y

    def calculate_heading_velocities(self):
        """
        Build the polar diagram used to determine best heading.
        The idea is to generate a certain number of possible headings, then assign them a value based on the direction
        of the wind. Currently, the magnitudes are pretty meaningless so feel free to play around with them
        :return: dictionary of possible headings, format is heading_angle: velocity
        """
        headings = {}
        for i in range(self.num_headings):
            # start at wind angle and do a full 360 to get headings
            heading = ((360.0 / self.num_headings) * i) + self.wind_angle
            heading = heading % 360
            # calc difference between heading and wind angle
            wind_diff = self.wind_angle - heading
            # get smallest possible angle difference, ex 350 (clockwise) -> -10 (counterclockwise)
            if wind_diff > 180:
                wind_diff = wind_diff - 360
            elif wind_diff < -180:
                wind_diff = wind_diff + 360
            # get absolute value of diff
            wind_diff = abs(wind_diff)

            # make shape of polar diagram
            if wind_diff <= 35:  # if the heading is directly upwind, possible velocity is 0
                vel = 0
            # elif wind_diff >= 145:  # if the heading is directly downwind, we don't want it as much
            #     vel = 180
            else:  # otherwise the velocity is constant
                vel = 200

            headings[heading] = vel

        return headings

    def calc_best_heading(self, target_angle, headings):
        """
        calculate the optimal heading to follow by projecting each possible heading onto the target vector. The heading
        with the largest projection is the quickest way to the target. The best heading is then compared to the
        current best heading * the beating parameter to avoid quickly tacking back and forth when heading directly upwind
        :param target_angle: direct heading to the target
        :param headings: dictionary of headings with the format heading angle: velocity
        :return: optimal heading angle to target
        """
        best_heading = 0
        max_projection = 0
        for heading in headings:
            # convert to numpy vectors
            target_vector = np.array(HeadingPublisher.polar_to_cart(target_angle, 330))  # magnitude is arbitrary
            velocity_vector = np.array(HeadingPublisher.polar_to_cart(heading, headings[heading]))

            # project predicted velocity onto target vector
            target_vector_norm = np.sqrt(sum(target_vector ** 2))
            proj_velocity_on_target = np.dot(velocity_vector, target_vector) / target_vector_norm

            # keep the heading that gets us there the quickest
            if proj_velocity_on_target > max_projection:
                best_heading = heading
                max_projection = proj_velocity_on_target

        # beat hysteresis, avoid constantly switching back and forth
        if self.current_best_projection * self.beating_factor > max_projection:
            return self.current_target_heading
        else:
            self.current_target_heading = best_heading
            self.current_best_projection = max_projection
            return best_heading

    def timer_callback(self):
        """
        putting it all together, gets the optimal heading and publishes it
        """
        msg = Int64()
        target_angle = self.calc_target_angle()
        headings = self.calculate_heading_velocities()
        best_heading = self.calc_best_heading(target_angle, headings)
        msg.data = int(best_heading)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    heading_publisher = HeadingPublisher()

    rclpy.spin(heading_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    heading_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()