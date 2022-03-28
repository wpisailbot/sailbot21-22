mport rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int64
import json
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


class RudderController(Node):
    # the rudder position extremes:
    middle_pos = 70
    max_turn_port = 115
    max_turn_starboard = 25

    def __init__(self):
        super().__init__('Rudder_Controller')
        self.publisher_ = self.create_publisher(String, 'pwm_control', 10)
        self.airmar_subscription = self.create_subscription(String, 'airmar_data', self.airmar_callback, 15)
        self.d_heading_subscription = self.create_subscription(Int64, 'd_heading', self.d_heading_callback, 15)
        # timer_period = 1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_heading = 0
        self.d_heading = 0
        self.rate_of_turn = 0
        self.current_rudder_pos = 70  # default to center

    def make_json_string(self, json_msg):
        json_str = json.dumps(json_msg)
        message = String()
        message.data = json_str
        return message

    def airmar_callback(self, msg):
        msg_dict = json.loads(msg.data)
        if "magnetic-sensor-heading" in msg_dict:
            self.current_heading = float(msg_dict["magnetic-sensor-heading"])
            self.new_heading_callback()
        elif "rate-of-turn" in msg_dict:
            self.rate_of_turn = float(msg_dict["rate-of-turn"])

    def d_heading_callback(self, msg):
        self.d_heading = msg.data
        print("Desired Heading: ", self.d_heading)
        print("Current Heading: ", self.current_heading)

    def new_heading_callback(self):
        direction_diff = self.d_heading - self.current_heading  # positive = want to turn to starboard
        self.get_logger().info('Direction Diff: {}'.format(direction_diff))

        # calc change in rudder pos with fuzzy logic
        desired_percent_change = RudderController.fuzzy_logic(direction_diff, self.rate_of_turn)

        # threshold for change so we can stay straight
        if 1.0 >= desired_percent_change >= -1.0:
            rudder_json = {"channel": "8", "angle": RudderController.middle_pos}  # publish middle pos
            self.current_rudder_pos = RudderController.middle_pos
            rudder_string = self.make_json_string(rudder_json)
            self.publisher_.publish(rudder_string)
            self.get_logger().info('Publishing: "%s"' % rudder_string)
            return  # don't do the rest of function

        self.get_logger().info('Desired Percent Change: {}'.format(desired_percent_change))
        desired_decimal_change = desired_percent_change / 100.0

        new_rudder_pos = self.current_rudder_pos - (90.0 * desired_decimal_change)  # total range of motion is 90 deg
        # make sure not to overshoot boundaries
        if new_rudder_pos > RudderController.max_turn_port:
            new_rudder_pos = RudderController.max_turn_port
        elif new_rudder_pos < RudderController.max_turn_starboard:
            new_rudder_pos = RudderController.max_turn_starboard

        # publish new rudder pos
        rudder_json = {"channel": "8", "angle": new_rudder_pos}
        self.current_rudder_pos = new_rudder_pos
        rudder_string = self.make_json_string(rudder_json)
        self.publisher_.publish(rudder_string)
        self.get_logger().info('Publishing: "%s"' % rudder_string)

    @staticmethod
    def fuzzy_logic(desired_heading_diff, rate_of_turn):
        """
        Fuzzy logic system to find desired rudder direction, notebook giving some more context for the membership
        functions and knowledge base can be found at:
        https://colab.research.google.com/drive/1Uzi_D_OQ96E0de71teNa2wVg3Dykhprw?usp=sharing

        :param desired_heading_diff: desired heading - current heading, positive means we want to turn to starboard
        :param rate_of_turn: taken from airmar, positive means currently turning to port
        :return:
        """
        # create membership function for desired direction
        desired_direction = ctrl.Antecedent(np.arange(-180, 180, 1), "desired direction")
        desired_direction["left"] = fuzz.trimf(desired_direction.universe, [-100, -50, 0])
        desired_direction["right"] = fuzz.trimf(desired_direction.universe, [0, 50, 100])
        desired_direction["strong left"] = fuzz.trapmf(desired_direction.universe, [-180, -180, -100, -50])
        desired_direction["strong right"] = fuzz.trapmf(desired_direction.universe, [50, 100, 180, 180])
        desired_direction["middle"] = fuzz.trapmf(desired_direction.universe, [-50, -15, 15, 50])

        # create membership functions for turn
        turn = ctrl.Antecedent(np.arange(-4000, 4000, 1), "turn")
        turn["neutral"] = fuzz.trimf(turn.universe, [-1200, 0, 1200])
        turn["left"] = fuzz.trapmf(turn.universe, [-4000, -4000, -1200, 0])
        turn["right"] = fuzz.trapmf(turn.universe, [0, 1200, 4000, 4000])

        # create membership functions for output
        rudder_change = ctrl.Consequent(np.arange(-20, 20, 1), "Rudder Change (% of max)")
        rudder_change["strong left"] = fuzz.trimf(rudder_change.universe, [-15, -15, -15])
        rudder_change["strong right"] = fuzz.trimf(rudder_change.universe, [15, 15, 15])
        rudder_change["left"] = fuzz.trimf(rudder_change.universe, [-5, -5, -5])
        rudder_change["right"] = fuzz.trimf(rudder_change.universe, [5, 5, 5])
        rudder_change["keep"] = fuzz.trimf(rudder_change.universe, [0, 0, 0])

        # Here we make the knowledge base, again check linked notebook for a better breakdown of this
        rule1 = ctrl.Rule(desired_direction["strong left"] & turn["left"], rudder_change["left"])
        rule2 = ctrl.Rule(desired_direction["strong left"] & turn["neutral"], rudder_change["strong left"])
        rule3 = ctrl.Rule(desired_direction["strong left"] & turn["right"], rudder_change["strong left"])
        rule4 = ctrl.Rule(desired_direction["left"] & turn["left"], rudder_change["keep"])
        rule5 = ctrl.Rule(desired_direction["left"] & turn["neutral"], rudder_change["left"])
        rule6 = ctrl.Rule(desired_direction["left"] & turn["right"], rudder_change["strong left"])
        rule7 = ctrl.Rule(desired_direction["middle"] & turn["left"], rudder_change["right"])
        rule8 = ctrl.Rule(desired_direction["middle"] & turn["neutral"], rudder_change["keep"])
        rule9 = ctrl.Rule(desired_direction["middle"] & turn["right"], rudder_change["left"])
        rule10 = ctrl.Rule(desired_direction["right"] & turn["left"], rudder_change["strong right"])
        rule11 = ctrl.Rule(desired_direction["right"] & turn["neutral"], rudder_change["right"])
        rule12 = ctrl.Rule(desired_direction["right"] & turn["right"], rudder_change["keep"])
        rule13 = ctrl.Rule(desired_direction["strong right"] & turn["left"], rudder_change["strong right"])
        rule14 = ctrl.Rule(desired_direction["strong right"] & turn["neutral"], rudder_change["strong right"])
        rule15 = ctrl.Rule(desired_direction["strong right"] & turn["right"], rudder_change["right"])

        rules = [rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14,
                 rule15]

        # create sim enviornment, this is what you plug inputs into
        rule_ctrl = ctrl.ControlSystem(rules)
        ctrl_sim = ctrl.ControlSystemSimulation(rule_ctrl)

        # Compute desired output
        ctrl_sim.input["desired direction"] = desired_heading_diff
        ctrl_sim.input["turn"] = rate_of_turn
        ctrl_sim.compute()

        # calculate desire change in output
        percent_change = ctrl_sim.output["Rudder Change (% of max)"]

        return percent_change


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
