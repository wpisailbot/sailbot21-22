import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

class BatteryMonitor(Node):

    def __init__(self):
        super().__init__('battery_monitor')
        self.publisher_ = self.create_publisher(String, 'battery_status', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Pin Definitions Need 5 board pins next to one another
        self.bat_p1 = 26  # 14V pin
        self.bat_p2 = 13  # 13V pin
        self.bat_p3 = 6  # 12V pin
        self.bat_p4 = 5  # 11V pin

        # Pin Setup:
        # Board pin-numbering scheme
        GPIO.setmode(GPIO.BCM)  # Set pins low for high inputs
        GPIO.setup(self.bat_p1, GPIO.IN)  # + 14V
        GPIO.setup(self.bat_p2, GPIO.IN)  # + 13V
        GPIO.setup(self.bat_p3, GPIO.IN)  # + 12V
        GPIO.setup(self.bat_p4, GPIO.IN)  # + 11V

    def timer_callback(self):
        msg = String()
        # #If Else Statements
        if GPIO.input(self.bat_p1):
            msg.data = "+14V you're balling"
        elif GPIO.input(self.bat_p2):
            msg.data = "+13V you're still keefin' Chief"
        elif GPIO.input(self.bat_p3):
            msg.data = "+12V recommend turn back to charge"
        elif GPIO.input(self.bat_p4):
            msg.data = "+11V Turn back Battery in danger"
        elif GPIO.input(self.bat_p1) is False and GPIO.input(self.bat_p2) is False and GPIO.input(self.bat_p3) is False and GPIO.input(self.bat_p4) is False:
            msg.data = "You done fucked up this time"
        else:
            msg.data = "Uh oh a fucky wucky happened"

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    bat_monitor = BatteryMonitor()

    rclpy.spin(bat_monitor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bat_monitor.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
    main()