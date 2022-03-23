import rclpy
from rclpy.node import Node
import requests

from std_msgs.msg import String, Float32, Int8

# Website URL and header #
ip = '192.168.17.20'
url = 'http://' + ip + ':3000/boat'
Headers = {'Content-type': 'application/json'}



class DebugInterface(Node):

    def __init__(self):
        super().__init__('debug_interface')
        
        # Create subcription to serial_rc topic
        self.serial_rc_subscription = self.create_subscription(
            String,
            'serial_rc',
            self.serial_rc_listener_callback,
            10)
        self.serial_rc_subscription
        
        # Create subscription to airmar_data
        self.airmar_data_subscription = self.create_subscription(
            String,
            'airmar_data',
            self.airmar_data_listener_callback,
            10)
        self.airmar_data_subscription
        
        # Create subscription to tt_telemetry
        self.trim_tab_telemetry_subscription = self.create_subscription(
            Float32,
            'tt_telemetry',
            self.trim_tab_telemetry_listener_callback,
            10)
        self.trim_tab_telemetry_subscription

        # Create subscription to tt_battery
        self.trim_tab_battery_subscription = self.create_subscription(
            Int8,
            'tt_battery',
            self.trim_tab_battery_listener_callback,
            10)
        self.trim_tab_battery_subscription

        # Create subscription to tt_control
        self.trim_tab_control_subscription = self.create_subscription(
            Int8,
            'tt_control',
            self.trim_tab_control_listener_callback,
            10)
        self.trim_tab_control_subscription

        # Create subscription to pwm_control
        self.pwm_control_subscription = self.create_subscription(
            String,
            'pwm_control',
            self.pwm_control_listener_callback,
            10)
        self.pwm_control_subscription

    def serial_rc_listener_callback(self, msg):
        self.get_logger().info('Serial msg: "%s"' % msg.data)
        requests.post(url, data=msg.data, headers=Headers)
        
    def airmar_data_listener_callback(self, msg):
        self.get_logger().info('Airmar data: "%s"' % msg.data)
        requests.post(url, data=msg.data, headers=Headers)
        # if data is in json format:
        #r = requests.post(url, data=(DATA_HERE), headers=Headers)
        # if data is in python dict format:
	#r = requests.post(url, json=(DATA_HERE))

    def trim_tab_telemetry_listener_callback(self, msg):
        self.get_logger().info('Trim Tab wind msg: "%s"' % msg.data)
        requests.post(url, data=msg.data, headers=Headers)

    def trim_tab_battery_listener_callback(self, msg):
        self.get_logger().info('Trim Tab battery msg: "%s"' % msg.data)
        requests.post(url, data=msg.data, headers=Headers)

    def trim_tab_control_listener_callback(self, msg):
        self.get_logger().info('Trim Tab control msg: "%s"' % msg.data)
        requests.post(url, data=msg.data, headers=Headers)
        
    def pwm_control_listener_callback(self, msg):
        self.get_logger().info('PWM msg: "%s"' % msg.data)
        requests.post(url, data=msg.data, headers=Headers)

    #JARIUS TBD - Add callbacks for {battery voltage, BT status, intended heading, current mag. heading from Airmar}
        #pull Trim Tab Battery - display as a percent
        #maybe convert Hull Battery Voltage to an expression (ie. GOOD, OKAY, LOW, DYING, DEAD, EMERGENCY)
    #def topic_name_listener_callback(self, msg):
        #self.get_logger().info('xxxxx msg: "%s"' % msg.data)
        #requests.post(url, data=msg.data, headers=Headers)

def main(args=None):
    rclpy.init(args=args)

    debug_interface = DebugInterface()
    #testing
    #requests.post(url, data=json.dumps({"A":"b"}), headers=Headers)
    #r = requests.post('http://192.168.17.20:3000/boat', json={"key": "value"})
    rclpy.spin(debug_interface)
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    debug_interface.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
