import rclpy
from rclpy.node import Node
import socket
import json
import requests

from std_msgs.msg import String

# Website URL and header #
ip = '192.168.17.20'
url = 'http://' + ip + ':3000/boat'
Headers = {'Content-type': 'application/json'}



class DebugInterface(Node):

    def __init__(self):
        super().__init__('debug_interface')
        
        #create subcription to serial_rc topic
        self.serial_rc_subscription = self.create_subscription(
            String,
            'serial_rc',
            self.serial_rc_listener_callback,
            10)
        self.serial_rc_subscription
        
        #create subscription to airmar_data
        self.airmar_data_subscription = self.create_subscription(
            String,
            'airmar_data',
            self.airmar_data_listener_callback,
            10)
        self.airmar_data_subscription
        
        #create subscription to teensy_status
        self.teensy_status_subscription = self.create_subscription(
            String,
            'teensy_status',
            self.teensy_status_listener_callback,
            10)
        self.teensy_status_subscription

        #create subscription to teensy_control
        self.teensy_control_subscription = self.create_subscription(
            String,
            'teensy_control',
            self.teensy_control_listener_callback,
            10)
        self.teensy_control_subscription

        #create subscription to pwm_control
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

    def teensy_status_listener_callback(self, msg):
        self.get_logger().info('Teensy msg: "%s"' % msg.data)
        requests.post(url, data=msg.data, headers=Headers)

    def teensy_control_listener_callback(self, msg):
        self.get_logger().info('Teensy msg: "%s"' % msg.data)
        requests.post(url, data=msg.data, headers=Headers)
        
    def pwm_control_listener_callback(self, msg):
        self.get_logger().info('PWM msg: "%s"' % msg.data)
        requests.post(url, data=msg.data, headers=Headers)


    

        

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
