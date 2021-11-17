import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
import time
import sailbot.autonomous.p2p as p2p

class ControlSystem(Node): #gathers data from some nodes and distributes it to others

    def __init__(self):
        super().__init__('control_system')
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
        
        #create publisher to pwm_control
        self.pwm_control_publisher_ = self.create_publisher(String, 'pwm_control', 10)

        #create publisher to teensy_control
        self.teensy_control_publisher_ = self.create_publisher(String, 'teensy_control', 10)

        #create instance vars for subscribed topics to update
        self.serial_rc = {}
        self.airmar_data = {}
        self.teensy_status = {}

        #create instance var for keeping queue of wind data
        self.lastWinds = []
        self.p2p_alg = None
        

    def serial_rc_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)
        msg_dict = json.loads(msg.data)
        for i in msg_dict:
            self.serial_rc[i] = msg_dict[i]
        
    def airmar_data_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)
        msg_dict = json.loads(msg.data)
        for i in msg_dict:
            self.airmar_data[i] = msg_dict[i]
        
    def teensy_status_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)

        try:
            msg_dict = json.loads(msg.data)
            for i in msg_dict:
                self.teensy_status[i] = msg_dict[i]
        except Exception as e:
            self.get_logger().error(str(e))

    def updateWinds(self, relativeWind):
        #check we have new wind
        if(len(self.lastWinds) != 0 and relativeWind == self.lastWinds[len(self.lastWinds) -1]):
            return       
        #first add wind to running list
        self.lastWinds.append(float(relativeWind))
        if(len(self.lastWinds) > 10):
            self.lastWinds.pop(0)
        #now find best trim tab state
        smoothAngle = self.median(self.lastWinds)
        return smoothAngle

    def findTrimTabState(self, relativeWind):
        smoothAngle = self.updateWinds(relativeWind)
        if(smoothAngle >= 45.0 and smoothAngle < 135):
            #max lift port
            toPub = self.makeJsonString({"state":"0"})
            self.teensy_control_publisher_.publish(toPub)
        elif(smoothAngle >= 135 and smoothAngle < 180):
            #max drag port
            toPub = self.makeJsonString({"state":"2"})
            self.teensy_control_publisher_.publish(toPub)
        elif(smoothAngle >= 180 and smoothAngle < 225):
            #max drag starboard
            toPub = self.makeJsonString({"state":"3"})
            self.teensy_control_publisher_.publish(toPub)
        elif(smoothAngle >= 225 and smoothAngle < 315):
            #max lift starboard
            toPub = self.makeJsonString({"state":"1"})
            self.teensy_control_publisher_.publish(toPub)
        else:
            #in irons, min lift
            toPub = self.makeJsonString({"state":"4"})
            self.teensy_control_publisher_.publish(toPub)
        

            
    def makeJsonString(self, jsonMsg):
        json_str = json.dumps(jsonMsg)
        message = String()
        message.data = json_str
        return message

           
    def median(self, lst):
        n = len(lst)
        s = sorted(lst)
        return (sum(s[n//2-1:n//2+1])/2.0, s[n//2])[n % 2] if n else None

    def ballastAlgorithm(self):
        #check wind angle, then check current tilt of boat, then adjust ballast accordingly
        if(len(self.lastWinds) == 0):
            return
        smoothAngle = self.median(self.lastWinds)
        ballastAngle = 0
        print("roll:" + self.airmar_data["roll"]);
        if(smoothAngle > 0 and smoothAngle <= 180):#starboard tack
            #go for 20 degrees
            if(float(self.airmar_data["roll"]) > -12):
                ballastAngle = 110
            elif(float(self.airmar_data["roll"]) < -20):
                ballastAngle = 80
        elif(smoothAngle > 180 and smoothAngle < 360):#port tack
            if (float(self.airmar_data["roll"]) < 12):
                ballastAngle = 80
            elif (float(self.airmar_data["roll"]) > 20):
                ballastAngle = 110

        ballastJson = {"channel": "12", "angle": ballastAngle}
        self.pwm_control_publisher_.publish(self.makeJsonString(ballastJson))

           
def main(args=None):
    rclpy.init(args=args)

    control_system = ControlSystem()
    
    while( rclpy.ok() ):
        
        rclpy.spin_once(control_system, timeout_sec=.5)
        # now we have new vals from subscribers in:
        # control_system.serial_rc
        # control_system.airmar_data
        # control_system.teensy_status

        # need to publish new values to both control topics based on new values
        # control_system.pwm_control_publisher_.publish()
        # control_system.teensy_control_publisher_.publish()

        #TODO ^^implement
        
        
        if(len(control_system.serial_rc) < 2):
            pass #don't have rc values
        elif(float(control_system.serial_rc["state2"]) > 600): #in RC
            if(float(control_system.serial_rc["state1"]) < 400):
                #manual
                manualAngle = int((float(control_system.serial_rc["manual"]) / 2000) * 100) + 65
                toPub = control_system.makeJsonString({"state":"5","angle":manualAngle})
                control_system.teensy_control_publisher_.publish(toPub)
            elif("wind-angle-relative" in control_system.airmar_data ):
		#print(control_system.airmar_data["wind-angle-relative"])
                try:
                    control_system.findTrimTabState(control_system.airmar_data["wind-angle-relative"])
                except Exception as e:
                    control_system.get_logger().error(str(e))
            else:
                print("No wind angle values")
            if(float(control_system.serial_rc["state1"]) < 800):
                ballastAngle = 0
                if (control_system.serial_rc["ballast"] > 1200):
                    ballastAngle = 110
                elif (control_system.serial_rc["ballast"] < 800):
                    ballastAngle = 80
                ballastJson = {"channel" : "12", "angle" : ballastAngle}
                control_system.pwm_control_publisher_.publish(control_system.makeJsonString(ballastJson))
            else:
                control_system.ballastAlgorithm()
            rudderAngle = (float(control_system.serial_rc["rudder"]) / 2000 * 90) + 25
            rudderJson = {"channel": "8", "angle": rudderAngle}
            control_system.pwm_control_publisher_.publish(control_system.makeJsonString(rudderJson))
        elif(float(control_system.serial_rc["state2"]) < 600):
            destinations = [(42.277055,-71.799924),(42.276692,-71.799912)] 
            if('Latitude' in control_system.airmar_data and 'Longitude' in control_system.airmar_data):
                
                try:
                    if(control_system.p2p_alg == None): #instantiate new
                        control_system.p2p_alg = p2p.P2P((float(control_system.airmar_data['Latitude']), float(control_system.airmar_data['Longitude'])), destinations[0])

                    wind = control_system.updateWinds(control_system.airmar_data["wind-angle-relative"])
                    action = control_system.p2p_alg.getAction(wind,float(control_system.airmar_data["magnetic-sensor-heading"]),float(control_system.airmar_data["track-degrees-true"]))
                    control_system.get_logger().error(str(control_system.p2p_alg.getdistance()))
                    control_system.get_logger().error(str(action))
                    if(action['status'] == 'DONE'):
                        if(control_system.p2p_alg.dest == destinations[0]):
                            control_system.p2p_alg = p2p.P2P((control_system.airmar_data['Latitude'], control_system.airmar_data['Longitude']), destinations[1])
                        else:
                            control_system.p2p_alg = p2p.P2P((control_system.airmar_data['Latitude'], control_system.airmar_data['Longitude']), destinations[0])
                    else: #we have a non-done action (either trim tab or rudders)
                        if('tt-state' in action):
                            toPub = control_system.makeJsonString({"state" : int(action['tt-state'])})
                            control_system.teensy_control_publisher_.publish(toPub)
                        elif('rudder-angle' in action):
                            rudderJson = {"channel": "8", "angle": int(action['rudder-angle'])}
                            control_system.pwm_control_publisher_.publish(control_system.makeJsonString(rudderJson))
                        control_system.ballastAlgorithm()
                except Exception as e:
                    control_system.get_logger().error(str(e))
            else:
                control_system.get_logger().error("No latitude and longitude data")
            
            

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_system.destroy_node()
    rclpy.shutdown()








if __name__ == '__main__':
    main()
