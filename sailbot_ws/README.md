
# General Jetson Info
- The Jetson is running a version of ubuntu 18.04 provided by Nvidia. Info on the Jetson and the OS can be found here: https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit
- The Jetson uses rc-local.service to run a script on startup which configures a few things including allowing non-root access to some serial devices and setting up the ethernet bridge connection. The sartup script can be found in the ~/ directory, and is called startup.sh. To check the status of the startup service, you can use ```systemctl status rc-local.service```. Using this to start the wifi unfortunately fails.
- The ROS 2 distribution used is Dashing Diademata, and installation directions can be found here: https://docs.ros.org/en/dashing/Installation/Linux-Install-Debians.html 
- Username is "sailbot" and password is "admin"


# Overview of the ROS Architecture

## Node and Topic Structure
The ROS workspace uses the following nodes and topics:

Nodes:
- airmar_reader
- pwm_controller
- serial_rc_receiver
- control_system
- teensy_comms
- debug_interface

Topics:
- serial_rc
- pwm_control
- airmar_data
- teensy_control
- teensy_status

Node Subscriptions and Publishing:
- airmar_reader
  - publishes to `airmar_data`
- pwm_controller
  - subscribes to `pwm_control`
- serial_rc_receiver
  - publishes to `serial_rc`
- control_system
  - subscribes to `airmar_data`, `serial_rc`, `teensy_status`
  - publishes to `pwm_control`, `teensy_control`
- teensy_comms
  - publishes to `teensy_status`
  - subscribes to `teensy_control`
- debug_interface
  - subscribes to `serial_rc`, `pwm_control`, `airmar_data`, `teensy_control`, `teensy_status`

## ROS Architecture Summary

From a high level, the nodes are set up with the following intended functionallity: The `control_system` node takes in information from sensors and RC (`airmar_reader`, `serial_rc_receiver`) to form decisions, which are executed by other nodes (`pwm_controller`, `teensy_comms`). The `debug_interface` takes in all information from across all nodes, and relays it to a webserver which shows the information on a dashboard, as well as logs it for later use.

On a per node basis, each node does the following:

The Airmar reader handes the interpretation of all airmar information. The airmar communicates to the Maretron, which the jetson connects to over usb. The Maretron shows NEMA2000 messages which must be decoded into readable format. These messages are then published

The PWM controller handles control of both the rudders and of the ballast. The node takes messages with a channel and angle. At time of writing, the rudder is wired to channel 8, and the ballast to channel 12. The rudders move with a servo, so only an angle is needed. The ballast uses a motor controller, so the value will control speed.

The serial rc receiver node connects to the FR Sky remote control, and publishes the values from 6 channels corresponding to differnet parts of the controller. 

The control system takes in values and makes decisions on how to set he rudders, ballast, and trim tab based on the mode it is opperating in, and the values passed it. For example, toggling the state 1 switch to the middle position on the radio controller will enter a mode where trim tab control is automated. 

The teensy comms node connects to the trim tab board, which was the teensy now is the MKR1010, over a open websocket and sends commands to move the trim tab as either states or manual angles. The node also recives relative wind angles back from the trim tab

The debug interface runs with the telemetry in order to gather data and show the current status of the boat.


## ROS general notes
All the code is in sailbot20-21/sailbot_ws/src/sailbot/sailbot
The path has to be this long unfortunately since we need our git repo, then our ros workspace, then our ros package, then our python package

To ensure you have dependancies installed, run 
```rosdep install -i --from-path src --rosdistro dashing -y``` 
from this directory (sailbot20-21/sailbot_ws)

# Connection Via SSH

### On PC

On PC you need this app to connect to the boat via ssh

http://ftp1.digi.com/support/utilities/changer_digi.zip

Assuming the boat is powered and the ethernet bridge is powered and plugged in, open the changer digi tool and hit search.

The tool should show two bridges. The one with IP ending in 17 is on the boat. The one ending in 20 is what you are connected to.

If you do not see either bridge, try turning off wifi and hitting search again.
If that doesn't solve, try restarting your pc/jetson, turing off your firewall, etc.

Once you see both bridges you may be able to ssh, but we've found it wont work unless you follow these steps:
1. double click the ip ending in 20, and change the "target device new parameters" to be IP:192.168.17.21. keep all else the same.
2. Hit apply
3. Then hit yes
4. Hit search again and wait for it to discover (takes a moment)
5. Then double click and change back to 192.168.17.20
6. Hit apply. This may fail. (If it does, carry on)
7. Hit apply again a second time. This should work.
8. Lastly make sure the ethernet in your network connections is a static IPv4 which is 192.168.17.20
9. Now if you run ipconfig you should see the ethernet as 192.168.17.20 and should be able to ping 192.168.17.17

Now you should be able to shh with putty at sailbot@192.168.17.20

The password is "admin"

### On Mac
<i>this is intended for those running macOS Catalina. you can find further instuctions for your version online by searching "how to manually set your IP address - Mac"</i>

On Mac, you can first connect the powered ethernet bridge to your machine. Your Mac should recognize that a new network connection is available

1. go to your network settings
2. navigate to a network with the name "USB 10/100/1000 LAN"
3. change the "Configure IPv4" setting to "Manually"
4. set the IP address to be anything on the 192.168.17 (Class C) subnet, you may not use address 17 or 20 as those are reserved for the target and the express ethernet bridge you are connected to. For my purposes, I set the IP to 192.168.17.4 as 4 is my favorite number (must be between 0 and 255)
5. set the subnet mask to be "255.255.255.0"
6. then click "apply"
7. test by pinging 192.168.17.17 (the target IP for the express ethernet bridge in the hull)

# Starting the WiFi

You will only need to run this once, but it will need to be run every time you restart the jetson.

```
sudo nmcli device wifi hotspot con-name sailbothot ssid sailbothot band bg password salad123
```


# Running the Nodes
To run the boat, first connect over SSH, and navigate into this directory (sailbot20-21/sailbot_ws)

Once in the sailbot_ws directory, you need to build the ros package with the following commands:

```
colcon build --packages-select sailbot
source /opt/ros/dashing/setup.bash 
. install/setup.bash
```

Once built and sourced, you can use the ros2 commands to start nodes. The possible nodes you can start and their corresponding commands are:

```
ros2 run sailbot serial_rc_receiver
ros2 run sailbot pwm_controller
ros2 run sailbot teensy_comms
ros2 run sailbot airmar_reader
ros2 run sailbot control_system
ros2 run sailbot debug_interface
```

If you want to moniter the nodes individually, you can use multiple ssh clients each with their own node running. 


If you would like to start all of the nodes use:

```
ros2 launch sailbot full_launch.py
```

And to start with info messages use:

```
ros2 launch sailbot full_debug.py
```

When running the nodes via launch files, the message logging will default to only showing error messages (info messages, debug messages and in code print statements will not be shown). The full_debug launch will show debug messages. If you would like to see info messages or in code print statments, it is recomended to run a node (or several) individually in their own terminal. Again, multiple ssh clients can be used to acomplish this, but you will need to source, build, and install the ros package for each client. 

If you ever need to refresh your workspace (i.e.: you build but it runs old code or other wierd things happen), you can always delete your log, build, and install directories with rm -rf <directory> and then rebuild from scratch.

# Uploading Code
To upload code, its best to connect the boat to ethernet in the lab and pull from git. There is an ethernet cable connected to a network switch in the lab (along the side wall where some old PCs are sitting) which is long enough to reach the boat. IMPORTANT: Before you can upload code, you will need to let the eth0 interface autoconfigure (by default it is configured by the startup.sh script for use with telemetry). This means you will need to modify the startup.sh file and comment out the ```ifconfig eth0 ...``` command with a #. You then must restart the jetson. This will let the boat connect over ethernet to the internet. Once you've uploaded code, be sure to uncomment the line and either reboot or run it in terminal, otherwise you wont be able to ssh into the boat with the telemetry.


# Additional Notes

Rememeber to rebuild and resource every time you change the code! (otherwise you will run the old code)

You can always connect over the micro usb with serial if needed

More information on the autonomous systems can be found in `sailbot20-21/sailbot_ws/src/sailbot/sailbot/autonomous/`

If you have questions feel free to email nick.eusman@gmail.com



