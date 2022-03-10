# Sailbot
Sailbot ROS code for 2021-2022 MQP

## Getting Started

You've come to the right place! Read through the sections below to learn more about setting up your development environment. If you are looking for more specific details about working with the current WPI Sailbot, be sure to check out the [Wiki](https://github.com/wpisailbot/sailbot21-22/wiki/Getting-Started).

This software runs on Python 3 and was developed for ROS Dashing Diademata (installation directions can be found here: [https://docs.ros.org/en/dashing/Installation/Linux-Install-Debians.html](https://docs.ros.org/en/dashing/Installation/Linux-Install-Debians.html)).

## Overview of the ROS Architecture

### Node and Topic Structure
The ROS workspace consists of the following nodes and topics:

Nodes:
- airmar_reader
- pwm_controller
- serial_rc_receiver
- control_system
- trim_tab_comms
- debug_interface

Topics:
- serial_rc
- pwm_control
- airmar_data
- tt_control
- tt_telemetry
- tt_battery

Node Subscriptions and Publishing:
- airmar_reader
  - publishes to `airmar_data`
- pwm_controller
  - subscribes to `pwm_control`
- serial_rc_receiver
  - publishes to `serial_rc`
- control_system
  - subscribes to `airmar_data`, `serial_rc`, `tt_telemetry`
  - publishes to `pwm_control`, `tt_control`
- trim_tab_comms
  - publishes to `tt_telemetry`, `tt_battery`
  - subscribes to `tt_control`
- debug_interface
  - subscribes to `serial_rc`, `pwm_control`, `airmar_data`, `tt_control`, `tt_telemetry`, `tt_battery`

### ROS Architecture Summary

From a high level, the nodes are set up with the following intended functionality: The `control_system` node takes in information from sensors and RC (`airmar_reader`, `serial_rc_receiver`) to form decisions, which are executed by other nodes (`pwm_controller`, `trim_tab_comms`). The `debug_interface` takes in all information from across all nodes, and relays it to a webserver which shows the information on a dashboard, as well as logs it for later use.

On a per node basis, each node does the following:

The Airmar reader handles the interpretation of all airmar information. The airmar communicates to the Maretron, which the main controller (a Jetson nano in our case) connects to over usb. The Maretron shows NMEA2000 messages which must be decoded into readable format. These messages are then published.

The PWM controller handles control of both the rudders and of the ballast. The node takes in messages with a channel and angle. Currently, the rudder is wired to channel 8, and the ballast to channel 12. The rudders move with a servo, so only an angle is needed. The ballast uses a motor controller, so the value will control speed.

The serial rc receiver node connects to a FR Sky remote control, and publishes the values from 6 channels corresponding to different parts of the controller. 

The control system takes in values and makes decisions on how to set the rudders, ballast, and trim tab based on the mode it is operating in, and the values passed to it. For example, toggling the state 1 switch to the middle position on the radio controller will enter a mode where trim tab control is automated. 

The trim tab comms node connects to the trim tab controller, over a Bluetooth Low Energy (BLE) connection and sends commands to move the trim tab as either states or manual angles. The node also receives relative wind angles and battery level information from the trim tab.

The debug interface runs with the telemetry in order to gather data and show the current status of the boat.


## ROS Dependencies

To ensure you have dependencies installed, run 
```rosdep install -i --from-path src --rosdistro dashing -y``` 
from this directory [(sailbot21-22/sailbot_ws)](/sailbot_ws).


## Running the Nodes
To run the nodes, first connect to the main controller over SSH, and navigate into the ROS workspace [(sailbot21-22/sailbot_ws)](/sailbot_ws).

Once in the workspace, you'll need to build the ROS package with the following commands:

```
colcon build --packages-select sailbot
source /opt/ros/dashing/setup.bash 
. install/setup.bash
```

Once built and sourced, you can use the `ros2` commands to start nodes. The nodes you can start and their corresponding commands are listed below:

```
ros2 run sailbot serial_rc_receiver
ros2 run sailbot pwm_controller
ros2 run sailbot trim_tab_comms
ros2 run sailbot airmar_reader
ros2 run sailbot control_system
ros2 run sailbot debug_interface
```

If you want to monitor the nodes individually, you can use multiple SSH clients each with their own node running. 


If you would like to start all of the nodes at once use:

```
ros2 launch sailbot full_launch.py
```

And to start with info messages use:

```
ros2 launch sailbot full_debug.py
```

When running the nodes via launch files, the message logging will default to only showing error messages (informational and debug messages will not be shown). The `full_debug` launch file will show these messages. If you would like to see informational or debug messages, it is recommended to run a node (or several) individually in their own terminal. Again, multiple SSH clients can be used to accomplish this, but you will need to source, build, and install the ROS package for each client. 

If you ever need to refresh your workspace (i.e. you build but it runs old code or other weird things happen), you can always delete your log, build, and install directories with `rm -rf <directory>` and then rebuild from scratch.


## Related Repositories

The software in this repository leverages connections to a number of additional subsystems on the boat. The following repositories contain the software for these subsystems:

- Trim Tab ([rigid_wing](https://github.com/wpisailbot/rigid_wing))
- Telemetry Dashboard ([telemetry](https://github.com/wpisailbot/telemetry))


## Additional Notes

Remember to rebuild and re-source every time you change the code (otherwise you will run the old code)!

You can always connect to the main controller over micro usb with serial if needed.

More information on the autonomous systems can be found in [sailbot_ws/src/sailbot/sailbot/autonomous/](/sailbot_ws/src/sailbot/sailbot/autonomous).

If you have any questions feel free to email [gr-sailbot2122@wpi.edu](mailto:gr-sailbot2122@wpi.edu).
