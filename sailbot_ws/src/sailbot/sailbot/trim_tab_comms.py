import struct
import asyncio

from bleak import BleakClient
from bleak.uuids import uuid16_dict
import rclpy
from rclpy.node import Node
from enum import Enum

from std_msgs.msg import Int8, Float32


ble_address = "F0:08:D1:CE:D8:52"  # Trim Tab controller BLE address
uuid16_dict = {v: k for k, v in uuid16_dict.items()}

# Characteristic UUIDs
APPARENT_WIND_UUID = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(
    uuid16_dict.get("Apparent Wind Direction")
)
BATTERY_UUID = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(
    uuid16_dict.get("Battery Level")
)
TAB_STATE_UUID = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(
    uuid16_dict.get("LN Feature")
)
TAB_ANGLE_UUID = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(
    uuid16_dict.get("LN Control Point")
)


class TRIM_STATE(int, Enum):
    MAX_LIFT_PORT: int = 0
    MAX_LIFT_STBD: int = 1
    MAX_DRAG_PORT: int = 2
    MAX_DRAG_STBD: int = 3
    MIN_LIFT: int = 4
    MANUAL: int = 5


# Local variables
state = TRIM_STATE.MIN_LIFT
angle = 0
wind_dir = 0.0
battery_level = 100


class TrimTabComms(Node):
    def __init__(self):
        super(TrimTabComms, self).__init__('tt_comms')

        self.tt_telemetry_publisher = self.create_publisher(Float32, 'tt_telemetry', 10)  # Wind direction
        self.tt_battery_publisher = self.create_publisher(Int8, 'tt_battery', 10)  # Battery level
        self.tt_control_subscriber = self.create_subscription(Int8, 'tt_control', self.listener_callback, 10)  # Trim tab state (currently does not support manual angle)

        # Set up timer for retrieving variables over BLE
        timer_period = 0.5  # Fetch data every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Set up BLE client and attempt to connect
        self.client = BleakClient(ble_address)
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._ble_connect())

    def disconnect(self):
        """ Disconnect from the trim tab controller """
        loop = asyncio.get_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._ble_disconnect())

    async def _ble_connect(self):
        """
        Attempt to establish a connection over BLE [Asynchronous]
        
        # TODO: If this fails, we may want to try again
        """
        try:
            await self.client.connect()
            self.get_logger().info("Connected")
        except Exception as e:
            self.get_logger().error(str(e))

    async def _ble_disconnect(self):
        """ Attempt to disconnect from BLE [Asynchronous] """
        try:
            await self.client.disconnect()
            self.get_logger().info("Disconnected")
        except Exception as e:
            self.get_logger().error(str(e))

    async def ble_read(self):
        """ Retrieve characteristic values from the trim tab controller [Asynchronous] """
        try:
            global wind_dir
            global battery_level
            wind_dir = struct.unpack('f', await self.client.read_gatt_char(APPARENT_WIND_UUID))[0]
            battery_data = await self.client.read_gatt_char(BATTERY_UUID)
            battery_level = int(battery_data[0])
        except Exception as e:
            self.get_logger().error(str(e))

    async def ble_write(self):
        """ Update the state of the trim tab controller [Asynchronous] """
        try:
            global state
            global angle
            await self.client.write_gatt_char(TAB_STATE_UUID, state.to_bytes(1, 'little'))
            if state == TRIM_STATE.MANUAL:
                await self.client.write_gatt_char(TAB_ANGLE_UUID, angle.to_bytes(1, 'little'))
        except Exception as e:
            self.get_logger().error(str(e))

    def timer_callback(self):
        """
        Handles timer event. Retrieves the new values from the trim tab controller and publishes them over ROS.
        """
        self.get_logger().debug("Attempting to read BLE variables")
        global battery_level
        global wind_dir
        
        # Read current values from trim tab controller
        loop = asyncio.get_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.ble_read())

        # Publish ROS messages
        telemetry_msg = Float32()
        battery_msg = Int8()
        telemetry_msg.data = float(wind_dir)
        battery_msg.data = battery_level
        self.tt_telemetry_publisher.publish(telemetry_msg)
        self.tt_battery_publisher.publish(battery_msg)

    def listener_callback(self, msg):
        """ Subscription handler for trim tab state commands """
        self.get_logger().debug("Sending command to trim tab controller...")
        global state
        state = msg.data
        
        # Send new state to trim tab controller
        loop = asyncio.get_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.ble_write())


def main(args=None):
    rclpy.init(args=args)

    tt_comms = TrimTabComms()

    try:
        rclpy.spin(tt_comms)

        # Clean up when we are done
        tt_comms.disconnect()
        tt_comms.destroy_node()
        rclpy.shutdown()
    except:
        tt_comms.disconnect()


if __name__ == '__main__':
    main()
