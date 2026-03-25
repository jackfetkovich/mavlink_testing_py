#!/usr/bin/env python3
"""
barebones_mavlink_ros2.py

Purpose:
- Keep your existing MAVLink script working
- Add a simple ROS 2 publisher
- Publish every MAVLink message received from QGC on a ROS 2 topic: /mavtopic

This is meant to be the FIRST step only.
No action server yet.
No custom ROS messages yet.
Just simple publishing so you can verify MAVLink -> ROS 2 flow.
"""

# Standard Python module for time handling
import time

# ROS 2 Python library
import rclpy
from builtins import getattr

# Base class for creating a ROS 2 node in Python
from rclpy.node import Node

# Standard ROS 2 message type containing a single string field: data
from std_msgs.msg import String

# pymavlink helper library for MAVLink communication
from pymavlink import mavutil


# ============================================================
#                    SYSTEM IDENTITY
# ============================================================
# These IDs identify your Raspberry Pi / MCU system on the MAVLink network.
# QGroundControl usually uses system ID 255.
MVL_SYSID = 1
MVL_COMPID = 1
GCS_SYSID = 255


# ============================================================
#                  PERIODIC SEND INTERVALS
# ============================================================
# These control how often we transmit heartbeat and status messages back.
HB_INTERVAL = 1.0          # send heartbeat every 1 second
SYS_STAT_INTERVAL = 0.1    # send status every 0.1 second


# ============================================================
#                      GLOBAL STATE
# ============================================================
mvl_armed = False
mission_upload_active = False
mission_items = -1
mission_items_idx = -1
sys_stat_count = 0


# ============================================================
#                    TIME HELPER FUNCTIONS
# ============================================================
def now_s() -> float:
    """
    Returns monotonic time in seconds.
    Monotonic means it always moves forward and is safe for interval timing.
    """
    return time.monotonic()


def millis() -> int:
    """
    Returns monotonic time in milliseconds.
    Useful for MAVLink messages like ATTITUDE_QUATERNION.
    """
    return int(now_s() * 1000)


# ============================================================
#              EXISTING MAVLINK HANDLER FUNCTIONS
# ============================================================
def handle_manual_control(m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
    """
    Handles MANUAL_CONTROL received from QGC.

    Right now, this simply echoes the same manual control values back.
    This is your existing behavior preserved.
    """
    master.mav.manual_control_send(
        target=m.target,
        x=m.x,
        y=m.y,
        z=m.z,
        r=m.r,
        buttons=m.buttons
    )


def handle_param_request_list(_m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
    """
    Handles PARAM_REQUEST_LIST from QGC.

    Here we reply with a single dummy parameter.
    This helps QGC see that the MAVLink endpoint is alive and responding.
    """
    param_id = b"a_parm"

    master.mav.param_value_send(
        param_id=param_id,
        param_value=123.456,
        param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        param_count=1,
        param_index=0
    )


def handle_mission_request_list(_m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
    """
    Handles MISSION_REQUEST_LIST from QGC.

    For now, this sends a request for mission item index 0.
    This keeps your original behavior.
    """
    master.mav.mission_request_int_send(
        target_system=MVL_SYSID,
        target_component=MVL_COMPID,
        seq=0
    )


def handle_mission_count(_m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
    """
    Handles MISSION_COUNT from QGC.

    This tells us how many mission items QGC wants to send.
    We store the count, mark upload as active, then request the first item.
    """
    global mission_upload_active
    global mission_items_idx
    global mission_items

    print(_m)

    if not mission_upload_active:
        mission_upload_active = True
        mission_items = _m.count
        print(f"# Mission items: {mission_items}")

    send_mission_request_int(_m, master)


def handle_mission_item_int(_m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile):
    """
    Handles one MISSION_ITEM_INT from QGC.

    Right now, we only print it and request the next mission item.
    Later this is where you may publish waypoint data to a dedicated ROS topic.
    """
    print(_m)
    send_mission_request_int(_m, master)


def handle_command_long(m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
    """
    Handles COMMAND_LONG sent from QGC.

    Examples:
    - request autopilot capabilities
    - arm/disarm
    - waypoint-related commands
    """
    global mvl_armed

    cmd = m.command

    # QGC is asking what this MAVLink endpoint supports
    if cmd == mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
        if int(m.param1) == 1:
            capabilities = 0
            capabilities |= mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
            capabilities |= mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MAVLINK2

            master.mav.autopilot_version_send(
                capabilities=capabilities,
                flight_sw_version=2,
                middleware_sw_version=1,
                os_sw_version=0,
                board_version=1,
                flight_custom_version=b"\x00" * 8,
                middleware_custom_version=b"\x00" * 8,
                os_custom_version=b"\x00" * 8,
                vendor_id=10101,
                product_id=20202,
                uid=0
            )

    # QGC is asking to arm or disarm
    elif cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        mvl_armed = (int(m.param1) == 1)

        master.mav.command_ack_send(
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )

    # Waypoint/nav command received
    elif cmd == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
        print("WAYPOINT COMMAND RECEIVED")

    else:
        print("OTHER CMD RECEIVED")


def send_mission_request_int(_m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile):
    """
    Requests mission items one by one from QGC.

    When all items are received, we send mission ACK.
    """
    global mission_upload_active
    global mission_items_idx
    global mission_items

    mission_items_idx += 1

    if mission_items_idx < mission_items:
        print(f"Mission item index: {mission_items_idx}")
        master.mav.mission_request_int_send(
            target_system=MVL_SYSID,
            target_component=MVL_COMPID,
            seq=mission_items_idx
        )
    else:
        print("Accepting mission...")
        master.mav.mission_ack_send(
            target_system=MVL_SYSID,
            target_component=MVL_COMPID,
            type=0
        )
        mission_upload_active = False
        mission_items_idx = -1
        mission_items = 0


# ============================================================
#              PERIODIC OUTGOING MAVLINK MESSAGES
# ============================================================
def send_heartbeat(master: mavutil.mavfile) -> None:
    """
    Send heartbeat periodically so QGC knows this MAVLink component is alive.
    """
    base_mode = (
        mavutil.mavlink.MAV_MODE_MANUAL_ARMED
        if mvl_armed else mavutil.mavlink.MAV_MODE_MANUAL_DISARMED
    )

    # Add custom mode enabled flag
    base_mode |= mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

    master.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_SUBMARINE,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        base_mode=base_mode,
        custom_mode=0xABBA,
        system_status=mavutil.mavlink.MAV_STATE_ACTIVE
    )


def send_sys_status_and_att(master: mavutil.mavfile) -> None:
    """
    Periodically send SYS_STATUS and ATTITUDE_QUATERNION.
    This helps QGC display telemetry from your system.
    """
    global sys_stat_count

    voltage_mv = sys_stat_count * 100 + 10000
    sys_stat_count = (sys_stat_count + 1) % 60

    master.mav.sys_status_send(
        onboard_control_sensors_present=0,
        onboard_control_sensors_enabled=0,
        onboard_control_sensors_health=0,
        load=0,
        voltage_battery=voltage_mv,
        current_battery=-1,
        battery_remaining=-1,
        drop_rate_comm=0,
        errors_comm=0,
        errors_count1=0,
        errors_count2=0,
        errors_count3=0,
        errors_count4=0
    )

    master.mav.attitude_quaternion_send(
        time_boot_ms=millis(),
        q1=1.0,
        q2=0.0,
        q3=0.0,
        q4=0.0,
        rollspeed=0.0,
        pitchspeed=0.0,
        yawspeed=0.0
    )


# ============================================================
#                    ROS 2 NODE DEFINITION
# ============================================================
class MavlinkRosPublisher(Node):
    """
    A very simple ROS 2 node that publishes MAVLink messages
    on the /mavtopic topic as std_msgs/String.
    """

    def __init__(self):
        # Name of this ROS 2 node
        super().__init__('mavlink_ros_publisher')

        # Create a publisher on topic /mavtopic
        # Queue size = 10 means ROS buffers up to 10 outgoing messages
        self.publisher_ = self.create_publisher(String, '/mavtopic', 10)

    def publish_mavlink_message(self, m: mavutil.mavlink.MAVLink_message):
        """
        Convert the received MAVLink message into a human-readable string
        and publish it on /mavtopic.
        """
        msg = String()

        # Build a readable string using message type, id, source, and full payload
        msg.data = (
            f"type={m.get_type()}, "
            f"msgid={m.get_msgId()}, "
            f"src_sys={m.get_srcSystem()}, "
            f"src_comp={m.get_srcComponent()}, "
            f"payload={m.to_dict()}"
        )

        # Publish the ROS 2 message
        self.publisher_.publish(msg)

        # Also log it in the terminal from the ROS node side
        self.get_logger().info(f"Published on /mavtopic: {msg.data}")


# ============================================================
#                        MAIN FUNCTION
# ============================================================
def main() -> None:
    """
    Main execution flow:

    1. Start ROS 2
    2. Create ROS 2 node
    3. Open MAVLink connection
    4. Continuously:
       - process ROS callbacks
       - receive MAVLink messages
       - publish them to ROS
       - handle MAVLink protocol logic
       - send periodic heartbeat/status
    """

    # Start ROS 2
    rclpy.init()

    # Create our publisher node
    ros_node = MavlinkRosPublisher()

    # Open MAVLink connection
    # Change this URI later if needed for serial, e.g. /dev/ttyUSB0
    master = mavutil.mavlink_connection(
        "udpout:192.168.10.2:14550",
        source_system=MVL_SYSID,
        source_component=MVL_COMPID
    )

    # Track the last time heartbeat and status were sent
    t_last_hb = now_s()
    t_last_sys = now_s()

    try:
        # Main loop runs while ROS 2 is alive
        while rclpy.ok():

            # Let ROS process callbacks once without blocking
            # Since this example only publishes, this is light-weight
            rclpy.spin_once(ros_node, timeout_sec=0.0)

            # Try to receive one MAVLink message without blocking
            m = master.recv_match(blocking=False)

            if m is not None and getattr(m, "get_srcSystem", None):

                # Only process messages coming from QGC
                if m.get_srcSystem() == GCS_SYSID:

                    # NEW PART:
                    # Publish every received MAVLink message to ROS topic /mavtopic
                    ros_node.publish_mavlink_message(m)

                    # Continue your existing MAVLink handling logic
                    mid = m.get_msgId()

                    if mid == mavutil.mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL:
                        handle_manual_control(m, master)

                    elif mid == mavutil.mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                        handle_param_request_list(m, master)

                    elif mid == mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_LONG:
                        handle_command_long(m, master)

                    elif mid == mavutil.mavlink.MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                        handle_mission_request_list(m, master)

                    elif mid == mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT:
                        # We ignore incoming heartbeat for now
                        pass

                    elif mid == mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT:
                        handle_mission_item_int(m, master)

                    elif mid == mavutil.mavlink.MAVLINK_MSG_ID_MISSION_COUNT:
                        handle_mission_count(m, master)

                    else:
                        # Print unknown/unhandled message IDs
                        print(mid)

            # Send heartbeat periodically
            t = now_s()
            if (t - t_last_hb) >= HB_INTERVAL:
                send_heartbeat(master)
                t_last_hb = t

            # Send status + attitude periodically
            if (t - t_last_sys) >= SYS_STAT_INTERVAL:
                send_sys_status_and_att(master)
                t_last_sys = t

            # Small sleep to avoid maxing CPU
            time.sleep(0.002)

    except KeyboardInterrupt:
        # Allow Ctrl+C to stop program cleanly
        pass

    finally:
        # Cleanly destroy node and shutdown ROS
        ros_node.destroy_node()
        rclpy.shutdown()


# Entry point
if __name__ == "__main__":
    main()