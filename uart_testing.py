#!/usr/bin/env python3
"""
barebones_mavlink.py
Python/pymavlink port of barebones_MAVLink.ino behavior.

Test with QGroundControl by pointing QGC at the serial port (or use UDP instead).
"""

import math
import time
from pymavlink import mavutil

# ------------------- Identity (matches your Arduino sketch) -------------------
MVL_SYSID = 1
MVL_COMPID = 1

# QGC typically uses sysid=255; we filter on that like your code
GCS_SYSID = 255

# ------------------- Periods (seconds) -------------------
HB_INTERVAL = 1.0
SYS_STAT_INTERVAL = 0.1

# ------------------- State -------------------
mvl_armed = False
sys_stat_count = 0

def now_s() -> float:
    return time.monotonic()

def millis() -> int:
    return int(now_s() * 1000)

# ------------------- Handlers -------------------
def handle_manual_control(m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
    # Decode not strictly needed; message already has fields in pymavlink,
    # but we keep the same behavior: retransmit the same message back.
    master.mav.manual_control_send(
        target=m.target,
        x=m.x, y=m.y, z=m.z, r=m.r,
        buttons=m.buttons
    )

def handle_param_request_list(_m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
    # Send one fake parameter like your sketch
    param_id = b"a_parm"  # <=16 bytes; pymavlink will pad with nulls
    master.mav.param_value_send(
        param_id=param_id,
        param_value=123.456,
        param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        param_count=1,
        param_index=0
    )

def handle_mission_request_list(_m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
    # Reply that we have zero mission items
    master.mav.mission_count_send(
        target_system=MVL_SYSID,
        target_component=MVL_COMPID,
        count=0
    )

def handle_command_long(m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
    global mvl_armed

    cmd = m.command

    if cmd == mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
        # In your code: only respond if param1 == 1
        if int(m.param1) == 1:
            capabilities = 0
            capabilities |= mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
            capabilities |= mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MAVLINK2

            # AUTOPILOT_VERSION has a bunch of fields; pymavlink allows sending with required ones.
            # We'll fill what you filled and leave the rest zeroed.
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

    elif cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        mvl_armed = (int(m.param1) == 1)

        # Acknowledge (your sketch always accepts)
        master.mav.command_ack_send(
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )

# ------------------- Periodic TX -------------------
def send_heartbeat(master: mavutil.mavfile) -> None:
    base_mode = (mavutil.mavlink.MAV_MODE_MANUAL_ARMED if mvl_armed
                 else mavutil.mavlink.MAV_MODE_MANUAL_DISARMED)
    base_mode |= mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

    master.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_SUBMARINE,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        base_mode=base_mode,
        custom_mode=0xABBA,
        system_status=mavutil.mavlink.MAV_STATE_ACTIVE
    )

def send_sys_status_and_att(master: mavutil.mavfile) -> None:
    global sys_stat_count

    # Ramp 10–16 V as 10000–16000 mV over 60 ticks (like your sketch)
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

    # Your current Arduino code leaves attitude quaternion fields commented out,
    # so it's basically zeros. We'll mirror that.
    master.mav.attitude_quaternion_send(
        time_boot_ms=millis(),
        q1=1.0, q2=0.0, q3=0.0, q4=0.0,   # identity quaternion is safer than all-zeros
        rollspeed=0.0,
        pitchspeed=0.0,
        yawspeed=0.0
    )

    # Optional: uncomment to mimic your “rocking” demo
    # pfreq = 0.2
    # phaserad = 2 * math.pi * pfreq * (millis() / 1000.0)
    # maxang = 0.0873
    # roll = maxang * math.sin(phaserad)
    # pitch = maxang * math.cos(phaserad)
    # yaw = phaserad
    # cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    # cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    # cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    # q1 = cy * cr * cp + sy * sr * sp
    # q2 = cy * sr * cp - sy * cr * sp
    # q3 = cy * cr * sp + sy * sr * cp
    # q4 = sy * cr * cp - cy * sr * sp
    # master.mav.attitude_quaternion_send(
    #     time_boot_ms=millis(),
    #     q1=q1, q2=q2, q3=q3, q4=q4,
    #     rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0
    # )

# ------------------- Main loop -------------------
def main() -> None:
    # SERIAL example (most like Arduino):
    #   /dev/ttyACM0 or /dev/ttyUSB0, baud 115200
    #
    # UDP example (if you want to talk to QGC over network instead):
    #   mavutil.mavlink_connection("udpout:192.168.10.1:14550", source_system=MVL_SYSID, source_component=MVL_COMPID)

    master = mavutil.mavlink_connection("udpout:192.168.10.2:14550", source_system=1, source_component=1)
    # If you want, wait for a heartbeat from QGC (not required)
    # master.wait_heartbeat(timeout=5)

    t_last_hb = now_s()
    t_last_sys = now_s()

    while True:
        # Non-blocking receive (like reading Serial.available())
        m = master.recv_match(blocking=False)
        if m is not None and getattr(m, "get_srcSystem", None):
            if m.get_srcSystem() == GCS_SYSID:
                mid = m.get_msgId()

                if mid == mavutil.mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL:
                    handle_manual_control(m, master)
                elif mid == mavutil.mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                    handle_param_request_list(m, master)
                elif mid == mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_LONG:
                    handle_command_long(m, master)
                elif mid == mavutil.mavlink.MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                    handle_mission_request_list(m, master)

        # Periodic sends
        t = now_s()
        if (t - t_last_hb) >= HB_INTERVAL:
            send_heartbeat(master)
            t_last_hb = t

        if (t - t_last_sys) >= SYS_STAT_INTERVAL:
            send_sys_status_and_att(master)
            t_last_sys = t

        # Small sleep to avoid busy-spin; keep it short so RX stays responsive
        time.sleep(0.002)

if __name__ == "__main__":
    main()