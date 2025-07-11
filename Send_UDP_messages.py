import socket
import struct
import time
# python3 -m pip install pymavlink if you do not have it already installed
from pymavlink.dialects.v20 import common as mavlink

# Setup UDP socket
UDP_IP = "192.168.194.214"  # read seeduino IP once it is connected to your WiFi
UDP_PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mav = mavlink.MAVLink(None)

# Example data for Mocap


def make_mocap_example():
    """Create a mock packet to simulate sending motion capture data."""
    pos = (0.0, 10.0, 10.0)  # position x,y,z
    quat = (0.0, 0.0, 0.707, 0.707)  # quaternion x,y,z,w
    packet = struct.pack('<7f', *pos, *quat)  # Little endian, 7 floats
    return packet


def make_arm_packet(target_system=1, target_component=1, force_arm=True):
    """Build a MAVLink COMMAND_LONG packet to arm (or disarm) the vehicle."""
    param1 = 1.0 if force_arm else 0.0         # param1 = 1.0 to arm, 0.0 to disarm
    # param2 = “force arm” key (21196) or 0
    param2 = 21196.0 if force_arm else 0.0

    msg = mav.command_long_encode(
        target_system,
        target_component,
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        param1,
        param2,
        0, 0, 0, 0, 0
    )
    return msg.pack(mav)


if __name__ == "__main__":

    # Send arm command to the UAV
    arm_packet = make_arm_packet(force_arm=False)  # True = arm, False = disarm
    print(f"Sending ARM command to {UDP_IP}:{UDP_PORT}…")
    sock.sendto(arm_packet, (UDP_IP, UDP_PORT))
    print("→ ARM packet sent")
    time.sleep(1.0)

    # Loop to send test of MoCap data-like
    # while True:
    #     packet = make_mocap_example()
    #     sock.sendto(packet, (UDP_IP, UDP_PORT))
    #     time.sleep(1)  # 1 Hz
