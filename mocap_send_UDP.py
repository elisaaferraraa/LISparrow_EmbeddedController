#!/usr/bin/env python3

import rospy
import socket
import struct
from geometry_msgs.msg import PoseStamped

class MocapUDPBridge:
    def __init__(self):
        self.udp_ip = "192.168.209.92" # Seeduino IP
        self.udp_port = 8888
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        rospy.init_node('mocap_udp_bridge', anonymous=True)

        # Latest pose
        self.latest_pose = None
        rospy.Subscriber("/vrpn_client_node/indUAV/pose", PoseStamped, self.mocap_callback)

        # Send at 100 Hz
        self.send_timer = rospy.Timer(rospy.Duration(1.0 / 100.0), self.send_udp_callback)

        rospy.loginfo("MoCap → UDP bridge initialized (100 Hz)")

    def mocap_callback(self, msg):
        self.latest_pose = msg

    def send_udp_callback(self, event):
        if self.latest_pose is None:
            return

        p = self.latest_pose.pose.position
        o = self.latest_pose.pose.orientation

        # Pack data: [x, y, z, qx, qy, qz, qw]
        packet = struct.pack('<7f', p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        self.sock.sendto(packet, (self.udp_ip, self.udp_port))

        rospy.loginfo_throttle(1.0, f"Sent pose to {self.udp_ip}:{self.udp_port}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        bridge = MocapUDPBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass



# INTRUCTIONS ON HOW TO RUN IT:
#chmod +x mocap_udp_bridge.py
#rosrun your_package_name mocap_udp_bridge.py
