#!/usr/bin/env python3

import socket
import struct
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 9998))

while True:
    data, _ = sock.recvfrom(1024)
    if len(data) == 72:
        values = struct.unpack('<18f', data)

        # Parse fields
        roll, pitch, yaw = values[0:3]
        p, q, r = values[3:6]
        x, y, z = values[6:9]
        vx, vy, vz = values[9:12]
        u = values[12:17]

        print("\n=== Telemetry Update ===")
        print(f"Attitude       [rad]: Roll={roll:+.3f}, Pitch={pitch:+.3f}, Yaw={yaw:+.3f}")
        print(f"Angular Rates  [rad/s]: p={p:+.3f}, q={q:+.3f}, r={r:+.3f}")
        print(f"Position       [m]: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        print(f"Velocity       [m/s]: vx={vx:+.2f}, vy={vy:+.2f}, vz={vz:+.2f}")
        print(f"Actuator Inputs [-1 to 1]:")
        print("  u0 (thrust)   = {:.3f}".format(u[0]))
        print("  u1 (sweep L)  = {:.3f}".format(u[1]))
        print("  u2 (sweep R)  = {:.3f}".format(u[2]))
        print("  u3 (elevator) = {:.3f}".format(u[3]))
        print("  u4 (rudder)   = {:.3f}".format(u[4]))

    #sleep 1 ms
    time.sleep(0.05)  # Uncomment if you want to limit the rate of processing
