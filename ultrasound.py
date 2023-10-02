#!/usr/bin/env python3

from getch import getch
import rtde_control
import rtde_receive
from math import pi

from getch import getch

import time

rtde_c = rtde_control.RTDEControlInterface("192.168.0.100")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.100")

tf = [0, 0, 0, 0, 1.0472, 0]
selection_vector = [0, 0, 1, 0, 0, 0]
wrench_down = [0, 0, -9, 0, 0, 0]
zero = [0, 0, 0, 0, 0, 0]
wrench_up = [0, 0, 9, 0, 0, 0]
force_type = 2
limits = [2, 2, 0.2, 1, 1, 1]
dt = 1.0/500  # 2ms
b=[-0.47956611978987035, -0.15146267999392335, 0.5069463782918725, 0.5833426813796129, -0.667834347842564, -1.360682483587835]


rtde_c.setTcp([0, -0.055, 0.058, 0, 0, 0]) #Setting tool center point of robot's end-effector

variables = ["timestamp", "actual_TCP_pose", "actual_TCP_force"]

rtde_r.startFileRecording("IECBES.csv", variables)
print("Data recording started")

p=rtde_r.getActualTCPPose()


rtde_c.moveL(b, 1, 0.1, False) #move in vector position

s = [-0.03, 0, -0.02, 0, 0, 0]
rtde_c.moveUntilContact(s) #move until robot's end-effector is touching any object

for i in range(6100):
    start = time.time()

    if i > 6000:
        rtde_c.forceMode(tf, selection_vector, wrench_up, force_type, limits)
    else:
        rtde_c.forceMode(tf, selection_vector, wrench_down, force_type, limits)
    end = time.time()
    duration = end - start
    if duration < dt:
        time.sleep(dt - duration)


rtde_c.forceModeStop()
rtde_r.stopFileRecording()
print("\nData recording stopped.")

rtde_c.stopScript()


