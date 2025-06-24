#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import threading
import pygame
import time
import signal
import numpy as np
import sys

"""
This script is used to compare the yaw outputs of two different IMU filters.

1. Madgwick Filter
    <!-- Madgwick Filter -->
    <node pkg="imu_filter_madgwick" type="imu_filter_node" name="imu_filter_madgwick" output="screen">
        <remap from="imu/data_raw" to="imu/data_raw"/>
        <remap from="imu/data" to="imu/data_madgwick"/>
        <param name="fixed_frame" value="$(arg namespace)/base_link"/>
        <param name="use_mag" value="false"/>
        <param name="publish_tf" value="false"/>
        <param name="world_frame" value="enu"/>
    </node>

2. Complementary Filter
    <!-- Complementary Filter -->
    <node pkg="imu_complementary_filter" type="complementary_filter_node" name="imu_complementary_filter" output="screen">
        <remap from="imu/data_raw" to="imu/data_raw"/>
        <remap from="imu/data" to="imu/data_complementary"/>
        <param name="use_mag" value="false"/>
        <param name="publish_tf" value="false"/>
        <param name="fixed_frame" value="$(arg namespace)/base_link"/>
    </node>
"""

# Collected data
madgwick_data = []
complementary_data = []
timestamps = []
start_time = None


def calculate_displacement(madgwick_yaws, complementary_yaws):
    global madgwick_data, complementary_data
    if len(madgwick_yaws) != len(complementary_yaws):
        min_length = min(len(madgwick_yaws), len(complementary_yaws)) // 2
        madgwick_yaws = madgwick_yaws[:min_length]
        complementary_yaws = complementary_yaws[:min_length]
    return np.mean(complementary_yaws - madgwick_yaws)


def imu_callback_madgwick(msg):
    q = msg.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    t = msg.header.stamp.to_sec() - start_time
    madgwick_data.append((t, yaw))


def imu_callback_complementary(msg):
    q = msg.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    t = msg.header.stamp.to_sec() - start_time
    complementary_data.append((t, yaw))


def listen_space():
    pygame.init()
    screen = pygame.display.set_mode((200, 100))
    pygame.display.set_caption("Press SPACE to mark event")
    global running
    while running:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    t = time.time() - wall_start
                    timestamps.append(t)
                    print(f"[SPACE] at {t:.2f}s")
                elif event.key == pygame.K_ESCAPE:
                    running = False
            elif event.type == pygame.QUIT:
                running = False
    pygame.quit()


def generate_plot():
    global madgwick_data, complementary_data, timestamps
    madgwick_data.sort()
    complementary_data.sort()

    # Apply displacement correction
    if madgwick_data and complementary_data:
        madgwick_yaws = np.array([yaw for _, yaw in madgwick_data])
        complementary_yaws = np.array([yaw for _, yaw in complementary_data])
        displacement = calculate_displacement(madgwick_yaws, complementary_yaws)
        madgwick_yaws += displacement
        madgwick_data = [(t, yaw + displacement) for (t, yaw) in madgwick_data]

    tm, ym = zip(*madgwick_data) if madgwick_data else ([], [])
    tc, yc = zip(*complementary_data) if complementary_data else ([], [])

    plt.figure(figsize=(10, 5))
    if tm: plt.plot(tm, ym, label='Simple Filter')
    if tc: plt.plot(tc, yc, label='Complementary Filter')
    for i, t_evt in enumerate(timestamps):
        plt.axvline(t_evt, color='red', linestyle='--', label="")
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw [rad]')
    plt.title('Yaw Comparison Between Filters')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig("yaw_rotate2.png")
    print("Saved plot to yaw_comparison.png")


def signal_handler(sig, frame):
    global running
    global madgwick_data, complementary_data
    running = False
    rospy.signal_shutdown("User interrupt")
    generate_plot()
    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('imu_graphs', anonymous=True)
    start_time = rospy.Time.now().to_sec()
    wall_start = time.time()
    running = True

    rospy.Subscriber("/bot02/imu/data_madgwick", Imu, imu_callback_madgwick)
    rospy.Subscriber("/bot02/imu/data_complementary", Imu, imu_callback_complementary)

    signal.signal(signal.SIGINT, signal_handler)

    space_thread = threading.Thread(target=listen_space)
    space_thread.start()

    rospy.spin()
