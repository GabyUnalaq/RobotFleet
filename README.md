# RobotFleet

This project implements multi-robot coordination and control using ROS1 Noetic. It includes formation control and flocking behavior based on separation, cohesion, and alignment, with real-time deployment on Yahboom Transbot robots equipped with Jetson Nano boards. The system supports both simulation and real-world experiments.

The system is easily scalable, with multiple sensors, control algorithms and visualisers that can be added.

## Hardware

 - Ubuntu 20.04 (either using WSL1 or running natively);
 - Multiple robots of type Yahboom Transbot with Jetson Nano.

## Features

- Multi-robot setup with ROS namespaces
- Real-time Lidar-based obstacle avoidance
- IMU filtering and sensor fusion (EKF and complementary filters)
- Flocking controller with target-seeking
- Custom visualizer and metrics logger
- Fully documented setup and experiment pipeline

## Setup

For setup, please check [Setup.md](Setup.md)

### Useful links

- [Official Transbot docs](http://www.yahboom.net/study/Transbot-jetson_nano)
- [Jetson Nano microSD card write](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write)

## Achievements

- Final dissertation project (Master's degree)
- Presented at UnitBV Scientific Communication Session 2025 — **2nd place award**