## General

- Check if the used mulity [launch file](transbot_ws/src/transbot_mulity/launch/transbot_mulity_control.launch) contains everything this robot might be used for. If not, create one that adapts to the needs.

## Simulator

- Optimise Lidar raytracing algorithm, by improving the logic and also extending it to use the GPU.
- Extend the Obstacle class to accomodate different shapes and more complex obstacles.

## Transbot

- Extend the topics and test the Arm and the Camera, along with the rotating platform of the camera.
- Improve the Frames class to ensure the static frames are reliably fetched.
- Implement a watchdog for the battery.
- Implement a GPS topic that can be used instead of the Odometry one in position (or this can be done directly from the Robot).
