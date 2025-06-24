# Setup project

This tutorial aims at documenting the setup process of the robot fleet. There are multiple ways to use Ubuntu, either using WSL (WSL1, ros does not work as it should over WSL2) or native Ubuntu 20.04. The final project works using Native ubuntu.

## Connection setup

The first step is connecting the robot to the master and checking that the ROS nodes are visible from the master.

### WSL1 Setup

After starting the WSL1 and setting the Ubuntu 20.04 image, we need to check that the network works. Running `ping google.com` should reveal its status. It might be needed to overwrite the `/etc/resolv.conf` with `nameserver 8.8.8.8` and to add the following to the /etc/wsl.conf`:

```bash
[network]
generateResolvConf=false
```

Then, to start setting up ROS, install ros noetic as per this [tutorial](http://wiki.ros.org/Installation/Ubuntu) and then add the following at the end of `~/.bashrc` file:

```bash
NET_TYPE="eth0"  # eth0 for cable, wlan for wifi. Check type by running ifconfig
MY_IP=$(ip -4 addr show $NET_TYPE | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
export ROS_HOSTNAME=$MY_IP
export ROS_MASTER_URI=http://$MY_IP:11311
echo ------------------------
echo MY_IP: $MY_IP
echo ROS_HOSTNAME: $ROS_HOSTNAME
echo ROS_MASTER_URI: $ROS_MASTER_URI
echo ------------------------
source /opt/ros/noetic/setup.bash
source ~/RobotFleet/transbot_ws/source_py_env.sh
```

In case of connectivity issues, the firewall or VPN might be the ones who cause this. Using WSL1 should allow us to tap into the same network as the Windows.

### Native Ubuntu setup

After starting the Ubuntu machine, install the required tools, including ros noetic, as per this [tutorial](http://wiki.ros.org/Installation/Ubuntu). After this, modify the `~/.bashrc` file by adding the following:

```bash
NET_TYPE="eth0"  # eth0 for cable, wlan for wifi. Check type by running ifconfig
MY_IP=$(ip -4 addr show $NET_TYPE | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
export ROS_HOSTNAME=$MY_IP
export ROS_MASTER_URI=http://$MY_IP:11311
echo ------------------------
echo MY_IP: $MY_IP
echo ROS_HOSTNAME: $ROS_HOSTNAME
echo ROS_MASTER_URI: $ROS_MASTER_URI
echo ------------------------
source /opt/ros/noetic/setup.bash
source ~/RobotFleet/transbot_ws/source_py_env.sh
```

### Transbot setup

For the Transbot, you will need a USB stick with at least 32 Gb (My Jetson Nano boots from the USB stick). There are tutorials on the Transbot official page [Transbot-jetson nano](http://www.yahboom.net/study/Transbot-jetson_nano) on how to flash the image (System image-Udisk) onto the stick. After plugging it in the correct slot and starting the robot, the cmd will open and attempt to setup ROS. The same setup as in the WSL1 must be done in the Transbot, with a few adjustments. Running `nano ~/.bashrc` will open the file in editor mode. At the bottom, the ROS variables are already exported. They need to be changed the following way:

```bash
export ROS_MASTER_URI=http://XX.XX.XX.XX:11311  # Ubuntu IP
export ROS_HOSTNAME=$ROS_IP  # Transbot IP
```

After setting the environment variables, you will need to change the transbot launch file so taht it does not start with the default one, but with one that allows us to configure namespaces, so taht we can use multiple bots at the same time. For this, we need to edit the `start_transbot.sh` script, found in `~/Transbot/transbot/`. Looking into the script, we need to comment the current gnome-terminal line (that starts the default launch file) and add the following section under:

```bash
export ROBOT_NAME="botXX"  # The name is important and must be unique
gnome-terminal -- bash -i -c "cd /home/jetson/transbot_ws/src/transbot_mulity/launch;sleep 3;roslaunch transbot_mulity transbot_mulity_control.launch namespace:=$ROBOT_NAME;exec bash"
```

After restarting the robot, it should work as expected.

## Testing the setup

Firstly, we can test without running the ros system, by pinging:
```bash
# On Ubuntu
ping <Transbot IP>  # should work

# On Transbot
ping <Ubuntu IP>  # should work
```

For the ROS system, it must be running on the Ubuntu machine. This can be done by running `roscore` in a terminal and ensuring it starts without errors and then hangs. After this, by starting the Transbot, the `start_transbot.sh` will be executed and it will connect to the Ubuntu roscore as required.

Then, running `rosnode list` on Transbot should reveal a number of nodes, one in particular called `/rosout`. This one is the one started by the ROS master in Ubuntu. If this appears, the connection is established.

Running `rosnode list` and `rostopic list` on Ubuntu should reveal a number of nodes / topics that start with the robot name provided in the start script. Running `rostopic echo <topic_name>` should print in the console the information published to this topic, from the Transbot.

If everything worked so far, the setup is complete.

# Improvements

There are some changes necesarry to be able to run the application at it's full capacitiy.

## Imu fixes

The IMU sensor contains a gyroscope and accelerometer. The current mulity setup computes the imu data with a drift, making the imu data unusable. To fix this, modify the content of the launch file at `~/transbot_ws/src/transbot_mulity/launch/transbot_mulity_control.launch` by first commenting out the current imu node (called `imu_filter_madgwick`) and adding our own:

```xml
<!-- Improve IMU sensor -->
<node pkg="imu_complementary_filter" type="complementary_filter_node" name="imu_complementary_filter" output="screen" respawn="true">
    <remap from="imu/data_raw" to="imu/data_raw"/>
    <param name="use_mag" value="false"/>
    <param name="publish_tf" value="false"/>
    <param name="fixed_frame" value="$(arg namespace)/base_link"/>
</node>
```

As you can see in the new node, we have changed from the default computation method to a complementary filter node for the imu. Thus, we need to also install this new filter in the system. This can be done by running:

```bash
sudo apt install ros-melodic-imu-complementary-filter
```

Also, in `transbot_ws/src/transbot_bringup/scripts/transbot_driver.py`, the imu data is mising 2 fields in the `pub_data` method:

```py
def pub_data(self):
    while not rospy.is_shutdown():
        ...
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        # From here
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "imu_link"
        # To here
        ...
```
