# SETUP of wsl, Ros2 env, Micro-Ros and Manual teleop controller

## 1) Install wsl Ubuntu-22.04
(for windows only)
<br>
If you haven't installed WSL yet, run:
```
wsl --install
```
Run this specific command to ensure version 22.04 is installed:
```
wsl --install -d Ubuntu-22.04
```
- restart comupter when prompted
- open the Ubuntu application from the Start Menu to complete the initial setup, creating a username and password

# 2) Install ROS 2 HUMBLE
```
# ROS 2 requires a locale that supports UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# You need to add the ROS 2 apt repository to your system
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG Key:
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the Repository to your sources list:
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

# Install Development Tools:
sudo apt install ros-dev-tools

# Environment Setup (Permanent Sourcing)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Check ROS 2 installation:
echo $ROS_DISTRO
```

# 3) Install and setup Micro-ROS
```
# Source your ROS 2 environment
source /opt/ros/humble/setup.bash

# Create and enter the workspace
mkdir -p ~/microros_ws/src
cd ~/microros_ws

# Clone the micro-ROS setup repo
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build the setup package
colcon build
source install/local_setup.bash
```
Install the micro-ROS Agent <br>
The Agent is a node that runs on your WSL/Ubuntu machine. It listens for data from the microcontroller and "translates" it for ROS 2.
```
# Create the agent's workspace
ros2 run micro_ros_setup create_agent_ws.sh

# Build the agent
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
To run the agent later (for a Serial/USB connection):
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
# 4) Build the Swarm Workspace
- Downloade the swarm_ws folder from github.
- Transfer the folder to your ubuntu \home\User_ID <br>
(mostly the address of your Ubuntu files will be \\wsl.localhost\Ubuntu-22.04\home\User_ID)
- Now go to Ubuntu and extract the folder and build
```
tar xzf swarm_ws.tar.gz
cd swarm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
permanent sourcing:
```
echo "source install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
- Now everything is set up perfectly!
# 5) Implementing
- Downloade the Arduino IDE code of the bot, change the WIFI_SSID, WIFI_PASS, AGENT_IP
- You can find AGENT_IP by typing this in your ubuntu termincal:
```
hostname -I
```
Select the first ip it shows and paste it in the AGENT_IP field <br>
(NOTE: Both your bot and laptop should be connected to the same wifi network. eg: your mobile data)
<br><br>
Flash the ESP-32 with the code,check serial monitor. IT will try to connect to wifi two times, once to your mobile data, and once to your laptop. The second one will only connect after you do the next step. <br><br>
Now in yout Ubuntu terminal, start the micro-ros agent 
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```
Now if you check the serial monitor in arduino ide, it should show connected and print values. <br>
Open another terminal and start the teleop control:
```
ros2 run swarm_controller teleop
```
If the bot is connected with your computer through wifi now, you will see repeated lines coming on the micro-ros agent. Now if you use the teleop control, you can move the bot. <br>
To see the topics and messages, open a new terminal,
```
ros2 topic list
ros2 topic echo /topic_name
```

# 6) Common Errors
- if you are not able to flash to esp, and it outputs exit status 1, then while its uploading, press the boot button on the esp.
- if it returns exit status 2, check the com port and the device selected on the arduino ide
- if the esp 32 isnt connecting to the micro ros agent, do this:
```
# Kill the agent
Ctrl+C on the agent terminal

# Kill any stale agent processes
pkill -f micro_ros_agent

# Wait 3 seconds
sleep 3

# Restart agent fresh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```
- If you are using your mobile hotspot as a wifi network, your laptop ip might keep on changing whenever it disconnects and reconnects, so keep that in mind.
- If nothing shows up when you try ros2 topic list, try starting the daemon manually:
```
ros2 daemon start
ros2 topic list
```
