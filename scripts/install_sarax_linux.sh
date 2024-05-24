#!/bin/sh

# Creating a dependency worskpace
mkdir -p ~/dependencies/sarax
cd ~/dependencies/sarax

# Install PX4 and ROS dependencies
echo "Installing PX4 dependencies"
wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh && wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/requirements.txt && chmod +x ubuntu.sh && ./ubuntu.sh

echo "Installing binaries for MAVROS"
sudo apt-get install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras

echo "Install Geographic Datasets"
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && chmod +x install_geographiclib_datasets.sh && ./install_geographiclib_datasets.sh

echo "Install ROS building dependencies"
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

echo "Install catkin dependencies"
sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y

echo "Install rqt dependency for GUI"
sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins -y

# Create Sarax workspace and build
echo "Creating Sarax workspace"
mkdir -p ~/sarax_ws/src && cd ~/sarax_ws && catkin init && wstool init src
git clone --recursive -b v1.13.2-sarax-sim https://github.com/SaxionMechatronics/PX4-Autopilot.git
cd src && git clone https://github.com/SaxionMechatronics/sarax.git
cd .. && source /opt/ros/noetic/setup.bash  && rosdep install --from-paths src --ignore-src -r -y --skip-keys="python-scipy"
cd ~/sarax_ws/ && catkin build

# Set the sarax workspace as an environment variable
cd ~/sarax_ws/
echo "export SARAX_WS=$PWD" >> ~/.bashrc
echo "source \$SARAX_WS/devel/setup.bash" >> ~/.bashrc

# Inform user of workspace
echo "Sarax workspace is created in the following path: $PWD"
source ~/.bashrc