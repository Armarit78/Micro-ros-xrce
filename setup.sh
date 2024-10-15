#!/bin/bash

# Stop the script if any command fails
set -e

# Check if ROS 2 Iron is already installed
if dpkg -s ros-iron-desktop >/dev/null 2>&1; then
    echo "ROS 2 Iron already installed. Skipping ROS installation."
else
    echo "ROS 2 Iron not found. Installing ROS 2 Iron Desktop..."

    # Setting up locale to support UTF-8
    echo "Setting up the locale to support UTF-8..."
    sudo apt update && sudo apt install locales -y
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Verify locale settings
    locale

    # Enabling Ubuntu Universe repository
    echo "Enabling Ubuntu Universe repository..."
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y

    # Adding ROS 2 GPG key and repository
    echo "Adding ROS 2 GPG key and repository..."
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # Updating and upgrading packages
    echo "Updating and upgrading system packages to ensure the system is up to date..."
    sudo apt update
    sudo apt upgrade -y

    # Install development tools (optional)
    echo "Installing development tools..."
    sudo apt update && sudo apt install build-essential cmake python3-pip python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

    # Install ROS 2 Iron
    echo "Installing ROS 2 Iron Desktop..."
    sudo apt install ros-iron-desktop -y

    # Set up the ROS 2 environment
    echo "Setting up ROS 2 environment..."
    echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
    source /opt/ros/iron/setup.bash

    echo "ROS 2 Iron installation and setup completed successfully!"
fi

cd ~

# Micro-ROS installation starts here
# Create a micro-ROS workspace
source /opt/ros/$ROS_DISTRO/setup.bash

mkdir -p ~/microros_ws
cd ~/microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install Python pip
sudo apt-get install python3-pip -y

# Build the micro-ROS tools
colcon build
source install/local_setup.bash

# Create the firmware workspace
ros2 run micro_ros_setup create_firmware_ws.sh host

# Build the firmware
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash

ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash

# Create and build the micro-ROS agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

cp -r test/ping ~/microros_ws/src/uros/micro-ROS-demos/rclc/
cp -r test/pong ~/microros_ws/src/uros/micro-ROS-demos/rclc/
cp -r test/rosbagsub ~/microros_ws/src/uros/micro-ROS-demos/rclc/
cp -r test/rosbagpub ~/microros_ws/src/uros/micro-ROS-demos/rclc/
cp test/test.yaml ~/microros_ws/
cp -r test/subset ~/microros_ws/
rm  ~/microros_ws/src/uros/micro-ROS-demos/rclc/CMakeLists.txt
cp  test/CMakeLists.txt ~/microros_ws/src/uros/micro-ROS-demos/rclc/

ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash

echo "Installation complete! micro-ROS are ready to use. Please restart your terminal or run 'source ~/.bashrc' to apply all changes."

