#!/bin/bash

set -e
echo "[INFO] Starting Interbotix full setup..."

##########################################
### STEP 1: System & ROS Core Dependencies
##########################################
echo "[STEP 1] Installing system dependencies..."
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y build-essential cmake git wget curl
sudo apt-get install -y python3-rosdep python3-rosinstall-generator python3-vcstool python3-rosinstall python3-catkin-tools python3-colcon-common-extensions
sudo apt-get install -y libboost-all-dev libudev-dev

##########################################
### STEP 2: Setup Dynamixel SDK Workspace
##########################################
##########################################
### STEP 2: Setup Dynamixel SDK Workspace
##########################################
echo "[STEP 2] Installing DynamixelSDK (CMake-based) in its own workspace..."
if [ ! -d ~/dynamixel_sdk_ws/src/DynamixelSDK ]; then
    mkdir -p ~/dynamixel_sdk_ws/src
    cd ~/dynamixel_sdk_ws/src
    git clone -b master https://github.com/ROBOTIS-GIT/DynamixelSDK.git
fi
cd ~/dynamixel_sdk_ws
catkin_make
source devel/setup.bash

##############################################
### STEP 2b: Setup Dynamixel Workbench + Msgs
##############################################
echo "[STEP 2b] Installing Dynamixel Workbench and msgs..."
if [ ! -d ~/dynamixel_ws/src/dynamixel-workbench ]; then
    mkdir -p ~/dynamixel_ws/src
    cd ~/dynamixel_ws/src
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
    cd dynamixel-workbench
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
fi
cd ~/dynamixel_ws
catkin_make
source devel/setup.bash


##########################################
### STEP 3: Install librealsense from source
##########################################
echo "[STEP 3] Installing librealsense..."
sudo apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
if [ ! -d ~/librealsense ]; then
    git clone https://github.com/IntelRealSense/librealsense.git ~/librealsense
fi
cd ~/librealsense
mkdir -p build && cd build
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
make -j$(nproc)
sudo make install
sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo ldconfig

##############################################
### STEP 4: Manually Clone Interbotix Packages
##############################################
echo "[STEP 4] Cloning Interbotix packages..."
mkdir -p ~/interbotix_ws/src
cd ~/interbotix_ws/src
[ ! -d interbotix_ros_core ] && git clone https://github.com/Interbotix/interbotix_ros_core.git
[ ! -d interbotix_ros_toolboxes ] && git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git
[ ! -d interbotix_ros_manipulators ] && git clone https://github.com/Interbotix/interbotix_ros_manipulators.git

#################################################
### STEP 5: Move unused Interbotix packages out
#################################################
echo "[STEP 5] Moving unused Interbotix packages to ~/NOT_BUILT..."
mkdir -p ~/NOT_BUILT
[ -d ~/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_uxarms ] && \
    mv ~/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_uxarms ~/NOT_BUILT/
[ -d ~/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_rpi_toolbox ] && \
    mv ~/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_rpi_toolbox ~/NOT_BUILT/

##################################################
### STEP 6: Install Additional ROS & Python Deps
##################################################
echo "[STEP 6] Installing ROS and Python dependencies..."
sudo apt install -y ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-moveit \
  ros-noetic-moveit-visual-tools ros-noetic-industrial-msgs ros-noetic-industrial-robot-simulator \
  ros-noetic-industrial-utils ros-noetic-controller-manager ros-noetic-effort-controllers \
  ros-noetic-joint-state-controller ros-noetic-position-controllers ros-noetic-joint-trajectory-controller \
  ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-rviz ros-noetic-tf \
  ros-noetic-tf2-ros ros-noetic-tf2-tools ros-noetic-image-transport ros-noetic-cv-bridge \
  ros-noetic-vision-opencv ros-noetic-depth-image-proc ros-noetic-camera-info-manager \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-interactive-markers \
  ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-joint-state-publisher-gui

pip3 install opencv-python numpy scipy matplotlib

##################################
### STEP 7: Build apriltag (C lib)
##################################
echo "[STEP 7] Building apriltag native library..."
if [ ! -d ~/apriltag ]; then
    git clone https://github.com/AprilRobotics/apriltag.git ~/apriltag
fi
cd ~/apriltag
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig

############################################
### STEP 8: Clone apriltag_ros + realsense-ros
############################################
echo "[STEP 8] Cloning apriltag_ros and realsense-ros..."
cd ~/interbotix_ws/src
[ ! -d apriltag_ros ] && git clone https://github.com/AprilRobotics/apriltag_ros.git

echo "[STEP 8] Setting up realsense-ros (ros1-legacy) in ~/realsense_ws..."

# Create realsense_ws and clone realsense-ros
if [ ! -d ~/realsense_ws/src ]; then
    mkdir -p ~/realsense_ws/src
    cd ~/realsense_ws/src

    echo "[INFO] Cloning realsense-ros repository..."
    git clone https://github.com/IntelRealSense/realsense-ros.git

    cd realsense-ros
    echo "[INFO] Checking out ros1-legacy branch..."
    git checkout ros1-legacy
    cd ..

    # Clone ddynamic_reconfigure if not available via apt
    if ! rospack find ddynamic_reconfigure &> /dev/null; then
        echo "[INFO] Cloning ddynamic_reconfigure from source..."
        git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
    fi

    echo "[INFO] Initializing catkin workspace..."
    catkin_init_workspace
else
    echo "[INFO] ~/realsense_ws/src already exists. Skipping clone and init..."
fi

# Build workspace
cd ~/realsense_ws
echo "[INFO] Cleaning and building realsense_ws..."
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install

# Source workspace
source devel/setup.bash
if ! grep -Fxq "source ~/realsense_ws/devel/setup.bash" ~/.bashrc; then
    echo "source ~/realsense_ws/devel/setup.bash" >> ~/.bashrc
    echo "[INFO] Added realsense_ws to ~/.bashrc"
fi

#################################################
### STEP 9: Build entire Interbotix workspace
#################################################
echo "[STEP 9] Building Interbotix catkin workspace..."
cd ~/interbotix_ws

# Make sure Interbotix can find DynamixelSDK
# Let Interbotix find all other workspaces
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/dynamixel_sdk_ws/devel:~/dynamixel_ws/devel

catkin_make -j$(nproc)

# Source setup and persist
source devel/setup.bash
if ! grep -Fxq "source ~/interbotix_ws/devel/setup.bash" ~/.bashrc; then
    echo "source ~/interbotix_ws/devel/setup.bash" >> ~/.bashrc
fi

########################################
### STEP 10: Install Interbotix Udev Rules
########################################
echo "[STEP 10] Installing Interbotix udev rules..."
cd ~/interbotix_ws/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "[✅ COMPLETE] Interbotix + Dynamixel SDK + RealSense + Apriltag setup is DONE!"

