# Fast-Planner with ROS 2 Humble and MAVROS Support

*TO DO: Finish updating launch files (ADD MAVROS BRIDGE), add PX4 with custom world and model, add notes on running and project structure*

*Original Work - [Fast-Planner by HKUST Aerial Robotics](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)*

This repository is an updated version of **Fast-Planner** developed at **HKUST Aerial Robotics**. It introduces:
* **ROS 2** Humble support
* Custom **MAVROS** bridge
* **Gazebo** setup for simulated testing
* Simplified configuration step
* Simplified launch files
* Simplified file structure

### Dependencies

This repository has been tested with the following dependencies:
* Ubuntu 22
* ROS 2 Humble
* Eigen3 3.4.0
* nlopt 2.10.0
* OpenCV 4.5.4
* PCL 1.12.1
* Armadillo 10.8.2

Clone this repository, including **PX4** submodules:
```bash
git clone https://github.com/damiankryzia71/Fast-Planner-ROS2.git --recursive
cd Fast-Planner-ROS2/
```

Install **PX4** dependencies, then reboot:
```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
reboot
```

Install **ROS 2 Humble** by following the [official guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

Install necessary libraries with **apt**:
```bash
sudo apt update
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
sudo apt install libeigen3-dev libopencv-dev libpcl-dev libarmadillo-dev 
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-mavros
```

Download **QGroundControl**:
```bash
cd Fast-Planner-ROS2/
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage
```

Build and install **nlopt** from source:
```bash
git clone -b v2.10.0 https://github.com/stevengj/nlopt.git
cd nlopt/
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

Install other **ROS 2** dependencies with **rosdep**:
```bash
cd Fast-Planner-ROS2/
rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Build and Run

**Terminal 1** Build **PX4**. The Baylands world, with a modified odometry + depth camera drone was used for testing.
```bash
cd PX4-Autopilot/
make px4_sitl gz_x500_vision_baylands
```

**Terminal 2** Launch QGroundControl:
```bash
chmod +x QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```

Source the **ROS 2** overlay:
```bash
source /opt/ros/humble/setup.bash
```

Build the project with **colcon**:
```bash
cd Fast-Planner-ROS2/
colcon build
```

Source the project:
```bash
source install/setup.bash
```
