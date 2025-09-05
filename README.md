# Fast-Planner with ROS 2 Humble and MAVROS Support

*TO DO: Update topo.launch.py, include Gazebo playground with correct launch parameters for the planner.*

*Original Work - [Fast-Planner by HKUST Aerial Robotics](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)*

This repository is an updated version of **Fast-Planner** developed at **HKUST Aerial Robotics**. It introduces:
* **ROS 2** Humble support
* Custom **MAVROS** bridge
* **PX4** submodule for simulated testing
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
git clone --recursive https://github.com/UAV-SAR/Fast-Planner-ROS2.git
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

Finally, modify an existing PX4 model to include both odometry and a depth camera.
Open the file `PX4-Autopilot/Tools/simulation/gz/models/x500_vision/model.sdf` and change the base model from `x500` to `x500_depth`.
```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500_vision'>
    <include merge='true'>
      <!-- <uri>x500</uri> -->
      <uri>x500_depth</uri>
    </include>
    <plugin
      filename="gz-sim-odometry-publisher-system"
      name="gz::sim::systems::OdometryPublisher">
      <dimensions>3</dimensions>
    </plugin>
  </model>
</sdf>
```

### Build and Run

Build the project with **colcon**:
```bash
source /opt/ros/humble/setup.bash
cd Fast-Planner-ROS2/
colcon build --packages-select quadrotor_msgs multi_map_msgs bspline_msgs swarmtal_msgs util_interfaces
source install/setup.bash
colcon build
```

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

Source the **ROS 2** overlay in each new terminal:
```bash
source /opt/ros/humble/setup.bash
```

**Terminal 3** Source the project and run the **PX4**, **Gazebo**, and **MAVROS** utils launch:
```bash
source install/setup.bash
ros2 launch utils px4.launch.py
```

**Terminal 4** Source the project and run the **Fast-Planner** system:
```bash
source install/setup.bash
ros2 launch plan_manage kino.launch.py
```
*NOTE: In case of any naming or setup mismatch, change parameters in kino.launch.py and px4.launch.py accordingly.*

**Terminal 5** Publish a goal waypoint for the planner. The */send_goal* service has been set up to accept a name of a **Gazebo** object that will serve as a target. You can read the frame ID from **Gazebo** and input it here. Example with "box":
```bash
source install/setup.bash
ros2 service call util_interfaces/srv/SetFrameAsGoal "{frame_id: 'box'}"
```

**Terminal 6** First, have the drone take off via QGroundControl. Then, use **MAVROS** to set the mode to **OFFBOARD**. At this point, the simulated drone should fly towards the target you set in the previous step.
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"
```
