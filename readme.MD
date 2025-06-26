# QR Navigation - `qr_nav`

This is a ROS 2 package for autonomous robot navigation using QR code detection. It uses an OAK-D Lite camera with the `depthai-ros` driver and detects QR codes using the `zbar` library. Detected QR codes are published as poses and visualization markers to be used in navigation and visualized in RViz.

---

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- OAK-D Lite camera
- `zbar` for QR detection
- `depthai-ros` drivers (clone from Luxonis)
- Dependencies: `cv_bridge`, `image_transport`, `OpenCV`, etc.

---

## 1. Clone Repositories

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone depthai-ros
git clone https://github.com/luxonis/depthai-ros.git

# Clone qr_nav
git clone https://github.com/plescaevelyn/qr_nav.git

---

## 2. Install Dependencies

```cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Install system dependencies

sudo apt update
sudo apt install libzbar-dev

---

## 3. Build Workspace

```cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

---

## 4. Run the System
```# Terminal 1:
ros2 launch depthai_ros_driver camera.launch.py

# Terminal 2:
source ~/ros2_ws/install/setup.bash
ros2 run qr_nav qr_detector_node

# Terminal 3:
source ~/ros2_ws/install/setup.bash
rviz2

---

## 4. Debug

```ros2 topic list
ros2 topic echo /qr_pose
ros2 topic echo /qr_marker
