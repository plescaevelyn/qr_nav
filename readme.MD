# QR Navigation

This is a ROS 2 package for autonomous robot navigation using QR code detection. It uses an OAK-D Lite camera with the `depthai-ros` driver and detects QR codes using the `zbar` library. Detected QR codes are published as poses and visualization markers to be used in navigation and visualized in RViz.

---

## 1. Requirements

- Ubuntu 22.04
- ROS 2 Humble
- OAK-D Lite camera
- `zbar` for QR detection
- `depthai-ros` drivers (clone from Luxonis)
- ROS 2 dependencies: `cv_bridge`, `image_transport`, `OpenCV`, etc.

---

## 2. Clone Repositories

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone depthai-ros
git clone https://github.com/luxonis/depthai-ros.git

# Clone qr_nav
git clone https://github.com/plescaevelyn/qr_nav.git
```

---

## 3. Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Install system packages:

```bash
sudo apt update
sudo apt install libzbar-dev
```

---

## 4. Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 5. Run the System

### Terminal 1 – Start the Camera Node

```bash
ros2 launch depthai_ros_driver camera.launch.py
```

### Terminal 2 – Start the QR Detector Node

```bash
source ~/ros2_ws/install/setup.bash
ros2 run qr_nav_ros2 tag_detector
```

### Terminal 3 – Launch RViz for Visualization

```bash
source ~/ros2_ws/install/setup.bash
rviz2
```

In RViz:
- Add a `Marker` display subscribed to `/qr_marker`
- Add a `TF` display (optional)
- Add an `Image` display subscribed to `/color/image` (optional)

---

## 6. Debug and Inspect Topics

List active topics:

```bash
ros2 topic list
```

Check published pose:

```bash
ros2 topic echo /qr_pose
```

Check visualization markers:

```bash
ros2 topic echo /qr_marker
```

---

## 7. Log Detected Tags (Optional)

To log detected markers (e.g., for mapping or later analysis), run the logger node:

```bash
ros2 run qr_logger log_node
```

This node listens for detections (likely on `/qr_pose` or `/tag_detections`) and logs them. Check its source code for file paths or logging details.

---

## License

MIT License

---

## Author

[Evelyn-Iulia Pleșca](https://github.com/plescaevelyn)
