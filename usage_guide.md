# video_ws — Usage Guide

## Overview

ROS 2 Jazzy workspace for HDMI camera streaming and ROI-based telemetry extraction.

### Packages

| Package | Description | Topics |
|---------|-------------|--------|
| `hdmi_camera_driver` | V4L2 driver publishing raw frames | `/camera/image_raw` (`sensor_msgs/msg/Image`) |
| `image_roi_splitter` | Crops RGB screen + telemetry ROIs | `/camera/rgb_roi`, `/telemetry/location_roi` |

### Topic Pipeline

```
/camera/image_raw (1920×1080 @ 60 Hz, bgr8)
       │
       ├──► /camera/rgb_roi          (1439×876)  ──► Computer vision / display
       │
       └──► /telemetry/location_roi  (185×57)    ──► OCR processor (lat + lon stacked)
```

---

## 1. ROI Calibration (One-Time Setup)

ROI coordinates are hardcoded in `src/image_roi_splitter/src/roi_splitter_node.cpp`. To recalibrate:

### 1.1 Capture a Reference Frame
```bash
# With camera running, save one frame:
ros2 run image_view image_saver image:=/camera/image_raw __name:=saver
# Output: frame_0001.png (1920×1080)
```

### 1.2 Run Interactive Calibration
```bash
# In container with X11 forwarding:
cd /workspace/video_ws/dataset/roi_calibration
python3 calibrate_rois.py ../frame_0001.png
# Or use default reference image:
python3 calibrate_rois.py
```

**Usage:**
1. Window opens with the reference image (scaled to fit screen)
2. Click **2 points per ROI** (top-left → bottom-right)
3. Order: **RGB → Latitude → Longitude**
4. Coordinates print to terminal
5. Press **ESC** to save and exit

### 1.3 Output File
Creates/updates `dataset/roi_calibration/roi_calibration.txt`:
```
# ROI Calibration - frame_0001.png
# Resolution: 1920x1080
# Format: name x y w h

rgb_roi: 240 69 1439 876
latitude_roi: 840 1048 185 28
longitude_roi: 1032 1047 163 29
```

### 1.4 Update Hardcoded Values
Copy the three values into `src/image_roi_splitter/src/roi_splitter_node.cpp` (lines 30–32):
```cpp
cv::Rect rgb_rect(240, 69, 1439, 876);
cv::Rect lat_rect(840, 1048, 185, 28);
cv::Rect lon_rect(1032, 1047, 163, 29);
```

### 1.5 Rebuild
```bash
colcon build --packages-select image_roi_splitter --symlink-install
```

> **Note:** Future work will make ROIs configurable via ROS 2 parameters.

---

## 2. Building the Workspace

### Prerequisites
- Docker with ROS 2 Jazzy image (`rosstudy_env:jazzy`)
- NVIDIA GPU (for hardware acceleration) or CPU-only mode
- X11 forwarding for GUI tools

### Container Setup (Recommended)
```bash
# Host: Enable X11 access
xhost +local:root

# Host: Start persistent development container
docker run -d --rm \
  --name ros2_dev \
  --net=host --ipc=host --pid=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/psampaioc/Workspaces:/workspace \
  -w /workspace/video_ws \
  rosstudy_env:jazzy \
  sleep infinity

# Host: Enter container
docker exec -it ros2_dev bash
```

### Build
```bash
# Inside container:
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hdmi_camera_driver image_roi_splitter --symlink-install
source install/setup.bash
```

### Clean Build (Validation)
```bash
rm -rf build install log
colcon build --packages-select hdmi_camera_driver image_roi_splitter --symlink-install
```

---

## 3. Running the Pipeline

### Terminal 1 — Camera Driver (Requires Hardware)
```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
source /workspace/video_ws/install/setup.bash

# Default device: /dev/video4
ros2 run hdmi_camera_driver camera_publisher_node --ros-args -p device_path:=/dev/video4
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `device_path` | `/dev/video4` | V4L2 device path |
| `width` | `1920` | Frame width |
| `height` | `1080` | Frame height |
| `fps` | `60.0` | Target frame rate |

**Custom parameters:**
```bash
ros2 run hdmi_camera_driver camera_publisher_node \
  --ros-args -p device_path:=/dev/video4 -p width:=1920 -p height:=1080 -p fps:=60.0
```

### Terminal 2 — ROI Splitter
```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
source /workspace/video_ws/install/setup.bash
ros2 run image_roi_splitter roi_splitter_node
```

### Terminal 3 — Verification
```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash

# List topics
ros2 topic list
# Expected: /camera/image_raw, /camera/rgb_roi, /telemetry/location_roi, /parameter_events, /rosout

# Inspect RGB ROI
ros2 topic echo /camera/rgb_roi --once
# Verify: width: 1439, height: 876

# Inspect Location ROI
ros2 topic echo /telemetry/location_roi --once
# Verify: width: 185, height: 57 (28 + 29 stacked)

# Check frame rate
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/rgb_roi
```

### Terminal 4 — Visualization (Optional)
```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
source /workspace/video_ws/install/setup.bash

# RGB screen view
ros2 run image_view image_view image:=/camera/rgb_roi

# Telemetry stack (latitude on top, longitude below)
ros2 run image_view image_view image:=/telemetry/location_roi
```

---

## 4. Container-Based Validation (CI / Demo)

Test build and run in a fresh container without persistent state:

```bash
# Host: Start ephemeral test container
docker run -d --rm \
  --name ros2_test \
  --net=host --ipc=host --pid=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/psampaioc/Workspaces:/workspace \
  -w /workspace/video_ws \
  rosstudy_env:jazzy \
  sleep infinity

# Host: Enable X11
xhost +local:root

# Host: Clean build
docker exec ros2_test bash -c "
  source /opt/ros/jazzy/setup.bash &&
  colcon build --packages-select hdmi_camera_driver image_roi_splitter --symlink-install
"

# Host: Test driver (requires /dev/video4)
docker exec -it ros2_test bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /workspace/video_ws/install/setup.bash &&
  timeout 5 ros2 run hdmi_camera_driver camera_publisher_node --ros-args -p device_path:=/dev/video4
"

# Host: Test splitter
docker exec -it ros2_test bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /workspace/video_ws/install/setup.bash &&
  timeout 5 ros2 run image_roi_splitter roi_splitter_node
"

# Host: Verify topics
docker exec ros2_test bash -c "
  source /opt/ros/jazzy/setup.bash &&
  ros2 topic list
"

# Host: Cleanup
docker stop ros2_test
```

---

## 5. Validation Checklist

| Check | Command | Expected |
|-------|---------|----------|
| **Clean build** | `colcon build ...` | 0 errors, 2 packages built |
| **Driver starts** | `ros2 run hdmi_camera_driver ...` | "Publishing frame..." at ~60 Hz |
| **Splitter starts** | `ros2 run image_roi_splitter ...` | "ROIs calibrated (1920x1080)" |
| **Topics exist** | `ros2 topic list` | 4+ topics including rgb_roi + location_roi |
| **RGB ROI size** | `ros2 topic echo /camera/rgb_roi --once` | width: 1439, height: 876 |
| **Location ROI size** | `ros2 topic echo /telemetry/location_roi --once` | width: 185, height: 57 |
| **Timestamps preserved** | `ros2 topic echo /camera/rgb_roi --once` | `header.stamp` = capture time |
| **Frame ID correct** | `ros2 topic echo /camera/rgb_roi --once` | `header.frame_id: "camera_frame"` |

---

## 6. Project Structure

```
video_ws/
├── src/
│   ├── hdmi_camera_driver/
│   │   ├── include/hdmi_camera_driver/camera_publisher_node.hpp
│   │   ├── src/camera_publisher_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── image_roi_splitter/
│       ├── include/image_roi_splitter/roi_splitter_node.hpp
│       ├── src/roi_splitter_node.cpp
│       ├── CMakeLists.txt
│       └── package.xml
├── dataset/                    # gitignored — personal data
│   ├── drone_test_flight.mkv
│   ├── resolution_image.png
│   ├── resolution_image_live.png
│   └── roi_calibration/
│       ├── calibrate_rois.py
│       └── roi_calibration.txt
├── build/                      # gitignored
├── install/                    # gitignored
├── log/                        # gitignored
├── .gitignore
├── CLAUDE.md                   # Project instructions (internal)
├── usage_guide.md              # This file
└── .memsearch/                 # gitignored
```

---

## 7. Quick Reference

```bash
# Topic rates
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/rgb_roi

# Driver parameters
ros2 param list /camera_publisher_node
ros2 param get /camera_publisher_node device_path

# Record bag
ros2 bag record /camera/image_raw /camera/rgb_roi /telemetry/location_roi

# Playback bag
ros2 bag play recording.db3 --loop
```

---

## 8. Troubleshooting

| Issue | Resolution |
|-------|------------|
| `cv2.error: Can't initialize GTK backend` | Run `xhost +local:root` on host before `docker exec` |
| `/dev/video4` not found | `ls /dev/video*` — adjust `device_path` parameter |
| Black frames / no data | Check HDMI cable, capture card, `v4l2-ctl --list-devices` |
| ROI out of bounds | Recalibrate with `python3 calibrate_rois.py` — source must be 1920×1080 |
| Build fails: `cv_bridge` missing | Install `ros-jazzy-cv-bridge` in base Dockerfile |
| Timestamp mismatch | Driver sets `header.stamp = now()` at capture (correct) |

---

## 9. Roadmap

| Feature | Effort | Priority |
|---------|--------|----------|
| Launch file (composable driver + splitter) | 30 min | High |
| Dynamic parameters (`OnSetParametersCallback`) | 45 min | High |
| DiagnosticUpdater (FPS, dropped frames, device health) | 30 min | Medium |
| OCR telemetry node (ONNX Runtime CRNN) | 2–3 hrs | High |
| Unit tests (v4l2loopback mock, encoding/timestamp validation) | 1 hr | Medium |
| Full pipeline launch file | 30 min | High |

---

*video_ws — Naval Rex project*