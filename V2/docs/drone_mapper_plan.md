# Implementation Plan: drone_mapper Package (Updated with Research & Requirements)

## Context

The V2 `drone_vision` package is complete and built. It publishes JSON telemetry on `/telemetry/data` containing latitude, longitude, heading, height, and speed extracted via OCR from HDMI camera feed.

**Objective:** Create a new ROS 2 package `drone_mapper` with **three nodes** that work together with `drone_vision`:

1. **`map_publisher`** - Loads pre-converted `map.pcd`, publishes static PointCloud2 once on `/map/cloud` (latched QoS)
2. **`tf_publisher`** - Publishes static TF: `map` → `utm_zone_XX` (auto-detects UTM zone from first telemetry message)
3. **`points_publisher`** - Subscribes to `/telemetry/data`, converts to UTM, publishes `MarkerArray` on `/map/telemetry_points` with trajectory, headings, and drone axis marker

The map is published **statically once** (latched/transient_local QoS) — no Open3D needed, just `pcl::io::loadPCDFile` + `pcl_conversions`.

---

## Key Decisions (Updated)

| Decision | Choice | Rationale |
|----------|--------|-----------|
| **Config Location** | **Single global `V2/config.yaml`** (merged with existing drone_vision params) | Follows existing pattern; one config file for entire workspace |
| **Launch File** | **`drone_mapper` only** (3 nodes: map_publisher, tf_publisher, points_publisher) | `drone_vision` launched separately; decoupled workflows |
| **PLY→PCD Converter** | **Python script** (`scripts/convert_ply_to_pcd.py`) | One-time tool; python-pcl is simpler; no C++ build dependency |
| **Map File Location** | `V2/config/map.ply` (user provides) → `V2/config/map.pcd` (generated, gitignored) | Shared config directory at workspace level |

---

## Package Structure (Updated)

```
video_ws/
├── V2/                              # ACTIVE ROS 2 WORKSPACE
│   ├── config.yaml                  # GLOBAL CONFIG (merged: drone_vision + drone_mapper params)
│   ├── config/
│   │   ├── map.ply                  # Source map (user provides, ~211MB, 7.8M points)
│   │   └── map.pcd                  # Generated binary PCD (gitignored, cached)
│   ├── src/
│   │   ├── drone_vision/            # Existing package
│   │   └── drone_mapper/            # NEW PACKAGE
│   │       ├── CMakeLists.txt
│   │       ├── package.xml
│   │       ├── include/drone_mapper/
│   │       │   ├── map_publisher.hpp
│   │       │   ├── tf_publisher.hpp
│   │       │   └── points_publisher.hpp
│   │       ├── src/
│   │       │   ├── map_publisher.cpp
│   │       │   ├── tf_publisher.cpp
│   │       │   └── points_publisher.cpp
│   │       ├── scripts/
│   │       │   └── convert_ply_to_pcd.py    # One-time Python converter
│   │       ├── rviz/
│   │       │   └── drone_mapper.rviz
│   │       └── launch/
│   │           └── drone_mapper.launch.py   # Launches ONLY drone_mapper nodes
│   ├── build/                       # Colcon build artifacts (gitignored)
│   ├── install/                     # Colcon install space (gitignored)
│   └── log/                         # Colcon logs (gitignored)
```

---

## Global Config: V2/config.yaml (Merged)

```yaml
# ============================================================
# drone_vision parameters (existing)
# ============================================================
/unified_vision_node:
  ros__parameters:
    input_mode: "topic"
    device_path: "/dev/video4"
    roi_rgb: [80, 60, 480, 360]
    roi_lat: [280, 467, 62, 11]
    roi_lon: [344, 467, 59, 12]
    roi_heading: [210, 467, 23, 12]
    roi_height: [278, 422, 37, 20]
    roi_speed: [213, 446, 28, 16]

# ============================================================
# drone_mapper parameters (NEW - add to same file)
# ============================================================
/map_publisher:
  ros__parameters:
    pcd_file_path: "config/map.pcd"   # Relative to package share dir
    map_frame: "map"
    topic_name: "/map/cloud"
    publish_once: true

/tf_publisher:
  ros__parameters:
    map_frame: "map"
    telemetry_topic: "/telemetry/data"
    auto_detect: true
    # Fallback if auto_detect=false:
    origin_latitude: 40.9788
    origin_longitude: -8.9345
    origin_altitude: 0.0

/points_publisher:
  ros__parameters:
    telemetry_topic: "/telemetry/data"
    marker_topic: "/map/telemetry_points"
    utm_frame: ""  # Empty = auto from TF, or override
    map_frame: "map"
    max_history_points: 1000
    publish_rate_hz: 10.0
    
    # Drone marker (axes)
    drone_axis_scale: 3.0      # Length of each axis arrow
    drone_axis_shaft_dia: 0.3  # Line thickness
    
    # Trajectory
    trajectory_line_width: 0.2
    trajectory_color: [0.0, 1.0, 0.0, 0.8]   # RGBA green
    
    # Points
    point_scale: 0.5
    point_color: [1.0, 0.0, 0.0, 1.0]        # RGBA red
    
    # Heading arrow
    arrow_length: 5.0
    arrow_shaft_dia: 0.3
    arrow_head_dia: 0.6
    arrow_color: [0.0, 0.0, 1.0, 1.0]        # RGBA blue
    
    # Drone axes colors (X=Red, Y=Green, Z=Blue)
    axis_x_color: [1.0, 0.0, 0.0, 1.0]
    axis_y_color: [0.0, 1.0, 0.0, 1.0]
    axis_z_color: [0.0, 0.0, 1.0, 1.0]
```

**Node parameter loading:** Each node loads its namespace from the same global `config.yaml`:
```cpp
// In each node constructor:
this->declare_parameter("pcd_file_path", "config/map.pcd");
// ...
auto config_path = ament_index_cpp::get_package_share_directory("drone_mapper") + "/config.yaml";
// But actually: nodes use their private namespace, so parameters are at /map_publisher/pcd_file_path etc.
// Launch file passes the global config.yaml to each node via parameters=[config_path]
```

---

## Dependencies (package.xml)

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>       <!-- PointCloud2 -->
<depend>tf2_ros</depend>           <!-- StaticTransformBroadcaster -->
<depend>tf2_geometry_msgs</depend> <!-- TF geometry utilities -->
<depend>visualization_msgs</depend> <!-- MarkerArray -->
<depend>geodesy</depend>           <!-- WGS84/UTM conversion -->
<depend>geographic_msgs</depend>   <!-- Geographic messages -->
<depend>pcl_ros</depend>           <!-- PCL-ROS conversions -->
<depend>pcl_conversions</depend>   <!-- PCL ↔ ROS conversion -->
<depend>nlohmann_json</depend>     <!-- JSON parsing -->
```

---

## Node 1: map_publisher (Simplified - Loads PCD Only)

**File:** `src/map_publisher.cpp`

**Purpose:** Load pre-converted `map.pcd`, publish once as latched `PointCloud2` on `/map/cloud`.

**Key Implementation:**
```cpp
class MapPublisher : public rclcpp::Node {
public:
    MapPublisher();
private:
    void loadAndPublishMap();
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    std::string pcd_file_path_;
    std::string map_frame_;  // "map"
};

// Load logic (SIMPLE - no conversion):
// 1. Resolve pcd_file_path relative to package share directory
// 2. Load PCD via pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *cloud)
// 3. Convert to ROS: pcl::toROSMsg(cloud, ros_cloud)
// 4. Set header.frame_id = map_frame_
// 5. Publish with QoS: transient_local + reliable + depth 1 (LATCHED)
```

---

## Node 2: tf_publisher

**File:** `src/tf_publisher.cpp`

**Purpose:** Auto-detect UTM zone from first `/telemetry/data`, publish static TF `map` → `utm_zone_XX`.

**Key Implementation:**
```cpp
class TfPublisher : public rclcpp::Node {
public:
    TfPublisher();
private:
    void telemetryCallback(const std_msgs::msg::String::SharedPtr msg);
    void publishStaticTransform(const geodesy::UTMPoint& utm_origin);
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    std::string map_frame_;      // "map"
    std::string utm_frame_;      // "utm_zone_29N" (computed)
    bool published_ = false;
};

// Logic:
// 1. Subscribe to /telemetry/data (QoS: reliable, depth 1)
// 2. On first message: parse JSON, extract lat/lon/alt
// 3. Create geographic_msgs::msg::GeoPoint → geodesy::UTMPoint
// 4. utm_frame_ = "utm_zone_" + zone + band (e.g., "utm_zone_29N")
// 5. Publish static transform: map → utm_frame_ (identity)
// 6. Unsubscribe after publishing (one-shot)
```

---

## Node 3: points_publisher

**File:** `src/points_publisher.cpp`

**Purpose:** Subscribe to `/telemetry/data`, convert to UTM, publish `MarkerArray` on `/map/telemetry_points`.

**Key Implementation:**
```cpp
class PointsPublisher : public rclcpp::Node {
public:
    PointsPublisher();
private:
    void telemetryCallback(const std_msgs::msg::String::SharedPtr msg);
    void publishMarkers();
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    struct TelemetryPoint {
        double easting, northing, altitude;
        double heading_rad;  // ROS convention (0=East, CCW)
        rclcpp::Time stamp;
    };
    std::vector<TelemetryPoint> history_;
    size_t max_history_;
    std::string utm_frame_;  // "utm_zone_XX" (from param or TF)
    std::string map_frame_;  // "map"
};

// MarkerArray Composition (published at 10 Hz or on each callback):
// 1. TRAJECTORY (LINE_STRIP, ns="trajectory", id=0)
//    - points: all history_ positions (x=easting, y=northing, z=altitude)
//    - color: green, scale.x=0.2 (line width)
//
// 2. POINTS (SPHERE_LIST, ns="points", id=0)
//    - points: all history_ positions
//    - color: red, scale.x=scale.y=scale.z=0.5
//
// 3. DRONE AXES (LINE_LIST, ns="drone_axes", id=0) — 3 lines from origin
//    - Line 0: X-axis (Red)   — length=drone_scale, direction=heading
//    - Line 1: Y-axis (Green) — length=drone_scale, direction=heading+90°
//    - Line 2: Z-axis (Blue)  — length=drone_scale/2, direction=UP
//    - scale.x=0.3 (shaft diameter), colors per vertex
//
// 4. HEADING ARROW (ARROW, ns="heading", id=0) — optional, at drone position
//    - pose: position=current, orientation=quaternion from heading_rad
//    - scale.x=arrow_length, scale.y=0.3, scale.z=0.2
//    - color: blue
```

**Heading Conversion (Critical):**
```cpp
// Input: heading_deg from JSON (0=North, CW)
// ROS/UTM: 0=East, CCW
double heading_ros_rad = (90.0 - heading_deg) * M_PI / 180.0;

// Quaternion for arrow (Z-up):
tf2::Quaternion q;
q.setRPY(0, 0, heading_ros_rad);
```

---

## PLY → PCD Conversion (Separate Python Tool - Run Once)

**File:** `src/drone_mapper/scripts/convert_ply_to_pcd.py` (standalone, run manually once)

```python
#!/usr/bin/env python3
"""Convert PLY to PCD for drone_mapper. Run ONCE after placing map.ply in V2/config/."""
import sys
import os

def convert_ply_to_pcd(ply_path, pcd_path):
    if not os.path.exists(ply_path):
        print(f"ERROR: {ply_path} not found")
        return False
    
    try:
        import pcl
        cloud = pcl.load(ply_path)  # Auto-detects format (ASCII/binary PLY)
        pcl.save(cloud, pcd_path, format='pcd', binary=True)
        print(f"Converted {ply_path} -> {pcd_path} ({cloud.size} points)")
        return True
    except ImportError:
        print("ERROR: python3-pcl not installed. Install with: pip install pcl")
        return False

if __name__ == "__main__":
    # Runs from V2/ directory: python3 src/drone_mapper/scripts/convert_ply_to_pcd.py
    base = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ply_path = os.path.join(base, "config", "map.ply")
    pcd_path = os.path.join(base, "config", "map.pcd")
    convert_ply_to_pcd(ply_path, pcd_path)
```

**Usage (run once manually from V2/):**
```bash
cd /workspace/video_ws/V2
python3 src/drone_mapper/scripts/convert_ply_to_pcd.py
```

**Result:** Creates `config/map.pcd` (binary PCD) — cached, not regenerated at runtime.

---

## Launch File (drone_mapper.launch.py — ONLY drone_mapper nodes)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('drone_mapper'),
        'config.yaml'
    )
    
    return LaunchDescription([
        # drone_mapper nodes only (drone_vision launched separately)
        Node(
            package='drone_mapper',
            executable='map_publisher',
            name='map_publisher',
            parameters=[config_path],
            output='screen'
        ),
        Node(
            package='drone_mapper',
            executable='tf_publisher',
            name='tf_publisher',
            parameters=[config_path],
            output='screen'
        ),
        Node(
            package='drone_mapper',
            executable='points_publisher',
            name='points_publisher',
            parameters=[config_path],
            output='screen'
        ),
    ])
```

---

## RViz Config (drone_mapper.rviz)

```yaml
Visualization Manager:
  Global Options:
    Fixed Frame: map
    Frame Rate: 30
  Displays:
    - Class: rviz/PointCloud2
      Name: Map Cloud
      Topic: /map/cloud
      Color Transformer: FlatColor
      Style: Points
      Size (Pixels): 2
      Color: 255; 255; 255
      Alpha: 1.0
      Enabled: true
    - Class: rviz/MarkerArray
      Name: Telemetry Points
      Topic: /map/telemetry_points
      Enabled: true
    - Class: rviz/TF
      Name: TF Tree
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Show Names: true
      Show Axes: true
      Show Arrows: true
      Marker Scale: 1.0
      Enabled: true
    - Class: rviz/Grid
      Name: Grid
      Plane: XY
      Reference Frame: map
      Enabled: true
```

---

## CMakeLists.txt (drone_mapper)

```cmake
cmake_minimum_required(VERSION 3.8)
project(drone_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nlohmann_json REQUIRED)

# PCL components
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(include ${PCL_INCLUDE_DIRS})

# Map Publisher
add_executable(map_publisher src/map_publisher.cpp)
target_link_libraries(map_publisher ${PCL_LIBRARIES} nlohmann_json::nlohmann_json)
ament_target_dependencies(map_publisher
  rclcpp std_msgs sensor_msgs pcl_ros pcl_conversions
)

# TF Publisher
add_executable(tf_publisher src/tf_publisher.cpp)
ament_target_dependencies(tf_publisher
  rclcpp tf2_ros geodesy geographic_msgs std_msgs
)

# Points Publisher
add_executable(points_publisher src/points_publisher.cpp)
target_link_libraries(points_publisher nlohmann_json::nlohmann_json)
ament_target_dependencies(points_publisher
  rclcpp std_msgs visualization_msgs geodesy geographic_msgs
)

# Install
install(TARGETS map_publisher tf_publisher points_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()
```

---

## Integration with drone_vision (Complete Pipeline)

### Topic Pipeline
```
/camera/image_raw (drone_vision, 60 Hz)
       │
       ├──► /camera/rgb_roi, /camera/*_roi (drone_vision) ──► Visual inspection
       │
       └──► /telemetry/data (drone_vision, JSON) ──► tf_publisher (1st msg: UTM zone)
              │                                      │
              │                                      └──► Static TF: map → utm_zone_XX
              │
              └──► points_publisher (every msg)
                     │
                     ├──► /map/telemetry_points (MarkerArray) ──► RViz
                     │        ├── trajectory (LINE_STRIP)
                     │        ├── points (SPHERE_LIST)
                     │        ├── drone_axes (LINE_LIST: X/Y/Z arrows)
                     │        └── heading (ARROW)
                     │
                     └──► (history buffer, max 1000)
```

### TF Tree (REP-105 Compliant)
```
map (fixed world frame, RViz fixed frame)
  └── utm_zone_29N (static, identity from tf_publisher)
        ├── /map/cloud (PointCloud2, frame_id=map)
        └── /map/telemetry_points (MarkerArray, frame_id=utm_zone_29N)
```

**Note:** Publish `/map/cloud` in `map` frame (simpler). Markers in `utm_zone_29N` frame. TF is identity so RViz aligns perfectly.

---

## Verification Plan

1. **Prepare Map:** Place your `map.ply` in `V2/config/`
2. **Convert (one-time, manual):** `cd V2 && python3 src/drone_mapper/scripts/convert_ply_to_pcd.py` → creates `config/map.pcd`
3. **Update Config:** Add drone_mapper params to `V2/config.yaml` (see merged config above)
4. **Build:** `cd /workspace/video_ws/V2 && colcon build --packages-select drone_mapper --symlink-install`
5. **Launch drone_vision (Terminal 1):** `source install/setup.bash && ros2 run drone_vision unified_vision_node --ros-args --params-file config.yaml -p input_mode:=topic`
6. **Launch drone_mapper (Terminal 2):** `source install/setup.bash && ros2 launch drone_mapper drone_mapper.launch.py`
7. **Play Rosbag (Terminal 3):** `ros2 bag play /path/to/file.mcap`
8. **RViz (Terminal 4):** `rviz2 -d install/drone_mapper/share/drone_mapper/rviz/drone_mapper.rviz`
9. **Verify:**
   - ✅ `/map/cloud` shows static point cloud (persists, latched)
   - ✅ TF tree: `map` → `utm_zone_XX` (static, identity)
   - ✅ `/map/telemetry_points` shows:
     - Green trajectory line connecting points
     - Red spheres at each historical position
     - **Large drone axes at current position:** Red X (forward/heading), Green Y (left), Blue Z (up)
     - Blue heading arrow at drone position
   - ✅ Markers accumulate (max 1000)
   - ✅ Drone axes rotate with heading (X always points forward)

---

## Questions Resolved from Research

| Question | Decision | Rationale |
|----------|----------|-----------|
| PLY→PCD conversion | **Separate Python script** (run once manually) | User runs explicitly; map_publisher only loads PCD; no runtime conversion logic |
| UTM origin | Auto-detect from first telemetry | Robust, no manual config needed |
| Map cloud frame | `map` frame | Simpler, TF is identity anyway |
| Heading format | Degrees, 0=North → convert to ROS (0=East, CCW) | Standard compass → ENU conversion |
| History limit | 1000 (configurable) | Start conservative, adjust based on memory |
| Drone marker | 3-axis LINE_LIST (X=Red, Y=Green, Z=Blue) | Clear orientation, no mesh needed, performant |
| Static TF | `tf2_ros::StaticTransformBroadcaster` | Published to `/tf_static`, latched, standard practice |
| Point cloud QoS | `transient_local` + `reliable` + `depth=1` | Latched behavior for static map |
| Config location | **Global `V2/config.yaml`** (merged) | Follows existing drone_vision pattern |
| Launch scope | **drone_mapper only** | Decoupled from drone_vision; separate launch |

---

## Next Steps (After Plan Approval)

1. **Create package structure** (`V2/src/drone_mapper/` with all dirs)
2. **Update global `V2/config.yaml`** with drone_mapper parameters (merged)
3. **Write `scripts/convert_ply_to_pcd.py`** 
4. **Implement `map_publisher.cpp/hpp`** (PCD load, latched publish)
5. **Implement `tf_publisher.cpp/hpp`** (telemetry sub, geodesy UTM, static TF)
6. **Implement `points_publisher.cpp/hpp`** (JSON parse, UTM convert, MarkerArray)
7. **Write `CMakeLists.txt`** and `package.xml`
8. **Write `launch/drone_mapper.launch.py`** and `rviz/drone_mapper.rviz`
9. **Build in Docker** → Test with rosbag → RViz verification