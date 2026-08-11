# Comparison & Implementation Plan: Vasco GTSAM Factor Graph → Our drone_mapper + drone_vision

---

## 📊 Executive Summary

**Vasco's `online_drone_localization_jazzy`** is a production-grade, single-node Python pipeline that:
- Subscribes to `/telemetry/data` (from OCR/drone_vision)
- Converts WGS84 → UTM via **PROJ `cs2cs`** (keeps process alive for performance)
- Applies map metadata offset (`applied_translation` + bbox)
- **Robust filtering**: coordinate validation, bbox check, jump detection (max speed + time gate), negative longitude repair
- Publishes: `Path` (latched), `Marker` (latched), `PoseStamped`, `Odometry`, `TF` (map→drone_reference), status JSON
- Separate map publisher with voxel downsampling

**Our current `drone_mapper`** (3 C++ nodes):
- `map_publisher`: Loads pre-downsampled PCD, publishes latched PointCloud2
- `tf_publisher`: One-shot UTM zone detection from first telemetry, publishes static `map→utm_zone_XX`
- `points_publisher`: Real-time WGS84→UTM via `geodesy`, publishes heavy `MarkerArray` (LINE_STRIP + SPHERE_LIST + LINE_LIST + ARROW)

---

## 🔍 Detailed Comparison

| Aspect | Vasco (online_drone_localization_jazzy) | Our drone_mapper | Gap |
|--------|-----------------------------------------|------------------|-----|
| **Architecture** | 1 Python node + 1 map publisher | 3 C++ nodes + launch | Ours over-engineered |
| **WGS84→UTM** | PROJ `cs2cs` (persistent subprocess) | `geodesy::UTMPoint` | Vasco faster, more robust |
| **Map Offset** | JSON metadata (`applied_translation`, bbox) | Hardcoded in config.yaml | Vasco data-driven |
| **Coordinate Validation** | Lat/lon ranges, bbox, regex parsing | Basic JSON parse | Vasco production-grade |
| **Jump Detection** | Max speed + time gate + tolerance | **None** | Critical gap |
| **Longitude Repair** | Auto-detect & fix negative from OCR | **None** | OCR bug workaround |
| **Height/Heading Fallback** | Configurable defaults with counters | **None** | Resilience gap |
| **Trajectory Output** | `nav_msgs/Path` (latched) + `Marker` | `MarkerArray` (heavy) | Vasco lighter, native RViz |
| **TF** | Dynamic `map→drone_reference` | Static `map→utm_zone_XX` | Vasco correct for tracking |
| **Status/Monitoring** | JSON status topic with filter stats | **None** | Observability gap |
| **Configuration** | `.env.local` (shell-sourced) | Global `config.yaml` | Both valid |
| **Map Publisher** | Voxel downsample at runtime (0.15m) | Pre-downsampled PCD (1M pts) | Vasco more flexible |
| **QoS** | Transient local for Path/Marker | Transient local for map only | Vasco latches trajectory |
| **Lines of Code** | ~420 (1 file) + ~120 (map) | ~600+ across 6+ files | Vasco simpler |

---

## 🎯 Target Architecture: Unified drone_mapper (C++ Port of Vasco)

### Design Principles
1. **Single C++ node** (`drone_localization_node`) replacing 3 nodes
2. **PROJ C++ API** (not subprocess) for WGS84→UTM — faster, no fork overhead
3. **Map metadata JSON** for offset/bbox (data-driven, no hardcoded values)
4. **Robust filtering pipeline** (validation → projection → bbox → jump gate)
5. **Latched Path + Marker** for RViz (native displays, not MarkerArray)
6. **Dynamic TF** (`map→drone_reference`) updated per accepted pose
7. **Status topic** for monitoring filter health
8. **Global config.yaml** for all parameters (consistent with drone_vision)

---

## 📁 File Structure (Updated)

```
V2/src/drone_mapper/
├── CMakeLists.txt
├── package.xml
├── include/drone_mapper/
│   ├── drone_localization_node.hpp    # NEW: Unified node header
│   ├── map_metadata.hpp               # NEW: Map metadata loader
│   ├── utm_projector.hpp              # NEW: PROJ C++ wrapper
│   ├── telemetry_filter.hpp           # NEW: Filtering pipeline
│   └── trajectory_publisher.hpp       # NEW: Path + Marker publisher
├── src/
│   ├── drone_localization_node.cpp    # NEW: Main node
│   ├── map_metadata.cpp               # NEW
│   ├── utm_projector.cpp              # NEW
│   ├── telemetry_filter.cpp           # NEW
│   └── trajectory_publisher.cpp       # NEW
├── scripts/
│   ├── convert_ply_to_pcd.py          # Keep (pre-downsample for map)
│   └── downsample_pcd.py              # Keep
├── launch/
│   └── drone_mapper.launch.py         # Updated: single node
├── rviz/
│   └── drone_mapper.rviz              # Updated: Path + Pose displays
└── config/
    └── map_metadata.json              # NEW: applied_translation + bbox
```

---

## 🔧 Technical Specification

### 1. Map Metadata JSON (`config/map_metadata.json`)

```json
{
  "applied_translation": [-549860.2645008239, -4448439.33050061, -87.23800010871888],
  "bbox_min_after": [-107.0, -84.0, 0.0],
  "bbox_max_after": [107.0, 84.0, 27.0],
  "utm_zone": 29,
  "utm_hemisphere": "N"
}
```

**Source**: Generated from map processing pipeline (same as Vasco's `map.json`)

### 2. UTM Projector (`utm_projector.hpp/.cpp`)

```cpp
// Uses PROJ C++ API (libproj-dev), not subprocess
#include <proj.h>

class UtmProjector {
public:
  UtmProjector(int zone, bool south);
  ~UtmProjector();
  
  // Returns {easting, northing} in meters
  std::pair<double, double> project(double latitude, double longitude) const;
  
private:
  PJ* proj_ctx_ = nullptr;
  PJ* proj_utm_ = nullptr;
};
```

**Build**: Add `proj` to `find_package()` and `ament_target_dependencies`

### 3. Telemetry Filter (`telemetry_filter.hpp/.cpp`)

```cpp
struct TelemetryData {
  double timestamp;
  double latitude;
  double longitude;
  double height;
  double heading;
  double speed;
  bool longitude_sign_repaired = false;
};

struct FilterConfig {
  // Coordinate bounds
  double lat_min, lat_max;
  double lon_min, lon_max;
  double height_min, height_max;
  bool repair_negative_longitude = true;
  
  // Fallbacks
  double default_height;
  double default_heading;
  double max_reported_speed;
  
  // Map bounds (from metadata)
  double map_min_x, map_max_x;
  double map_min_y, map_max_y;
  double map_margin;
  
  // Jump gate
  double max_speed_mps;
  double max_gate_dt;
  double position_tolerance_m;
  
  // Path thinning
  double path_min_distance;
};

class TelemetryFilter {
public:
  TelemetryFilter(const FilterConfig& config, const UtmProjector& projector);
  
  // Returns nullopt if rejected, otherwise filtered data with map coords
  std::optional<FilteredTelemetry> process(const TelemetryData& raw);
  
  // Statistics for status topic
  struct Stats { ... };
  Stats getStats() const;
  
private:
  // Validation steps in order:
  // 1. JSON parse + regex number extraction
  // 2. Lat/lon range check
  // 3. Negative longitude repair
  // 4. Height/heading fallback
  // 5. PROJ projection (WGS84→UTM)
  // 6. Apply map translation
  // 7. Map bbox check (+ margin)
  // 8. Jump gate (speed + time + tolerance)
  // 9. Path thinning (min distance)
};
```

### 4. Trajectory Publisher (`trajectory_publisher.hpp/.cpp`)

```cpp
class TrajectoryPublisher {
public:
  TrajectoryPublisher(rclcpp::Node* node, const std::string& frame_id);
  
  void appendPose(const geometry_msgs::msg::PoseStamped& pose);
  void publishPath();
  void publishMarker();
  void publishPose(const geometry_msgs::msg::PoseStamped& pose);
  void publishOdometry(const geometry_msgs::msg::PoseStamped& pose);
  void publishTF(const geometry_msgs::msg::PoseStamped& pose);
  
private:
  // Latched QoS for Path and Marker
  rclcpp::QoS transient_qos_;
  
  nav_msgs::msg::Path path_msg_;
  visualization_msgs::msg::Marker marker_msg_;
  geometry_msgs::msg::PoseStamped last_path_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};
```

### 5. Main Node (`drone_localization_node.hpp/.cpp`)

```cpp
class DroneLocalizationNode : public rclcpp::Node {
public:
  DroneLocalizationNode(const rclcpp::NodeOptions& options);
  
private:
  void telemetryCallback(const std_msgs::msg::String::SharedPtr msg);
  void publishPathTimer();
  void publishStatusTimer();
  void loadMapMetadata();
  
  // Components
  std::unique_ptr<UtmProjector> projector_;
  std::unique_ptr<TelemetryFilter> filter_;
  std::unique_ptr<TrajectoryPublisher> traj_pub_;
  
  // Config (from config.yaml)
  FilterConfig filter_config_;
  std::string map_metadata_path_;
  std::string output_csv_path_;
  
  // CSV logging
  std::ofstream csv_file_;
};
```

### 6. Global Config (Add to `V2/config.yaml`)

```yaml
# ============================================================
# drone_mapper parameters (UPDATED - unified node)
# ============================================================
/drone_localization_node:
  ros__parameters:
    # Map metadata
    map_metadata_path: "config/map_metadata.json"
    output_csv_path: "output/drone_reference_online.csv"
    
    # UTM projection
    utm_zone: 29
    utm_south: false
    frame_id: "map"
    child_frame: "drone_reference"
    
    # Coordinate validation (Portugal defaults)
    latitude_min: 39.0
    latitude_max: 41.0
    longitude_min: -9.0
    longitude_max: -7.0
    repair_negative_longitude: true
    
    # Height/heading
    height_min: 0.0
    height_max: 120.0
    default_height_m: 25.0
    default_heading_deg: 90.0
    reported_speed_max: 30.0
    
    # Map bounds (loaded from metadata, but can override)
    map_z_offset: 0.0
    map_margin_m: 5.0
    
    # Jump gate (critical for OCR noise)
    max_speed_mps: 8.0
    max_gate_dt: 10.0
    position_tolerance_m: 1.5
    
    # Path thinning
    path_min_distance: 0.05
    path_period: 0.5
    
    # Visualization
    marker_width: 0.4
    
    # Covariance (for odometry)
    xy_variance: 2.25
    z_variance: 4.0
    yaw_variance: 0.12
```

---

## 🗺️ RViz Config Updates (`drone_mapper.rviz`)

```yaml
# Fixed Frame: map
# Displays:
# 1. PointCloud2: /map/cloud (from map_publisher) - Style: Points, Size: 2, RGB8
# 2. Path: /drone25/reference/path (latched) - Color: 20;220;255, Line Width: 0.4
# 3. Pose: /drone25/reference/pose - Axes, Length: 3
# 4. Image: /camera/rgb_roi (optional)
# 5. Grid: XY plane
```

**Key**: Uses `/drone25/reference/path` (Path msg) + `/drone25/reference/pose` (PoseStamped) — native RViz displays, not MarkerArray.

---

## 📋 Implementation Phases

### Phase 1: Infrastructure (30 min)
- [ ] Add `proj` dependency to `package.xml` and `CMakeLists.txt`
- [ ] Create `config/map_metadata.json` from existing map data
- [ ] Update `CMakeLists.txt` for new source files

### Phase 2: Core Components (90 min)
- [ ] `map_metadata.cpp/hpp` — Load JSON metadata
- [ ] `utm_projector.cpp/hpp` — PROJ C++ wrapper
- [ ] `telemetry_filter.cpp/hpp` — Full filtering pipeline
- [ ] `trajectory_publisher.cpp/hpp` — Path + Marker + Pose + Odometry + TF

### Phase 3: Main Node (45 min)
- [ ] `drone_localization_node.cpp/hpp` — Unified node
- [ ] CSV logging
- [ ] Parameter loading from config.yaml
- [ ] Timers for path/status publishing

### Phase 4: Integration (30 min)
- [ ] Update `drone_mapper.launch.py` — Single node launch
- [ ] Update `drone_mapper.rviz` — Path + Pose displays
- [ ] Update `V2/config.yaml` — All parameters namespaced

### Phase 5: Build & Test (30 min)
- [ ] Build in Docker container
- [ ] Test with rosbag playback
- [ ] Verify RViz shows map + path + pose
- [ ] Verify status topic shows filter stats

---

## 🔑 Key Differences from Current Implementation

| Current | New (Vasco-inspired) |
|---------|---------------------|
| 3 C++ nodes | 1 C++ node |
| `geodesy` for UTM | PROJ C++ API |
| Hardcoded offset | JSON metadata (`applied_translation`) |
| No filtering | Full validation pipeline |
| No jump gate | Speed + time + tolerance gate |
| No longitude repair | Auto-repair negative from OCR |
| No fallbacks | Configurable height/heading defaults |
| `MarkerArray` (heavy) | `Path` + `Marker` (latched, native) |
| Static `map→utm_zone_XX` | Dynamic `map→drone_reference` |
| No status topic | JSON status with filter stats |
| Pre-downsampled PCD | Voxel downsample at runtime (optional) |

---

## 🐳 Docker Dependencies (Add to Dockerfile.dronevision)

```dockerfile
# PROJ for WGS84→UTM (C++ API)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libproj-dev \
    && rm -rf /var/lib/apt/lists/*
```

---

## ✅ Acceptance Criteria

1. **Build**: `colcon build --packages-select drone_mapper` passes (only PCL CMP0144 warning)
2. **Map**: `/map/cloud` publishes latched PointCloud2 (1M points, RGB)
3. **Telemetry**: Subscribes to `/telemetry/data`, filters, converts, publishes
4. **Path**: `/drone25/reference/path` (latched Path) visible in RViz
5. **Pose**: `/drone25/reference/pose` (PoseStamped) visible in RViz
6. **TF**: `map → drone_reference` dynamic transform published
7. **Status**: `/drone25/reference/status` shows accepted/rejected counts
8. **CSV**: `output/drone_reference_online.csv` logged
9. **RViz**: Fixed Frame = `map`, shows 3D map + blue path + red axes pose
10. **Performance**: < 5% CPU on RTX 4070 with 30 Hz telemetry

---

## 🚀 Run Commands (After Implementation)

```bash
# Terminal 1: drone_vision (OCR)
ros2 run drone_vision unified_vision_node --ros-args --params-file config.yaml -p input_mode:=topic

# Terminal 2: Rosbag
ros2 bag play /path/to/flight.mcap --loop

# Terminal 3: drone_mapper (single node)
ros2 launch drone_mapper drone_mapper.launch.py

# Terminal 4: RViz (host)
xhost +local:root && rviz2 -d install/drone_mapper/share/drone_mapper/rviz/drone_mapper.rviz
```

---

## 📝 Notes

- **Map metadata JSON**: Must be generated once from map processing. Contains `applied_translation` (UTM offset to local map frame) and `bbox_min_after`/`bbox_max_after` (map bounds in local frame).
- **PROJ vs geodesy**: PROJ C++ API is faster and more accurate. `libproj-dev` is standard Ubuntu package.
- **Jump gate**: Critical for OCR noise. Rejects poses that move faster than `max_speed_mps` over `dt` with `position_tolerance_m` buffer.
- **Longitude repair**: OCR sometimes reads `-8.413` as `8.413`. Auto-detects and flips if within valid range.
- **Latched Path/Marker**: QoS `TRANSIENT_LOCAL` + `RELIABLE` depth=1 — late RViz subscribers get full history.