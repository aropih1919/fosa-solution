# PARC Robot — ROS 2 Topics and Message Structures

> Snapshot: 2026-11-02 16:19 UTC · ROS 2 Jazzy · ROS_DOMAIN_ID=0 · NVIDIA Orin (aarch64).
> This is a **point-in-time ROS graph inventory**, not a guarantee of future availability or an authorization to publish to every topic.
> Endpoint counts and QoS reflect the capture time. Hostname, IP addresses, endpoint GIDs and network-service details have been excluded from this team-sharing version.

## Contents

1. [Integration quick reference](#integration-quick-reference)
2. [Topic overview](#topic-overview)
3. [Topic details](#topic-details)
4. [Message type reference](#message-type-reference)
5. [Nodes](#nodes)
6. [Services](#services)
7. [Actions](#actions)
8. [TF frame relationships](#tf-frame-relationships)

## Integration quick reference

| Capability | Observed interface(s) | Notes |
| --- | --- | --- |
| Audio | `/audio`, `/audio/channel0`–`/audio/channel5`, `/speech_audio` | `audio_common_msgs/msg/AudioData`; PCM encoding, sample rate and channel semantics not present in snapshot |
| Voice activity and direction | `/is_speeching`, `/sound_direction`, `/sound_localization` | Boolean, integer and pose outputs; units/meaning need verification |
| IMU | `/camera1/camera1/imu`, `/camera2/camera2/imu` | `sensor_msgs/msg/Imu`; publishers use best-effort QoS |
| RGB cameras | `/camera1/camera1/color/image_raw`, `/camera2/camera2/color/image_raw` | Camera calibration available on corresponding `camera_info` topics |
| Depth cameras | `/camera1/camera1/depth/image_rect_raw`, `/camera2/camera2/depth/image_rect_raw` | Additional camera-derived scans at `/camera1/scan`, `/camera2/scan` |
| Range scans | `/scan`, `/scan_filtered`, `/scan_merged` | Confirm source and filtering rules before choosing a primary scan |
| Odometry | `/botwheel_explorer/odom` | `nav_msgs/msg/Odometry` |
| Velocity commands | `/cmd_vel`, `/botwheel_explorer/cmd_vel`, `/stamped_vel` | **Do not assume all are team-authorized.** Confirm official safe command entry point with organizers |
| Transforms | `/tf`, `/tf_static` | Frame inventory below; no `map` frame observed in captured graph |

**Observed publication frequencies from separate operator tests:** `/audio` approximately 15.6 ROS messages/s; both `/camera1/camera1/imu` and `/camera2/camera2/imu` approximately 200 messages/s. Audio message frequency is **not** the audio sample rate.

## Topic overview

| Topic | Message type | Publishers | Subscribers |
| --- | --- | ---: | ---: |
| [`/accessory_view/command`](#topic-accessory-view-command) | [`caytu_msgs/msg/AccessoryCommand`](#type-caytu-msgs-msg-accessorycommand) | 0 | 1 |
| [`/accessory_view/state`](#topic-accessory-view-state) | [`std_msgs/msg/String`](#type-std-msgs-msg-string) | 1 | 0 |
| [`/audio`](#topic-audio) | [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata) | 1 | 0 |
| [`/audio/channel0`](#topic-audio-channel0) | [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata) | 1 | 0 |
| [`/audio/channel1`](#topic-audio-channel1) | [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata) | 1 | 0 |
| [`/audio/channel2`](#topic-audio-channel2) | [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata) | 1 | 0 |
| [`/audio/channel3`](#topic-audio-channel3) | [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata) | 1 | 0 |
| [`/audio/channel4`](#topic-audio-channel4) | [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata) | 1 | 0 |
| [`/audio/channel5`](#topic-audio-channel5) | [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata) | 1 | 0 |
| [`/auto_vel`](#topic-auto-vel) | [`geometry_msgs/msg/Twist`](#type-geometry-msgs-msg-twist) | 1 | 0 |
| [`/battery_level`](#topic-battery-level) | [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32) | 1 | 0 |
| [`/botwheel_explorer/cmd_vel`](#topic-botwheel-explorer-cmd-vel) | [`geometry_msgs/msg/TwistStamped`](#type-geometry-msgs-msg-twiststamped) | 1 | 1 |
| [`/botwheel_explorer/odom`](#topic-botwheel-explorer-odom) | [`nav_msgs/msg/Odometry`](#type-nav-msgs-msg-odometry) | 1 | 1 |
| [`/botwheel_explorer/transition_event`](#topic-botwheel-explorer-transition-event) | [`lifecycle_msgs/msg/TransitionEvent`](#type-lifecycle-msgs-msg-transitionevent) | 1 | 0 |
| [`/camera1/camera1/accel/imu_info`](#topic-camera1-camera1-accel-imu-info) | [`realsense2_camera_msgs/msg/IMUInfo`](#type-realsense2-camera-msgs-msg-imuinfo) | 1 | 0 |
| [`/camera1/camera1/accel/metadata`](#topic-camera1-camera1-accel-metadata) | [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata) | 1 | 0 |
| [`/camera1/camera1/accel/sample`](#topic-camera1-camera1-accel-sample) | [`sensor_msgs/msg/Imu`](#type-sensor-msgs-msg-imu) | 1 | 0 |
| [`/camera1/camera1/color/camera_info`](#topic-camera1-camera1-color-camera-info) | [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo) | 1 | 0 |
| [`/camera1/camera1/color/image_raw`](#topic-camera1-camera1-color-image-raw) | [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image) | 1 | 0 |
| [`/camera1/camera1/color/metadata`](#topic-camera1-camera1-color-metadata) | [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata) | 1 | 0 |
| [`/camera1/camera1/depth/camera_info`](#topic-camera1-camera1-depth-camera-info) | [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo) | 1 | 1 |
| [`/camera1/camera1/depth/image_rect_raw`](#topic-camera1-camera1-depth-image-rect-raw) | [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image) | 1 | 1 |
| [`/camera1/camera1/depth/metadata`](#topic-camera1-camera1-depth-metadata) | [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata) | 1 | 0 |
| [`/camera1/camera1/extrinsics/depth_to_accel`](#topic-camera1-camera1-extrinsics-depth-to-accel) | [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics) | 1 | 0 |
| [`/camera1/camera1/extrinsics/depth_to_color`](#topic-camera1-camera1-extrinsics-depth-to-color) | [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics) | 1 | 0 |
| [`/camera1/camera1/extrinsics/depth_to_gyro`](#topic-camera1-camera1-extrinsics-depth-to-gyro) | [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics) | 1 | 0 |
| [`/camera1/camera1/gyro/imu_info`](#topic-camera1-camera1-gyro-imu-info) | [`realsense2_camera_msgs/msg/IMUInfo`](#type-realsense2-camera-msgs-msg-imuinfo) | 1 | 0 |
| [`/camera1/camera1/gyro/metadata`](#topic-camera1-camera1-gyro-metadata) | [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata) | 1 | 0 |
| [`/camera1/camera1/gyro/sample`](#topic-camera1-camera1-gyro-sample) | [`sensor_msgs/msg/Imu`](#type-sensor-msgs-msg-imu) | 1 | 0 |
| [`/camera1/camera1/imu`](#topic-camera1-camera1-imu) | [`sensor_msgs/msg/Imu`](#type-sensor-msgs-msg-imu) | 1 | 0 |
| [`/camera1/scan`](#topic-camera1-scan) | [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan) | 1 | 1 |
| [`/camera2/camera2/accel/imu_info`](#topic-camera2-camera2-accel-imu-info) | [`realsense2_camera_msgs/msg/IMUInfo`](#type-realsense2-camera-msgs-msg-imuinfo) | 1 | 0 |
| [`/camera2/camera2/accel/metadata`](#topic-camera2-camera2-accel-metadata) | [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata) | 1 | 0 |
| [`/camera2/camera2/accel/sample`](#topic-camera2-camera2-accel-sample) | [`sensor_msgs/msg/Imu`](#type-sensor-msgs-msg-imu) | 1 | 0 |
| [`/camera2/camera2/color/camera_info`](#topic-camera2-camera2-color-camera-info) | [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo) | 1 | 0 |
| [`/camera2/camera2/color/image_raw`](#topic-camera2-camera2-color-image-raw) | [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image) | 1 | 1 |
| [`/camera2/camera2/color/metadata`](#topic-camera2-camera2-color-metadata) | [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata) | 1 | 0 |
| [`/camera2/camera2/depth/camera_info`](#topic-camera2-camera2-depth-camera-info) | [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo) | 1 | 1 |
| [`/camera2/camera2/depth/image_rect_raw`](#topic-camera2-camera2-depth-image-rect-raw) | [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image) | 1 | 2 |
| [`/camera2/camera2/depth/metadata`](#topic-camera2-camera2-depth-metadata) | [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata) | 1 | 0 |
| [`/camera2/camera2/extrinsics/depth_to_accel`](#topic-camera2-camera2-extrinsics-depth-to-accel) | [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics) | 1 | 0 |
| [`/camera2/camera2/extrinsics/depth_to_color`](#topic-camera2-camera2-extrinsics-depth-to-color) | [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics) | 1 | 0 |
| [`/camera2/camera2/extrinsics/depth_to_gyro`](#topic-camera2-camera2-extrinsics-depth-to-gyro) | [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics) | 1 | 0 |
| [`/camera2/camera2/gyro/imu_info`](#topic-camera2-camera2-gyro-imu-info) | [`realsense2_camera_msgs/msg/IMUInfo`](#type-realsense2-camera-msgs-msg-imuinfo) | 1 | 0 |
| [`/camera2/camera2/gyro/metadata`](#topic-camera2-camera2-gyro-metadata) | [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata) | 1 | 0 |
| [`/camera2/camera2/gyro/sample`](#topic-camera2-camera2-gyro-sample) | [`sensor_msgs/msg/Imu`](#type-sensor-msgs-msg-imu) | 1 | 0 |
| [`/camera2/camera2/imu`](#topic-camera2-camera2-imu) | [`sensor_msgs/msg/Imu`](#type-sensor-msgs-msg-imu) | 1 | 0 |
| [`/camera2/scan`](#topic-camera2-scan) | [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan) | 1 | 0 |
| [`/client_count`](#topic-client-count) | [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32) | 1 | 0 |
| [`/closest_gap`](#topic-closest-gap) | [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2) | 1 | 0 |
| [`/cmd_vel`](#topic-cmd-vel) | [`geometry_msgs/msg/Twist`](#type-geometry-msgs-msg-twist) | 0 | 1 |
| [`/collision_obstacles`](#topic-collision-obstacles) | [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2) | 1 | 0 |
| [`/connected_clients`](#topic-connected-clients) | [`rosbridge_msgs/msg/ConnectedClients`](#type-rosbridge-msgs-msg-connectedclients) | 1 | 0 |
| [`/controller_manager/activity`](#topic-controller-manager-activity) | [`controller_manager_msgs/msg/ControllerManagerActivity`](#type-controller-manager-msgs-msg-controllermanageractivity) | 1 | 0 |
| [`/controller_manager/introspection_data/full`](#topic-controller-manager-introspection-data-full) | [`pal_statistics_msgs/msg/Statistics`](#type-pal-statistics-msgs-msg-statistics) | 1 | 0 |
| [`/controller_manager/introspection_data/names`](#topic-controller-manager-introspection-data-names) | [`pal_statistics_msgs/msg/StatisticsNames`](#type-pal-statistics-msgs-msg-statisticsnames) | 1 | 0 |
| [`/controller_manager/introspection_data/values`](#topic-controller-manager-introspection-data-values) | [`pal_statistics_msgs/msg/StatisticsValues`](#type-pal-statistics-msgs-msg-statisticsvalues) | 1 | 0 |
| [`/controller_manager/statistics/full`](#topic-controller-manager-statistics-full) | [`pal_statistics_msgs/msg/Statistics`](#type-pal-statistics-msgs-msg-statistics) | 1 | 0 |
| [`/controller_manager/statistics/names`](#topic-controller-manager-statistics-names) | [`pal_statistics_msgs/msg/StatisticsNames`](#type-pal-statistics-msgs-msg-statisticsnames) | 1 | 0 |
| [`/controller_manager/statistics/values`](#topic-controller-manager-statistics-values) | [`pal_statistics_msgs/msg/StatisticsValues`](#type-pal-statistics-msgs-msg-statisticsvalues) | 1 | 0 |
| [`/desired_traj`](#topic-desired-traj) | [`geometry_msgs/msg/PoseArray`](#type-geometry-msgs-msg-posearray) | 1 | 0 |
| [`/diagnostics`](#topic-diagnostics) | [`diagnostic_msgs/msg/DiagnosticArray`](#type-diagnostic-msgs-msg-diagnosticarray) | 2 | 0 |
| [`/dynamic_joint_states`](#topic-dynamic-joint-states) | [`control_msgs/msg/DynamicJointState`](#type-control-msgs-msg-dynamicjointstate) | 1 | 0 |
| [`/footprint`](#topic-footprint) | [`visualization_msgs/msg/Marker`](#type-visualization-msgs-msg-marker) | 1 | 0 |
| [`/gaps`](#topic-gaps) | [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2) | 1 | 0 |
| [`/goal`](#topic-goal) | [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped) | 0 | 1 |
| [`/is_speeching`](#topic-is-speeching) | [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool) | 1 | 0 |
| [`/joint_state_broadcaster/transition_event`](#topic-joint-state-broadcaster-transition-event) | [`lifecycle_msgs/msg/TransitionEvent`](#type-lifecycle-msgs-msg-transitionevent) | 1 | 0 |
| [`/joint_states`](#topic-joint-states) | [`sensor_msgs/msg/JointState`](#type-sensor-msgs-msg-jointstate) | 1 | 1 |
| [`/nav_goal`](#topic-nav-goal) | [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped) | 1 | 0 |
| [`/network/command`](#topic-network-command) | [`caytu_msgs/msg/NetworkCommand`](#type-caytu-msgs-msg-networkcommand) | 0 | 1 |
| [`/network/state`](#topic-network-state) | [`caytu_msgs/msg/NetworkInfo`](#type-caytu-msgs-msg-networkinfo) | 1 | 0 |
| [`/parameter_events`](#topic-parameter-events) | [`rcl_interfaces/msg/ParameterEvent`](#type-rcl-interfaces-msg-parameterevent) | 32 | 18 |
| [`/person_following_robot/color_data`](#topic-person-following-robot-color-data) | [`person_following_robot/msg/TrackedObject`](#type-person-following-robot-msg-trackedobject) | 0 | 1 |
| [`/person_following_robot/follow_enable`](#topic-person-following-robot-follow-enable) | [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool) | 1 | 1 |
| [`/person_following_robot/gui_enable`](#topic-person-following-robot-gui-enable) | [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool) | 1 | 1 |
| [`/person_following_robot/gui_state`](#topic-person-following-robot-gui-state) | [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool) | 1 | 0 |
| [`/robot_description`](#topic-robot-description) | [`std_msgs/msg/String`](#type-std-msgs-msg-string) | 1 | 1 |
| [`/ros_to_iot`](#topic-ros-to-iot) | [`std_msgs/msg/String`](#type-std-msgs-msg-string) | 1 | 0 |
| [`/rosout`](#topic-rosout) | [`rcl_interfaces/msg/Log`](#type-rcl-interfaces-msg-log) | 33 | 0 |
| [`/safe_vel`](#topic-safe-vel) | [`geometry_msgs/msg/Twist`](#type-geometry-msgs-msg-twist) | 1 | 0 |
| [`/scan`](#topic-scan) | [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan) | 1 | 1 |
| [`/scan_filtered`](#topic-scan-filtered) | [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan) | 1 | 1 |
| [`/scan_merged`](#topic-scan-merged) | [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan) | 1 | 1 |
| [`/sound_direction`](#topic-sound-direction) | [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32) | 1 | 0 |
| [`/sound_localization`](#topic-sound-localization) | [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped) | 1 | 0 |
| [`/speech_audio`](#topic-speech-audio) | [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata) | 1 | 0 |
| [`/stamped_vel`](#topic-stamped-vel) | [`geometry_msgs/msg/TwistStamped`](#type-geometry-msgs-msg-twiststamped) | 1 | 1 |
| [`/status_led`](#topic-status-led) | [`std_msgs/msg/ColorRGBA`](#type-std-msgs-msg-colorrgba) | 0 | 1 |
| [`/tf`](#topic-tf) | [`tf2_msgs/msg/TFMessage`](#type-tf2-msgs-msg-tfmessage) | 2 | 2 |
| [`/tf_static`](#topic-tf-static) | [`tf2_msgs/msg/TFMessage`](#type-tf2-msgs-msg-tfmessage) | 7 | 2 |
| [`/tracked_person/position`](#topic-tracked-person-position) | [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped) | 1 | 1 |
| [`/virt_gaps`](#topic-virt-gaps) | [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2) | 1 | 0 |
| [`/volume/command`](#topic-volume-command) | [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32) | 0 | 1 |
| [`/volume/state`](#topic-volume-state) | [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32) | 1 | 0 |
| [`/webrtc/command`](#topic-webrtc-command) | [`caytu_msgs/msg/ActionCommand`](#type-caytu-msgs-msg-actioncommand) | 0 | 1 |
| [`/webrtc/state`](#topic-webrtc-state) | [`std_msgs/msg/String`](#type-std-msgs-msg-string) | 1 | 0 |

## Topic details

### `/accessory_view/command`
<a id="topic-accessory-view-command"></a>

- **Message type:** [`caytu_msgs/msg/AccessoryCommand`](#type-caytu-msgs-msg-accessorycommand)
- **Publishers:** 0
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| SUBSCRIPTION | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/accessory_view/state`
<a id="topic-accessory-view-state"></a>

- **Message type:** [`std_msgs/msg/String`](#type-std-msgs-msg-string)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/audio`
<a id="topic-audio"></a>

- **Message type:** [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `respeaker_node` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/audio/channel0`
<a id="topic-audio-channel0"></a>

- **Message type:** [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `respeaker_node` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/audio/channel1`
<a id="topic-audio-channel1"></a>

- **Message type:** [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `respeaker_node` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/audio/channel2`
<a id="topic-audio-channel2"></a>

- **Message type:** [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `respeaker_node` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/audio/channel3`
<a id="topic-audio-channel3"></a>

- **Message type:** [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `respeaker_node` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/audio/channel4`
<a id="topic-audio-channel4"></a>

- **Message type:** [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `respeaker_node` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/audio/channel5`
<a id="topic-audio-channel5"></a>

- **Message type:** [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `respeaker_node` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/auto_vel`
<a id="topic-auto-vel"></a>

- **Message type:** [`geometry_msgs/msg/Twist`](#type-geometry-msgs-msg-twist)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/battery_level`
<a id="topic-battery-level"></a>

- **Message type:** [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/botwheel_explorer/cmd_vel`
<a id="topic-botwheel-explorer-cmd-vel"></a>

- **Message type:** [`geometry_msgs/msg/TwistStamped`](#type-geometry-msgs-msg-twiststamped)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (10) |
| SUBSCRIPTION | `botwheel_explorer` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/botwheel_explorer/odom`
<a id="topic-botwheel-explorer-odom"></a>

- **Message type:** [`nav_msgs/msg/Odometry`](#type-nav-msgs-msg-odometry)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `botwheel_explorer` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| SUBSCRIPTION | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/botwheel_explorer/transition_event`
<a id="topic-botwheel-explorer-transition-event"></a>

- **Message type:** [`lifecycle_msgs/msg/TransitionEvent`](#type-lifecycle-msgs-msg-transitionevent)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `botwheel_explorer` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera1/camera1/accel/imu_info`
<a id="topic-camera1-camera1-accel-imu-info"></a>

- **Message type:** [`realsense2_camera_msgs/msg/IMUInfo`](#type-realsense2-camera-msgs-msg-imuinfo)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/camera1/camera1/accel/metadata`
<a id="topic-camera1-camera1-accel-metadata"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera1/camera1/accel/sample`
<a id="topic-camera1-camera1-accel-sample"></a>

- **Message type:** [`sensor_msgs/msg/Imu`](#type-sensor-msgs-msg-imu)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | BEST_EFFORT | VOLATILE | KEEP_LAST (5) |

### `/camera1/camera1/color/camera_info`
<a id="topic-camera1-camera1-color-camera-info"></a>

- **Message type:** [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera1/camera1/color/image_raw`
<a id="topic-camera1-camera1-color-image-raw"></a>

- **Message type:** [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/camera1/camera1/color/metadata`
<a id="topic-camera1-camera1-color-metadata"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera1/camera1/depth/camera_info`
<a id="topic-camera1-camera1-depth-camera-info"></a>

- **Message type:** [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | VOLATILE | KEEP_LAST (10) |
| SUBSCRIPTION | `depthimage_to_laserscan_1` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/camera1/camera1/depth/image_rect_raw`
<a id="topic-camera1-camera1-depth-image-rect-raw"></a>

- **Message type:** [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| SUBSCRIPTION | `depthimage_to_laserscan_1` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/camera1/camera1/depth/metadata`
<a id="topic-camera1-camera1-depth-metadata"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera1/camera1/extrinsics/depth_to_accel`
<a id="topic-camera1-camera1-extrinsics-depth-to-accel"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/camera1/camera1/extrinsics/depth_to_color`
<a id="topic-camera1-camera1-extrinsics-depth-to-color"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/camera1/camera1/extrinsics/depth_to_gyro`
<a id="topic-camera1-camera1-extrinsics-depth-to-gyro"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/camera1/camera1/gyro/imu_info`
<a id="topic-camera1-camera1-gyro-imu-info"></a>

- **Message type:** [`realsense2_camera_msgs/msg/IMUInfo`](#type-realsense2-camera-msgs-msg-imuinfo)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/camera1/camera1/gyro/metadata`
<a id="topic-camera1-camera1-gyro-metadata"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera1/camera1/gyro/sample`
<a id="topic-camera1-camera1-gyro-sample"></a>

- **Message type:** [`sensor_msgs/msg/Imu`](#type-sensor-msgs-msg-imu)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | BEST_EFFORT | VOLATILE | KEEP_LAST (5) |

### `/camera1/camera1/imu`
<a id="topic-camera1-camera1-imu"></a>

- **Message type:** [`sensor_msgs/msg/Imu`](#type-sensor-msgs-msg-imu)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera1` | BEST_EFFORT | VOLATILE | KEEP_LAST (5) |

### `/camera1/scan`
<a id="topic-camera1-scan"></a>

- **Message type:** [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `depthimage_to_laserscan_1` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| SUBSCRIPTION | `scan_merger` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera2/camera2/accel/imu_info`
<a id="topic-camera2-camera2-accel-imu-info"></a>

- **Message type:** [`realsense2_camera_msgs/msg/IMUInfo`](#type-realsense2-camera-msgs-msg-imuinfo)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/camera2/camera2/accel/metadata`
<a id="topic-camera2-camera2-accel-metadata"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera2/camera2/accel/sample`
<a id="topic-camera2-camera2-accel-sample"></a>

- **Message type:** [`sensor_msgs/msg/Imu`](#type-sensor-msgs-msg-imu)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | BEST_EFFORT | VOLATILE | KEEP_LAST (5) |

### `/camera2/camera2/color/camera_info`
<a id="topic-camera2-camera2-color-camera-info"></a>

- **Message type:** [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera2/camera2/color/image_raw`
<a id="topic-camera2-camera2-color-image-raw"></a>

- **Message type:** [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| SUBSCRIPTION | `byte_tracker` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera2/camera2/color/metadata`
<a id="topic-camera2-camera2-color-metadata"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera2/camera2/depth/camera_info`
<a id="topic-camera2-camera2-depth-camera-info"></a>

- **Message type:** [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | VOLATILE | KEEP_LAST (10) |
| SUBSCRIPTION | `depthimage_to_laserscan_2` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/camera2/camera2/depth/image_rect_raw`
<a id="topic-camera2-camera2-depth-image-rect-raw"></a>

- **Message type:** [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image)
- **Publishers:** 1
- **Subscribers:** 2

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| SUBSCRIPTION | `byte_tracker` | RELIABLE | VOLATILE | KEEP_LAST (10) |
| SUBSCRIPTION | `depthimage_to_laserscan_2` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/camera2/camera2/depth/metadata`
<a id="topic-camera2-camera2-depth-metadata"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera2/camera2/extrinsics/depth_to_accel`
<a id="topic-camera2-camera2-extrinsics-depth-to-accel"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/camera2/camera2/extrinsics/depth_to_color`
<a id="topic-camera2-camera2-extrinsics-depth-to-color"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/camera2/camera2/extrinsics/depth_to_gyro`
<a id="topic-camera2-camera2-extrinsics-depth-to-gyro"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/camera2/camera2/gyro/imu_info`
<a id="topic-camera2-camera2-gyro-imu-info"></a>

- **Message type:** [`realsense2_camera_msgs/msg/IMUInfo`](#type-realsense2-camera-msgs-msg-imuinfo)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/camera2/camera2/gyro/metadata`
<a id="topic-camera2-camera2-gyro-metadata"></a>

- **Message type:** [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/camera2/camera2/gyro/sample`
<a id="topic-camera2-camera2-gyro-sample"></a>

- **Message type:** [`sensor_msgs/msg/Imu`](#type-sensor-msgs-msg-imu)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | BEST_EFFORT | VOLATILE | KEEP_LAST (5) |

### `/camera2/camera2/imu`
<a id="topic-camera2-camera2-imu"></a>

- **Message type:** [`sensor_msgs/msg/Imu`](#type-sensor-msgs-msg-imu)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | BEST_EFFORT | VOLATILE | KEEP_LAST (5) |

### `/camera2/scan`
<a id="topic-camera2-scan"></a>

- **Message type:** [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `depthimage_to_laserscan_2` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/client_count`
<a id="topic-client-count"></a>

- **Message type:** [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/closest_gap`
<a id="topic-closest-gap"></a>

- **Message type:** [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `obstacle_map` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/cmd_vel`
<a id="topic-cmd-vel"></a>

- **Message type:** [`geometry_msgs/msg/Twist`](#type-geometry-msgs-msg-twist)
- **Publishers:** 0
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| SUBSCRIPTION | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/collision_obstacles`
<a id="topic-collision-obstacles"></a>

- **Message type:** [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/connected_clients`
<a id="topic-connected-clients"></a>

- **Message type:** [`rosbridge_msgs/msg/ConnectedClients`](#type-rosbridge-msgs-msg-connectedclients)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/controller_manager/activity`
<a id="topic-controller-manager-activity"></a>

- **Message type:** [`controller_manager_msgs/msg/ControllerManagerActivity`](#type-controller-manager-msgs-msg-controllermanageractivity)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `controller_manager` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/controller_manager/introspection_data/full`
<a id="topic-controller-manager-introspection-data-full"></a>

- **Message type:** [`pal_statistics_msgs/msg/Statistics`](#type-pal-statistics-msgs-msg-statistics)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `controller_manager` | BEST_EFFORT | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/controller_manager/introspection_data/names`
<a id="topic-controller-manager-introspection-data-names"></a>

- **Message type:** [`pal_statistics_msgs/msg/StatisticsNames`](#type-pal-statistics-msgs-msg-statisticsnames)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `controller_manager` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/controller_manager/introspection_data/values`
<a id="topic-controller-manager-introspection-data-values"></a>

- **Message type:** [`pal_statistics_msgs/msg/StatisticsValues`](#type-pal-statistics-msgs-msg-statisticsvalues)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `controller_manager` | BEST_EFFORT | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/controller_manager/statistics/full`
<a id="topic-controller-manager-statistics-full"></a>

- **Message type:** [`pal_statistics_msgs/msg/Statistics`](#type-pal-statistics-msgs-msg-statistics)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `controller_manager` | BEST_EFFORT | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/controller_manager/statistics/names`
<a id="topic-controller-manager-statistics-names"></a>

- **Message type:** [`pal_statistics_msgs/msg/StatisticsNames`](#type-pal-statistics-msgs-msg-statisticsnames)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `controller_manager` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/controller_manager/statistics/values`
<a id="topic-controller-manager-statistics-values"></a>

- **Message type:** [`pal_statistics_msgs/msg/StatisticsValues`](#type-pal-statistics-msgs-msg-statisticsvalues)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `controller_manager` | BEST_EFFORT | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/desired_traj`
<a id="topic-desired-traj"></a>

- **Message type:** [`geometry_msgs/msg/PoseArray`](#type-geometry-msgs-msg-posearray)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/diagnostics`
<a id="topic-diagnostics"></a>

- **Message type:** [`diagnostic_msgs/msg/DiagnosticArray`](#type-diagnostic-msgs-msg-diagnosticarray)
- **Publishers:** 2
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `scan_filter` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| PUBLISHER | `controller_manager` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/dynamic_joint_states`
<a id="topic-dynamic-joint-states"></a>

- **Message type:** [`control_msgs/msg/DynamicJointState`](#type-control-msgs-msg-dynamicjointstate)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `joint_state_broadcaster` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/footprint`
<a id="topic-footprint"></a>

- **Message type:** [`visualization_msgs/msg/Marker`](#type-visualization-msgs-msg-marker)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/gaps`
<a id="topic-gaps"></a>

- **Message type:** [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `obstacle_map` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/goal`
<a id="topic-goal"></a>

- **Message type:** [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped)
- **Publishers:** 0
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| SUBSCRIPTION | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/is_speeching`
<a id="topic-is-speeching"></a>

- **Message type:** [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `respeaker_node` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/joint_state_broadcaster/transition_event`
<a id="topic-joint-state-broadcaster-transition-event"></a>

- **Message type:** [`lifecycle_msgs/msg/TransitionEvent`](#type-lifecycle-msgs-msg-transitionevent)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `joint_state_broadcaster` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/joint_states`
<a id="topic-joint-states"></a>

- **Message type:** [`sensor_msgs/msg/JointState`](#type-sensor-msgs-msg-jointstate)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `joint_state_broadcaster` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| SUBSCRIPTION | `robot_state_publisher` | BEST_EFFORT | VOLATILE | KEEP_LAST (5) |

### `/nav_goal`
<a id="topic-nav-goal"></a>

- **Message type:** [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/network/command`
<a id="topic-network-command"></a>

- **Message type:** [`caytu_msgs/msg/NetworkCommand`](#type-caytu-msgs-msg-networkcommand)
- **Publishers:** 0
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| SUBSCRIPTION | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/network/state`
<a id="topic-network-state"></a>

- **Message type:** [`caytu_msgs/msg/NetworkInfo`](#type-caytu-msgs-msg-networkinfo)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/parameter_events`
<a id="topic-parameter-events"></a>

- **Message type:** [`rcl_interfaces/msg/ParameterEvent`](#type-rcl-interfaces-msg-parameterevent)
- **Publishers:** 32
- **Subscribers:** 18

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| PUBLISHER | `person_follower` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `scan_merger` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `rplidar_node` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `camera2` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `byte_tracker` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `depthimage_to_laserscan_2` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `camera1` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `scan_filter` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `depthimage_to_laserscan_1` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `ros_to_iot` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `respeaker_node` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `_ros2cli_daemon_0_2d301807e34d487aac0587b2d78ac200` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `robot_state_publisher` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `_ros2cli_10535` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `camera1_tf` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `camera2_tf` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `obstacle_map` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `controller_manager` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `diffbot` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `joint_state_broadcaster` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `botwheel_explorer` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `static_tf2_broadcaster` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| PUBLISHER | `laser_tf` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `rplidar_node` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `camera2` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `depthimage_to_laserscan_2` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `camera1` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `scan_filter` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `depthimage_to_laserscan_1` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `robot_state_publisher` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `robot_state_publisher` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `camera1_tf` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `camera2_tf` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `transform_listener_impl_aaaaf2770f00` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `obstacle_map` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `controller_manager` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `diffbot` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `joint_state_broadcaster` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `botwheel_explorer` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `laser_tf` | RELIABLE | VOLATILE | KEEP_LAST (1000) |

### `/person_following_robot/color_data`
<a id="topic-person-following-robot-color-data"></a>

- **Message type:** [`person_following_robot/msg/TrackedObject`](#type-person-following-robot-msg-trackedobject)
- **Publishers:** 0
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| SUBSCRIPTION | `byte_tracker` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/person_following_robot/follow_enable`
<a id="topic-person-following-robot-follow-enable"></a>

- **Message type:** [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `byte_tracker` | RELIABLE | VOLATILE | KEEP_LAST (10) |
| SUBSCRIPTION | `person_follower` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/person_following_robot/gui_enable`
<a id="topic-person-following-robot-gui-enable"></a>

- **Message type:** [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `byte_tracker` | RELIABLE | VOLATILE | KEEP_LAST (10) |
| SUBSCRIPTION | `byte_tracker` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/person_following_robot/gui_state`
<a id="topic-person-following-robot-gui-state"></a>

- **Message type:** [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `byte_tracker` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/robot_description`
<a id="topic-robot-description"></a>

- **Message type:** [`std_msgs/msg/String`](#type-std-msgs-msg-string)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `robot_state_publisher` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| SUBSCRIPTION | `controller_manager` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/ros_to_iot`
<a id="topic-ros-to-iot"></a>

- **Message type:** [`std_msgs/msg/String`](#type-std-msgs-msg-string)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `ros_to_iot` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/rosout`
<a id="topic-rosout"></a>

- **Message type:** [`rcl_interfaces/msg/Log`](#type-rcl-interfaces-msg-log)
- **Publishers:** 33
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `person_follower` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `scan_merger` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `rplidar_node` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `camera2` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `byte_tracker` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `depthimage_to_laserscan_2` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `_ros2cli_10598` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `camera1` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `scan_filter` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `depthimage_to_laserscan_1` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `ros_to_iot` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `respeaker_node` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `_ros2cli_daemon_0_2d301807e34d487aac0587b2d78ac200` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `robot_state_publisher` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `camera1_tf` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `camera2_tf` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `obstacle_avoidance` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `transform_listener_impl_aaaaf2770f00` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `obstacle_map` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `controller_manager` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `diffbot` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `joint_state_broadcaster` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `botwheel_explorer` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `static_tf2_broadcaster` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |
| PUBLISHER | `laser_tf` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1000) |

### `/safe_vel`
<a id="topic-safe-vel"></a>

- **Message type:** [`geometry_msgs/msg/Twist`](#type-geometry-msgs-msg-twist)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/scan`
<a id="topic-scan"></a>

- **Message type:** [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `rplidar_node` | RELIABLE | VOLATILE | KEEP_LAST (10) |
| SUBSCRIPTION | `scan_filter` | BEST_EFFORT | VOLATILE | KEEP_LAST (5) |

### `/scan_filtered`
<a id="topic-scan-filtered"></a>

- **Message type:** [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `scan_filter` | RELIABLE | VOLATILE | KEEP_LAST (1000) |
| SUBSCRIPTION | `scan_merger` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/scan_merged`
<a id="topic-scan-merged"></a>

- **Message type:** [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `scan_merger` | RELIABLE | VOLATILE | KEEP_LAST (10) |
| SUBSCRIPTION | `obstacle_map` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/sound_direction`
<a id="topic-sound-direction"></a>

- **Message type:** [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `respeaker_node` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/sound_localization`
<a id="topic-sound-localization"></a>

- **Message type:** [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `respeaker_node` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |

### `/speech_audio`
<a id="topic-speech-audio"></a>

- **Message type:** [`audio_common_msgs/msg/AudioData`](#type-audio-common-msgs-msg-audiodata)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `respeaker_node` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/stamped_vel`
<a id="topic-stamped-vel"></a>

- **Message type:** [`geometry_msgs/msg/TwistStamped`](#type-geometry-msgs-msg-twiststamped)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `person_follower` | RELIABLE | VOLATILE | KEEP_LAST (10) |
| SUBSCRIPTION | `obstacle_avoidance` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/status_led`
<a id="topic-status-led"></a>

- **Message type:** [`std_msgs/msg/ColorRGBA`](#type-std-msgs-msg-colorrgba)
- **Publishers:** 0
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| SUBSCRIPTION | `respeaker_node` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/tf`
<a id="topic-tf"></a>

- **Message type:** [`tf2_msgs/msg/TFMessage`](#type-tf2-msgs-msg-tfmessage)
- **Publishers:** 2
- **Subscribers:** 2

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `robot_state_publisher` | RELIABLE | VOLATILE | KEEP_LAST (100) |
| PUBLISHER | `botwheel_explorer` | RELIABLE | VOLATILE | KEEP_LAST (1) |
| SUBSCRIPTION | `scan_merger` | RELIABLE | VOLATILE | KEEP_LAST (100) |
| SUBSCRIPTION | `transform_listener_impl_aaaaf2770f00` | RELIABLE | VOLATILE | KEEP_LAST (100) |

### `/tf_static`
<a id="topic-tf-static"></a>

- **Message type:** [`tf2_msgs/msg/TFMessage`](#type-tf2-msgs-msg-tfmessage)
- **Publishers:** 7
- **Subscribers:** 2

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `camera2` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `camera1` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `robot_state_publisher` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `camera1_tf` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `camera2_tf` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `static_tf2_broadcaster` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| PUBLISHER | `laser_tf` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (1) |
| SUBSCRIPTION | `scan_merger` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (100) |
| SUBSCRIPTION | `transform_listener_impl_aaaaf2770f00` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST (100) |

### `/tracked_person/position`
<a id="topic-tracked-person-position"></a>

- **Message type:** [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped)
- **Publishers:** 1
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `byte_tracker` | RELIABLE | VOLATILE | KEEP_LAST (10) |
| SUBSCRIPTION | `person_follower` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/virt_gaps`
<a id="topic-virt-gaps"></a>

- **Message type:** [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `obstacle_map` | RELIABLE | VOLATILE | KEEP_LAST (10) |

### `/volume/command`
<a id="topic-volume-command"></a>

- **Message type:** [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32)
- **Publishers:** 0
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| SUBSCRIPTION | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/volume/state`
<a id="topic-volume-state"></a>

- **Message type:** [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/webrtc/command`
<a id="topic-webrtc-command"></a>

- **Message type:** [`caytu_msgs/msg/ActionCommand`](#type-caytu-msgs-msg-actioncommand)
- **Publishers:** 0
- **Subscribers:** 1

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| SUBSCRIPTION | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |

### `/webrtc/state`
<a id="topic-webrtc-state"></a>

- **Message type:** [`std_msgs/msg/String`](#type-std-msgs-msg-string)
- **Publishers:** 1
- **Subscribers:** 0

| Endpoint | Node | Reliability | Durability | History |
| --- | --- | --- | --- | --- |
| PUBLISHER | `_NODE_NAME_UNKNOWN_` | RELIABLE | VOLATILE | KEEP_LAST (1) |

## Message type reference

Definitions below are transcribed from `ros2 interface show` output captured on the robot.

### `audio_common_msgs/msg/AudioData`
<a id="type-audio-common-msgs-msg-audiodata"></a>

**Used by topics:** [`/audio`](#topic-audio), [`/audio/channel0`](#topic-audio-channel0), [`/audio/channel1`](#topic-audio-channel1), [`/audio/channel2`](#topic-audio-channel2), [`/audio/channel3`](#topic-audio-channel3), [`/audio/channel4`](#topic-audio-channel4), [`/audio/channel5`](#topic-audio-channel5), [`/speech_audio`](#topic-speech-audio)

```text
uint8[] data
```

### `caytu_msgs/msg/AccessoryCommand`
<a id="type-caytu-msgs-msg-accessorycommand"></a>

**Used by topics:** [`/accessory_view/command`](#topic-accessory-view-command)

```text
string url
string action
```

### `caytu_msgs/msg/ActionCommand`
<a id="type-caytu-msgs-msg-actioncommand"></a>

**Used by topics:** [`/webrtc/command`](#topic-webrtc-command)

```text
string action
```

### `caytu_msgs/msg/NetworkCommand`
<a id="type-caytu-msgs-msg-networkcommand"></a>

**Used by topics:** [`/network/command`](#topic-network-command)

```text
string action
string ssid
string password
```

### `caytu_msgs/msg/NetworkInfo`
<a id="type-caytu-msgs-msg-networkinfo"></a>

**Used by topics:** [`/network/state`](#topic-network-state)

```text
string ssid
string internal_ip
string external_ip
string gateway
string device_mac
string connection_status
WiFiNetwork[] available_networks
    string ssid
    int32 signal_strength
```

### `controller_manager_msgs/msg/ControllerManagerActivity`
<a id="type-controller-manager-msgs-msg-controllermanageractivity"></a>

**Used by topics:** [`/controller_manager/activity`](#topic-controller-manager-activity)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

NamedLifecycleState[] controllers
    string name
    lifecycle_msgs/State state
        uint8 PRIMARY_STATE_UNKNOWN = 0
        uint8 PRIMARY_STATE_UNCONFIGURED = 1
        uint8 PRIMARY_STATE_INACTIVE = 2
        uint8 PRIMARY_STATE_ACTIVE = 3
        uint8 PRIMARY_STATE_FINALIZED = 4
        uint8 TRANSITION_STATE_CONFIGURING = 10
        uint8 TRANSITION_STATE_CLEANINGUP = 11
        uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
        uint8 TRANSITION_STATE_ACTIVATING = 13
        uint8 TRANSITION_STATE_DEACTIVATING = 14
        uint8 TRANSITION_STATE_ERRORPROCESSING = 15
        uint8 id
        string label

NamedLifecycleState[] hardware_components
    string name
    lifecycle_msgs/State state
        uint8 PRIMARY_STATE_UNKNOWN = 0
        uint8 PRIMARY_STATE_UNCONFIGURED = 1
        uint8 PRIMARY_STATE_INACTIVE = 2
        uint8 PRIMARY_STATE_ACTIVE = 3
        uint8 PRIMARY_STATE_FINALIZED = 4
        uint8 TRANSITION_STATE_CONFIGURING = 10
        uint8 TRANSITION_STATE_CLEANINGUP = 11
        uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
        uint8 TRANSITION_STATE_ACTIVATING = 13
        uint8 TRANSITION_STATE_DEACTIVATING = 14
        uint8 TRANSITION_STATE_ERRORPROCESSING = 15
        uint8 id
        string label
```

### `control_msgs/msg/DynamicJointState`
<a id="type-control-msgs-msg-dynamicjointstate"></a>

**Used by topics:** [`/dynamic_joint_states`](#topic-dynamic-joint-states)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

string[] joint_names
InterfaceValue[] interface_values
    string[] interface_names
    float64[] values
```

### `diagnostic_msgs/msg/DiagnosticArray`
<a id="type-diagnostic-msgs-msg-diagnosticarray"></a>

**Used by topics:** [`/diagnostics`](#topic-diagnostics)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id
DiagnosticStatus[] status
    byte OK=0
    byte WARN=1
    byte ERROR=2
    byte STALE=3
    byte level
    string name
    string message
    string hardware_id
    KeyValue[] values
        string key
        string value
```

### `geometry_msgs/msg/PoseArray`
<a id="type-geometry-msgs-msg-posearray"></a>

**Used by topics:** [`/desired_traj`](#topic-desired-traj)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

Pose[] poses
    Point position
        float64 x
        float64 y
        float64 z
    Quaternion orientation
        float64 x 0
        float64 y 0
        float64 z 0
        float64 w 1
```

### `geometry_msgs/msg/PoseStamped`
<a id="type-geometry-msgs-msg-posestamped"></a>

**Used by topics:** [`/goal`](#topic-goal), [`/nav_goal`](#topic-nav-goal), [`/sound_localization`](#topic-sound-localization), [`/tracked_person/position`](#topic-tracked-person-position)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id
Pose pose
    Point position
        float64 x
        float64 y
        float64 z
    Quaternion orientation
        float64 x 0
        float64 y 0
        float64 z 0
        float64 w 1
```

### `geometry_msgs/msg/Twist`
<a id="type-geometry-msgs-msg-twist"></a>

**Used by topics:** [`/auto_vel`](#topic-auto-vel), [`/cmd_vel`](#topic-cmd-vel), [`/safe_vel`](#topic-safe-vel)

```text
Vector3  linear
    float64 x
    float64 y
    float64 z
Vector3  angular
    float64 x
    float64 y
    float64 z
```

### `geometry_msgs/msg/TwistStamped`
<a id="type-geometry-msgs-msg-twiststamped"></a>

**Used by topics:** [`/botwheel_explorer/cmd_vel`](#topic-botwheel-explorer-cmd-vel), [`/stamped_vel`](#topic-stamped-vel)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id
Twist twist
    Vector3  linear
        float64 x
        float64 y
        float64 z
    Vector3  angular
        float64 x
        float64 y
        float64 z
```

### `lifecycle_msgs/msg/TransitionEvent`
<a id="type-lifecycle-msgs-msg-transitionevent"></a>

**Used by topics:** [`/botwheel_explorer/transition_event`](#topic-botwheel-explorer-transition-event), [`/joint_state_broadcaster/transition_event`](#topic-joint-state-broadcaster-transition-event)

```text
uint64 timestamp

Transition transition
    uint8 TRANSITION_CREATE = 0
    uint8 TRANSITION_CONFIGURE = 1
    uint8 TRANSITION_CLEANUP = 2
    uint8 TRANSITION_ACTIVATE = 3
    uint8 TRANSITION_DEACTIVATE = 4
    uint8 TRANSITION_UNCONFIGURED_SHUTDOWN  = 5
    uint8 TRANSITION_INACTIVE_SHUTDOWN = 6
    uint8 TRANSITION_ACTIVE_SHUTDOWN = 7
    uint8 TRANSITION_DESTROY = 8
    uint8 TRANSITION_ON_CONFIGURE_SUCCESS = 10
    uint8 TRANSITION_ON_CONFIGURE_FAILURE = 11
    uint8 TRANSITION_ON_CONFIGURE_ERROR = 12
    uint8 TRANSITION_ON_CLEANUP_SUCCESS = 20
    uint8 TRANSITION_ON_CLEANUP_FAILURE = 21
    uint8 TRANSITION_ON_CLEANUP_ERROR = 22
    uint8 TRANSITION_ON_ACTIVATE_SUCCESS = 30
    uint8 TRANSITION_ON_ACTIVATE_FAILURE = 31
    uint8 TRANSITION_ON_ACTIVATE_ERROR = 32
    uint8 TRANSITION_ON_DEACTIVATE_SUCCESS = 40
    uint8 TRANSITION_ON_DEACTIVATE_FAILURE = 41
    uint8 TRANSITION_ON_DEACTIVATE_ERROR = 42
    uint8 TRANSITION_ON_SHUTDOWN_SUCCESS = 50
    uint8 TRANSITION_ON_SHUTDOWN_FAILURE = 51
    uint8 TRANSITION_ON_SHUTDOWN_ERROR = 52
    uint8 TRANSITION_ON_ERROR_SUCCESS = 60
    uint8 TRANSITION_ON_ERROR_FAILURE = 61
    uint8 TRANSITION_ON_ERROR_ERROR = 62
    uint8 TRANSITION_CALLBACK_SUCCESS = 97
    uint8 TRANSITION_CALLBACK_FAILURE = 98
    uint8 TRANSITION_CALLBACK_ERROR = 99
    uint8 id
    string label

State start_state
    uint8 PRIMARY_STATE_UNKNOWN = 0
    uint8 PRIMARY_STATE_UNCONFIGURED = 1
    uint8 PRIMARY_STATE_INACTIVE = 2
    uint8 PRIMARY_STATE_ACTIVE = 3
    uint8 PRIMARY_STATE_FINALIZED = 4
    uint8 TRANSITION_STATE_CONFIGURING = 10
    uint8 TRANSITION_STATE_CLEANINGUP = 11
    uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
    uint8 TRANSITION_STATE_ACTIVATING = 13
    uint8 TRANSITION_STATE_DEACTIVATING = 14
    uint8 TRANSITION_STATE_ERRORPROCESSING = 15
    uint8 id
    string label

State goal_state
    uint8 PRIMARY_STATE_UNKNOWN = 0
    uint8 PRIMARY_STATE_UNCONFIGURED = 1
    uint8 PRIMARY_STATE_INACTIVE = 2
    uint8 PRIMARY_STATE_ACTIVE = 3
    uint8 PRIMARY_STATE_FINALIZED = 4
    uint8 TRANSITION_STATE_CONFIGURING = 10
    uint8 TRANSITION_STATE_CLEANINGUP = 11
    uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
    uint8 TRANSITION_STATE_ACTIVATING = 13
    uint8 TRANSITION_STATE_DEACTIVATING = 14
    uint8 TRANSITION_STATE_ERRORPROCESSING = 15
    uint8 id
    string label
```

### `nav_msgs/msg/Odometry`
<a id="type-nav-msgs-msg-odometry"></a>

**Used by topics:** [`/botwheel_explorer/odom`](#topic-botwheel-explorer-odom)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

string child_frame_id

geometry_msgs/PoseWithCovariance pose
    Pose pose
        Point position
            float64 x
            float64 y
            float64 z
        Quaternion orientation
            float64 x 0
            float64 y 0
            float64 z 0
            float64 w 1
    float64[36] covariance

geometry_msgs/TwistWithCovariance twist
    Twist twist
        Vector3  linear
            float64 x
            float64 y
            float64 z
        Vector3  angular
            float64 x
            float64 y
            float64 z
    float64[36] covariance
```

### `pal_statistics_msgs/msg/Statistics`
<a id="type-pal-statistics-msgs-msg-statistics"></a>

**Used by topics:** [`/controller_manager/introspection_data/full`](#topic-controller-manager-introspection-data-full), [`/controller_manager/statistics/full`](#topic-controller-manager-statistics-full)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

Statistic[] statistics
    string name
    float64 value
```

### `pal_statistics_msgs/msg/StatisticsNames`
<a id="type-pal-statistics-msgs-msg-statisticsnames"></a>

**Used by topics:** [`/controller_manager/introspection_data/names`](#topic-controller-manager-introspection-data-names), [`/controller_manager/statistics/names`](#topic-controller-manager-statistics-names)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

string[] names
uint32 names_version
```

### `pal_statistics_msgs/msg/StatisticsValues`
<a id="type-pal-statistics-msgs-msg-statisticsvalues"></a>

**Used by topics:** [`/controller_manager/introspection_data/values`](#topic-controller-manager-introspection-data-values), [`/controller_manager/statistics/values`](#topic-controller-manager-statistics-values)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

float64[] values
uint32 names_version
```

### `person_following_robot/msg/TrackedObject`
<a id="type-person-following-robot-msg-trackedobject"></a>

**Used by topics:** [`/person_following_robot/color_data`](#topic-person-following-robot-color-data)

```text
string name
int32 id
bool success
float32[] position
```

### `rcl_interfaces/msg/Log`
<a id="type-rcl-interfaces-msg-log"></a>

**Used by topics:** [`/rosout`](#topic-rosout)

```text
uint8 DEBUG=10

uint8 INFO=20

uint8 WARN=30

uint8 ERROR=40

uint8 FATAL=50

builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec

uint8 level

string name

string msg

string file

string function

uint32 line
```

### `rcl_interfaces/msg/ParameterEvent`
<a id="type-rcl-interfaces-msg-parameterevent"></a>

**Used by topics:** [`/parameter_events`](#topic-parameter-events)

```text
builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec

string node

Parameter[] new_parameters
    string name
    ParameterValue value
        uint8 type
        bool bool_value
        int64 integer_value
        float64 double_value
        string string_value
        byte[] byte_array_value
        bool[] bool_array_value
        int64[] integer_array_value
        float64[] double_array_value
        string[] string_array_value

Parameter[] changed_parameters
    string name
    ParameterValue value
        uint8 type
        bool bool_value
        int64 integer_value
        float64 double_value
        string string_value
        byte[] byte_array_value
        bool[] bool_array_value
        int64[] integer_array_value
        float64[] double_array_value
        string[] string_array_value

Parameter[] deleted_parameters
    string name
    ParameterValue value
        uint8 type
        bool bool_value
        int64 integer_value
        float64 double_value
        string string_value
        byte[] byte_array_value
        bool[] bool_array_value
        int64[] integer_array_value
        float64[] double_array_value
        string[] string_array_value
```

### `realsense2_camera_msgs/msg/Extrinsics`
<a id="type-realsense2-camera-msgs-msg-extrinsics"></a>

**Used by topics:** [`/camera1/camera1/extrinsics/depth_to_accel`](#topic-camera1-camera1-extrinsics-depth-to-accel), [`/camera1/camera1/extrinsics/depth_to_color`](#topic-camera1-camera1-extrinsics-depth-to-color), [`/camera1/camera1/extrinsics/depth_to_gyro`](#topic-camera1-camera1-extrinsics-depth-to-gyro), [`/camera2/camera2/extrinsics/depth_to_accel`](#topic-camera2-camera2-extrinsics-depth-to-accel), [`/camera2/camera2/extrinsics/depth_to_color`](#topic-camera2-camera2-extrinsics-depth-to-color), [`/camera2/camera2/extrinsics/depth_to_gyro`](#topic-camera2-camera2-extrinsics-depth-to-gyro)

```text
float64[9] rotation
float64[3] translation
```

### `realsense2_camera_msgs/msg/IMUInfo`
<a id="type-realsense2-camera-msgs-msg-imuinfo"></a>

**Used by topics:** [`/camera1/camera1/accel/imu_info`](#topic-camera1-camera1-accel-imu-info), [`/camera1/camera1/gyro/imu_info`](#topic-camera1-camera1-gyro-imu-info), [`/camera2/camera2/accel/imu_info`](#topic-camera2-camera2-accel-imu-info), [`/camera2/camera2/gyro/imu_info`](#topic-camera2-camera2-gyro-imu-info)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id
float64[12] data
float64[3] noise_variances
float64[3] bias_variances
```

### `realsense2_camera_msgs/msg/Metadata`
<a id="type-realsense2-camera-msgs-msg-metadata"></a>

**Used by topics:** [`/camera1/camera1/accel/metadata`](#topic-camera1-camera1-accel-metadata), [`/camera1/camera1/color/metadata`](#topic-camera1-camera1-color-metadata), [`/camera1/camera1/depth/metadata`](#topic-camera1-camera1-depth-metadata), [`/camera1/camera1/gyro/metadata`](#topic-camera1-camera1-gyro-metadata), [`/camera2/camera2/accel/metadata`](#topic-camera2-camera2-accel-metadata), [`/camera2/camera2/color/metadata`](#topic-camera2-camera2-color-metadata), [`/camera2/camera2/depth/metadata`](#topic-camera2-camera2-depth-metadata), [`/camera2/camera2/gyro/metadata`](#topic-camera2-camera2-gyro-metadata)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id
string json_data
```

### `rosbridge_msgs/msg/ConnectedClients`
<a id="type-rosbridge-msgs-msg-connectedclients"></a>

**Used by topics:** [`/connected_clients`](#topic-connected-clients)

```text
ConnectedClient[] clients
    string ip_address
    builtin_interfaces/Time connection_time
        int32 sec
        uint32 nanosec
```

### `sensor_msgs/msg/CameraInfo`
<a id="type-sensor-msgs-msg-camerainfo"></a>

**Used by topics:** [`/camera1/camera1/color/camera_info`](#topic-camera1-camera1-color-camera-info), [`/camera1/camera1/depth/camera_info`](#topic-camera1-camera1-depth-camera-info), [`/camera2/camera2/color/camera_info`](#topic-camera2-camera2-color-camera-info), [`/camera2/camera2/depth/camera_info`](#topic-camera2-camera2-depth-camera-info)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

uint32 height
uint32 width

string distortion_model

float64[] d

float64[9]  k

float64[9]  r

float64[12] p

uint32 binning_x
uint32 binning_y

RegionOfInterest roi
    uint32 x_offset
    uint32 y_offset
    uint32 height
    uint32 width
    bool do_rectify
```

### `sensor_msgs/msg/Image`
<a id="type-sensor-msgs-msg-image"></a>

**Used by topics:** [`/camera1/camera1/color/image_raw`](#topic-camera1-camera1-color-image-raw), [`/camera1/camera1/depth/image_rect_raw`](#topic-camera1-camera1-depth-image-rect-raw), [`/camera2/camera2/color/image_raw`](#topic-camera2-camera2-color-image-raw), [`/camera2/camera2/depth/image_rect_raw`](#topic-camera2-camera2-depth-image-rect-raw)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

uint32 height
uint32 width


string encoding

uint8 is_bigendian
uint32 step
uint8[] data
```

### `sensor_msgs/msg/Imu`
<a id="type-sensor-msgs-msg-imu"></a>

**Used by topics:** [`/camera1/camera1/accel/sample`](#topic-camera1-camera1-accel-sample), [`/camera1/camera1/gyro/sample`](#topic-camera1-camera1-gyro-sample), [`/camera1/camera1/imu`](#topic-camera1-camera1-imu), [`/camera2/camera2/accel/sample`](#topic-camera2-camera2-accel-sample), [`/camera2/camera2/gyro/sample`](#topic-camera2-camera2-gyro-sample), [`/camera2/camera2/imu`](#topic-camera2-camera2-imu)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

geometry_msgs/Quaternion orientation
    float64 x 0
    float64 y 0
    float64 z 0
    float64 w 1
float64[9] orientation_covariance

geometry_msgs/Vector3 angular_velocity
    float64 x
    float64 y
    float64 z
float64[9] angular_velocity_covariance

geometry_msgs/Vector3 linear_acceleration
    float64 x
    float64 y
    float64 z
float64[9] linear_acceleration_covariance
```

### `sensor_msgs/msg/JointState`
<a id="type-sensor-msgs-msg-jointstate"></a>

**Used by topics:** [`/joint_states`](#topic-joint-states)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

string[] name
float64[] position
float64[] velocity
float64[] effort
```

### `sensor_msgs/msg/LaserScan`
<a id="type-sensor-msgs-msg-laserscan"></a>

**Used by topics:** [`/camera1/scan`](#topic-camera1-scan), [`/camera2/scan`](#topic-camera2-scan), [`/scan`](#topic-scan), [`/scan_filtered`](#topic-scan-filtered), [`/scan_merged`](#topic-scan-merged)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

float32 angle_min
float32 angle_max
float32 angle_increment

float32 time_increment
float32 scan_time

float32 range_min
float32 range_max

float32[] ranges
float32[] intensities
```

### `sensor_msgs/msg/PointCloud2`
<a id="type-sensor-msgs-msg-pointcloud2"></a>

**Used by topics:** [`/closest_gap`](#topic-closest-gap), [`/collision_obstacles`](#topic-collision-obstacles), [`/gaps`](#topic-gaps), [`/virt_gaps`](#topic-virt-gaps)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

uint32 height
uint32 width

PointField[] fields
    uint8 INT8    = 1
    uint8 UINT8   = 2
    uint8 INT16   = 3
    uint8 UINT16  = 4
    uint8 INT32   = 5
    uint8 UINT32  = 6
    uint8 FLOAT32 = 7
    uint8 FLOAT64 = 8
    string name
    uint32 offset
    uint8  datatype
    uint32 count

bool    is_bigendian
uint32  point_step
uint32  row_step
uint8[] data

bool is_dense
```

### `std_msgs/msg/Bool`
<a id="type-std-msgs-msg-bool"></a>

**Used by topics:** [`/is_speeching`](#topic-is-speeching), [`/person_following_robot/follow_enable`](#topic-person-following-robot-follow-enable), [`/person_following_robot/gui_enable`](#topic-person-following-robot-gui-enable), [`/person_following_robot/gui_state`](#topic-person-following-robot-gui-state)

```text
bool data
```

### `std_msgs/msg/ColorRGBA`
<a id="type-std-msgs-msg-colorrgba"></a>

**Used by topics:** [`/status_led`](#topic-status-led)

```text
float32 r
float32 g
float32 b
float32 a
```

### `std_msgs/msg/Int32`
<a id="type-std-msgs-msg-int32"></a>

**Used by topics:** [`/battery_level`](#topic-battery-level), [`/client_count`](#topic-client-count), [`/sound_direction`](#topic-sound-direction), [`/volume/command`](#topic-volume-command), [`/volume/state`](#topic-volume-state)

```text
int32 data
```

### `std_msgs/msg/String`
<a id="type-std-msgs-msg-string"></a>

**Used by topics:** [`/accessory_view/state`](#topic-accessory-view-state), [`/robot_description`](#topic-robot-description), [`/ros_to_iot`](#topic-ros-to-iot), [`/webrtc/state`](#topic-webrtc-state)

```text
string data
```

### `tf2_msgs/msg/TFMessage`
<a id="type-tf2-msgs-msg-tfmessage"></a>

**Used by topics:** [`/tf`](#topic-tf), [`/tf_static`](#topic-tf-static)

```text
geometry_msgs/TransformStamped[] transforms
    std_msgs/Header header
        builtin_interfaces/Time stamp
            int32 sec
            uint32 nanosec
        string frame_id
    string child_frame_id
    Transform transform
        Vector3 translation
            float64 x
            float64 y
            float64 z
        Quaternion rotation
            float64 x 0
            float64 y 0
            float64 z 0
            float64 w 1
```

### `visualization_msgs/msg/Marker`
<a id="type-visualization-msgs-msg-marker"></a>

**Used by topics:** [`/footprint`](#topic-footprint)

```text
int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11
int32 ARROW_STRIP=12

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
    Point position
        float64 x
        float64 y
        float64 z
    Quaternion orientation
        float64 x 0
        float64 y 0
        float64 z 0
        float64 w 1

geometry_msgs/Vector3 scale
    float64 x
    float64 y
    float64 z

std_msgs/ColorRGBA color
    float32 r
    float32 g
    float32 b
    float32 a

builtin_interfaces/Duration lifetime
    int32 sec
    uint32 nanosec

bool frame_locked

geometry_msgs/Point[] points
    float64 x
    float64 y
    float64 z

std_msgs/ColorRGBA[] colors
    float32 r
    float32 g
    float32 b
    float32 a

string texture_resource

sensor_msgs/CompressedImage texture
    std_msgs/Header header
        builtin_interfaces/Time stamp
            int32 sec
            uint32 nanosec
        string frame_id
    string format
    uint8[] data

UVCoordinate[] uv_coordinates
    float32 u
    float32 v

string text

string mesh_resource
MeshFile mesh_file
    string filename
    uint8[] data
bool mesh_use_embedded_materials
```

## Nodes

- `/botwheel_explorer`
- `/camera1/camera1`
- `/camera1_tf`
- `/camera2/camera2`
- `/camera2_tf`
- `/controller_manager`
- `/depthimage_to_laserscan_1`
- `/depthimage_to_laserscan_2`
- `/diffbot`
- `/joint_state_broadcaster`
- `/laser_tf`
- `/obstacle_avoidance`
- `/obstacle_map`
- `/person_following_robot/byte_tracker`
- `/person_following_robot/person_follower`
- `/person_following_robot/static_tf2_broadcaster`
- `/respeaker_node`
- `/robot_state_publisher`
- `/ros_to_iot`
- `/rplidar_node`
- `/scan_filter`
- `/scan_merger`
- `/transform_listener_impl_aaaaf2770f00`

## Services

Service list is an observed inventory; parameter services may expose sensitive controls and are not necessarily participant-authorized.

- `/accessory_view_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/accessory_view_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/accessory_view_node/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/accessory_view_node/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/accessory_view_node/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/accessory_view_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/battery_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/battery_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/battery_node/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/battery_node/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/battery_node/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/battery_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/botwheel_explorer/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/botwheel_explorer/get_logger_levels [rcl_interfaces/srv/GetLoggerLevels]`
- `/botwheel_explorer/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/botwheel_explorer/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/botwheel_explorer/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/botwheel_explorer/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/botwheel_explorer/set_logger_levels [rcl_interfaces/srv/SetLoggerLevels]`
- `/botwheel_explorer/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/botwheel_explorer/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/camera1/camera1/calib_config_read [realsense2_camera_msgs/srv/CalibConfigRead]`
- `/camera1/camera1/calib_config_write [realsense2_camera_msgs/srv/CalibConfigWrite]`
- `/camera1/camera1/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/camera1/camera1/device_info [realsense2_camera_msgs/srv/DeviceInfo]`
- `/camera1/camera1/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/camera1/camera1/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/camera1/camera1/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/camera1/camera1/hw_reset [std_srvs/srv/Empty]`
- `/camera1/camera1/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/camera1/camera1/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/camera1/camera1/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/camera1_tf/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/camera1_tf/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/camera1_tf/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/camera1_tf/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/camera1_tf/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/camera1_tf/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/camera1_tf/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/camera2/camera2/calib_config_read [realsense2_camera_msgs/srv/CalibConfigRead]`
- `/camera2/camera2/calib_config_write [realsense2_camera_msgs/srv/CalibConfigWrite]`
- `/camera2/camera2/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/camera2/camera2/device_info [realsense2_camera_msgs/srv/DeviceInfo]`
- `/camera2/camera2/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/camera2/camera2/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/camera2/camera2/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/camera2/camera2/hw_reset [std_srvs/srv/Empty]`
- `/camera2/camera2/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/camera2/camera2/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/camera2/camera2/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/camera2_tf/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/camera2_tf/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/camera2_tf/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/camera2_tf/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/camera2_tf/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/camera2_tf/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/camera2_tf/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/controller_manager/cleanup_controller [controller_manager_msgs/srv/CleanupController]`
- `/controller_manager/configure_controller [controller_manager_msgs/srv/ConfigureController]`
- `/controller_manager/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/controller_manager/get_logger_levels [rcl_interfaces/srv/GetLoggerLevels]`
- `/controller_manager/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/controller_manager/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/controller_manager/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/controller_manager/list_controller_types [controller_manager_msgs/srv/ListControllerTypes]`
- `/controller_manager/list_controllers [controller_manager_msgs/srv/ListControllers]`
- `/controller_manager/list_hardware_components [controller_manager_msgs/srv/ListHardwareComponents]`
- `/controller_manager/list_hardware_interfaces [controller_manager_msgs/srv/ListHardwareInterfaces]`
- `/controller_manager/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/controller_manager/load_controller [controller_manager_msgs/srv/LoadController]`
- `/controller_manager/reload_controller_libraries [controller_manager_msgs/srv/ReloadControllerLibraries]`
- `/controller_manager/set_hardware_component_state [controller_manager_msgs/srv/SetHardwareComponentState]`
- `/controller_manager/set_logger_levels [rcl_interfaces/srv/SetLoggerLevels]`
- `/controller_manager/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/controller_manager/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/controller_manager/switch_controller [controller_manager_msgs/srv/SwitchController]`
- `/controller_manager/unload_controller [controller_manager_msgs/srv/UnloadController]`
- `/depthimage_to_laserscan_1/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/depthimage_to_laserscan_1/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/depthimage_to_laserscan_1/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/depthimage_to_laserscan_1/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/depthimage_to_laserscan_1/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/depthimage_to_laserscan_1/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/depthimage_to_laserscan_1/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/depthimage_to_laserscan_2/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/depthimage_to_laserscan_2/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/depthimage_to_laserscan_2/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/depthimage_to_laserscan_2/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/depthimage_to_laserscan_2/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/depthimage_to_laserscan_2/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/depthimage_to_laserscan_2/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/diffbot/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/diffbot/get_logger_levels [rcl_interfaces/srv/GetLoggerLevels]`
- `/diffbot/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/diffbot/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/diffbot/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/diffbot/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/diffbot/set_logger_levels [rcl_interfaces/srv/SetLoggerLevels]`
- `/diffbot/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/diffbot/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/joint_state_broadcaster/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/joint_state_broadcaster/get_logger_levels [rcl_interfaces/srv/GetLoggerLevels]`
- `/joint_state_broadcaster/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/joint_state_broadcaster/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/joint_state_broadcaster/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/joint_state_broadcaster/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/joint_state_broadcaster/set_logger_levels [rcl_interfaces/srv/SetLoggerLevels]`
- `/joint_state_broadcaster/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/joint_state_broadcaster/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/laser_tf/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/laser_tf/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/laser_tf/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/laser_tf/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/laser_tf/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/laser_tf/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/laser_tf/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/network_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/network_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/network_node/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/network_node/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/network_node/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/network_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/obstacle_avoidance/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/obstacle_avoidance/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/obstacle_avoidance/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/obstacle_avoidance/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/obstacle_avoidance/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/obstacle_avoidance/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/obstacle_avoidance/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/obstacle_map/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/obstacle_map/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/obstacle_map/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/obstacle_map/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/obstacle_map/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/obstacle_map/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/obstacle_map/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/person_following_robot/byte_tracker/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/person_following_robot/byte_tracker/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/person_following_robot/byte_tracker/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/person_following_robot/byte_tracker/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/person_following_robot/byte_tracker/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/person_following_robot/byte_tracker/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/person_following_robot/byte_tracker/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/person_following_robot/person_follower/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/person_following_robot/person_follower/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/person_following_robot/person_follower/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/person_following_robot/person_follower/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/person_following_robot/person_follower/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/person_following_robot/person_follower/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/person_following_robot/person_follower/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/person_following_robot/static_tf2_broadcaster/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/person_following_robot/static_tf2_broadcaster/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/person_following_robot/static_tf2_broadcaster/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/person_following_robot/static_tf2_broadcaster/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/person_following_robot/static_tf2_broadcaster/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/person_following_robot/static_tf2_broadcaster/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/person_following_robot/static_tf2_broadcaster/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/respeaker_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/respeaker_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/respeaker_node/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/respeaker_node/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/respeaker_node/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/respeaker_node/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/respeaker_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/robot_state_publisher/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/robot_state_publisher/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/robot_state_publisher/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/robot_state_publisher/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/robot_state_publisher/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/robot_state_publisher/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/robot_state_publisher/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/ros_to_iot/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/ros_to_iot/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/ros_to_iot/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/ros_to_iot/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/ros_to_iot/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/ros_to_iot/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/ros_to_iot/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/rosapi/action_feedback_details [rosapi_msgs/srv/ActionFeedbackDetails]`
- `/rosapi/action_goal_details [rosapi_msgs/srv/ActionGoalDetails]`
- `/rosapi/action_result_details [rosapi_msgs/srv/ActionResultDetails]`
- `/rosapi/action_servers [rosapi_msgs/srv/GetActionServers]`
- `/rosapi/action_type [rosapi_msgs/srv/ActionType]`
- `/rosapi/delete_param [rosapi_msgs/srv/DeleteParam]`
- `/rosapi/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/rosapi/get_param [rosapi_msgs/srv/GetParam]`
- `/rosapi/get_param_names [rosapi_msgs/srv/GetParamNames]`
- `/rosapi/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/rosapi/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/rosapi/get_ros_version [rosapi_msgs/srv/GetROSVersion]`
- `/rosapi/get_time [rosapi_msgs/srv/GetTime]`
- `/rosapi/has_param [rosapi_msgs/srv/HasParam]`
- `/rosapi/interfaces [rosapi_msgs/srv/Interfaces]`
- `/rosapi/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/rosapi/message_details [rosapi_msgs/srv/MessageDetails]`
- `/rosapi/node_details [rosapi_msgs/srv/NodeDetails]`
- `/rosapi/nodes [rosapi_msgs/srv/Nodes]`
- `/rosapi/publishers [rosapi_msgs/srv/Publishers]`
- `/rosapi/service_node [rosapi_msgs/srv/ServiceNode]`
- `/rosapi/service_providers [rosapi_msgs/srv/ServiceProviders]`
- `/rosapi/service_request_details [rosapi_msgs/srv/ServiceRequestDetails]`
- `/rosapi/service_response_details [rosapi_msgs/srv/ServiceResponseDetails]`
- `/rosapi/service_type [rosapi_msgs/srv/ServiceType]`
- `/rosapi/services [rosapi_msgs/srv/Services]`
- `/rosapi/services_for_type [rosapi_msgs/srv/ServicesForType]`
- `/rosapi/set_param [rosapi_msgs/srv/SetParam]`
- `/rosapi/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/rosapi/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/rosapi/subscribers [rosapi_msgs/srv/Subscribers]`
- `/rosapi/topic_type [rosapi_msgs/srv/TopicType]`
- `/rosapi/topics [rosapi_msgs/srv/Topics]`
- `/rosapi/topics_and_raw_types [rosapi_msgs/srv/TopicsAndRawTypes]`
- `/rosapi/topics_for_type [rosapi_msgs/srv/TopicsForType]`
- `/rosbridge_websocket/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/rosbridge_websocket/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/rosbridge_websocket/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/rosbridge_websocket/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/rosbridge_websocket/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/rosbridge_websocket/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/rplidar_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/rplidar_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/rplidar_node/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/rplidar_node/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/rplidar_node/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/rplidar_node/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/rplidar_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/scan_filter/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/scan_filter/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/scan_filter/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/scan_filter/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/scan_filter/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/scan_filter/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/scan_filter/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/scan_merger/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/scan_merger/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/scan_merger/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/scan_merger/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/scan_merger/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/scan_merger/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/scan_merger/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/start_motor [std_srvs/srv/Empty]`
- `/stop_motor [std_srvs/srv/Empty]`
- `/transform_listener_impl_aaaaf2770f00/get_type_description [type_description_interfaces/srv/GetTypeDescription]`
- `/volume_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/volume_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/volume_node/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/volume_node/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/volume_node/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/volume_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`
- `/webrtc_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]`
- `/webrtc_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]`
- `/webrtc_node/get_parameters [rcl_interfaces/srv/GetParameters]`
- `/webrtc_node/list_parameters [rcl_interfaces/srv/ListParameters]`
- `/webrtc_node/set_parameters [rcl_interfaces/srv/SetParameters]`
- `/webrtc_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]`

## Actions

- `/camera1/camera1/triggered_calibration [realsense2_camera_msgs/action/TriggeredCalibration]`
- `/camera2/camera2/triggered_calibration [realsense2_camera_msgs/action/TriggeredCalibration]`

## TF frame relationships

The TF graph was captured via `tf2_tools view_frames`. These are observed parent → child relationships, not geometric calibration values.

- `odom` → `base_link`
- `camera2_color_frame` → `ncamera2_color_optical_frame`
- `base_link` → `nleft_wheel`
- `base_link` → `nright_wheel`
- `base_link` → `ncaster_frontal_wheel`
- `base_link` → `ncaster_rear_wheel`
- `base_link` → `nlaser`
- `base_link` → `ncamera2_link`
- `camera1_link` → `ncamera1_depth_frame`
- `base_link` → `ncamera1_link`
- `camera1_depth_frame` → `ncamera1_depth_optical_frame`
- `camera1_link` → `ncamera1_color_frame`
- `camera1_color_frame` → `ncamera1_color_optical_frame`
- `camera1_link` → `ncamera1_accel_frame`
- `camera1_accel_frame` → `ncamera1_accel_optical_frame`
- `camera1_link` → `ncamera1_gyro_frame`
- `camera1_gyro_frame` → `ncamera1_gyro_optical_frame`
- `camera1_gyro_frame` → `ncamera1_imu_frame`
- `camera1_imu_frame` → `ncamera1_imu_optical_frame`
- `camera2_link` → `ncamera2_depth_frame`
- `camera2_depth_frame` → `ncamera2_depth_optical_frame`
- `camera2_link` → `ncamera2_color_frame`
- `camera2_link` → `ncamera2_accel_frame`
- `camera2_accel_frame` → `ncamera2_accel_optical_frame`
- `camera2_link` → `ncamera2_gyro_frame`
- `camera2_gyro_frame` → `ncamera2_gyro_optical_frame`
- `camera2_gyro_frame` → `ncamera2_imu_frame`
- `camera2_imu_frame` → `ncamera2_imu_optical_frame`
