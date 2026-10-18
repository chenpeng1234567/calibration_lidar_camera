# calibration_ws

Trajectory-based **LiDAR–camera hand-eye calibration** for ROS 2 Humble, designed for sensor setups
where the camera and LiDAR fields of view **do not overlap** (e.g. a rear-facing camera and a
top-mounted LiDAR).

Instead of observing a shared calibration target — impossible when the sensors look in opposite
directions — this workspace consumes two independent world-frame odometry streams (LiDAR-inertial
odometry from FAST-LIO and monocular visual odometry from ORB-SLAM3) and solves the fixed rigid
transform between them as a classic `AX = XB` hand-eye problem over relative motions.

## Packages

| Package | License | Description |
| --- | --- | --- |
| [`lidar_camera_handeye_calibrator`](src/lidar_camera_handeye_calibrator/) | Apache-2.0 | Synchronizes the two odometry streams, builds relative-motion pairs, and solves rotation, translation, and (optionally) the monocular scale factor. |
| [`orb_slam3_ros2`](src/orb_slam3_ros2/) | GPL-3.0-only | ROS 2 monocular wrapper around a local ORB-SLAM3 source tree. Generates the ORB-SLAM3 config from the rectified `CameraInfo`, runs tracking, and publishes odometry as `orb_world -> camera_optical_link`. |

## How it works

1. **FAST-LIO** publishes `/Odometry` (`camera_init -> body`, i.e. `T_world_body`).
2. **`orb_slam3_ros2`** subscribes to the rectified rear-camera image, runs ORB-SLAM3, inverts the
   internal `Tcw`, and publishes `/orb_slam3/camera_back/odometry`
   (`orb_world -> camera_back/camera_optical_link`).
3. **`lidar_camera_handeye_calibrator`** time-synchronizes both streams, forms relative motions
   between sample *i* and *i + stride*, and rejects pairs whose conjugate rotation angles disagree
   (a SLAM jump detector).
4. Rotation is solved as the one-dimensional quaternion null space of the `AX = XB` system
   (with a rank check that the null space is unique); translation and the monocular scale factor are
   then solved by least squares.
5. The result `X = T_body_camera_optical` is published on `~/result` and written to a YAML file:
   `p_body = X * p_camera_optical`.

The two independent world origins cancel because only relative motions are used, so neither stream
needs to share a map frame.

```
                      +---------------------+
 /rect/camera_back -->|     orb_slam3_ros2  |--> /orb_slam3/camera_back/odometry -----+
  image_gray,         |  (monocular ORB-    |--> /orb_slam3/camera_back/tracking_ok --+
  camera_info         |       SLAM3)        |                                          |
                      +---------------------+                                          v
                                                                        +--------------------------+
 /Odometry ---------------------------------------------------------------->| lidar_camera_handeye    |
 (FAST-LIO / FAST_LIO_SAM)                                                  | calibrator              |
                                                                            |  sync -> motion pairs   |
                                                                            |  -> AX=XB solve         |
                                                                            +--------------------------+
                                                                              |            |
                                                                       ~/result (TF)   output YAML
```

## Prerequisites

- ROS 2 Humble
- [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) source tree (built separately; the
  `orb_slam3_ros2` wrapper is GPL-3.0 because it links against ORB-SLAM3)
- A LiDAR-inertial odometry source publishing `nav_msgs/Odometry` (FAST-LIO / FAST_LIO_SAM)
- Eigen 3, `cv_bridge`, and the usual `rosdep` dependencies

## Build

Build ORB_SLAM3 first, then the workspace (replace `ORB_SLAM3_ROOT` with your local path):

```bash
cd <path-to>/ORB_SLAM3
chmod +x build.sh && ./build.sh

cd <path-to>/calibration_ws        # this repository
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --cmake-args \
  -DORB_SLAM3_ROOT=<path-to>/ORB_SLAM3 \
  -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Usage

### Online calibration

```bash
ros2 launch lidar_camera_handeye_calibrator calibrate.launch.py
```

Drive a varied-motion route (turns in both directions, slopes where safe — flat straight-line
motion is degenerate for 6-DoF calibration). Confirm both streams before solving:

```bash
ros2 topic hz /Odometry
ros2 topic hz /orb_slam3/camera_back/odometry
ros2 topic echo /orb_slam3/camera_back/tracking_ok --once
```

Collection is gated by `/orb_slam3/camera_back/tracking_ok`: no poses are collected until it is
`true`, and a transition to `false` immediately clears all pending synchronized poses, so motions
from different ORB atlas maps are never mixed after tracking loss. Set
`require_camera_tracking_ok: false` only if your camera odometry source has no equivalent
status topic.

Solve and write the result (default `/tmp/lidar_camera_handeye_result.yaml`):

```bash
ros2 service call /lidar_camera_handeye_calibrator/calibrate std_srvs/srv/Trigger '{}'
```

Clear samples before a new run:

```bash
ros2 service call /lidar_camera_handeye_calibrator/reset std_srvs/srv/Trigger '{}'
```

### Rosbag workflow

Record the raw sensor streams along with the generated odometry so a run can be reprocessed:

```bash
ros2 bag record -s mcap -o bags/camera_back_top_lidar \
  /Odometry \
  /orb_slam3/camera_back/odometry \
  /orb_slam3/camera_back/tracking_ok \
  /rect/camera_back/image_gray \
  /rect/camera_back/camera_info \
  /sensing/lidar/top/pointcloud_raw \
  /sensing/imu/imu_data \
  /tf /tf_static
```

Start the calibrator first, then play the bag. If the bag already contains both odometry topics,
the SLAM systems do not need to be relaunched.

## Key parameters

See [`config/calibrator.param.yaml`](src/lidar_camera_handeye_calibrator/config/calibrator.param.yaml)
for the full list with comments.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `estimate_camera_scale` | `true` | Keep `true` for pure monocular ORB-SLAM3 (arbitrary translation scale); `false` for metric stereo/VI odometry. |
| `camera_time_offset_sec` | `0.0` | Constant camera timestamp correction; not estimated automatically, and a wrong value biases translation. |
| `max_sync_error_sec` | `0.025` | Maximum timestamp mismatch when pairing lidar and camera poses. |
| `pair_stride` | `10` | Relative motions are formed between samples *i* and *i + stride*. |
| `min_motion_translation_m` | `0.20` | Minimum relative translation for a motion pair to be used. |
| `min_motion_rotation_deg` | `3.0` | Minimum relative rotation for a motion pair to be used. |
| `max_relative_rotation_difference_deg` | `2.0` | Conjugate rotations must match in angle — rejects SLAM jumps. |
| `min_motion_pairs` | `20` | Minimum number of accepted pairs required to solve. |
| `invert_lidar_pose` / `invert_camera_pose` | `false` | For third-party wrappers that publish inverted odometry. |

## Important limits

1. FAST-LIO's odometry child frame is `body` (normally its IMU frame), not necessarily the physical
   LiDAR, so the result is `T_body_camera_optical`. Combine it with the configured LiDAR-to-IMU
   extrinsic to obtain `T_lidar_camera_optical`.
2. The result targets `camera_optical_link`. Apply the fixed `camera_link -> camera_optical_link`
   rotation before writing an Autoware-style `camera_link` extrinsic entry.
3. Flat vehicle motion is degenerate for full 6-DoF calibration — use varied motion or fix
   height/roll/pitch from mechanical measurement.
4. Calibrate with one bag and validate trajectory residuals with a separate bag. Reject negative
   scale, large residuals, or poorly excited motion data — never write a result to the vehicle model
   solely because the service returned success.

## License

- [`lidar_camera_handeye_calibrator`](src/lidar_camera_handeye_calibrator/) — Apache-2.0
- [`orb_slam3_ros2`](src/orb_slam3_ros2/) — GPL-3.0-only (links against ORB-SLAM3)
