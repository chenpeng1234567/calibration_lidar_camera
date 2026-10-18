# lidar_camera_handeye_calibrator

Trajectory hand-eye calibration for a rigid rear camera and LiDAR/IMU body when their fields of
view do not overlap. It consumes two independent world-frame odometry streams and solves the fixed
transform from the camera optical frame into the FAST-LIO reference/body frame.

## Conventions used by the supplied nodes

- FAST_LIO_SAM `/Odometry`: `camera_init -> body` (`T_world_body`).
- `orb_slam3_ros2` odometry: `orb_world -> camera_back/camera_optical_link`
  (`T_world_camera`, already inverted from ORB-SLAM3's internal `Tcw`).
- Result: `p_body = T_body_camera_optical * p_camera_optical`.

The independent world origins cancel because the solver uses relative motions. Do not remap either
world frame to Autoware `map`.

## Build in the standalone workspace

Both this package and `orb_slam3_ros2` already live in the calibration workspace:

```bash
cd /root/autoware/src/ys/calibration/calibration_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --symlink-install \
  --cmake-args \
  -DORB_SLAM3_ROOT=/root/autoware/src/ys/calibration/ORB_SLAM3 \
  -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Online collection

Start FAST-LIO, ORB-SLAM3, and the calibrator:

```bash
ros2 launch lidar_camera_handeye_calibrator calibrate.launch.py
```

Drive a varied-motion calibration route. Confirm both streams before solving:

```bash
ros2 topic hz /Odometry
ros2 topic hz /orb_slam3/camera_back/odometry
ros2 topic echo /orb_slam3/camera_back/tracking_ok --once
```

Collection is gated by `/orb_slam3/camera_back/tracking_ok` by default. No poses are collected
until it becomes `true`. If it changes to `false`, all pending and synchronized poses are cleared
immediately; collection resumes as a new segment the next time it becomes `true`. This prevents
motions from different ORB atlas maps being mixed after tracking loss. Set
`require_camera_tracking_ok: false` only when using a camera odometry source that has no equivalent
tracking-status topic.

Solve and save `/tmp/lidar_camera_handeye_result.yaml`:

```bash
ros2 service call \
  /lidar_camera_handeye_calibrator/calibrate \
  std_srvs/srv/Trigger '{}'
```

Clear samples before a new run:

```bash
ros2 service call \
  /lidar_camera_handeye_calibrator/reset \
  std_srvs/srv/Trigger '{}'
```

## Rosbag workflow

Recording raw sensor streams as well as generated odometry permits reprocessing:

```bash
ros2 bag record -s mcap \
  -o /root/autoware/src/ys/calibration/calibration_ws/bags/camera_back_top_lidar \
  /Odometry \
  /orb_slam3/camera_back/odometry \
  /orb_slam3/camera_back/tracking_ok \
  /rect/camera_back/image_gray \
  /rect/camera_back/camera_info \
  /sensing/lidar/top/pointcloud_raw \
  /sensing/imu/imu_data \
  /tf \
  /tf_static
```

To collect from a recorded bag, start the calibrator first and then play the bag. If the bag contains
both odometry topics there is no need to rerun the SLAM systems.

## Important limits

1. Pure monocular ORB-SLAM3 has arbitrary translation scale. Keep `estimate_camera_scale: true`.
2. FAST-LIO's output child is `body` (normally its IMU frame), not necessarily the physical LiDAR.
   The result is therefore `T_body_camera_optical`. Combine it with the configured LiDAR-to-IMU
   extrinsic to obtain `T_lidar_camera_optical`. In the checked `fairy.yaml`, the configured
   LiDAR translation with respect to IMU/body is `[0.01365, 0.00782, 0.00492]` m and its rotation
   is identity; verify those values on the vehicle before applying the conversion.
3. Flat vehicle motion is degenerate for full 6-DoF calibration. Use turns in both directions and
   slopes where safe, or fix height/roll/pitch from mechanical measurement.
4. `camera_time_offset_sec` is not estimated automatically. A wrong time offset biases translation.
5. The result targets `camera_optical_link`. Convert through the fixed
   `camera_link -> camera_optical_link` rotation before writing Autoware's
   `camera_back/camera_link` YAML entry.
6. Calibrate with one bag and validate trajectory residuals with a separate bag.

Never write a result into the vehicle model solely because the service returned success. Reject
negative scale, large residuals, or poorly excited/singular motion data.
