# orb_slam3_ros2

ROS 2 Humble monocular wrapper used to feed `camera_back` into the local ORB-SLAM3 source tree.

The node subscribes to:

- `/rect/camera_back/image_gray`
- `/rect/camera_back/camera_info`

It generates `/tmp/orb_slam3_camera_back.yaml` from the rectified `CameraInfo`, starts ORB-SLAM3,
inverts ORB-SLAM3's internal `Tcw`, and publishes:

- `/orb_slam3/camera_back/odometry` (`orb_world -> camera_back/camera_optical_link`)
- `/orb_slam3/camera_back/path`
- `/orb_slam3/camera_back/tracking_state` (`2` means normal tracking)
- `/orb_slam3/camera_back/tracking_ok`

`publish_tf` defaults to false because a monocular ORB world is not an Autoware metric/map frame.

## Build

Build the third-party library first:

```bash
cd /root/autoware/src/ys/calibration/ORB_SLAM3
chmod +x build.sh
./build.sh
```

Then build the wrapper. The ORB root is a CMake cache variable, not a ROS parameter:

```bash
cd /root/autoware
colcon build --symlink-install --packages-select orb_slam3_ros2 \
  --cmake-args \
  -DORB_SLAM3_ROOT=/root/autoware/src/ys/calibration/ORB_SLAM3 \
  -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

Start the camera through the normal Autoware sensing launch, then:

```bash
ros2 launch orb_slam3_ros2 camera_back_mono.launch.py
```

Verification:

```bash
ros2 topic hz /orb_slam3/camera_back/odometry
ros2 topic echo /orb_slam3/camera_back/tracking_ok --once
ros2 topic echo /orb_slam3/camera_back/odometry --once
```

The generated calibration is only as good as `/rect/camera_back/camera_info`. Do not copy the
front-camera intrinsics to camera_back. Pure monocular translation has arbitrary scale; the hand-eye
calibrator must be run with `estimate_camera_scale: true`.
