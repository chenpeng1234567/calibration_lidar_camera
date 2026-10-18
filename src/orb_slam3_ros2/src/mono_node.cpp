#include <System.h>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace orb_slam3_ros2
{

class MonoNode : public rclcpp::Node
{
public:
  MonoNode() : Node("orb_slam3_camera_back")
  {
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/rect/camera_back/image_gray");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/rect/camera_back/camera_info");
    vocabulary_file_ = declare_parameter<std::string>("vocabulary_file", "");
    settings_file_ = declare_parameter<std::string>("settings_file", "");
    generated_settings_file_ = declare_parameter<std::string>(
      "generated_settings_file", "/tmp/orb_slam3_camera_back.yaml");
    world_frame_ = declare_parameter<std::string>("world_frame", "orb_world");
    camera_frame_ = declare_parameter<std::string>(
      "camera_frame", "camera_back/camera_optical_link");
    use_viewer_ = declare_parameter<bool>("use_viewer", false);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    assume_rectified_ = declare_parameter<bool>("assume_rectified", true);
    save_keyframe_trajectory_ = declare_parameter<bool>("save_keyframe_trajectory", true);
    keyframe_trajectory_file_ = declare_parameter<std::string>(
      "keyframe_trajectory_file", "/tmp/orb_slam3_camera_back_keyframes.txt");
    path_max_length_ = declare_parameter<int>("path_max_length", 20000);
    n_features_ = declare_parameter<int>("orb.n_features", 1800);
    scale_factor_ = declare_parameter<double>("orb.scale_factor", 1.2);
    n_levels_ = declare_parameter<int>("orb.n_levels", 8);
    ini_th_fast_ = declare_parameter<int>("orb.ini_th_fast", 20);
    min_th_fast_ = declare_parameter<int>("orb.min_th_fast", 7);
    camera_fps_ = declare_parameter<int>("camera_fps", 15);

    if (vocabulary_file_.empty() || !std::filesystem::exists(vocabulary_file_)) {
      throw std::runtime_error("vocabulary_file does not exist: " + vocabulary_file_);
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(10));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(1).transient_local());
    tracking_state_pub_ = create_publisher<std_msgs::msg::Int32>("tracking_state", 10);
    tracking_ok_pub_ = create_publisher<std_msgs::msg::Bool>("tracking_ok", 10);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    path_.header.frame_id = world_frame_;

    if (!settings_file_.empty()) {
      initialize_slam(settings_file_);
    } else {
      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&MonoNode::camera_info_callback, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(), "Waiting for CameraInfo on %s to generate ORB-SLAM3 settings",
        camera_info_topic_.c_str());
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MonoNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ORB-SLAM3 image input: %s", image_topic_.c_str());
  }

  ~MonoNode() override
  {
    std::scoped_lock lock(slam_mutex_);
    if (slam_) {
      slam_->Shutdown();
      if (save_keyframe_trajectory_) {
        slam_->SaveKeyFrameTrajectoryTUM(keyframe_trajectory_file_);
      }
    }
  }

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    std::scoped_lock lock(slam_mutex_);
    if (slam_) {
      return;
    }

    const double fx = msg->p[0] > 0.0 ? msg->p[0] : msg->k[0];
    const double fy = msg->p[5] > 0.0 ? msg->p[5] : msg->k[4];
    const double cx = msg->p[2] != 0.0 ? msg->p[2] : msg->k[2];
    const double cy = msg->p[6] != 0.0 ? msg->p[6] : msg->k[5];
    if (msg->width == 0 || msg->height == 0 || fx <= 0.0 || fy <= 0.0) {
      RCLCPP_ERROR(get_logger(), "CameraInfo is invalid; width/height/fx/fy must be positive");
      return;
    }

    std::ofstream out(generated_settings_file_, std::ios::trunc);
    if (!out) {
      RCLCPP_ERROR(
        get_logger(), "Cannot create generated settings: %s", generated_settings_file_.c_str());
      return;
    }
    const auto distortion = [msg](std::size_t i) {
        return i < msg->d.size() ? msg->d[i] : 0.0;
      };
    // ORB-SLAM3 checks the OpenCV FileNode type and rejects integral nodes for camera
    // parameters. Keep the decimal point even when CameraInfo contains values such as 360.0.
    out << std::fixed << std::setprecision(12) << std::showpoint;
    out << "%YAML:1.0\n"
        << "File.version: \"1.0\"\n"
        << "Camera.type: \"PinHole\"\n"
        << "Camera1.fx: " << fx << "\n"
        << "Camera1.fy: " << fy << "\n"
        << "Camera1.cx: " << cx << "\n"
        << "Camera1.cy: " << cy << "\n"
        << "Camera1.k1: " << (assume_rectified_ ? 0.0 : distortion(0)) << "\n"
        << "Camera1.k2: " << (assume_rectified_ ? 0.0 : distortion(1)) << "\n"
        << "Camera1.p1: " << (assume_rectified_ ? 0.0 : distortion(2)) << "\n"
        << "Camera1.p2: " << (assume_rectified_ ? 0.0 : distortion(3)) << "\n"
        << "Camera.width: " << msg->width << "\n"
        << "Camera.height: " << msg->height << "\n"
        << "Camera.fps: " << camera_fps_ << "\n"
        // The local v4l2_camera driver publishes rect_color as rgb8.
        << "Camera.RGB: 1\n"
        << "ORBextractor.nFeatures: " << n_features_ << "\n"
        << "ORBextractor.scaleFactor: " << scale_factor_ << "\n"
        << "ORBextractor.nLevels: " << n_levels_ << "\n"
        << "ORBextractor.iniThFAST: " << ini_th_fast_ << "\n"
        << "ORBextractor.minThFAST: " << min_th_fast_ << "\n"
        << "Viewer.KeyFrameSize: 0.05\n"
        << "Viewer.KeyFrameLineWidth: 1.0\n"
        << "Viewer.GraphLineWidth: 0.9\n"
        << "Viewer.PointSize: 2.0\n"
        << "Viewer.CameraSize: 0.08\n"
        << "Viewer.CameraLineWidth: 3.0\n"
        << "Viewer.ViewpointX: 0.0\n"
        << "Viewer.ViewpointY: -0.7\n"
        << "Viewer.ViewpointZ: -1.8\n"
        << "Viewer.ViewpointF: 500.0\n";
    out.close();

    RCLCPP_INFO(
      get_logger(),
      "Generated ORB-SLAM3 camera settings: %ux%u, fx=%.6f, fy=%.6f, cx=%.6f, cy=%.6f",
      msg->width, msg->height, fx, fy, cx, cy);

    camera_frame_ = msg->header.frame_id.empty() ? camera_frame_ : msg->header.frame_id;
    initialize_slam(generated_settings_file_);
    camera_info_sub_.reset();
  }

  void initialize_slam(const std::string & settings)
  {
    if (!std::filesystem::exists(settings)) {
      throw std::runtime_error("settings_file does not exist: " + settings);
    }
    slam_ = std::make_unique<ORB_SLAM3::System>(
      vocabulary_file_, settings, ORB_SLAM3::System::MONOCULAR, use_viewer_);
    RCLCPP_INFO(
      get_logger(), "ORB-SLAM3 initialized with settings %s; output frame %s -> %s",
      settings.c_str(), world_frame_.c_str(), camera_frame_.c_str());
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::scoped_lock lock(slam_mutex_);
    if (!slam_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping images while waiting for CameraInfo");
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "cv_bridge conversion failed: %s", error.what());
      return;
    }

    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    const Sophus::SE3f t_camera_world = slam_->TrackMonocular(cv_ptr->image, stamp);
    const int state = slam_->GetTrackingState();
    const bool tracking_ok = state == 2 || state == 5;

    if (state != last_tracking_state_) {
      RCLCPP_INFO(
        get_logger(), "ORB-SLAM3 tracking state changed: %d -> %d (encoding=%s, %dx%d)",
        last_tracking_state_, state, msg->encoding.c_str(), cv_ptr->image.cols, cv_ptr->image.rows);
      last_tracking_state_ = state;
    }

    std_msgs::msg::Int32 state_msg;
    state_msg.data = state;
    tracking_state_pub_->publish(state_msg);
    std_msgs::msg::Bool ok_msg;
    ok_msg.data = tracking_ok;
    tracking_ok_pub_->publish(ok_msg);
    if (!tracking_ok) {
      return;
    }

    // ORB-SLAM3 returns Tcw. ROS odometry below publishes Twc.
    const Sophus::SE3f t_world_camera = t_camera_world.inverse();
    const Eigen::Vector3f translation = t_world_camera.translation();
    Eigen::Quaternionf rotation(t_world_camera.rotationMatrix());
    rotation.normalize();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = camera_frame_;
    odom.pose.pose.position.x = translation.x();
    odom.pose.pose.position.y = translation.y();
    odom.pose.pose.position.z = translation.z();
    odom.pose.pose.orientation.x = rotation.x();
    odom.pose.pose.orientation.y = rotation.y();
    odom.pose.pose.orientation.z = rotation.z();
    odom.pose.pose.orientation.w = rotation.w();
    odom.pose.covariance.fill(0.0);
    odom.pose.covariance[0] = -1.0;  // ORB-SLAM3 does not provide pose covariance.
    odom_pub_->publish(odom);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    path_.header.stamp = msg->header.stamp;
    path_.poses.push_back(pose);
    if (path_max_length_ > 0 && static_cast<int>(path_.poses.size()) > path_max_length_) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + path_.poses.size() / 10);
    }
    path_pub_->publish(path_);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = camera_frame_;
      transform.transform.translation.x = translation.x();
      transform.transform.translation.y = translation.y();
      transform.transform.translation.z = translation.z();
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string vocabulary_file_;
  std::string settings_file_;
  std::string generated_settings_file_;
  std::string world_frame_;
  std::string camera_frame_;
  std::string keyframe_trajectory_file_;
  bool use_viewer_{false};
  bool publish_tf_{false};
  bool assume_rectified_{true};
  bool save_keyframe_trajectory_{true};
  int path_max_length_{20000};
  int n_features_{1800};
  double scale_factor_{1.2};
  int n_levels_{8};
  int ini_th_fast_{20};
  int min_th_fast_{7};
  int camera_fps_{15};
  int last_tracking_state_{-1};

  std::mutex slam_mutex_;
  std::unique_ptr<ORB_SLAM3::System> slam_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tracking_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tracking_ok_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Path path_;
};

}  // namespace orb_slam3_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<orb_slam3_ros2::MonoNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("orb_slam3_camera_back"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
