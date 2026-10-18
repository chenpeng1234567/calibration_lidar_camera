#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lidar_camera_handeye_calibrator
{

constexpr double kRadToDeg = 57.2957795130823208768;

struct TimedPose
{
  double stamp{};
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
};

struct SynchronizedPose
{
  TimedPose lidar;
  TimedPose camera;
  double sync_error{};
};

struct MotionPair
{
  Eigen::Isometry3d lidar{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d camera{Eigen::Isometry3d::Identity()};
};

class HandeyeCalibratorNode : public rclcpp::Node
{
public:
  HandeyeCalibratorNode() : Node("lidar_camera_handeye_calibrator")
  {
    lidar_odometry_topic_ = declare_parameter<std::string>("lidar_odometry_topic", "/Odometry");
    camera_odometry_topic_ = declare_parameter<std::string>(
      "camera_odometry_topic", "/orb_slam3/camera_back/odometry");
    camera_tracking_ok_topic_ = declare_parameter<std::string>(
      "camera_tracking_ok_topic", "/orb_slam3/camera_back/tracking_ok");
    require_camera_tracking_ok_ = declare_parameter<bool>("require_camera_tracking_ok", true);
    reference_frame_ = declare_parameter<std::string>("reference_frame", "body");
    camera_frame_ = declare_parameter<std::string>(
      "camera_frame", "camera_back/camera_optical_link");
    output_yaml_ = declare_parameter<std::string>(
      "output_yaml", "/tmp/lidar_camera_handeye_result.yaml");
    camera_time_offset_sec_ = declare_parameter<double>("camera_time_offset_sec", 0.0);
    max_sync_error_sec_ = declare_parameter<double>("max_sync_error_sec", 0.025);
    max_queue_size_ = declare_parameter<int>("max_queue_size", 1000);
    pair_stride_ = declare_parameter<int>("pair_stride", 10);
    min_motion_translation_m_ = declare_parameter<double>("min_motion_translation_m", 0.20);
    min_motion_rotation_deg_ = declare_parameter<double>("min_motion_rotation_deg", 3.0);
    max_relative_rotation_difference_deg_ = declare_parameter<double>(
      "max_relative_rotation_difference_deg", 2.0);
    min_motion_pairs_ = declare_parameter<int>("min_motion_pairs", 20);
    estimate_camera_scale_ = declare_parameter<bool>("estimate_camera_scale", true);
    invert_lidar_pose_ = declare_parameter<bool>("invert_lidar_pose", false);
    invert_camera_pose_ = declare_parameter<bool>("invert_camera_pose", false);

    if (max_sync_error_sec_ <= 0.0 || max_queue_size_ < 2 || pair_stride_ < 1 ||
      min_motion_pairs_ < 3)
    {
      throw std::invalid_argument("Invalid synchronization or motion-pair parameter");
    }

    lidar_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      lidar_odometry_topic_, rclcpp::QoS(200),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odometry_callback(*msg, true);
      });
    camera_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      camera_odometry_topic_, rclcpp::QoS(200),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odometry_callback(*msg, false);
      });
    if (require_camera_tracking_ok_) {
      camera_tracking_ok_sub_ = create_subscription<std_msgs::msg::Bool>(
        camera_tracking_ok_topic_, rclcpp::QoS(10),
        std::bind(
          &HandeyeCalibratorNode::camera_tracking_ok_callback, this,
          std::placeholders::_1));
    }

    result_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
      "~/result", rclcpp::QoS(1).transient_local());
    calibrate_service_ = create_service<std_srvs::srv::Trigger>(
      "~/calibrate",
      std::bind(
        &HandeyeCalibratorNode::calibrate_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      std::bind(
        &HandeyeCalibratorNode::reset_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "Collecting FAST-LIO poses from %s and camera poses from %s",
      lidar_odometry_topic_.c_str(), camera_odometry_topic_.c_str());
    if (require_camera_tracking_ok_) {
      RCLCPP_INFO(
        get_logger(), "Waiting for camera tracking confirmation on %s",
        camera_tracking_ok_topic_.c_str());
    }
    RCLCPP_INFO(
      get_logger(), "Call /lidar_camera_handeye_calibrator/calibrate after a varied-motion run");
  }

private:
  static double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
  }

  static bool odometry_to_pose(const nav_msgs::msg::Odometry & msg, Eigen::Isometry3d & pose)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q_msg = msg.pose.pose.orientation;
    Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !q.coeffs().allFinite() || q.norm() < 1.0e-6)
    {
      return false;
    }
    q.normalize();
    pose = Eigen::Isometry3d::Identity();
    pose.linear() = q.toRotationMatrix();
    pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    return true;
  }

  static double rotation_angle(const Eigen::Matrix3d & rotation)
  {
    const double cos_angle = std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cos_angle);
  }

  static Eigen::Matrix4d quaternion_left(const Eigen::Quaterniond & input)
  {
    const Eigen::Quaterniond q = input.normalized();
    Eigen::Matrix4d matrix;
    matrix << q.w(), -q.x(), -q.y(), -q.z(),
      q.x(), q.w(), -q.z(), q.y(),
      q.y(), q.z(), q.w(), -q.x(),
      q.z(), -q.y(), q.x(), q.w();
    return matrix;
  }

  static Eigen::Matrix4d quaternion_right(const Eigen::Quaterniond & input)
  {
    const Eigen::Quaterniond q = input.normalized();
    Eigen::Matrix4d matrix;
    matrix << q.w(), -q.x(), -q.y(), -q.z(),
      q.x(), q.w(), q.z(), -q.y(),
      q.y(), -q.z(), q.w(), q.x(),
      q.z(), q.y(), -q.x(), q.w();
    return matrix;
  }

  void clear_collected_poses_locked()
  {
    lidar_queue_.clear();
    camera_queue_.clear();
    synchronized_.clear();
  }

  void camera_tracking_ok_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    std::scoped_lock lock(mutex_);
    const bool state_changed = !camera_tracking_state_received_ ||
      camera_tracking_ok_ != msg->data;
    if (!state_changed) {
      return;
    }

    camera_tracking_state_received_ = true;
    camera_tracking_ok_ = msg->data;
    clear_collected_poses_locked();

    if (camera_tracking_ok_) {
      RCLCPP_INFO(
        get_logger(),
        "ORB-SLAM3 tracking is OK; cleared stale poses and started a new calibration segment");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "ORB-SLAM3 tracking is not OK; cleared all calibration poses and paused collection");
    }
  }

  void odometry_callback(const nav_msgs::msg::Odometry & msg, bool is_lidar)
  {
    TimedPose sample;
    sample.stamp = stamp_seconds(msg.header.stamp);
    if (!odometry_to_pose(msg, sample.pose)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected invalid odometry pose");
      return;
    }
    if ((is_lidar && invert_lidar_pose_) || (!is_lidar && invert_camera_pose_)) {
      sample.pose = sample.pose.inverse();
    }

    std::scoped_lock lock(mutex_);
    if (require_camera_tracking_ok_ &&
      (!camera_tracking_state_received_ || !camera_tracking_ok_))
    {
      return;
    }
    auto & queue = is_lidar ? lidar_queue_ : camera_queue_;
    if (!queue.empty() && sample.stamp <= queue.back().stamp) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Rejected non-increasing %s timestamp",
        is_lidar ? "LiDAR" : "camera");
      return;
    }
    queue.push_back(sample);
    while (static_cast<int>(queue.size()) > max_queue_size_) {
      queue.pop_front();
    }
    synchronize_queues();
  }

  void synchronize_queues()
  {
    while (!lidar_queue_.empty() && !camera_queue_.empty()) {
      const double corrected_camera_stamp =
        camera_queue_.front().stamp + camera_time_offset_sec_;
      const double error = corrected_camera_stamp - lidar_queue_.front().stamp;
      if (std::abs(error) <= max_sync_error_sec_) {
        synchronized_.push_back(
          {lidar_queue_.front(), camera_queue_.front(), std::abs(error)});
        lidar_queue_.pop_front();
        camera_queue_.pop_front();
        if (synchronized_.size() % 100 == 0) {
          RCLCPP_INFO(get_logger(), "Collected %zu synchronized poses", synchronized_.size());
        }
      } else if (error < 0.0) {
        camera_queue_.pop_front();
      } else {
        lidar_queue_.pop_front();
      }
    }
  }

  std::vector<MotionPair> make_motion_pairs(const std::vector<SynchronizedPose> & samples) const
  {
    std::vector<MotionPair> motions;
    if (samples.size() <= static_cast<std::size_t>(pair_stride_)) {
      return motions;
    }
    for (std::size_t i = 0; i + pair_stride_ < samples.size(); ++i) {
      const std::size_t j = i + pair_stride_;
      MotionPair motion;
      motion.lidar = samples[i].lidar.pose.inverse() * samples[j].lidar.pose;
      motion.camera = samples[i].camera.pose.inverse() * samples[j].camera.pose;
      const double translation = motion.lidar.translation().norm();
      const double lidar_rotation_deg = rotation_angle(motion.lidar.rotation()) * kRadToDeg;
      const double camera_rotation_deg = rotation_angle(motion.camera.rotation()) * kRadToDeg;
      const bool rotation_consistent =
        std::abs(lidar_rotation_deg - camera_rotation_deg) <=
        max_relative_rotation_difference_deg_;
      if (rotation_consistent &&
        (translation >= min_motion_translation_m_ ||
        lidar_rotation_deg >= min_motion_rotation_deg_))
      {
        motions.push_back(motion);
      }
    }
    return motions;
  }

  bool solve_rotation(
    const std::vector<MotionPair> & motions, Eigen::Matrix3d & rotation,
    Eigen::Vector4d & singular_values) const
  {
    Eigen::MatrixXd system(4 * motions.size(), 4);
    for (std::size_t i = 0; i < motions.size(); ++i) {
      Eigen::Quaterniond q_lidar(motions[i].lidar.rotation());
      Eigen::Quaterniond q_camera(motions[i].camera.rotation());
      if (q_lidar.w() < 0.0) {
        q_lidar.coeffs() *= -1.0;
      }
      if (q_camera.w() < 0.0) {
        q_camera.coeffs() *= -1.0;
      }
      system.block<4, 4>(4 * i, 0) =
        quaternion_left(q_lidar) - quaternion_right(q_camera);
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(system, Eigen::ComputeFullV);
    singular_values = svd.singularValues().head<4>();
    // AX=XB must have exactly one quaternion null-space direction. Two or more
    // near-zero singular values indicate insufficient rotation-axis excitation.
    if (singular_values(2) < 1.0e-6) {
      return false;
    }
    const Eigen::Vector4d q_vector = svd.matrixV().col(3);
    Eigen::Quaterniond q(q_vector(0), q_vector(1), q_vector(2), q_vector(3));
    if (!q.coeffs().allFinite() || q.norm() < 1.0e-8) {
      return false;
    }
    q.normalize();
    rotation = q.toRotationMatrix();
    return true;
  }

  bool solve_translation_and_scale(
    const std::vector<MotionPair> & motions, const Eigen::Matrix3d & rotation,
    Eigen::Vector3d & translation, double & scale, Eigen::VectorXd & singular_values) const
  {
    const int columns = estimate_camera_scale_ ? 4 : 3;
    Eigen::MatrixXd system(3 * motions.size(), columns);
    Eigen::VectorXd target(3 * motions.size());
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    for (std::size_t i = 0; i < motions.size(); ++i) {
      const auto row = static_cast<Eigen::Index>(3 * i);
      system.block<3, 3>(row, 0) = motions[i].lidar.rotation() - identity;
      if (estimate_camera_scale_) {
        system.block<3, 1>(row, 3) = -rotation * motions[i].camera.translation();
        target.segment<3>(row) = -motions[i].lidar.translation();
      } else {
        target.segment<3>(row) =
          rotation * motions[i].camera.translation() - motions[i].lidar.translation();
      }
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      system, Eigen::ComputeThinU | Eigen::ComputeThinV);
    singular_values = svd.singularValues();
    if (svd.rank() < columns) {
      return false;
    }
    const Eigen::VectorXd solution = svd.solve(target);
    if (!solution.allFinite()) {
      return false;
    }
    translation = solution.head<3>();
    scale = estimate_camera_scale_ ? solution(3) : 1.0;
    return std::isfinite(scale);
  }

  void compute_residuals(
    const std::vector<MotionPair> & motions, const Eigen::Matrix3d & rotation,
    const Eigen::Vector3d & translation, double scale, double & rotation_rms_deg,
    double & translation_rms_m) const
  {
    double rotation_squared_sum = 0.0;
    double translation_squared_sum = 0.0;
    for (const auto & motion : motions) {
      const Eigen::Matrix3d left_rotation = motion.lidar.rotation() * rotation;
      const Eigen::Matrix3d right_rotation = rotation * motion.camera.rotation();
      const double angle = rotation_angle(left_rotation * right_rotation.transpose());
      rotation_squared_sum += angle * angle;

      const Eigen::Vector3d residual =
        motion.lidar.rotation() * translation + motion.lidar.translation() -
        (rotation * (scale * motion.camera.translation()) + translation);
      translation_squared_sum += residual.squaredNorm();
    }
    rotation_rms_deg =
      std::sqrt(rotation_squared_sum / static_cast<double>(motions.size())) * kRadToDeg;
    translation_rms_m =
      std::sqrt(translation_squared_sum / static_cast<double>(motions.size()));
  }

  static Eigen::Vector3d rotation_to_rpy(const Eigen::Matrix3d & r)
  {
    const double pitch = std::asin(std::clamp(-r(2, 0), -1.0, 1.0));
    const double cos_pitch = std::cos(pitch);
    double roll;
    double yaw;
    if (std::abs(cos_pitch) > 1.0e-8) {
      roll = std::atan2(r(2, 1), r(2, 2));
      yaw = std::atan2(r(1, 0), r(0, 0));
    } else {
      roll = 0.0;
      yaw = std::atan2(-r(0, 1), r(1, 1));
    }
    return {roll, pitch, yaw};
  }

  bool write_result(
    const Eigen::Matrix3d & rotation, const Eigen::Vector3d & translation, double scale,
    std::size_t synchronized_count, std::size_t motion_count, double mean_sync_error,
    double rotation_rms_deg, double translation_rms_m,
    const Eigen::Vector4d & rotation_singular_values,
    const Eigen::VectorXd & translation_singular_values) const
  {
    std::ofstream out(output_yaml_, std::ios::trunc);
    if (!out) {
      return false;
    }
    Eigen::Quaterniond q(rotation);
    q.normalize();
    const Eigen::Vector3d rpy = rotation_to_rpy(rotation);
    out << std::setprecision(15)
        << "result:\n"
        << "  convention: 'p_reference = T_reference_camera * p_camera'\n"
        << "  parent_frame: '" << reference_frame_ << "'\n"
        << "  child_frame: '" << camera_frame_ << "'\n"
        << "  translation: {x: " << translation.x() << ", y: " << translation.y()
        << ", z: " << translation.z() << "}\n"
        << "  quaternion: {x: " << q.x() << ", y: " << q.y() << ", z: " << q.z()
        << ", w: " << q.w() << "}\n"
        << "  rpy_rad: {roll: " << rpy.x() << ", pitch: " << rpy.y()
        << ", yaw: " << rpy.z() << "}\n"
        << "  camera_translation_scale_m_per_orb_unit: " << scale << "\n"
        << "  matrix_row_major:\n";
    for (int row = 0; row < 3; ++row) {
      out << "    - [" << rotation(row, 0) << ", " << rotation(row, 1) << ", "
          << rotation(row, 2) << ", " << translation(row) << "]\n";
    }
    out << "    - [0.0, 0.0, 0.0, 1.0]\n"
        << "quality:\n"
        << "  synchronized_pose_count: " << synchronized_count << "\n"
        << "  motion_pair_count: " << motion_count << "\n"
        << "  mean_sync_error_sec: " << mean_sync_error << "\n"
        << "  rotation_rms_deg: " << rotation_rms_deg << "\n"
        << "  translation_rms_m: " << translation_rms_m << "\n"
        << "  rotation_system_singular_values: [";
    for (Eigen::Index i = 0; i < rotation_singular_values.size(); ++i) {
      out << (i == 0 ? "" : ", ") << rotation_singular_values(i);
    }
    out << "]\n"
        << "  translation_system_singular_values: [";
    for (Eigen::Index i = 0; i < translation_singular_values.size(); ++i) {
      out << (i == 0 ? "" : ", ") << translation_singular_values(i);
    }
    out << "]\n"
        << "configuration:\n"
        << "  camera_time_offset_sec: " << camera_time_offset_sec_ << "\n"
        << "  pair_stride: " << pair_stride_ << "\n"
        << "  estimate_camera_scale: " << (estimate_camera_scale_ ? "true" : "false") << "\n";
    return true;
  }

  void calibrate_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::vector<SynchronizedPose> samples;
    {
      std::scoped_lock lock(mutex_);
      if (require_camera_tracking_ok_ &&
        (!camera_tracking_state_received_ || !camera_tracking_ok_))
      {
        response->success = false;
        response->message =
          "Camera tracking is not OK; collection is paused and no calibration can be solved";
        return;
      }
      samples = synchronized_;
    }
    const auto motions = make_motion_pairs(samples);
    if (motions.size() < static_cast<std::size_t>(min_motion_pairs_)) {
      std::ostringstream message;
      message << "Only " << motions.size() << " usable motion pairs from " << samples.size()
              << " synchronized poses; need at least " << min_motion_pairs_;
      response->success = false;
      response->message = message.str();
      return;
    }

    Eigen::Matrix3d rotation;
    Eigen::Vector4d rotation_singular_values;
    if (!solve_rotation(motions, rotation, rotation_singular_values)) {
      response->success = false;
      response->message = "Rotation solve failed";
      return;
    }
    Eigen::Vector3d translation;
    double scale = 1.0;
    Eigen::VectorXd translation_singular_values;
    if (!solve_translation_and_scale(
        motions, rotation, translation, scale, translation_singular_values))
    {
      response->success = false;
      response->message = "Translation/scale solve failed";
      return;
    }
    if (scale <= 0.0) {
      response->success = false;
      response->message =
        "Solved camera scale is not positive; check Tcw/Twc direction, timestamps, and motion "
        "observability";
      return;
    }

    double rotation_rms_deg = 0.0;
    double translation_rms_m = 0.0;
    compute_residuals(
      motions, rotation, translation, scale, rotation_rms_deg, translation_rms_m);
    double mean_sync_error = 0.0;
    for (const auto & sample : samples) {
      mean_sync_error += sample.sync_error;
    }
    mean_sync_error /= static_cast<double>(samples.size());

    if (!write_result(
        rotation, translation, scale, samples.size(), motions.size(), mean_sync_error,
        rotation_rms_deg, translation_rms_m, rotation_singular_values,
        translation_singular_values))
    {
      response->success = false;
      response->message = "Cannot write result file: " + output_yaml_;
      return;
    }

    Eigen::Quaterniond q(rotation);
    q.normalize();
    geometry_msgs::msg::TransformStamped result;
    result.header.stamp = now();
    result.header.frame_id = reference_frame_;
    result.child_frame_id = camera_frame_;
    result.transform.translation.x = translation.x();
    result.transform.translation.y = translation.y();
    result.transform.translation.z = translation.z();
    result.transform.rotation.x = q.x();
    result.transform.rotation.y = q.y();
    result.transform.rotation.z = q.z();
    result.transform.rotation.w = q.w();
    result_pub_->publish(result);

    std::ostringstream message;
    message << std::fixed << std::setprecision(6) << "Saved " << output_yaml_
            << "; pairs=" << motions.size() << ", scale=" << scale
            << ", rotation_rms_deg=" << rotation_rms_deg
            << ", translation_rms_m=" << translation_rms_m;
    response->success = true;
    response->message = message.str();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::scoped_lock lock(mutex_);
    clear_collected_poses_locked();
    response->success = true;
    response->message = camera_tracking_ok_ || !require_camera_tracking_ok_ ?
      "Cleared all collected poses; collection is active" :
      "Cleared all collected poses; waiting for camera tracking_ok=true";
  }

  std::string lidar_odometry_topic_;
  std::string camera_odometry_topic_;
  std::string camera_tracking_ok_topic_;
  std::string reference_frame_;
  std::string camera_frame_;
  std::string output_yaml_;
  double camera_time_offset_sec_{0.0};
  double max_sync_error_sec_{0.025};
  int max_queue_size_{1000};
  int pair_stride_{10};
  double min_motion_translation_m_{0.2};
  double min_motion_rotation_deg_{3.0};
  double max_relative_rotation_difference_deg_{2.0};
  int min_motion_pairs_{20};
  bool estimate_camera_scale_{true};
  bool invert_lidar_pose_{false};
  bool invert_camera_pose_{false};
  bool require_camera_tracking_ok_{true};
  bool camera_tracking_state_received_{false};
  bool camera_tracking_ok_{false};

  std::mutex mutex_;
  std::deque<TimedPose> lidar_queue_;
  std::deque<TimedPose> camera_queue_;
  std::vector<SynchronizedPose> synchronized_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr camera_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_tracking_ok_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr result_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};

}  // namespace lidar_camera_handeye_calibrator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<lidar_camera_handeye_calibrator::HandeyeCalibratorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_camera_handeye_calibrator"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
