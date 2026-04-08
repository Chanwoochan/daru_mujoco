#include "daru_mujoco/gr00t_infer/daru_v4_mujoco_gr00t_infer_ros.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

constexpr std::array<const char*, 29> kgr00tJointNames = {
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow_pitch",
    "left_elbow_yaw",      "left_wrist_roll",    "left_wrist_pitch",  "right_shoulder_pitch",
    "right_shoulder_roll", "right_shoulder_yaw", "right_elbow_pitch", "right_elbow_yaw",
    "right_wrist_roll",    "right_wrist_pitch",  "left_hand_1",       "left_hand_2",
    "left_hand_3",         "left_hand_4",        "left_hand_5",       "left_hand_6",
    "right_hand_1",        "right_hand_2",       "right_hand_3",      "right_hand_4",
    "right_hand_5",        "right_hand_6",       "head_yaw",          "head_pitch",
    "waist_yaw"};

constexpr std::array<const char*, 29> kGr00tActionNames = {
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow_pitch",
    "left_elbow_yaw",      "left_wrist_roll",    "left_wrist_pitch",  "right_shoulder_pitch",
    "right_shoulder_roll", "right_shoulder_yaw", "right_elbow_pitch", "right_elbow_yaw",
    "right_wrist_roll",    "right_wrist_pitch",  "left_hand_1",       "left_hand_2",
    "left_hand_3",         "left_hand_4",        "left_hand_5",       "left_hand_6",
    "right_hand_1",        "right_hand_2",       "right_hand_3",      "right_hand_4",
    "right_hand_5",        "right_hand_6",       "head_yaw",          "head_pitch",
    "waist_yaw"};

sensor_msgs::msg::JointState Makegr00tJointStateMsg(const rclcpp::Time& stamp) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name.assign(kgr00tJointNames.begin(), kgr00tJointNames.end());
  msg.position.assign(kgr00tJointNames.size(), 0.0);
  msg.velocity.assign(kgr00tJointNames.size(), 0.0);
  msg.effort.assign(kgr00tJointNames.size(), 0.0);
  return msg;
}

sensor_msgs::msg::CompressedImage ToCompressedMsg(const std::vector<uint8_t>& rgb,
                                                  int width,
                                                  int height,
                                                  const std_msgs::msg::Header& header) {
  sensor_msgs::msg::CompressedImage msg;
  msg.header = header;
  msg.format = "jpeg";

  cv::Mat rgb_image(height, width, CV_8UC3, const_cast<uint8_t*>(rgb.data()));
  cv::Mat bgr_image;
  cv::cvtColor(rgb_image, bgr_image, cv::COLOR_RGB2BGR);
  const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, 90};
  cv::imencode(".jpg", bgr_image, msg.data, encode_params);
  return msg;
}

std_msgs::msg::Float64MultiArray ToMsg(const std::array<double, 15>& values) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(values.begin(), values.end());
  return msg;
}

std_msgs::msg::Float64MultiArray ToMsg(const std::array<double, 14>& values) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(values.begin(), values.end());
  return msg;
}

bool ExtractNamedActionPositions(const sensor_msgs::msg::JointState& msg,
                                 std::array<double, daru_mj::DaruV4MuJoCoCustomController::kGr00tDof>& out) {
  if (msg.name.size() != msg.position.size()) {
    return false;
  }

  for (std::size_t i = 0; i < kGr00tActionNames.size(); ++i) {
    const auto it = std::find(msg.name.begin(), msg.name.end(), kGr00tActionNames[i]);
    if (it == msg.name.end()) {
      return false;
    }
    const std::size_t idx = static_cast<std::size_t>(std::distance(msg.name.begin(), it));
    out[i] = msg.position[idx];
  }
  return true;
}

}  // namespace

namespace daru_mj {

DaruV4MuJoCoCustomRosBridge::DaruV4MuJoCoCustomRosBridge(DaruV4MuJoCoCustomController& controller,
                                             const std::string& node_name,
                                             int wall_timer_hz)
    : controller_(controller), node_(std::make_shared<rclcpp::Node>(node_name)) {
  arm_pos_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("kcwarm/position", 10);
  arm_vel_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("kcwarm/velocity", 10);
  arm_acc_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("kcwarm/acceleration", 10);
  arm_eff_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("kcwarm/effort", 10);
  arm_ref_pos_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("kcwarm/ref_position", 10);

  des_ee_pos_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("kcwarm/EE/ref_pos", 10);
  act_ee_pos_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("kcwarm/EE/pos", 10);
  real_ee_pos_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("kcwarm/EE/pos_real", 10);
  lhand_feedback_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("robot/left_hand_feedback", 10);
  rhand_feedback_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("robot/right_hand_feedback", 10);

  l_head_cam_pub_ = image_transport::create_publisher(node_.get(), "zed_left_lerobot/image_raw");
  r_head_cam_pub_ = image_transport::create_publisher(node_.get(), "zed_right_lerobot/image_raw");
  l_wrist_cam_pub_ = image_transport::create_publisher(node_.get(), "/camera/left_hand/color/image_rect_raw");
  r_wrist_cam_pub_ = image_transport::create_publisher(node_.get(), "/camera/right_hand/color/image_rect_raw");
  l_head_cam_info_pub_ =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>("zed_left/camera_info", 10);
  r_head_cam_info_pub_ =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>("zed_right/camera_info", 10);
  l_wrist_cam_info_pub_ =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/left_hand/color/image_rect_raw/camera_info", 10);
  r_wrist_cam_info_pub_ =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/right_hand/color/image_rect_raw/camera_info", 10);
  l_head_cam_compressed_pub_ =
      node_->create_publisher<sensor_msgs::msg::CompressedImage>("zed_left/image_raw/compressed", 10);
  r_head_cam_compressed_pub_ =
      node_->create_publisher<sensor_msgs::msg::CompressedImage>("zed_right/image_raw/compressed", 10);
  l_wrist_cam_compressed_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera/left_hand/color/image_rect_raw/compressed", 10);
  r_wrist_cam_compressed_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera/right_hand/color/image_rect_raw/compressed", 10);
  gr00t_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("gr00t/state", 10);

  joy_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&DaruV4MuJoCoCustomRosBridge::JoyCallback, this, std::placeholders::_1));
  gr00t_action_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "gr00t/action", 10,
      std::bind(&DaruV4MuJoCoCustomRosBridge::PoseGR00TCallback, this, std::placeholders::_1));

  executor_.add_node(node_);

  const int hz = std::max(1, wall_timer_hz);
  const auto period = std::chrono::microseconds(1000000 / hz);
  wall_timer_ = node_->create_wall_timer(period, std::bind(&DaruV4MuJoCoCustomRosBridge::OnWallTimer, this));

  hand_control_running_.store(true);
  hand_control_thread_ = std::thread(&DaruV4MuJoCoCustomRosBridge::HandControlLoop, this);
}

DaruV4MuJoCoCustomRosBridge::~DaruV4MuJoCoCustomRosBridge() {
  hand_control_running_.store(false);
  if (hand_control_thread_.joinable()) {
    hand_control_thread_.join();
  }
}

void DaruV4MuJoCoCustomRosBridge::SpinSome() { executor_.spin_some(); }

bool DaruV4MuJoCoCustomRosBridge::ConsumeResetRequest() {
  const bool requested = reset_requested_;
  reset_requested_ = false;
  return requested;
}

bool DaruV4MuJoCoCustomRosBridge::ConsumeTaskResetRequest() {
  const bool requested = task_reset_requested_;
  task_reset_requested_ = false;
  return requested;
}

void DaruV4MuJoCoCustomRosBridge::OnWallTimer() { Publish(); }

void DaruV4MuJoCoCustomRosBridge::HandControlLoop() {
  auto next_tick = std::chrono::steady_clock::now();
  const auto period = std::chrono::milliseconds(20);

  while (hand_control_running_.load()) {
    next_tick += period;
    std::this_thread::sleep_until(next_tick);
  }
}

void DaruV4MuJoCoCustomRosBridge::Publish() {
  arm_pos_publisher_->publish(ToMsg(controller_.JointPosition()));
  arm_ref_pos_publisher_->publish(ToMsg(controller_.JointRefPosition()));
  arm_vel_publisher_->publish(ToMsg(controller_.JointVelocity()));
  arm_acc_publisher_->publish(ToMsg(controller_.JointAcceleration()));
  arm_eff_publisher_->publish(ToMsg(controller_.JointEffort()));

  des_ee_pos_publisher_->publish(ToMsg(controller_.RefEePose()));
  act_ee_pos_publisher_->publish(ToMsg(controller_.ActEePose()));
  real_ee_pos_publisher_->publish(ToMsg(controller_.RealEePose()));
  auto hand_tau = controller_.GetFingerForceState();

  double max_abs_r = 0.0;
  for (int i = 2; i < 5; ++i) {
    max_abs_r = std::max(max_abs_r, std::abs(hand_tau[i]));
  }
  double max_abs_l = 0.0;
  for (int i = 8; i < 11; ++i) {
    max_abs_l = std::max(max_abs_l, std::abs(hand_tau[i]));
  }

  std_msgs::msg::Float32 lhand_msg;
  lhand_msg.data = std::max(0.0, max_abs_l / 56.);
  lhand_feedback_publisher_->publish(lhand_msg);

  std_msgs::msg::Float32 rhand_msg;
  rhand_msg.data = std::max(0.0, max_abs_r / 56.);
  rhand_feedback_publisher_->publish(rhand_msg);

  const auto arm_pos = controller_.JointPosition();
  const auto arm_vel = controller_.JointVelocity();
  const auto arm_eff = controller_.JointEffort();
  const auto head_pos = controller_.HeadPosition();
  const auto head_vel = controller_.HeadVelocity();
  const auto head_eff = controller_.HeadEffort();
  const auto hand_pos = controller_.HandPosition();
  const auto hand_vel = controller_.HandVelocity();
  const auto hand_eff = controller_.HandEffort();

  auto gr00t_state_msg = Makegr00tJointStateMsg(node_->get_clock()->now());

  for (int i = 0; i < 7; ++i) {
    gr00t_state_msg.position[i] = arm_pos[1 + i];
    gr00t_state_msg.velocity[i] = arm_vel[1 + i];
    gr00t_state_msg.effort[i] = arm_eff[1 + i];
  }

  for (int i = 0; i < 7; ++i) {
    gr00t_state_msg.position[7 + i] = arm_pos[8 + i];
    gr00t_state_msg.velocity[7 + i] = arm_vel[8 + i];
    gr00t_state_msg.effort[7 + i] = arm_eff[8 + i];
  }

  for (int i = 0; i < 6; ++i) {
    gr00t_state_msg.position[14 + i] = hand_pos[6 + i];
    gr00t_state_msg.velocity[14 + i] = hand_vel[6 + i];
    gr00t_state_msg.effort[14 + i] = hand_eff[6 + i];

    gr00t_state_msg.position[20 + i] = hand_pos[i];
    gr00t_state_msg.velocity[20 + i] = hand_vel[i];
    gr00t_state_msg.effort[20 + i] = hand_eff[i];
  }

  gr00t_state_msg.position[26] = head_pos[0];
  gr00t_state_msg.position[27] = head_pos[1];
  gr00t_state_msg.velocity[26] = head_vel[0];
  gr00t_state_msg.velocity[27] = head_vel[1];
  gr00t_state_msg.effort[26] = head_eff[0];
  gr00t_state_msg.effort[27] = head_eff[1];

  gr00t_state_msg.position[28] = arm_pos[0];
  gr00t_state_msg.velocity[28] = arm_vel[0];
  gr00t_state_msg.effort[28] = arm_eff[0];

  gr00t_state_pub_->publish(gr00t_state_msg);
}

void DaruV4MuJoCoCustomRosBridge::PublishStereoFrames(const std::vector<uint8_t>& left_rgb,
                                                const std::vector<uint8_t>& right_rgb,
                                                int width,
                                                int height,
                                                const rclcpp::Time& stamp) {
  if (static_cast<int>(left_rgb.size()) != width * height * 3 ||
      static_cast<int>(right_rgb.size()) != width * height * 3) {
    RCLCPP_WARN(node_->get_logger(), "Stereo frame size mismatch");
    return;
  }

  sensor_msgs::msg::Image l_img;
  l_img.header.stamp = stamp;
  l_img.header.frame_id = "l_zed_optical_frame";
  l_img.height = height;
  l_img.width = width;
  l_img.encoding = "rgb8";
  l_img.is_bigendian = false;
  l_img.step = width * 3;
  l_img.data = left_rgb;
  l_head_cam_pub_.publish(l_img);
  l_head_cam_compressed_pub_->publish(ToCompressedMsg(left_rgb, width, height, l_img.header));
  sensor_msgs::msg::CameraInfo l_info;
  l_info.header = l_img.header;
  l_info.height = height;
  l_info.width = width;
  l_head_cam_info_pub_->publish(l_info);

  sensor_msgs::msg::Image r_img;
  r_img.header.stamp = stamp;
  r_img.header.frame_id = "r_zed_optical_frame";
  r_img.height = height;
  r_img.width = width;
  r_img.encoding = "rgb8";
  r_img.is_bigendian = false;
  r_img.step = width * 3;
  r_img.data = right_rgb;
  r_head_cam_pub_.publish(r_img);
  r_head_cam_compressed_pub_->publish(ToCompressedMsg(right_rgb, width, height, r_img.header));
  sensor_msgs::msg::CameraInfo r_info;
  r_info.header = r_img.header;
  r_info.height = height;
  r_info.width = width;
  r_head_cam_info_pub_->publish(r_info);
}

void DaruV4MuJoCoCustomRosBridge::PublishWristFrames(const std::vector<uint8_t>& left_rgb,
                                               const std::vector<uint8_t>& right_rgb,
                                               int width,
                                               int height,
                                               const rclcpp::Time& stamp) {
  if (static_cast<int>(left_rgb.size()) != width * height * 3 ||
      static_cast<int>(right_rgb.size()) != width * height * 3) {
    RCLCPP_WARN(node_->get_logger(), "Wrist frame size mismatch");
    return;
  }

  sensor_msgs::msg::Image l_img;
  l_img.header.stamp = stamp;
  l_img.header.frame_id = "l_wrist_cam_optical_frame";
  l_img.height = height;
  l_img.width = width;
  l_img.encoding = "rgb8";
  l_img.is_bigendian = false;
  l_img.step = width * 3;
  l_img.data = left_rgb;
  l_wrist_cam_pub_.publish(l_img);
  l_wrist_cam_compressed_pub_->publish(ToCompressedMsg(left_rgb, width, height, l_img.header));

  sensor_msgs::msg::CameraInfo l_info;
  l_info.header = l_img.header;
  l_info.height = height;
  l_info.width = width;
  l_wrist_cam_info_pub_->publish(l_info);

  sensor_msgs::msg::Image r_img;
  r_img.header.stamp = stamp;
  r_img.header.frame_id = "r_wrist_cam_optical_frame";
  r_img.height = height;
  r_img.width = width;
  r_img.encoding = "rgb8";
  r_img.is_bigendian = false;
  r_img.step = width * 3;
  r_img.data = right_rgb;
  r_wrist_cam_pub_.publish(r_img);
  r_wrist_cam_compressed_pub_->publish(ToCompressedMsg(right_rgb, width, height, r_img.header));

  sensor_msgs::msg::CameraInfo r_info;
  r_info.header = r_img.header;
  r_info.height = height;
  r_info.width = width;
  r_wrist_cam_info_pub_->publish(r_info);
}

void DaruV4MuJoCoCustomRosBridge::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (!msg) return;
}

void DaruV4MuJoCoCustomRosBridge::PoseLCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!msg) return;

  RigidBodyDynamics::Math::Vector3d pos(msg->pose.position.x + 0.1,
                                        msg->pose.position.y,
                                        msg->pose.position.z + 0.5725);

  const Eigen::Quaterniond quat(msg->pose.orientation.w,
                                -msg->pose.orientation.z,
                                msg->pose.orientation.x,
                                -msg->pose.orientation.y);

  const Eigen::Quaterniond quat_res = quat * Eigen::Quaterniond(0.707, 0, -0.707, 0);
  controller_.SetLeftEeTarget(pos, quat_res.normalized());
}

void DaruV4MuJoCoCustomRosBridge::PoseRCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!msg) return;

  RigidBodyDynamics::Math::Vector3d pos(msg->pose.position.x + 0.1,
                                        msg->pose.position.y,
                                        msg->pose.position.z + 0.5725);

  const Eigen::Quaterniond quat(msg->pose.orientation.w,
                                -msg->pose.orientation.z,
                                msg->pose.orientation.x,
                                -msg->pose.orientation.y);

  const Eigen::Quaterniond quat_res = quat * Eigen::Quaterniond(0.707, 0, -0.707, 0);
  controller_.SetRightEeTarget(pos, quat_res.normalized());
}

void DaruV4MuJoCoCustomRosBridge::LeftHandJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!msg) return;
}

void DaruV4MuJoCoCustomRosBridge::RightHandJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!msg) return;
}

void DaruV4MuJoCoCustomRosBridge::poseHeadCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
    Eigen::Quaterniond quat(-msg->quaternion.w,
                          msg->quaternion.x,
                          msg->quaternion.y,
                          msg->quaternion.z);
    controller_.SetHeadTarget(quat.normalized());
}

void DaruV4MuJoCoCustomRosBridge::LeftControllerCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (!msg) return;

  const double trig = msg->axes[2];
  const double grip = msg->axes[3];
  const double stick_x = msg->axes[0];
  // const double thumb = std::max(trig, grip);

  double pos[6] = {
      grip * 0.0135, grip * 0.0135,
      trig * 0.0135, trig * 0.0135,
      trig * 0.01, (1 + stick_x) * 0.5 * 0.01};

  for (int i = 0; i < 6; ++i) {
    pending_left_hand_target_.pos[i] +=
        std::clamp(pos[i] - pending_left_hand_target_.pos[i], -0.00035, 0.00035);
  }
  controller_.SetLeftHandLinearTargets(pending_left_hand_target_.pos);
}

void DaruV4MuJoCoCustomRosBridge::RightControllerCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (!msg) return;

  const double trig = msg->axes[2];
  const double grip = msg->axes[3];
  const double stick_x = msg->axes[0];
  const bool a_pressed = msg->buttons.size() > 4 && msg->buttons[4] != 0;
  // const double thumb = std::max(trig, grip);

  double pos[6] = {
      grip * 0.0135, grip * 0.0135,
      trig * 0.0135, trig * 0.0135,
      trig * 0.01, (1 - stick_x) * 0.5 * 0.01};

  for (int i = 0; i < 6; ++i) {
    pending_right_hand_target_.pos[i] +=
        std::clamp(pos[i] - pending_right_hand_target_.pos[i], -0.00035, 0.00035);
  }
  controller_.SetRightHandLinearTargets(pending_right_hand_target_.pos);

  if (a_pressed && !right_a_prev_) {
    task_reset_requested_ = true;
  }
  right_a_prev_ = a_pressed;
}

void DaruV4MuJoCoCustomRosBridge::PoseGR00TCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!msg) {
    return;
  }

  std::array<double, DaruV4MuJoCoCustomController::kGr00tDof> action_pos{};
  bool using_named_mapping = false;
  if (!msg->name.empty()) {
    if (!ExtractNamedActionPositions(*msg, action_pos)) {
      RCLCPP_WARN(node_->get_logger(),
                  "gr00t/action names do not match the expected 29-joint schema; dropping message");
      return;
    }
    using_named_mapping = true;
  } else {
    if (msg->position.size() < DaruV4MuJoCoCustomController::kGr00tDof) {
      RCLCPP_WARN(node_->get_logger(), "gr00t/action requires %d joint positions, got %zu",
                  DaruV4MuJoCoCustomController::kGr00tDof, msg->position.size());
      return;
    }
    std::copy_n(msg->position.begin(), DaruV4MuJoCoCustomController::kGr00tDof, action_pos.begin());
  }

  const auto& p = action_pos;
  std::array<double, DaruV4MuJoCoCustomController::kGr00tDof> targets{};

  targets[0] = std::clamp(p[0], -kPi * 168.0 / 180.0, kPi * 168.0 / 180.0);
  targets[1] = std::clamp(p[1], -kPi / 9.0, kPi * 10.0 / 9.0);
  targets[2] = std::clamp(p[2], -kPi * 165.0 / 180.0, kPi * 165.0 / 180.0);
  targets[3] = std::clamp(p[3], -kPi * 3.0 / 4.0, 0.0);
  targets[4] = std::clamp(p[4], -kPi / 2.0, kPi / 2.0);
  targets[5] = std::clamp(p[5], -kPi / 5.0, kPi / 5.0);
  targets[6] = std::clamp(p[6], -kPi / 3.0, kPi / 3.0);

  targets[7] = std::clamp(p[7], -kPi * 168.0 / 180.0, kPi * 168.0 / 180.0);
  targets[8] = std::clamp(p[8], -kPi * 10.0 / 9.0, kPi / 9.0);
  targets[9] = std::clamp(p[9], -kPi * 165.0 / 180.0, kPi * 165.0 / 180.0);
  targets[10] = std::clamp(p[10], -kPi * 3.0 / 4.0, 0.0);
  targets[11] = std::clamp(p[11], -kPi / 2.0, kPi / 2.0);
  targets[12] = std::clamp(p[12], -kPi / 5.0, kPi / 5.0);
  targets[13] = std::clamp(p[13], -kPi / 3.0, kPi / 3.0);

  for (int i = 14; i <= 25; ++i) {
    targets[i] =  p[i];
  }

  targets[26] = std::clamp(p[26], -kPi * 90.0 / 180.0, kPi * 90.0 / 180.0);
  targets[27] = std::clamp(p[27], -kPi * 19.0 / 180.0, kPi / 4.0);
  targets[28] = std::clamp(p[28], -kPi * 90.0 / 180.0, kPi * 90.0 / 180.0);

  if (!using_named_mapping && !msg->name.empty()) {
    RCLCPP_WARN(node_->get_logger(), "gr00t/action fell back to positional mapping");
  }
  controller_.SetGr00tRefTargets(targets);
}

}  // namespace daru_mj
