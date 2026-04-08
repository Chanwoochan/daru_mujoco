#include "daru_mujoco/teleop/daru_v4_mujoco_teleop_ros.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

constexpr auto kLeRobotNextPublishCooldown = std::chrono::milliseconds(1000);

constexpr std::array<const char*, 29> kLeRobotStateNames = {
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow_pitch",
    "left_elbow_yaw",      "left_wrist_roll",    "left_wrist_pitch",  "right_shoulder_pitch",
    "right_shoulder_roll", "right_shoulder_yaw", "right_elbow_pitch", "right_elbow_yaw",
    "right_wrist_roll",    "right_wrist_pitch",  "left_hand_1",       "left_hand_2",
    "left_hand_3",         "left_hand_4",        "left_hand_5",       "left_hand_6",
    "right_hand_1",        "right_hand_2",       "right_hand_3",      "right_hand_4",
    "right_hand_5",        "right_hand_6",       "head_yaw",          "head_pitch",
    "waist_yaw"};

constexpr std::array<const char*, 29> kLeRobotActionNames = {
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow_pitch",
    "left_elbow_yaw",      "left_wrist_roll",    "left_wrist_pitch",  "right_shoulder_pitch",
    "right_shoulder_roll", "right_shoulder_yaw", "right_elbow_pitch", "right_elbow_yaw",
    "right_wrist_roll",    "right_wrist_pitch",  "left_hand_1",       "left_hand_2",
    "left_hand_3",         "left_hand_4",        "left_hand_5",       "left_hand_6",
    "right_hand_1",        "right_hand_2",       "right_hand_3",      "right_hand_4",
    "right_hand_5",        "right_hand_6",       "head_yaw",          "head_pitch",
    "waist_yaw"};

template <std::size_t N>
sensor_msgs::msg::JointState MakeLeRobotJointStateMsg(const rclcpp::Time& stamp,
                                                      const std::array<const char*, N>& names) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name.assign(names.begin(), names.end());
  msg.position.assign(names.size(), 0.0);
  msg.velocity.assign(names.size(), 0.0);
  msg.effort.assign(names.size(), 0.0);
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

}  // namespace

namespace daru_mj {

DaruV4MuJoCoRosBridge::DaruV4MuJoCoRosBridge(DaruV4MuJoCoController& controller,
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
  lerobot_next_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/lerobot/next", 10);
  lerobot_reset_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/lerobot/reset", 10);

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
  lerobot_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("lerobot/state", 10);
  lerobot_action_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("lerobot/action", 10);

  joy_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&DaruV4MuJoCoRosBridge::JoyCallback, this, std::placeholders::_1));

  pose_l_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "left_controller_pose", 10,
      std::bind(&DaruV4MuJoCoRosBridge::PoseLCallback, this, std::placeholders::_1));
  pose_r_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "right_controller_pose", 10,
      std::bind(&DaruV4MuJoCoRosBridge::PoseRCallback, this, std::placeholders::_1));

  left_hand_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "left_hand_joint_states", rclcpp::SensorDataQoS(),
      std::bind(&DaruV4MuJoCoRosBridge::LeftHandJointCallback, this, std::placeholders::_1));
  right_hand_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "right_hand_joint_states", rclcpp::SensorDataQoS(),
      std::bind(&DaruV4MuJoCoRosBridge::RightHandJointCallback, this, std::placeholders::_1));
  pose_head_ = node_->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "vr_headset_rotation", 10,
      std::bind(&DaruV4MuJoCoRosBridge::poseHeadCallback, this, std::placeholders::_1));

  left_hand_controller_states_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "left_controller_input", 10,
      std::bind(&DaruV4MuJoCoRosBridge::LeftControllerCallback, this, std::placeholders::_1));
  right_hand_controller_states_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "right_controller_input", 10,
      std::bind(&DaruV4MuJoCoRosBridge::RightControllerCallback, this, std::placeholders::_1));

  executor_.add_node(node_);

  const int hz = std::max(1, wall_timer_hz);
  const auto period = std::chrono::microseconds(1000000 / hz);
  wall_timer_ = node_->create_wall_timer(period, std::bind(&DaruV4MuJoCoRosBridge::OnWallTimer, this));

  hand_control_running_.store(true);
  hand_control_thread_ = std::thread(&DaruV4MuJoCoRosBridge::HandControlLoop, this);
}

DaruV4MuJoCoRosBridge::~DaruV4MuJoCoRosBridge() {
  hand_control_running_.store(false);
  if (hand_control_thread_.joinable()) {
    hand_control_thread_.join();
  }
}

void DaruV4MuJoCoRosBridge::SpinSome() { executor_.spin_some(); }

bool DaruV4MuJoCoRosBridge::ConsumeResetRequest() {
  const bool requested = reset_requested_;
  reset_requested_ = false;
  return requested;
}

bool DaruV4MuJoCoRosBridge::ConsumeTaskResetRequest() {
  const bool requested = task_reset_requested_;
  task_reset_requested_ = false;
  return requested;
}

void DaruV4MuJoCoRosBridge::OnWallTimer() { Publish(); }

void DaruV4MuJoCoRosBridge::HandControlLoop() {
  auto next_tick = std::chrono::steady_clock::now();
  const auto period = std::chrono::milliseconds(20);

  while (hand_control_running_.load()) {
    next_tick += period;
    std::this_thread::sleep_until(next_tick);
  }
}

void DaruV4MuJoCoRosBridge::Publish() {
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
  const auto arm_ref = controller_.JointRefPosition();
  const auto arm_vel = controller_.JointVelocity();
  const auto arm_eff = controller_.JointEffort();
  const auto head_pos = controller_.HeadPosition();
  const auto head_ref = controller_.HeadRefPosition();
  const auto head_vel = controller_.HeadVelocity();
  const auto head_eff = controller_.HeadEffort();
  const auto hand_pos = controller_.HandPosition();
  const auto hand_ref = controller_.HandRefPosition();
  const auto hand_vel = controller_.HandVelocity();
  const auto hand_eff = controller_.HandEffort();

  auto lerobot_state_msg = MakeLeRobotJointStateMsg(node_->get_clock()->now(), kLeRobotStateNames);
  auto lerobot_action_msg = MakeLeRobotJointStateMsg(lerobot_state_msg.header.stamp, kLeRobotActionNames);

  for (int i = 0; i < 7; ++i) {
    lerobot_state_msg.position[i] = arm_pos[1 + i];
    lerobot_state_msg.velocity[i] = arm_vel[1 + i];
    lerobot_state_msg.effort[i] = arm_eff[1 + i];
    lerobot_action_msg.position[i] = arm_ref[1 + i];
  }

  for (int i = 0; i < 7; ++i) {
    lerobot_state_msg.position[7 + i] = arm_pos[8 + i];
    lerobot_state_msg.velocity[7 + i] = arm_vel[8 + i];
    lerobot_state_msg.effort[7 + i] = arm_eff[8 + i];
    lerobot_action_msg.position[7 + i] = arm_ref[8 + i];
  }

  for (int i = 0; i < 6; ++i) {
    lerobot_state_msg.position[14 + i] = hand_pos[6 + i];
    lerobot_state_msg.velocity[14 + i] = hand_vel[6 + i];
    lerobot_state_msg.effort[14 + i] = hand_eff[6 + i];
    lerobot_action_msg.position[14 + i] = hand_ref[6 + i];

    lerobot_state_msg.position[20 + i] = hand_pos[i];
    lerobot_state_msg.velocity[20 + i] = hand_vel[i];
    lerobot_state_msg.effort[20 + i] = hand_eff[i];
    lerobot_action_msg.position[20 + i] = hand_ref[i];
  }

  lerobot_state_msg.position[26] = head_pos[0];
  lerobot_state_msg.position[27] = head_pos[1];
  lerobot_state_msg.velocity[26] = head_vel[0];
  lerobot_state_msg.velocity[27] = head_vel[1];
  lerobot_state_msg.effort[26] = head_eff[0];
  lerobot_state_msg.effort[27] = head_eff[1];
  lerobot_action_msg.position[26] = head_ref[0];
  lerobot_action_msg.position[27] = head_ref[1];

  lerobot_state_msg.position[28] = arm_pos[0];
  lerobot_state_msg.velocity[28] = arm_vel[0];
  lerobot_state_msg.effort[28] = arm_eff[0];
  lerobot_action_msg.position[28] = arm_ref[0];

  lerobot_state_pub_->publish(lerobot_state_msg);
  lerobot_action_pub_->publish(lerobot_action_msg);
}

void DaruV4MuJoCoRosBridge::PublishStereoFrames(const std::vector<uint8_t>& left_rgb,
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

void DaruV4MuJoCoRosBridge::PublishWristFrames(const std::vector<uint8_t>& left_rgb,
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

void DaruV4MuJoCoRosBridge::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (!msg) return;
}

void DaruV4MuJoCoRosBridge::PoseLCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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

void DaruV4MuJoCoRosBridge::PoseRCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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

void DaruV4MuJoCoRosBridge::LeftHandJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!msg) return;
}

void DaruV4MuJoCoRosBridge::RightHandJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!msg) return;
}

void DaruV4MuJoCoRosBridge::poseHeadCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
    Eigen::Quaterniond quat(-msg->quaternion.w,
                          msg->quaternion.x,
                          msg->quaternion.y,
                          msg->quaternion.z);
    controller_.SetHeadTarget(quat.normalized());
}

void DaruV4MuJoCoRosBridge::LeftControllerCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (!msg) return;

  const double trig = msg->axes[2];
  const double grip = msg->axes[3];
  const double stick_x = msg->axes[0];
  const bool button4_pressed = msg->buttons.size() > 4 && msg->buttons[4] != 0;
  const bool button5_pressed = msg->buttons.size() > 5 && msg->buttons[5] != 0;
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

  if (button4_pressed && !left_button4_prev_) {
    const rclcpp::Time now = node_->get_clock()->now();
    if (last_lerobot_next_publish_time_.nanoseconds() == 0 ||
        (now - last_lerobot_next_publish_time_) >=
            rclcpp::Duration::from_nanoseconds(kLeRobotNextPublishCooldown.count() * 1000000LL)) {
      std_msgs::msg::Bool next_msg;
      next_msg.data = true;
      lerobot_next_publisher_->publish(next_msg);
      last_lerobot_next_publish_time_ = now;
    }
  }
  if (button5_pressed && !left_button5_prev_) {
    const rclcpp::Time now = node_->get_clock()->now();
    if (last_lerobot_reset_publish_time_.nanoseconds() == 0 ||
        (now - last_lerobot_reset_publish_time_) >=
            rclcpp::Duration::from_nanoseconds(kLeRobotNextPublishCooldown.count() * 1000000LL)) {
      std_msgs::msg::Bool reset_msg;
      reset_msg.data = true;
      lerobot_reset_publisher_->publish(reset_msg);
      last_lerobot_reset_publish_time_ = now;
    }
  }
  left_button4_prev_ = button4_pressed;
  left_button5_prev_ = button5_pressed;
}

void DaruV4MuJoCoRosBridge::RightControllerCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
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

}  // namespace daru_mj
