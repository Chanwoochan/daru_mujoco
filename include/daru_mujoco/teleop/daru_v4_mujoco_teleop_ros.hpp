#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "daru_mujoco/teleop/daru_v4_mujoco_teleop_controller.hpp"

namespace daru_mj {

class DaruV4MuJoCoRosBridge {
 public:
  explicit DaruV4MuJoCoRosBridge(DaruV4MuJoCoController& controller,
                                 const std::string& node_name = "daru_v4_mujoco_teleop_sim",
                                 int wall_timer_hz = 500);
  ~DaruV4MuJoCoRosBridge();

  void SpinSome();
  bool ConsumeResetRequest();
  bool ConsumeTaskResetRequest();
  void Publish();

  void PublishStereoFrames(const std::vector<uint8_t>& left_rgb,
                           const std::vector<uint8_t>& right_rgb,
                           int width,
                           int height,
                           const rclcpp::Time& stamp);
  void PublishWristFrames(const std::vector<uint8_t>& left_rgb,
                          const std::vector<uint8_t>& right_rgb,
                          int width,
                          int height,
                          const rclcpp::Time& stamp);

 private:
  struct PendingHandTarget {
    bool valid = false;
    std::array<double, 6> pos{};
  };

  void OnWallTimer();
  void HandControlLoop();

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void PoseLCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void PoseRCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void LeftHandJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void RightHandJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void poseHeadCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);
  void LeftControllerCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void RightControllerCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

 private:
  DaruV4MuJoCoController& controller_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pos_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_ref_pos_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_acc_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_eff_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr act_ee_pos_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr des_ee_pos_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr real_ee_pos_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lhand_feedback_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rhand_feedback_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lerobot_next_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lerobot_reset_publisher_;

  image_transport::Publisher l_head_cam_pub_;
  image_transport::Publisher r_head_cam_pub_;
  image_transport::Publisher l_wrist_cam_pub_;
  image_transport::Publisher r_wrist_cam_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr l_head_cam_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr r_head_cam_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr l_wrist_cam_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr r_wrist_cam_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr l_head_cam_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr r_head_cam_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr l_wrist_cam_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr r_wrist_cam_compressed_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_l_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_r_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_hand_joint_states_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_hand_joint_states_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr lerobot_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr lerobot_action_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr left_hand_controller_states_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr right_hand_controller_states_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr pose_head_;
  rclcpp::TimerBase::SharedPtr wall_timer_;

  std::thread hand_control_thread_;
  std::atomic<bool> hand_control_running_{false};
  std::mutex hand_target_mutex_;
  PendingHandTarget pending_left_hand_target_;
  PendingHandTarget pending_right_hand_target_;

  bool reset_requested_ = false;
  bool task_reset_requested_ = false;
  bool ps_prev_ = false;
  bool left_button4_prev_ = false;
  bool left_button5_prev_ = false;
  bool right_a_prev_ = false;
  rclcpp::Time last_lerobot_next_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lerobot_reset_publish_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace daru_mj
