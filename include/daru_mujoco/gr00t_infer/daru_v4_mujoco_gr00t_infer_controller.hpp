#pragma once

#include <array>
#include <mutex>
#include <string>

#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "daru_mujoco/daru_v4_rbdl.hpp"

namespace daru_mj {

class DaruV4MuJoCoCustomController {
 public:
  static constexpr int kDof = 15;
  static constexpr int kGr00tDof = 29;

  DaruV4MuJoCoCustomController(mjModel* model, mjData* data, const std::string& urdf_path);

  void Step();
  void Reset();
  void ResetToMode2();
  void ApplyWorkReadyPose();
  int mode() const { return mode_; }

  void SetLeftEeTarget(const RigidBodyDynamics::Math::Vector3d& pos, const Eigen::Quaterniond& quat);
  void SetRightEeTarget(const RigidBodyDynamics::Math::Vector3d& pos, const Eigen::Quaterniond& quat);
  void SetHeadTarget(const Eigen::Quaterniond& quat);
  void SetLeftEePositionTarget(const RigidBodyDynamics::Math::Vector3d& pos);
  void SetLeftHandLinearTargets(const std::array<double, 6>& targets);
  void SetRightHandLinearTargets(const std::array<double, 6>& targets);
  void SetGr00tRefTargets(const std::array<double, kGr00tDof>& targets);

  std::array<double, kDof> JointPosition() const;
  std::array<double, kDof> JointRefPosition() const;
  std::array<double, kDof> JointVelocity() const;
  std::array<double, kDof> JointAcceleration() const;
  std::array<double, kDof> JointEffort() const;
  std::array<double, 2> HeadPosition() const;
  std::array<double, 2> HeadRefPosition() const;
  std::array<double, 2> HeadVelocity() const;
  std::array<double, 2> HeadEffort() const;
  std::array<double, 12> HandPosition() const;
  std::array<double, 12> HandRefPosition() const;
  std::array<double, 12> HandVelocity() const;
  std::array<double, 12> HandEffort() const;

  std::array<double, 14> RefEePose() const;
  std::array<double, 14> ActEePose() const;
  std::array<double, 14> RealEePose();
  std::array<double, 12> GetFingerForceState();

  mjModel* GetModel() const { return m_; }
  mjData* GetData() const { return d_; }

 private:
  void ResetInternal(int start_mode);
  void StartMode2HomeBlend();
  void ApplyMode2HomeBlend();
  void InitializeHomeModeIfNeeded();
  void StateUpdate();
  void ApplyArmTorque();
  void HandControlTick();
  void HeadPdControl();
  void ApplyGr00tTargets();
  void GetCurrentState();
  void SetZeroPos();
  void RbdlStateUpdate();
  void IKSolver(RigidBodyDynamics::Math::Vector3d l_pos_des,
                RigidBodyDynamics::Math::Vector3d r_pos_des,
                Eigen::Quaterniond l_quat_des,
                Eigen::Quaterniond r_quat_des);
  void RefTorqueInit();
  void CGCompensation(double k);
  void JointPdControl(bool cur_reset = true);
  void JointTorqueControl(bool cur_reset = true);

 private:
  mjModel* m_;
  mjData* d_;
  RigidBodyDynamics::DARU_RBDL daru_;

  std::array<int, kDof> map_joint_;
  std::array<int, kDof> map_qpos_;
  std::array<int, kDof> map_dof_;
  std::array<int, kDof> map_actuator_;

  std::array<JOINT_, kDof> joints_{};
  std::array<JOINT_, kDof> upper_joint_{};

  unsigned long long world_cnt_ = 0;
  unsigned long long cnt_ = 0;
  int mode_ = 0;
  int reset_start_mode_ = 1;
  bool initialized_ = false;

  double last_update_time_ = 0.0;
  double dt_ = 0.002;

  double pos_init_[kDof]{};
  double des_pos_[kDof]{};
  double des_torque_[kDof]{};
  double pd_des_current_[kDof]{};

  RigidBodyDynamics::Math::VectorNd ref_pos_ = RigidBodyDynamics::Math::VectorNd::Zero(kDof);

  RigidBodyDynamics::Math::Vector3d l_pos_des_ = RigidBodyDynamics::Math::Vector3d::Zero();
  RigidBodyDynamics::Math::Vector3d r_pos_des_ = RigidBodyDynamics::Math::Vector3d::Zero();
  Eigen::Quaterniond l_quat_des_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond r_quat_des_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond head_quat_des_ = Eigen::Quaterniond::Identity();
  mutable std::mutex target_mutex_;
  mutable std::mutex hand_target_mutex_;
  mutable std::mutex gr00t_target_mutex_;
  std::array<double, kGr00tDof> gr00t_ref_pos_{};
  bool gr00t_target_received_ = false;

  RigidBodyDynamics::Math::Vector3d l_pos_act_ = RigidBodyDynamics::Math::Vector3d::Zero();
  RigidBodyDynamics::Math::Vector3d r_pos_act_ = RigidBodyDynamics::Math::Vector3d::Zero();
  RigidBodyDynamics::Math::Matrix3d l_rot_act_ = RigidBodyDynamics::Math::Matrix3d::Identity();
  RigidBodyDynamics::Math::Matrix3d r_rot_act_ = RigidBodyDynamics::Math::Matrix3d::Identity();
  RigidBodyDynamics::Math::Vector3d phi_l_ = RigidBodyDynamics::Math::Vector3d::Zero();
  RigidBodyDynamics::Math::Vector3d phi_r_ = RigidBodyDynamics::Math::Vector3d::Zero();
  bool mode2_home_blend_active_ = false;
  double mode2_home_blend_elapsed_ = 0.0;

  static constexpr int kHeadDof = 2;
  std::array<int, kHeadDof> head_map_joint_{};
  std::array<int, kHeadDof> head_map_dof_{};
  std::array<int, kHeadDof> head_map_actuator_{};
  std::array<double, kHeadDof> head_pos_{};
  std::array<double, kHeadDof> head_vel_{};
  std::array<double, kHeadDof> head_tau_{};
  std::array<double, kHeadDof> head_pos_init_{};
  std::array<double, kHeadDof> head_des_pos_{};
  std::array<double, kHeadDof> head_cmd_tau_{};
  bool head_initialized_ = false;

  static constexpr int kHandPerSide = 6;
  static constexpr int kHandDof = 12;
  std::array<int, kHandDof> hand_map_joint_{};
  std::array<int, kHandDof> hand_map_qpos_{};
  std::array<int, kHandDof> hand_map_dof_{};
  std::array<int, kHandDof> hand_map_actuator_{};
  std::array<int, kHandDof> hand_force_sensor_{};
  std::array<int, kHandDof> hand_force_sensor_adr_{};
  std::array<int, kHandDof> hand_ctrl_mode_{};
  std::array<double, kHandDof> hand_pos_{};
  std::array<double, kHandDof> hand_vel_{};
  std::array<double, kHandDof> hand_tau_{};
  std::array<double, kHandDof> hand_pub_pos_{};
  std::array<double, kHandDof> hand_pub_vel_{};
  std::array<double, kHandDof> hand_pub_tau_{};
  std::array<double, kHandDof> hand_des_pos_{};
  std::array<double, kHandDof> hand_pub_des_pos_{};
  std::array<double, kHandDof> hand_cmd_{};
  double hand_control_accumulator_ = 0.0;
  bool hand_initialized_ = false;
};

}  // namespace daru_mj
