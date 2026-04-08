#include "daru_mujoco/teleop/daru_v4_mujoco_teleop_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
constexpr double kPi = 3.14159265358979323846;
const std::array<const char*, daru_mj::DaruV4MuJoCoController::kDof> kJointNames = {
    "Waist_Joint", "LSP_Joint", "LSR_Joint", "LSY_Joint", "LEP_Joint", "LEY_Joint", "LWR_Joint", "LWP_Joint",
    "RSP_Joint", "RSR_Joint", "RSY_Joint", "REP_Joint", "REY_Joint", "RWR_Joint", "RWP_Joint"};

const std::array<const char*, 2> kHeadJointNames = {"HY_Joint", "HP_Joint"};
const std::array<const char*, 12> kHandJointNames = {
    "rh_f1_Motor_Linear_Joint",  "rh_f2_Motor_Linear_Joint",  "rh_f3_Motor_Linear_Joint",
    "rh_f4_Motor_Linear_Joint",  "rh_c_t_Motor_Linear_Joint", "rh_c_Motor_Linear_Joint",
    "lh_f1_Motor_Linear_Joint",  "lh_f2_Motor_Linear_Joint",  "lh_f3_Motor_Linear_Joint",
    "lh_f4_Motor_Linear_Joint",  "lh_c_t_Motor_Linear_Joint", "lh_c_Motor_Linear_Joint"};

const std::array<double, daru_mj::DaruV4MuJoCoController::kDof> kWorkReadyJointPose = {
    0.0, 0.0, 0.0, 0.0, -kPi / 2.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -kPi / 2.0, 0.0, 0.0, 0.0};

const std::array<double, 2> kWorkReadyHeadPose = {0.0, 0.0};

const RigidBodyDynamics::Math::Vector3d kDefaultLeftEePos(0.2930, 0.225, 0.0725);
const RigidBodyDynamics::Math::Vector3d kDefaultRightEePos(0.2930, -0.225, 0.0725);
const Eigen::Quaterniond kDefaultLeftEeQuat(0.707, 0.0, -0.707, 0.0);
const Eigen::Quaterniond kDefaultRightEeQuat(0.707, 0.0, -0.707, 0.0);
const Eigen::Quaterniond kDefaultHeadQuat = Eigen::Quaterniond::Identity();
constexpr double kMode2HomeBlendDuration = 1.0;

const std::array<const char*, 12> kHandForceSensorNames = {
    "rh_f1_force",  "rh_f2_force",  "rh_f3_force",  "rh_f4_force",  "rh_f5_force",  "rh_f6_force",
    "lh_f1_force",  "lh_f2_force",  "lh_f3_force",  "lh_f4_force",  "lh_f5_force",  "lh_f6_force"};

constexpr int kHandCtrlNone = 0;
constexpr int kHandCtrlForce = 1;
constexpr int kHandCtrlDirect = 2;

struct JointActuatorMatch {
  int force_actuator = -1;
  int first_joint_actuator = -1;
};

const char* SafeName(const mjModel* m, mjtObj objtype, int id) {
  const char* name = mj_id2name(m, objtype, id);
  return name ? name : "<unnamed>";
}

bool IsJointTransmissionActuator(const mjModel* m, int actuator_id) {
  const int trn_type = m->actuator_trntype[actuator_id];
  return trn_type == mjTRN_JOINT || trn_type == mjTRN_JOINTINPARENT;
}

bool IsForceControlActuator(const mjModel* m, int actuator_id) {
  return IsJointTransmissionActuator(m, actuator_id) && m->actuator_dyntype[actuator_id] == mjDYN_NONE &&
         m->actuator_gaintype[actuator_id] == mjGAIN_FIXED && m->actuator_biastype[actuator_id] == mjBIAS_NONE;
}

const char* DynTypeName(int dyn_type) {
  switch (dyn_type) {
    case mjDYN_NONE:
      return "none";
    case mjDYN_INTEGRATOR:
      return "integrator";
    case mjDYN_FILTER:
      return "filter";
    case mjDYN_FILTEREXACT:
      return "filterexact";
    case mjDYN_MUSCLE:
      return "muscle";
    case mjDYN_USER:
      return "user";
    default:
      return "unknown";
  }
}

const char* GainTypeName(int gain_type) {
  switch (gain_type) {
    case mjGAIN_FIXED:
      return "fixed";
    case mjGAIN_AFFINE:
      return "affine";
    case mjGAIN_MUSCLE:
      return "muscle";
    case mjGAIN_USER:
      return "user";
    default:
      return "unknown";
  }
}

const char* BiasTypeName(int bias_type) {
  switch (bias_type) {
    case mjBIAS_NONE:
      return "none";
    case mjBIAS_AFFINE:
      return "affine";
    case mjBIAS_MUSCLE:
      return "muscle";
    case mjBIAS_USER:
      return "user";
    default:
      return "unknown";
  }
}

JointActuatorMatch FindActuatorForJoint(const mjModel* m, int joint_id) {
  JointActuatorMatch match;
  for (int i = 0; i < m->nu; ++i) {
    if (!IsJointTransmissionActuator(m, i) || m->actuator_trnid[2 * i] != joint_id) {
      continue;
    }
    if (match.first_joint_actuator < 0) {
      match.first_joint_actuator = i;
    }
    if (IsForceControlActuator(m, i)) {
      match.force_actuator = i;
      break;
    }
  }
  return match;
}

double ClampActuatorTorqueCommand(const mjModel* m, int actuator_id, double fallback_limit, double command) {
  double lower = -fallback_limit;
  double upper = fallback_limit;
  if (actuator_id >= 0 && m->actuator_ctrllimited[actuator_id]) {
    lower = m->actuator_ctrlrange[2 * actuator_id + 0];
    upper = m->actuator_ctrlrange[2 * actuator_id + 1];
  }
  return std::clamp(command, lower, upper);
}

double ClampHandPositionCommand(const mjModel* m, int actuator_id, int joint_id, double command) {
  double lower = command;
  double upper = command;
  bool limited = false;
  if (actuator_id >= 0 && m->actuator_ctrllimited[actuator_id]) {
    lower = m->actuator_ctrlrange[2 * actuator_id + 0];
    upper = m->actuator_ctrlrange[2 * actuator_id + 1];
    limited = true;
  }
  if (!limited && joint_id >= 0 && m->jnt_limited[joint_id]) {
    lower = m->jnt_range[2 * joint_id + 0];
    upper = m->jnt_range[2 * joint_id + 1];
    limited = true;
  }
  return limited ? std::clamp(command, lower, upper) : command;
}

}  // namespace

namespace daru_mj {

DaruV4MuJoCoController::DaruV4MuJoCoController(mjModel* model, mjData* data, const std::string& urdf_path)
    : m_(model), d_(data), daru_(urdf_path.c_str()) {
  if (!m_ || !d_) {
    throw std::runtime_error("MuJoCo model/data is null");
  }

  l_pos_des_ = kDefaultLeftEePos;
  r_pos_des_ = kDefaultRightEePos;
  l_quat_des_ = kDefaultLeftEeQuat;
  r_quat_des_ = kDefaultRightEeQuat;
  head_quat_des_ = kDefaultHeadQuat;

  map_joint_.fill(-1);
  map_qpos_.fill(-1);
  map_dof_.fill(-1);
  map_actuator_.fill(-1);
  head_map_joint_.fill(-1);
  head_map_dof_.fill(-1);
  head_map_actuator_.fill(-1);
  hand_map_joint_.fill(-1);
  hand_map_qpos_.fill(-1);
  hand_map_dof_.fill(-1);
  hand_map_actuator_.fill(-1);
  hand_force_sensor_.fill(-1);
  hand_force_sensor_adr_.fill(-1);
  hand_ctrl_mode_.fill(kHandCtrlNone);

  for (int i = 0; i < kDof; ++i) {
    const int jid = mj_name2id(m_, mjOBJ_JOINT, kJointNames[i]);
    if (jid < 0) {
      throw std::runtime_error(std::string("Joint not found in MJCF: ") + kJointNames[i]);
    }
    map_joint_[i] = jid;
    map_qpos_[i] = m_->jnt_qposadr[jid];
    map_dof_[i] = m_->jnt_dofadr[jid];

    const JointActuatorMatch actuator_match = FindActuatorForJoint(m_, jid);
    map_actuator_[i] = actuator_match.force_actuator;

    if (actuator_match.force_actuator >= 0) {
      std::cout << "[DARU_MJ] joint '" << kJointNames[i] << "' -> actuator '"
                << SafeName(m_, mjOBJ_ACTUATOR, actuator_match.force_actuator) << "' (force control)" << std::endl;
      continue;
    }

    if (actuator_match.first_joint_actuator >= 0) {
      const int aid = actuator_match.first_joint_actuator;
      std::ostringstream oss;
      oss << "Joint '" << kJointNames[i] << "' is connected to actuator '"
          << SafeName(m_, mjOBJ_ACTUATOR, aid) << "', but that actuator is not force-controlled "
          << "(dyn=" << DynTypeName(m_->actuator_dyntype[aid]) << ", gain=" << GainTypeName(m_->actuator_gaintype[aid])
          << ", bias=" << BiasTypeName(m_->actuator_biastype[aid])
          << "). Use a <motor> or equivalent force actuator for torque control.";
      throw std::runtime_error(oss.str());
    }

    std::cout << "[DARU_MJ] joint '" << kJointNames[i] << "' has no actuator, using qfrc_applied fallback"
              << std::endl;
  }

  for (int i = 0; i < kHeadDof; ++i) {
    const int jid = mj_name2id(m_, mjOBJ_JOINT, kHeadJointNames[i]);
    if (jid < 0) {
      continue;
    }

    head_map_joint_[i] = jid;
    head_map_dof_[i] = m_->jnt_dofadr[jid];

    const JointActuatorMatch actuator_match = FindActuatorForJoint(m_, jid);
    head_map_actuator_[i] = actuator_match.force_actuator;

    if (actuator_match.force_actuator >= 0) {
      std::cout << "[DARU_MJ] head joint '" << kHeadJointNames[i] << "' -> actuator '"
                << SafeName(m_, mjOBJ_ACTUATOR, actuator_match.force_actuator) << "' (head PD)" << std::endl;
      continue;
    }

    if (actuator_match.first_joint_actuator >= 0) {
      const int aid = actuator_match.first_joint_actuator;
      std::ostringstream oss;
      oss << "Head joint '" << kHeadJointNames[i] << "' is connected to actuator '"
          << SafeName(m_, mjOBJ_ACTUATOR, aid) << "', but that actuator is not force-controlled "
          << "(dyn=" << DynTypeName(m_->actuator_dyntype[aid]) << ", gain=" << GainTypeName(m_->actuator_gaintype[aid])
          << ", bias=" << BiasTypeName(m_->actuator_biastype[aid])
          << "). Use a <motor> or equivalent force actuator for head PD torque control.";
      throw std::runtime_error(oss.str());
    }

    std::cout << "[DARU_MJ] head joint '" << kHeadJointNames[i] << "' has no actuator, using qfrc_applied fallback"
              << std::endl;
  }

  for (int i = 0; i < kHandDof; ++i) {
    const int jid = mj_name2id(m_, mjOBJ_JOINT, kHandJointNames[i]);
    if (jid < 0) {
      continue;
    }

    hand_map_joint_[i] = jid;
    hand_map_qpos_[i] = m_->jnt_qposadr[jid];
    hand_map_dof_[i] = m_->jnt_dofadr[jid];
    hand_force_sensor_[i] = mj_name2id(m_, mjOBJ_SENSOR, kHandForceSensorNames[i]);
    if (hand_force_sensor_[i] >= 0) {
      hand_force_sensor_adr_[i] = m_->sensor_adr[hand_force_sensor_[i]];
      std::cout << "[DARU_MJ] hand joint '" << kHandJointNames[i] << "' -> sensor '" << kHandForceSensorNames[i]
                << "'" << std::endl;
    }

    const JointActuatorMatch actuator_match = FindActuatorForJoint(m_, jid);
    if (actuator_match.force_actuator >= 0) {
      hand_map_actuator_[i] = actuator_match.force_actuator;
      hand_ctrl_mode_[i] = kHandCtrlForce;
      std::cout << "[DARU_MJ] hand joint '" << kHandJointNames[i] << "' -> actuator '"
                << SafeName(m_, mjOBJ_ACTUATOR, actuator_match.force_actuator) << "' (20ms hand force PD)"
                << std::endl;
      continue;
    }

    if (actuator_match.first_joint_actuator >= 0) {
      hand_map_actuator_[i] = actuator_match.first_joint_actuator;
      hand_ctrl_mode_[i] = kHandCtrlDirect;
      std::cout << "[DARU_MJ] hand joint '" << kHandJointNames[i] << "' -> actuator '"
                << SafeName(m_, mjOBJ_ACTUATOR, actuator_match.first_joint_actuator) << "' (20ms hand direct ctrl)"
                << std::endl;
      continue;
    }

    std::cout << "[DARU_MJ] hand joint '" << kHandJointNames[i]
              << "' has no actuator, using qfrc_applied fallback for 20ms hand control" << std::endl;
    hand_ctrl_mode_[i] = kHandCtrlForce;
  }
}

void DaruV4MuJoCoController::Reset() { ResetInternal(1); }

void DaruV4MuJoCoController::ResetToMode2() { ResetInternal(2); }

void DaruV4MuJoCoController::ResetInternal(int start_mode) {
  world_cnt_ = 0;
  cnt_ = 0;
  mode_ = 0;
  reset_start_mode_ = std::clamp(start_mode, 1, 3);
  initialized_ = false;
  last_update_time_ = 0.0;
  dt_ = m_->opt.timestep;

  head_initialized_ = false;
  hand_initialized_ = false;
  hand_control_accumulator_ = 0.0;

  for (int i = 0; i < kDof; ++i) {
    joints_[i] = JOINT_{};
    upper_joint_[i] = JOINT_{};
    pos_init_[i] = 0.0;
    des_pos_[i] = 0.0;
    des_torque_[i] = 0.0;
    pd_des_current_[i] = 0.0;
  }

  head_pos_.fill(0.0);
  head_vel_.fill(0.0);
  head_tau_.fill(0.0);
  head_pos_init_.fill(0.0);
  head_des_pos_.fill(0.0);
  head_cmd_tau_.fill(0.0);
  hand_pos_.fill(0.0);
  hand_vel_.fill(0.0);
  hand_tau_.fill(0.0);
  hand_pub_pos_.fill(0.0);
  hand_pub_vel_.fill(0.0);
  hand_pub_tau_.fill(0.0);
  hand_des_pos_.fill(0.0);
  hand_pub_des_pos_.fill(0.0);
  hand_cmd_.fill(0.0);

  ref_pos_.setZero();
  l_pos_act_.setZero();
  r_pos_act_.setZero();
  l_rot_act_.setIdentity();
  r_rot_act_.setIdentity();
  mode2_home_blend_active_ = false;
  mode2_home_blend_elapsed_ = 0.0;
  l_pos_des_ = kDefaultLeftEePos;
  r_pos_des_ = kDefaultRightEePos;
  l_quat_des_ = kDefaultLeftEeQuat;
  r_quat_des_ = kDefaultRightEeQuat;
  head_quat_des_ = kDefaultHeadQuat;
}

void DaruV4MuJoCoController::StartMode2HomeBlend() {
  mode2_home_blend_active_ = true;
  mode2_home_blend_elapsed_ = 0.0;
}

void DaruV4MuJoCoController::ApplyMode2HomeBlend() {
  if (!mode2_home_blend_active_) {
    return;
  }

  const double clamped_time = std::min(mode2_home_blend_elapsed_, kMode2HomeBlendDuration);
  const double alpha = 0.5 * (1.0 - std::cos(kPi * clamped_time / kMode2HomeBlendDuration));

  for (int i = 0; i < kDof; ++i) {
    const double blended = alpha * joints_[i].R_pos + (1.0 - alpha) * kWorkReadyJointPose[i];
    des_pos_[i] = blended;
    joints_[i].R_pos = blended;
    ref_pos_(i) = blended;
  }

  mode2_home_blend_elapsed_ = std::min(mode2_home_blend_elapsed_ + dt_, kMode2HomeBlendDuration);
  if (mode2_home_blend_elapsed_ >= kMode2HomeBlendDuration) {
    mode2_home_blend_active_ = false;
  }
}

void DaruV4MuJoCoController::ApplyWorkReadyPose() {
  for (int i = 0; i < kDof; ++i) {
    const int qadr = map_qpos_[i];
    const int dadr = map_dof_[i];
    if (qadr >= 0) {
      d_->qpos[qadr] = kWorkReadyJointPose[i];
    }
    if (dadr >= 0) {
      d_->qvel[dadr] = 0.0;
    }
  }

  for (int i = 0; i < kHeadDof; ++i) {
    const int jid = head_map_joint_[i];
    const int did = head_map_dof_[i];
    if (jid >= 0) {
      d_->qpos[m_->jnt_qposadr[jid]] = kWorkReadyHeadPose[i];
    }
    if (did >= 0) {
      d_->qvel[did] = 0.0;
    }
  }

  mju_zero(d_->qacc_warmstart, m_->nv);
  mju_zero(d_->qacc, m_->nv);
  mju_zero(d_->qfrc_applied, m_->nv);
  mju_zero(d_->xfrc_applied, 6 * m_->nbody);
  if (m_->nu > 0) {
    mju_zero(d_->ctrl, m_->nu);
  }
  if (m_->na > 0) {
    mju_zero(d_->act, m_->na);
  }
}

void DaruV4MuJoCoController::SetLeftEeTarget(const RigidBodyDynamics::Math::Vector3d& pos,
                                              const Eigen::Quaterniond& quat) {
  const std::lock_guard<std::mutex> lock(target_mutex_);
  l_pos_des_ = pos;
  l_quat_des_ = quat.normalized();
}

void DaruV4MuJoCoController::SetRightEeTarget(const RigidBodyDynamics::Math::Vector3d& pos,
                                               const Eigen::Quaterniond& quat) {
  const std::lock_guard<std::mutex> lock(target_mutex_);
  r_pos_des_ = pos;
  r_quat_des_ = quat.normalized();
}

void DaruV4MuJoCoController::SetHeadTarget(const Eigen::Quaterniond& quat) 
{
  const std::lock_guard<std::mutex> lock(target_mutex_);
  head_quat_des_ = quat.normalized();
}

void DaruV4MuJoCoController::SetLeftEePositionTarget(const RigidBodyDynamics::Math::Vector3d& pos) {
  const std::lock_guard<std::mutex> lock(target_mutex_);
  l_pos_des_ = pos;
}

void DaruV4MuJoCoController::SetLeftHandLinearTargets(const std::array<double, 6>& targets) {
  const std::lock_guard<std::mutex> lock(hand_target_mutex_);
  for (int i = 0; i < kHandPerSide; ++i) {
    const int idx = kHandPerSide + i;
    if (hand_map_joint_[idx] < 0) {
      continue;
    }
    hand_des_pos_[idx] = targets[i];
  }
}

void DaruV4MuJoCoController::SetRightHandLinearTargets(const std::array<double, 6>& targets) {
  const std::lock_guard<std::mutex> lock(hand_target_mutex_);
  for (int i = 0; i < kHandPerSide; ++i) {
    if (hand_map_joint_[i] < 0) {
      continue;
    }
    hand_des_pos_[i] = targets[i];
  }
}

void DaruV4MuJoCoController::InitializeHomeModeIfNeeded() {
  if (initialized_) return;
  StateUpdate();
  GetCurrentState();
  for (int i = 0; i < kDof; ++i) {
    des_pos_[i] = pos_init_[i];
    ref_pos_(i) = pos_init_[i];
  }
  for (int i = 0; i < kHeadDof; ++i) {
    if (head_map_joint_[i] < 0) {
      continue;
    }
    head_pos_init_[i] = head_pos_[i];
    head_des_pos_[i] = head_pos_[i];
  }
  {
    const std::lock_guard<std::mutex> lock(hand_target_mutex_);
    for (int i = 0; i < kHandDof; ++i) {
      if (hand_map_joint_[i] < 0) {
        continue;
      }
      hand_des_pos_[i] = hand_pos_[i];
    }
  }
  head_initialized_ = true;
  hand_initialized_ = true;
  mode_ = reset_start_mode_;
  cnt_ = 0;
  if (mode_ == 2) {
    StartMode2HomeBlend();
  }
  initialized_ = true;
  std::cout << "[DARU_MJ] start in MODE " << mode_;
  if (mode_ == 1) {
    std::cout << " (home pose PD)";
  } else if (mode_ == 2) {
    std::cout << " (direct control)";
  }
  std::cout << std::endl;
}

void DaruV4MuJoCoController::Step() {
  InitializeHomeModeIfNeeded();
  const double current_time = d_->time;
  dt_ = current_time - last_update_time_;
  if (dt_ <= 1e-9) {
    dt_ = m_->opt.timestep;
  }
  hand_control_accumulator_ += dt_;

  StateUpdate();
  RefTorqueInit();

  if (mode_ == 1) {
    RbdlStateUpdate();
    SetZeroPos();
    CGCompensation(1.2);
    ++cnt_;
    if (cnt_ > 6000) {
      mode_ = 2;
      cnt_ = 0;
      GetCurrentState();
      StartMode2HomeBlend();
    }
  } else if (mode_ == 2 || mode_ == 3) {
    RbdlStateUpdate();
    RigidBodyDynamics::Math::Vector3d l_pos_des;
    RigidBodyDynamics::Math::Vector3d r_pos_des;
    Eigen::Quaterniond l_quat_des;
    Eigen::Quaterniond r_quat_des;

    {
      const std::lock_guard<std::mutex> lock(target_mutex_);
      l_pos_des = l_pos_des_;
      r_pos_des = r_pos_des_;
      l_quat_des = l_quat_des_;
      r_quat_des = r_quat_des_;
    }

    for (int i = 0; i < kDof; ++i) {
      ref_pos_(i) = des_pos_[i];
    }

    Eigen::Matrix3d R = head_quat_des_.toRotationMatrix();
    head_des_pos_[1] =  std::asin(R(2,0));
    head_des_pos_[0] = -std::atan2(R(1,0), R(0,0)) - head_pos_[0];

    head_des_pos_[0] = std::clamp(head_des_pos_[0], -kPi/2, kPi/2);
    head_des_pos_[1] = std::clamp(head_des_pos_[1], -kPi*19/180, kPi/4);

    IKSolver(l_pos_des, r_pos_des, l_quat_des, r_quat_des);
    if (mode_ == 2) {
      ApplyMode2HomeBlend();
    }
    CGCompensation(1.2);
    
    ++cnt_;
  }

  if (mode_ == 1 || mode_ == 2 || mode_ == 3) {
    JointPdControl(true);
    JointTorqueControl(false);
    HeadPdControl();
    HandControlTick();
  }

  ApplyArmTorque();
  ++world_cnt_;
  last_update_time_ = current_time;
}

void DaruV4MuJoCoController::StateUpdate() {
  for (int i = 0; i < kDof; ++i) {
    const int qadr = map_qpos_[i];
    const int dadr = map_dof_[i];
    joints_[i].A_pos = d_->qpos[qadr];
    joints_[i].A_vel = d_->qvel[dadr];
    joints_[i].A_vel_lpf = 0.05 * joints_[i].A_vel_lpf + 0.95 * joints_[i].A_vel;
    joints_[i].A_acc = (joints_[i].A_vel - joints_[i].A_vel_old) / dt_;
    joints_[i].A_tau = d_->qfrc_actuator[dadr] + d_->qfrc_applied[dadr];
    joints_[i].A_pos_old = joints_[i].A_pos;
    joints_[i].A_vel_old = joints_[i].A_vel;
  }

  for (int i = 0; i < kHeadDof; ++i) {
    const int jid = head_map_joint_[i];
    const int did = head_map_dof_[i];
    if (jid < 0 || did < 0) {
      continue;
    }
    head_pos_[i] = d_->qpos[m_->jnt_qposadr[jid]];
    head_vel_[i] = d_->qvel[did];
    head_tau_[i] = d_->qfrc_actuator[did] + d_->qfrc_applied[did];
  }

  for (int i = 0; i < kHandDof; ++i) {
    const int qadr = hand_map_qpos_[i];
    const int did = hand_map_dof_[i];
    if (qadr < 0 || did < 0) {
      continue;
    }
    hand_pos_[i] = d_->qpos[qadr];
    hand_vel_[i] = d_->qvel[did];
    hand_tau_[i] = d_->qfrc_actuator[did];
  }

}

void DaruV4MuJoCoController::ApplyArmTorque() {
  mju_zero(d_->qfrc_applied, m_->nv);
  for (int i = 0; i < kDof; ++i) {
    const int aid = map_actuator_[i];
    const int did = map_dof_[i];
    if (aid >= 0) {
      d_->ctrl[aid] = joints_[i].R_tau;
    } else {
      d_->qfrc_applied[did] = joints_[i].R_tau;
    }
  }

  for (int i = 0; i < kHeadDof; ++i) {
    const int did = head_map_dof_[i];
    if (did < 0) {
      continue;
    }
    const int aid = head_map_actuator_[i];
    if (aid >= 0) {
      d_->ctrl[aid] = head_cmd_tau_[i];
    } else {
      d_->qfrc_applied[did] = head_cmd_tau_[i];
    }
  }

  for (int i = 0; i < kHandDof; ++i) {
    const int did = hand_map_dof_[i];
    if (did < 0) {
      continue;
    }
    const int aid = hand_map_actuator_[i];
    if (aid >= 0) {
      d_->ctrl[aid] = hand_cmd_[i];
    } else if (hand_ctrl_mode_[i] == kHandCtrlForce) {
      d_->qfrc_applied[did] += hand_cmd_[i];
    }
  }
}

void DaruV4MuJoCoController::GetCurrentState() {
  for (int n = 0; n < kDof; ++n) {
    pos_init_[n] = joints_[n].A_pos;
    joints_[n].R_pos = pos_init_[n];
  }
}

std::array<double, 12> DaruV4MuJoCoController::GetFingerForceState()
{
  std::array<double, 12> out{};
  for (int i = 0; i < 12; ++i) {
    if (hand_force_sensor_adr_[i] >= 0) {
      out[i] = d_->sensordata[hand_force_sensor_adr_[i]];
    } else {
      out[i] = hand_tau_[i];
    }
  }
  return out; 
}

void DaruV4MuJoCoController::SetZeroPos() {
  auto cosine_blend = [](double start, double goal, unsigned long long count, double duration) {
    return start + (goal - start) * 0.5 * (1.0 - std::cos(kPi * static_cast<double>(count) / duration));
  };

  constexpr int kWaist = 0;
  constexpr int kLsp = 1;
  constexpr int kLsr = 2;
  constexpr int kLsy = 3;
  constexpr int kLep = 4;
  constexpr int kLey = 5;
  constexpr int kLwr = 6;
  constexpr int kLwp = 7;
  constexpr int kRsp = 8;
  constexpr int kRsr = 9;
  constexpr int kRsy = 10;
  constexpr int kRep = 11;
  constexpr int kRey = 12;
  constexpr int kRwr = 13;
  constexpr int kRwp = 14;

  for (int n = 0; n < kDof; ++n) {
    des_pos_[n] = pos_init_[n];
  }
  for (int i = 0; i < kHeadDof; ++i) {
    head_des_pos_[i] = head_pos_init_[i];
  }

  if (cnt_ < 1000) {
    des_pos_[kLsr] = cosine_blend(pos_init_[kLsr], kPi / 6.0, cnt_, 1000.0);
    des_pos_[kRsr] = cosine_blend(pos_init_[kRsr], -kPi / 6.0, cnt_, 1000.0);
    des_pos_[kWaist] = cosine_blend(pos_init_[kWaist], 0.0, cnt_, 1000.0);
    head_des_pos_[0] = cosine_blend(head_pos_init_[0], 0.0, cnt_, 1000.0);
    head_des_pos_[1] = cosine_blend(head_pos_init_[1], 0.0, cnt_, 1000.0);
  } else {
    des_pos_[kLsr] = kPi / 6.0;
    des_pos_[kRsr] = -kPi / 6.0;
    des_pos_[kWaist] = 0.0;
    head_des_pos_[0] = 0.0;
    head_des_pos_[1] = 0.0;
  }

  if (cnt_ >= 1000 && cnt_ < 2000) {
    const auto local_cnt = cnt_ - 1000;
    des_pos_[kLsp] = cosine_blend(pos_init_[kLsp], 0.0, local_cnt, 1000.0);
    des_pos_[kLsy] = cosine_blend(pos_init_[kLsy], 0.0, local_cnt, 1000.0);
    des_pos_[kLep] = cosine_blend(pos_init_[kLep], 0.0, local_cnt, 1000.0);
    des_pos_[kRsp] = cosine_blend(pos_init_[kRsp], 0.0, local_cnt, 1000.0);
    des_pos_[kRsy] = cosine_blend(pos_init_[kRsy], 0.0, local_cnt, 1000.0);
    des_pos_[kRep] = cosine_blend(pos_init_[kRep], 0.0, local_cnt, 1000.0);
    des_pos_[kLey] = cosine_blend(pos_init_[kLey], 0.0, local_cnt, 1000.0);
    des_pos_[kLwr] = cosine_blend(pos_init_[kLwr], 0.0, local_cnt, 1000.0);
    des_pos_[kLwp] = cosine_blend(pos_init_[kLwp], 0.0, local_cnt, 1000.0);
    des_pos_[kRey] = cosine_blend(pos_init_[kRey], 0.0, local_cnt, 1000.0);
    des_pos_[kRwr] = cosine_blend(pos_init_[kRwr], 0.0, local_cnt, 1000.0);
    des_pos_[kRwp] = cosine_blend(pos_init_[kRwp], 0.0, local_cnt, 1000.0);
  } else if (cnt_ >= 2000) {
    des_pos_[kLsp] = 0.0;
    des_pos_[kLsy] = 0.0;
    des_pos_[kLep] = 0.0;
    des_pos_[kRsp] = 0.0;
    des_pos_[kRsy] = 0.0;
    des_pos_[kRep] = 0.0;
    des_pos_[kLey] = 0.0;
    des_pos_[kLwr] = 0.0;
    des_pos_[kLwp] = 0.0;
    des_pos_[kRey] = 0.0;
    des_pos_[kRwr] = 0.0;
    des_pos_[kRwp] = 0.0;
  }

  if (cnt_ >= 2000 && cnt_ < 4000) {
    const auto local_cnt = cnt_ - 2000;
    des_pos_[kLsp] = cosine_blend(0.0, kPi * 3.0 / 8.0, local_cnt, 2000.0);
    des_pos_[kRsp] = cosine_blend(0.0, kPi * 3.0 / 8.0, local_cnt, 2000.0);
    des_pos_[kLsr] = cosine_blend(kPi / 6.0, 0.0, local_cnt, 2000.0);
    des_pos_[kRsr] = cosine_blend(-kPi / 6.0, 0.0, local_cnt, 2000.0);
    des_pos_[kLep] = cosine_blend(0.0, -kPi * 3.0 / 4.0, local_cnt, 2000.0);
    des_pos_[kRep] = cosine_blend(0.0, -kPi * 3.0 / 4.0, local_cnt, 2000.0);
  } else if (cnt_ >= 4000) {
    des_pos_[kLsp] = kPi * 3.0 / 8.0;
    des_pos_[kRsp] = kPi * 3.0 / 8.0;
    des_pos_[kLsr] = 0.0;
    des_pos_[kRsr] = 0.0;
    des_pos_[kLep] = -kPi * 3.0 / 4.0;
    des_pos_[kRep] = -kPi * 3.0 / 4.0;
  }

  if (cnt_ >= 4000 && cnt_ < 6000) {
    const auto local_cnt = cnt_ - 4000;
    des_pos_[kLsp] = cosine_blend(kPi * 3.0 / 8.0, 0.0, local_cnt, 2000.0);
    des_pos_[kRsp] = cosine_blend(kPi * 3.0 / 8.0, 0.0, local_cnt, 2000.0);
    des_pos_[kLep] = cosine_blend(-kPi * 3.0 / 4.0, -kPi / 2.0, local_cnt, 2000.0);
    des_pos_[kRep] = cosine_blend(-kPi * 3.0 / 4.0, -kPi / 2.0, local_cnt, 2000.0);
  } else if (cnt_ >= 6000) {
    des_pos_[kLsp] = 0.0;
    des_pos_[kRsp] = 0.0;
    des_pos_[kLep] = -kPi / 2.0;
    des_pos_[kRep] = -kPi / 2.0;
  }

  for (int n = 0; n < kDof; ++n) {
    joints_[n].R_pos = des_pos_[n];
  }
}

void DaruV4MuJoCoController::RbdlStateUpdate() {
  for (int i = 0; i < kDof; ++i) {
    upper_joint_[i].A_pos = joints_[i].A_pos;
    upper_joint_[i].A_vel = joints_[i].A_vel;
    upper_joint_[i].A_acc = upper_joint_[i].A_vel - upper_joint_[i].A_vel_old / 0.002;
    upper_joint_[i].A_tau = joints_[i].A_tau;
  }
  for (unsigned int i = 0; i < daru_.Dof(); ++i) {
    upper_joint_[i].A_pos_old = upper_joint_[i].A_pos;
    upper_joint_[i].A_vel_old = upper_joint_[i].A_vel;
  }
  daru_.updateQ(upper_joint_.data());
}

void DaruV4MuJoCoController::IKSolver(RigidBodyDynamics::Math::Vector3d l_pos_des,
                                      RigidBodyDynamics::Math::Vector3d r_pos_des,
                                      Eigen::Quaterniond l_quat_des,
                                      Eigen::Quaterniond r_quat_des) {
  daru_.calEEPosq(l_pos_act_, l_rot_act_, r_pos_act_, r_rot_act_, ref_pos_);

  RigidBodyDynamics::Math::Vector3d shoulder_L(-0.225*sin(ref_pos_[0]),  0.225*cos(ref_pos_[0]), 0.3725); // 왼팔 어깨 좌표
  RigidBodyDynamics::Math::Vector3d shoulder_R(0.225*sin(ref_pos_[0]),  -0.225*cos(ref_pos_[0]), 0.3725); // 오른팔 어깨 좌표

    // 최대 팔 길이 (링크 길이 합보다 살짝 작게 잡으면 안정적)
    double ARM_MAX_LEN = 0.53; // [m] 예시

    // 왼팔 목표 제한
    RigidBodyDynamics::Math::Vector3d vecL = l_pos_des - shoulder_L;
    double dL = vecL.norm();
    if (dL > ARM_MAX_LEN) {
        l_pos_des = shoulder_L + vecL.normalized() * ARM_MAX_LEN;
    }

    // 오른팔 목표 제한
    RigidBodyDynamics::Math::Vector3d vecR = r_pos_des - shoulder_R;
    double dR = vecR.norm();
    if (dR > ARM_MAX_LEN) {
        r_pos_des = shoulder_R + vecR.normalized() * ARM_MAX_LEN;
    }

  l_quat_des = Eigen::Quaterniond(-l_quat_des.w(), l_quat_des.x(), l_quat_des.y(), l_quat_des.z());
  r_quat_des = Eigen::Quaterniond(-r_quat_des.w(), r_quat_des.x(), r_quat_des.y(), r_quat_des.z());
  l_quat_des.normalize();
  r_quat_des.normalize();
  const auto l_rot_des = l_quat_des.toRotationMatrix();
  const auto r_rot_des = r_quat_des.toRotationMatrix();

  const auto l_pos_e = l_pos_des - l_pos_act_;
  const auto r_pos_e = r_pos_des - r_pos_act_;
  RigidBodyDynamics::Math::Matrix3d l_rot_e = l_rot_des.transpose() * l_rot_act_;
  RigidBodyDynamics::Math::Matrix3d r_rot_e = r_rot_des.transpose() * r_rot_act_;

  daru_.Log(l_rot_e, phi_l_);
  daru_.Log(r_rot_e, phi_r_);

  RigidBodyDynamics::Math::VectorNd x = RigidBodyDynamics::Math::VectorNd::Zero(12);
  RigidBodyDynamics::Math::VectorNd x_null = RigidBodyDynamics::Math::VectorNd::Zero(12);
  RigidBodyDynamics::Math::VectorNd q = RigidBodyDynamics::Math::VectorNd::Zero(15);
  RigidBodyDynamics::Math::VectorNd q_null = RigidBodyDynamics::Math::VectorNd::Zero(15);

  x << 0 * phi_l_, 1 * l_pos_e, 0 * phi_r_, 1 * r_pos_e;
  x_null << 1 * phi_l_, 0 * l_pos_e, 1 * phi_r_, 0 * r_pos_e;

  for (int i = 0; i < 12; ++i) {
    if (i == 3 || i == 4 || i == 5 || i == 9 || i == 10 || i == 11) {
      x[i] = std::clamp(x[i], -0.01, 0.01);
      x_null[i] = std::clamp(x_null[i], -0.003, 0.003);
    } else {
      x[i] = std::clamp(x[i], -kPi * 0.005, kPi * 0.005);
      x_null[i] = std::clamp(x_null[i], -kPi * 0.005, kPi * 0.005);
    }
  }

  RigidBodyDynamics::Math::MatrixNd n = RigidBodyDynamics::Math::MatrixNd::Zero(15, 15);
  daru_.calJqWaist(ref_pos_);
  daru_.calIK(x, q);
  daru_.calIK(x_null, q_null);
  daru_.calJqWaistPos(ref_pos_);
  daru_.calNullSpace(n);

  q_null(0) += 0.0 * (0 - ref_pos_(0));
  q_null(2) += 0.05 * (kPi / 12 - ref_pos_(2));
  q_null(9) += 0.05 * (-kPi / 12 - ref_pos_(9));

  q = q + n * q_null;
  ref_pos_ += q;

  ref_pos_[0] = std::clamp(ref_pos_[0], -kPi * 90 / 180, kPi * 90 / 180);
  ref_pos_[1] = std::clamp(ref_pos_[1], -kPi * 168 / 180, kPi * 168 / 180);
  ref_pos_[2] = std::clamp(ref_pos_[2], -kPi / 9, kPi * 10 / 9);
  ref_pos_[3] = std::clamp(ref_pos_[3], -kPi * 165 / 180, kPi * 165 / 180);
  ref_pos_[4] = std::clamp(ref_pos_[4], -kPi * 3 / 4, -0.001);
  ref_pos_[5] = std::clamp(ref_pos_[5], -kPi / 2, kPi / 2);
  ref_pos_[6] = std::clamp(ref_pos_[6], -kPi / 3, kPi / 3);
  ref_pos_[7] = std::clamp(ref_pos_[7], -kPi / 3, kPi / 3);

  ref_pos_[8] = std::clamp(ref_pos_[8], -kPi * 168 / 180, kPi * 168 / 180);
  ref_pos_[9] = std::clamp(ref_pos_[9], -kPi * 10 / 9, kPi / 9);
  ref_pos_[10] = std::clamp(ref_pos_[10], -kPi * 165 / 180, kPi * 165 / 180);
  ref_pos_[11] = std::clamp(ref_pos_[11], -kPi * 3 / 4, -0.001);
  ref_pos_[12] = std::clamp(ref_pos_[12], -kPi / 2, kPi / 2);
  ref_pos_[13] = std::clamp(ref_pos_[13], -kPi / 3, kPi / 3);
  ref_pos_[14] = std::clamp(ref_pos_[14], -kPi / 3, kPi / 3);

  for (int nidx = 0; nidx < kDof; ++nidx) {
    des_pos_[nidx] = ref_pos_[nidx];
    joints_[nidx].R_pos = des_pos_[nidx];
  }
}

void DaruV4MuJoCoController::RefTorqueInit() { std::memset(des_torque_, 0, sizeof(double) * kDof); }

void DaruV4MuJoCoController::CGCompensation(double k) {
  RigidBodyDynamics::Math::VectorNd cg = RigidBodyDynamics::Math::VectorNd::Zero(daru_.Dof());
  daru_.calCG(cg);
  for (int i = 1; i < kDof; ++i) {
    des_torque_[i] += cg[i] * k;
  }
}

std::array<double, DaruV4MuJoCoController::kDof> DaruV4MuJoCoController::JointPosition() const {
  std::array<double, kDof> out{};
  for (int i = 0; i < kDof; ++i) {
    out[i] = joints_[i].A_pos;
  }
  return out;
}

std::array<double, DaruV4MuJoCoController::kDof> DaruV4MuJoCoController::JointRefPosition() const {
  std::array<double, kDof> out{};
  for (int i = 0; i < kDof; ++i) {
    out[i] = joints_[i].R_pos;
  }
  return out;
}

std::array<double, DaruV4MuJoCoController::kDof> DaruV4MuJoCoController::JointVelocity() const {
  std::array<double, kDof> out{};
  for (int i = 0; i < kDof; ++i) {
    out[i] = joints_[i].A_vel;
  }
  return out;
}

std::array<double, DaruV4MuJoCoController::kDof> DaruV4MuJoCoController::JointAcceleration() const {
  std::array<double, kDof> out{};
  for (int i = 0; i < kDof; ++i) {
    out[i] = joints_[i].A_acc;
  }
  return out;
}

std::array<double, DaruV4MuJoCoController::kDof> DaruV4MuJoCoController::JointEffort() const {
  std::array<double, kDof> out{};
  for (int i = 0; i < kDof; ++i) {
    out[i] = joints_[i].A_tau;
  }
  return out;
}

std::array<double, 2> DaruV4MuJoCoController::HeadPosition() const {
  std::array<double, 2> out{};
  for (int i = 0; i < kHeadDof; ++i) {
    out[i] = head_pos_[i];
  }
  return out;
}

std::array<double, 2> DaruV4MuJoCoController::HeadRefPosition() const {
  std::array<double, 2> out{};
  for (int i = 0; i < kHeadDof; ++i) {
    out[i] = head_des_pos_[i];
  }
  return out;
}

std::array<double, 2> DaruV4MuJoCoController::HeadVelocity() const {
  std::array<double, 2> out{};
  for (int i = 0; i < kHeadDof; ++i) {
    out[i] = head_vel_[i];
  }
  return out;
}

std::array<double, 2> DaruV4MuJoCoController::HeadEffort() const {
  std::array<double, 2> out{};
  for (int i = 0; i < kHeadDof; ++i) {
    out[i] = head_tau_[i];
  }
  return out;
}

std::array<double, 12> DaruV4MuJoCoController::HandPosition() const {
  std::array<double, 12> out{};
  for (int i = 0; i < kHandDof; ++i) {
    out[i] = hand_pub_pos_[i];
  }
  return out;
}

std::array<double, 12> DaruV4MuJoCoController::HandRefPosition() const {
  std::array<double, 12> out{};
  for (int i = 0; i < kHandDof; ++i) {
    out[i] = hand_pub_des_pos_[i];
  }
  return out;
}

std::array<double, 12> DaruV4MuJoCoController::HandVelocity() const {
  std::array<double, 12> out{};
  for (int i = 0; i < kHandDof; ++i) {
    out[i] = hand_pub_vel_[i];
  }
  return out;
}

std::array<double, 12> DaruV4MuJoCoController::HandEffort() const {
  std::array<double, 12> out{};
  for (int i = 0; i < kHandDof; ++i) {
    out[i] = hand_pub_tau_[i];
  }
  return out;
}

std::array<double, 14> DaruV4MuJoCoController::RefEePose() const {
  std::array<double, 14> out{};
  RigidBodyDynamics::Math::Vector3d l_pos_des;
  RigidBodyDynamics::Math::Vector3d r_pos_des;
  Eigen::Quaterniond lq;
  Eigen::Quaterniond rq;

  {
    const std::lock_guard<std::mutex> lock(target_mutex_);
    l_pos_des = l_pos_des_;
    r_pos_des = r_pos_des_;
    lq = l_quat_des_.normalized();
    rq = r_quat_des_.normalized();
  }

  out[0] = l_pos_des[0];
  out[1] = l_pos_des[1];
  out[2] = l_pos_des[2];
  if (lq.w() < 0.0) {
    out[3] = -lq.w();
    out[4] = -lq.x();
    out[5] = -lq.y();
    out[6] = -lq.z();
  } else {
    out[3] = lq.w();
    out[4] = lq.x();
    out[5] = lq.y();
    out[6] = lq.z();
  }

  out[7] = r_pos_des[0];
  out[8] = r_pos_des[1];
  out[9] = r_pos_des[2];
  if (rq.w() < 0.0) {
    out[10] = -rq.w();
    out[11] = -rq.x();
    out[12] = -rq.y();
    out[13] = -rq.z();
  } else {
    out[10] = rq.w();
    out[11] = rq.x();
    out[12] = rq.y();
    out[13] = rq.z();
  }

  return out;
}

std::array<double, 14> DaruV4MuJoCoController::ActEePose() const {
  std::array<double, 14> out{};
  const Eigen::Quaterniond lq(l_rot_act_);
  const Eigen::Quaterniond rq(r_rot_act_);

  out[0] = l_pos_act_[0];
  out[1] = l_pos_act_[1];
  out[2] = l_pos_act_[2];
  if (lq.w() < 0.0) {
    out[3] = -lq.w();
    out[4] = -lq.x();
    out[5] = -lq.y();
    out[6] = -lq.z();
  } else {
    out[3] = lq.w();
    out[4] = lq.x();
    out[5] = lq.y();
    out[6] = lq.z();
  }

  out[7] = r_pos_act_[0];
  out[8] = r_pos_act_[1];
  out[9] = r_pos_act_[2];
  if (rq.w() < 0.0) {
    out[10] = -rq.w();
    out[11] = -rq.x();
    out[12] = -rq.y();
    out[13] = -rq.z();
  } else {
    out[10] = rq.w();
    out[11] = rq.x();
    out[12] = rq.y();
    out[13] = rq.z();
  }

  return out;
}

std::array<double, 14> DaruV4MuJoCoController::RealEePose() {
  std::array<double, 14> out{};

  std::array<JOINT_, kDof> tmp = upper_joint_;
  for (int i = 0; i < kDof; ++i) {
    tmp[i].A_pos = joints_[i].A_pos;
    tmp[i].A_vel = joints_[i].A_vel;
    tmp[i].A_acc = joints_[i].A_acc;
    tmp[i].A_tau = joints_[i].A_tau;
  }
  daru_.updateQ(tmp.data());

  RigidBodyDynamics::Math::Vector3d l_pos_real, r_pos_real;
  RigidBodyDynamics::Math::Matrix3d l_rot_real, r_rot_real;
  daru_.calEEPos(l_pos_real, l_rot_real, r_pos_real, r_rot_real);

  const Eigen::Quaterniond lq(l_rot_real);
  const Eigen::Quaterniond rq(r_rot_real);

  out[0] = l_pos_real[0];
  out[1] = l_pos_real[1];
  out[2] = l_pos_real[2];
  if (lq.w() < 0.0) {
    out[3] = -lq.w();
    out[4] = -lq.x();
    out[5] = -lq.y();
    out[6] = -lq.z();
  } else {
    out[3] = lq.w();
    out[4] = lq.x();
    out[5] = lq.y();
    out[6] = lq.z();
  }

  out[7] = r_pos_real[0];
  out[8] = r_pos_real[1];
  out[9] = r_pos_real[2];
  if (rq.w() < 0.0) {
    out[10] = -rq.w();
    out[11] = -rq.x();
    out[12] = -rq.y();
    out[13] = -rq.z();
  } else {
    out[10] = rq.w();
    out[11] = rq.x();
    out[12] = rq.y();
    out[13] = rq.z();
  }

  return out;
}

void DaruV4MuJoCoController::JointPdControl(bool cur_reset) {
  for (int n = 0; n < kDof; ++n) {
    pd_des_current_[n] = 40 * (des_pos_[n] - joints_[n].A_pos) + 3 * (0 - joints_[n].A_vel);
    if (n == 5 || n == 6 || n == 7 || n == 12 || n == 13 || n == 14) {
      pd_des_current_[n] = 30 * (des_pos_[n] - joints_[n].A_pos) + 0.8 * (0 - joints_[n].A_vel);
    }
    if (n == 0) {
      pd_des_current_[n] = 200 * (des_pos_[n] - joints_[n].A_pos) + 20 * (0 - joints_[n].A_vel);
    }
  }

  pd_des_current_[0] = std::clamp(pd_des_current_[0], -120.0, 120.0);
  
  pd_des_current_[1] = std::clamp(pd_des_current_[1], -60.0, 60.0);
  pd_des_current_[2] = std::clamp(pd_des_current_[2], -60.0, 60.0);
  pd_des_current_[3] = std::clamp(pd_des_current_[3], -60.0, 60.0);
  pd_des_current_[4] = std::clamp(pd_des_current_[4], -60.0, 60.0);
  pd_des_current_[5] = std::clamp(pd_des_current_[5], -10.0, 10.0);
  pd_des_current_[6] = std::clamp(pd_des_current_[6], -10.0, 10.0);
  pd_des_current_[7] = std::clamp(pd_des_current_[7], -10.0, 10.0);

  pd_des_current_[8] = std::clamp(pd_des_current_[8],   -60.0, 60.0);
  pd_des_current_[9] = std::clamp(pd_des_current_[9],   -60.0, 60.0);
  pd_des_current_[10] = std::clamp(pd_des_current_[10], -60.0, 60.0);
  pd_des_current_[11] = std::clamp(pd_des_current_[11], -60.0, 60.0);
  pd_des_current_[12] = std::clamp(pd_des_current_[12], -10.0, 10.0);
  pd_des_current_[13] = std::clamp(pd_des_current_[13], -10.0, 10.0);
  pd_des_current_[14] = std::clamp(pd_des_current_[14], -10.0, 10.0);

  for (int n = 0; n < kDof; ++n) {
    if (cur_reset) {
      joints_[n].R_tau = pd_des_current_[n];
    } else {
      joints_[n].R_tau += pd_des_current_[n];
    }
  }
}

void DaruV4MuJoCoController::JointTorqueControl(bool cur_reset) {
  for (int n = 0; n < kDof; ++n) {
    if (cur_reset) {
      joints_[n].R_tau = des_torque_[n];
    } else {
      joints_[n].R_tau += des_torque_[n];
    }
  }
}

void DaruV4MuJoCoController::HandControlTick() {
  if (!hand_initialized_) {
    hand_cmd_.fill(0.0);
    return;
  }

  constexpr double kHandControlPeriod = 0.020;
  if (hand_control_accumulator_ + 1e-9 < kHandControlPeriod) {
    return;
  }
  while (hand_control_accumulator_ >= kHandControlPeriod) {
    hand_control_accumulator_ -= kHandControlPeriod;
  }

  std::array<double, kHandDof> hand_des_pos;
  {
    const std::lock_guard<std::mutex> lock(hand_target_mutex_);
    hand_des_pos = hand_des_pos_;
  }

  for (int i = 0; i < kHandDof; ++i) {
    hand_pub_pos_[i] = hand_pos_[i];
    hand_pub_vel_[i] = hand_vel_[i];
    hand_pub_tau_[i] = (hand_force_sensor_adr_[i] >= 0) ? d_->sensordata[hand_force_sensor_adr_[i]] : hand_tau_[i];
    hand_pub_des_pos_[i] = hand_des_pos[i];
  }

  constexpr double kHandKp = 10000.0;
  constexpr double kHandKd = 100.0;
  constexpr double kHandFallbackForceLimit = 56.0;

  for (int i = 0; i < kHandDof; ++i) {
    const int jid = hand_map_joint_[i];
    const int aid = hand_map_actuator_[i];
    if (jid < 0) {
      hand_cmd_[i] = 0.0;
      continue;
    }

    const double des = ClampHandPositionCommand(m_, aid, jid, hand_des_pos[i]);
    if (hand_ctrl_mode_[i] == kHandCtrlDirect) {
      hand_cmd_[i] = des;
      continue;
    }

    if (hand_ctrl_mode_[i] == kHandCtrlForce) {
      const double force_cmd = kHandKp * (des - hand_pos_[i]) - kHandKd * hand_vel_[i];
      hand_cmd_[i] = ClampActuatorTorqueCommand(m_, aid, kHandFallbackForceLimit, force_cmd);
      continue;
    }

    hand_cmd_[i] = 0.0;
  }
}

void DaruV4MuJoCoController::HeadPdControl() {
  if (!head_initialized_) {
    head_cmd_tau_.fill(0.0);
    return;
  }

  constexpr std::array<double, kHeadDof> kHeadKp = {25.0, 25.0};
  constexpr std::array<double, kHeadDof> kHeadKd = {1.5, 1.5};
  constexpr std::array<double, kHeadDof> kHeadFallbackLimit = {7.0, 7.0};

  for (int i = 0; i < kHeadDof; ++i) {
    if (head_map_dof_[i] < 0) {
      head_cmd_tau_[i] = 0.0;
      continue;
    }

    const double command = kHeadKp[i] * (head_des_pos_[i] - head_pos_[i]) - kHeadKd[i] * head_vel_[i];
    head_cmd_tau_[i] =
        ClampActuatorTorqueCommand(m_, head_map_actuator_[i], kHeadFallbackLimit[i], command);
  }
}

}  // namespace daru_mj
