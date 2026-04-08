#pragma once

#include <functional>

#include <mujoco/mujoco.h>

#include "daru_mujoco/teleop/daru_v4_mujoco_teleop_controller.hpp"
#include "daru_mujoco/teleop/daru_v4_mujoco_teleop_ros.hpp"

namespace daru_mj {

int RunTeleopGui(mjModel* model,
                 mjData* data,
                 DaruV4MuJoCoController& controller,
                 DaruV4MuJoCoRosBridge& ros_bridge,
                 int max_steps,
                 const std::function<void()>& step_hook);

}  // namespace daru_mj
