#pragma once

#include <functional>

#include <mujoco/mujoco.h>

#include "daru_mujoco/gr00t_infer/daru_v4_mujoco_gr00t_infer_controller.hpp"
#include "daru_mujoco/gr00t_infer/daru_v4_mujoco_gr00t_infer_ros.hpp"

namespace daru_mj {

int RunGr00tInferGui(mjModel* model,
                     mjData* data,
                     DaruV4MuJoCoCustomController& controller,
                     DaruV4MuJoCoCustomRosBridge& ros_bridge,
                     int max_steps,
                     const std::function<void()>& step_hook);

}  // namespace daru_mj
