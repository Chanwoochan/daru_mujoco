#include <algorithm>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

#include "daru_mujoco/package_paths.hpp"
#include "daru_mujoco/gr00t_infer/daru_v4_mujoco_gr00t_infer_controller.hpp"
#include "daru_mujoco/gr00t_infer/daru_v4_mujoco_gr00t_infer_gui.hpp"
#include "daru_mujoco/gr00t_infer/daru_v4_mujoco_gr00t_infer_ros.hpp"

int main(int argc, char** argv) {
  const auto resolve_path = [](const char* raw_path) {
    const std::filesystem::path input(raw_path);
    const std::filesystem::path absolute =
        input.is_absolute() ? input : std::filesystem::absolute(input);

    std::error_code ec;
    const std::filesystem::path normalized =
        std::filesystem::weakly_canonical(absolute, ec);
    return ec ? absolute : normalized;
  };

  const std::filesystem::path default_mjcf = daru_mj::GetKcwarmMujocoDefaultMjcfPath();
  const std::filesystem::path default_urdf = daru_mj::GetKcwarmMujocoDefaultUrdfPath();

  if (argc > 4) {
    std::cerr << "Usage: daru_v4_mujoco_gr00t_infer_sim [mjcf.xml] [robot.urdf] [steps]\n";
    return 1;
  }

  const std::filesystem::path mjcf_path =
      (argc >= 2) ? resolve_path(argv[1]) : default_mjcf;
  const std::filesystem::path urdf_path =
      (argc >= 3) ? resolve_path(argv[2]) : default_urdf;

  int steps = 0;
  for (int i = 3; i < argc; ++i) {
    const std::string arg = argv[i];
    char* end = nullptr;
    const long parsed_steps = std::strtol(arg.c_str(), &end, 10);
    if (end != arg.c_str() && end && *end == '\0') {
      steps = (parsed_steps < 0) ? 0 : parsed_steps;
    }
  }

  if (!std::filesystem::exists(mjcf_path)) {
    std::cerr << "MJCF file not found: " << mjcf_path << "\n";
    return 1;
  }
  if (!std::filesystem::exists(urdf_path)) {
    std::cerr << "URDF file not found: " << urdf_path << "\n";
    return 1;
  }

  char error[1024] = {0};
  mjModel* model = mj_loadXML(mjcf_path.string().c_str(), nullptr, error, sizeof(error));
  if (!model) {
    std::cerr << "Failed to load MJCF: " << error << "\n";
    return 2;
  }

  mjData* data = mj_makeData(model);
  if (!data) {
    std::cerr << "Failed to allocate mjData\n";
    mj_deleteModel(model);
    return 3;
  }

  rclcpp::init(argc, argv);

  int rc = 0;
  try {
    daru_mj::DaruV4MuJoCoCustomController controller(model, data, urdf_path.string());
    daru_mj::DaruV4MuJoCoCustomRosBridge ros_bridge(controller, "daru_v4_mujoco_gr00t_infer_sim");

    mj_forward(model, data);

    auto ros_tick = [&]() {
      ros_bridge.SpinSome();
      if (ros_bridge.ConsumeResetRequest()) {
        mj_resetData(model, data);
        mj_forward(model, data);
        controller.Reset();
      }
    };

    rc = daru_mj::RunGr00tInferGui(model, data, controller, ros_bridge, steps, ros_tick);
  } catch (const std::exception& e) {
    std::cerr << "Simulation error: " << e.what() << "\n";
    rc = 4;
  }

  rclcpp::shutdown();
  mj_deleteData(data);
  mj_deleteModel(model);
  return rc;
}
