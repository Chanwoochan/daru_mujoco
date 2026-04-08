#include "daru_mujoco/package_paths.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdlib>
#include <stdexcept>
#include <string>

namespace daru_mj {
namespace {

std::filesystem::path ResolveExistingDirectory(const std::vector<std::filesystem::path>& candidates) {
  for (const auto& candidate : candidates) {
    if (!candidate.empty() && std::filesystem::exists(candidate) && std::filesystem::is_directory(candidate)) {
      return candidate;
    }
  }
  return {};
}

}  // namespace

std::filesystem::path GetKcwarmMujocoShareDir() {
  try {
    return ament_index_cpp::get_package_share_directory("daru_mujoco");
  } catch (const std::exception&) {
  }

  std::vector<std::filesystem::path> fallback_candidates;
  if (const char* env_path = std::getenv("KCWARM_MUJOCO_SHARE_DIR")) {
    fallback_candidates.emplace_back(env_path);
  }
  fallback_candidates.emplace_back(std::filesystem::current_path() / "install" / "daru_mujoco" / "share" / "daru_mujoco");
  fallback_candidates.emplace_back(std::filesystem::current_path());

  const auto resolved = ResolveExistingDirectory(fallback_candidates);
  if (!resolved.empty()) {
    return resolved;
  }

  throw std::runtime_error("Failed to locate daru_mujoco share directory");
}

std::filesystem::path GetKcwarmMujocoDefaultMjcfPath() {
  return GetKcwarmMujocoShareDir() / "urdf" / "DARU_NEW_260323" / "scene.xml";
}

std::filesystem::path GetKcwarmMujocoDefaultUrdfPath() {
  return GetKcwarmMujocoShareDir() / "urdf" / "DARU_V4.urdf";
}

std::vector<std::filesystem::path> GetKcwarmMujocoTextureSearchBases() {
  std::vector<std::filesystem::path> bases = {
      std::filesystem::current_path() / "resource" / "textures",
      std::filesystem::current_path() / "textures",
  };

  try {
    bases.push_back(GetKcwarmMujocoShareDir() / "resource" / "textures");
  } catch (const std::exception&) {
  }

  return bases;
}

}  // namespace daru_mj
