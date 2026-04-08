#pragma once

#include <filesystem>
#include <vector>

namespace daru_mj {

std::filesystem::path GetKcwarmMujocoShareDir();
std::filesystem::path GetKcwarmMujocoDefaultMjcfPath();
std::filesystem::path GetKcwarmMujocoDefaultUrdfPath();
std::vector<std::filesystem::path> GetKcwarmMujocoTextureSearchBases();

}  // namespace daru_mj
