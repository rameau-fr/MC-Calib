#include "utilities.hpp"

namespace McCalib {

std::filesystem::path convertStrToPath(const std::string &item_name) {
  const std::filesystem::path path(item_name);
  return path;
}

std::vector<std::filesystem::path>
convertVecStrToVecPath(const std::vector<std::string> &input) {
  std::vector<std::filesystem::path> out;
  out.reserve(input.size());
  for (const std::string &item : input) {
    const std::filesystem::path path(item);
    out.push_back(path);
  }
  return out;
}

} // namespace McCalib