
#include <filesystem>
#include <string>
#include <vector>

namespace McCalib {

std::filesystem::path convertStrToPath(const std::string &item_name);
std::vector<std::filesystem::path>
convertVecStrToVecPath(const std::vector<std::string> &input);

} // namespace McCalib