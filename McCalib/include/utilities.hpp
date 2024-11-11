
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>

namespace McCalib {

std::filesystem::path convertStrToPath(const std::string &item_name);
std::vector<std::filesystem::path>
convertVecStrToVecPath(const std::vector<std::string> &input);

#if (defined(CV_VERSION_MAJOR) && CV_VERSION_MAJOR <= 4 &&                     \
     defined(CV_VERSION_MINOR) && CV_VERSION_MINOR < 7)

std::map<int, cv::Ptr<cv::aruco::CharucoBoard>>
createCharucoBoards(const unsigned int num_board,
                    const std::vector<int> &number_x_square_per_board,
                    const std::vector<int> &number_y_square_per_board,
                    const float length_square, const float length_marker,
                    const cv::Ptr<cv::aruco::Dictionary> dict);
#else
std::map<int, cv::Ptr<cv::aruco::CharucoBoard>>
createCharucoBoards(const unsigned int num_board,
                    const std::vector<int> &number_x_square_per_board,
                    const std::vector<int> &number_y_square_per_board,
                    const float length_square, const float length_marker,
                    const cv::aruco::Dictionary &dict);
#endif

std::vector<cv::Mat>
createCharucoBoardsImages(const unsigned int num_board,
                          const std::vector<int> &number_x_square_per_board,
                          const std::vector<int> &number_y_square_per_board,
                          const float length_square, const float length_marker,
                          const std::vector<int> &resolution_x_per_board,
                          const std::vector<int> &resolution_y_per_board);

} // namespace McCalib