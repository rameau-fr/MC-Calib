#include "utilities.hpp"

#include <numeric>

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

#if (defined(CV_VERSION_MAJOR) && CV_VERSION_MAJOR <= 4 &&                     \
     defined(CV_VERSION_MINOR) && CV_VERSION_MINOR < 7)

std::map<int, cv::Ptr<cv::aruco::CharucoBoard>>
createCharucoBoards(const unsigned int num_board,
                    const std::vector<int> &number_x_square_per_board,
                    const std::vector<int> &number_y_square_per_board,
                    const float length_square, const float length_marker,
                    const cv::Ptr<cv::aruco::Dictionary> dict) {
  std::map<int, cv::Ptr<cv::aruco::CharucoBoard>> charuco_boards;
  int offset_count = 0;
  for (std::size_t i = 0; i < num_board; i++) {
    // declare the board
    cv::Ptr<cv::aruco::CharucoBoard> charuco = cv::aruco::CharucoBoard::create(
        number_x_square_per_board[i], number_y_square_per_board[i],
        length_square, length_marker, dict);
    // If it is the first board then just use the standard idx
    if (i != 0) {
      int id_offset = charuco_boards[i - 1]->ids.size() + offset_count;
      offset_count = id_offset;
      for (auto &id : charuco->ids) {
        id += id_offset;
      }
    }
    charuco_boards[i] = charuco;
  }
  assert(charuco_boards.size() == num_board);
  return charuco_boards;
}

std::vector<cv::Mat>
createCharucoBoardsImages(const unsigned int num_board,
                          const std::vector<int> &number_x_square_per_board,
                          const std::vector<int> &number_y_square_per_board,
                          const float length_square, const float length_marker,
                          const std::vector<int> &resolution_x_per_board,
                          const std::vector<int> &resolution_y_per_board) {
  cv::Ptr<cv::aruco::Dictionary> dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  const std::map<int, cv::Ptr<cv::aruco::CharucoBoard>> charuco_boards =
      createCharucoBoards(num_board, number_x_square_per_board,
                          number_y_square_per_board, length_square,
                          length_marker, dict);

  std::vector<cv::Mat> charuco_boards_images;
  charuco_boards_images.reserve(num_board);
  for (std::size_t i = 0u; i < charuco_boards.size(); ++i) {
    // create the charuco board image
    cv::Mat board_image;
    for (auto const &charuco_board_iter : charuco_boards) {
      charuco_board_iter.second->draw(
          cv::Size(resolution_x_per_board[i], resolution_y_per_board[i]),
          board_image, 10, 1);
      charuco_boards_images.push_back(board_image);
    }
  }
  return charuco_boards_images;
}

#else

std::map<int, cv::Ptr<cv::aruco::CharucoBoard>>
createCharucoBoards(const unsigned int num_board,
                    const std::vector<int> &number_x_square_per_board,
                    const std::vector<int> &number_y_square_per_board,
                    const float length_square, const float length_marker,
                    const cv::aruco::Dictionary &dict) {
  std::map<int, cv::Ptr<cv::aruco::CharucoBoard>> charuco_boards;
  int offset_count = 0;
  for (std::size_t i = 0; i < num_board; i++) {
    if (i == 0) {
      // if it is the first board then just use the standard idx
      charuco_boards[i] = cv::makePtr<cv::aruco::CharucoBoard>(
          cv::aruco::CharucoBoard(cv::Size(number_x_square_per_board[i],
                                           number_y_square_per_board[i]),
                                  length_square, length_marker, dict));
    } else {
      int id_offset = charuco_boards[i - 1]->getIds().size() + offset_count;
      offset_count = id_offset;

      const std::size_t num_idxs = charuco_boards[i - 1]->getIds().size();
      std::vector<int> cur_ids(num_idxs);
      std::iota(cur_ids.begin(), cur_ids.end(), id_offset);

      charuco_boards[i] = cv::makePtr<cv::aruco::CharucoBoard>(
          cv::aruco::CharucoBoard(cv::Size(number_x_square_per_board[i],
                                           number_y_square_per_board[i]),
                                  length_square, length_marker, dict, cur_ids));
    }
  }
  assert(charuco_boards.size() == num_board);
  return charuco_boards;
}

std::vector<cv::Mat>
createCharucoBoardsImages(const unsigned int num_board,
                          const std::vector<int> &number_x_square_per_board,
                          const std::vector<int> &number_y_square_per_board,
                          const float length_square, const float length_marker,
                          const std::vector<int> &resolution_x_per_board,
                          const std::vector<int> &resolution_y_per_board) {
  const cv::aruco::Dictionary dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  const std::map<int, cv::Ptr<cv::aruco::CharucoBoard>> charuco_boards =
      createCharucoBoards(num_board, number_x_square_per_board,
                          number_y_square_per_board, length_square,
                          length_marker, dict);

  std::vector<cv::Mat> charuco_boards_images;
  charuco_boards_images.reserve(num_board);
  for (std::size_t i = 0u; i < charuco_boards.size(); ++i) {
    // create the charuco board image
    cv::Mat board_image;
    for (auto const &charuco_board_iter : charuco_boards) {
      charuco_board_iter.second->generateImage(
          cv::Size(resolution_x_per_board[i], resolution_y_per_board[i]),
          board_image, 10, 1);
      charuco_boards_images.push_back(board_image);
    }
  }

  return charuco_boards_images;
}

#endif

} // namespace McCalib