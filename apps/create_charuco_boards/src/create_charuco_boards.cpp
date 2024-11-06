#include <filesystem>
#include <iomanip>
#include <numeric>
#include <stdio.h>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>

#if (defined(CV_VERSION_MAJOR) && CV_VERSION_MAJOR <= 4 &&                     \
     defined(CV_VERSION_MINOR) && CV_VERSION_MINOR < 7)

std::vector<cv::Mat>
createCharucoBoards(const unsigned int num_board,
                    const std::vector<int> &number_x_square_per_board,
                    const std::vector<int> &number_y_square_per_board,
                    const float length_square, const float length_marker,
                    const std::vector<int> &resolution_x_per_board,
                    const std::vector<int> &resolution_y_per_board) {
  std::vector<cv::Mat> charuco_boards_images;
  charuco_boards_images.reserve(num_board);
  cv::Ptr<cv::aruco::Dictionary> dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  std::vector<cv::Ptr<cv::aruco::CharucoBoard>> charuco_boards;
  charuco_boards.reserve(num_board);
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
    charuco_boards.push_back(charuco);

    // create the charuco board image
    cv::Mat board_image;
    charuco_boards[i]->draw(
        cv::Size(resolution_x_per_board[i], resolution_y_per_board[i]),
        board_image, 10, 1);

    charuco_boards_images.push_back(board_image);
  }
  return charuco_boards_images;
}

#else

std::vector<cv::Mat>
createCharucoBoards(const unsigned int num_board,
                    const std::vector<int> &number_x_square_per_board,
                    const std::vector<int> &number_y_square_per_board,
                    const float length_square, const float length_marker,
                    const std::vector<int> &resolution_x_per_board,
                    const std::vector<int> &resolution_y_per_board) {
  std::vector<cv::Mat> charuco_boards_images;
  charuco_boards_images.reserve(num_board);
  const cv::aruco::Dictionary dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  std::vector<cv::aruco::CharucoBoard> charuco_boards;
  charuco_boards.reserve(num_board);
  int offset_count = 0;
  for (std::size_t i = 0; i < num_board; i++) {
    if (i == 0) {
      // if it is the first board then just use the standard idx
      cv::aruco::CharucoBoard charuco = cv::aruco::CharucoBoard(
          cv::Size(number_x_square_per_board[i], number_y_square_per_board[i]),
          length_square, length_marker, dict);

      charuco_boards.push_back(charuco);
    } else {
      int id_offset = charuco_boards[i - 1].getIds().size() + offset_count;
      offset_count = id_offset;

      const std::size_t num_idxs = charuco_boards[i - 1].getIds().size();
      std::vector<int> cur_ids(num_idxs);
      std::iota(cur_ids.begin(), cur_ids.end(), id_offset);

      cv::aruco::CharucoBoard charuco = cv::aruco::CharucoBoard(
          cv::Size(number_x_square_per_board[i], number_y_square_per_board[i]),
          length_square, length_marker, dict, cur_ids);

      charuco_boards.push_back(charuco);
    }

    // create the charuco board image
    cv::Mat board_image;
    charuco_boards[i].generateImage(
        cv::Size(resolution_x_per_board[i], resolution_y_per_board[i]),
        board_image, 10, 1);

    charuco_boards_images.push_back(board_image);
  }

  return charuco_boards_images;
}
#endif

void saveBoards(const std::vector<cv::Mat> boards) {
  for (std::size_t board_idx = 0u; board_idx < boards.size(); ++board_idx) {
    // Save the markers
    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << board_idx;
    std::string s1 = ss.str();
    std::string charname = "charuco_board_";
    std::string extname = ".bmp";
    std::string savename = charname + s1;
    savename += extname;
    std::cout << "save_name " << savename << std::endl;
    cv::imwrite(savename, boards[board_idx]);
  }
}

int main(int argc, char *argv[]) {

  if (argc != 2) {
    std::cout << "You need to provide the absolute path to the calibration file"
              << std::endl;
    return -1;
  }

  // path of the configuration file
  std::filesystem::path pathInt = argv[1];
  // Read the parameters from the configuration file
  int num_x_square, num_y_square, res_x, res_y, NbBoard;
  float length_square, length_marker;
  std::vector<int> number_x_square_per_board, number_y_square_per_board;
  std::vector<int> resolution_x_per_board, resolution_y_per_board;
  std::vector<double> square_size_per_board;
  cv::FileStorage fs; // FileStorage to read calibration params from file
  const bool is_file_available = std::filesystem::exists(pathInt) &&
                                 pathInt.has_filename() &&
                                 pathInt.extension() == ".yml";
  if (!is_file_available) {
    std::cout << "Config path '" << pathInt << "' doesn't exist." << std::endl;
    return -1;
  }
  fs.open(pathInt, cv::FileStorage::READ);
  fs["number_x_square"] >> num_x_square;
  fs["number_y_square"] >> num_y_square;
  fs["length_square"] >> length_square;
  fs["length_marker"] >> length_marker;
  fs["resolution_x"] >> res_x;
  fs["resolution_y"] >> res_y;

  fs["number_board"] >> NbBoard;
  assert(NbBoard > 0);
  const unsigned int num_board = static_cast<int>(NbBoard);

  fs["square_size_per_board"] >> square_size_per_board;
  fs["number_x_square_per_board"] >> number_x_square_per_board;
  fs["number_y_square_per_board"] >> number_y_square_per_board;
  fs["resolution_x_per_board"] >> resolution_x_per_board;
  fs["resolution_y_per_board"] >> resolution_y_per_board;

  fs.release(); // close the input file

  // Check if multi-size boards are used or not
  if (square_size_per_board.size() == 0) {
    for (std::size_t i = 0; i < num_board; i++) {
      number_x_square_per_board.push_back(num_x_square);
      number_y_square_per_board.push_back(num_y_square);
      resolution_x_per_board.push_back(res_x);
      resolution_y_per_board.push_back(res_y);
    }
  }

  const std::vector<cv::Mat> charuco_boards_images = createCharucoBoards(
      num_board, number_x_square_per_board, number_y_square_per_board,
      length_square, length_marker, resolution_x_per_board,
      resolution_y_per_board);
  saveBoards(charuco_boards_images);

  return 0;
}
