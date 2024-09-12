#include <filesystem>
#include <iomanip>
#include <stdio.h>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char *argv[]) {

  if (argc != 2) {
    std::cout << "You need to provide the absolute path to the calibration file"
              << std::endl;
    return -1;
  }

  // path of the configuration file
  std::string pathInt = argv[1];
  // Read the parameters from the configuration file
  int num_x_square, num_y_square, res_x, res_y, NbBoard;
  float length_square, length_marker;
  std::vector<int> number_x_square_per_board, number_y_square_per_board;
  std::vector<int> resolution_x_per_board, resolution_y_per_board;
  std::vector<double> square_size_per_board;
  cv::FileStorage fs; // FileStorage to read calibration params from file
  const bool is_file_available =
      std::filesystem::exists(pathInt) && pathInt.length() > 0;
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
  unsigned int num_board = static_cast<int>(NbBoard);

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

  // Create the charuco
  cv::Ptr<cv::aruco::Dictionary> dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  std::vector<cv::Ptr<cv::aruco::CharucoBoard>> charucoBoards;
  int offset_count = 0;
  for (std::size_t i = 0; i < num_board; i++) {
    // declare the board
    cv::Ptr<cv::aruco::CharucoBoard> charuco = cv::aruco::CharucoBoard::create(
        number_x_square_per_board[i], number_y_square_per_board[i],
        length_square, length_marker, dict);
    // If it is the first board then just use the standard idx
    if (i != 0) {
      int id_offset = charucoBoards[i - 1]->ids.size() + offset_count;
      offset_count = id_offset;
      for (auto &id : charuco->ids) {
        id += id_offset;
      }
    }
    // create the charuco board
    charucoBoards.push_back(charuco);
    cv::Mat boardImage;
    charucoBoards[i]->draw(
        cv::Size(resolution_x_per_board[i], resolution_y_per_board[i]),
        boardImage, 10, 1);
    // Display marker
    // cv::imshow("My Charuco", boardImage);
    // cv::waitKey(1);
    // Save the marker
    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << i;
    std::string s1 = ss.str();
    std::string charname = "charuco_board_";
    std::string extname = ".bmp";
    std::string savename = charname + s1;
    savename += extname;
    std::cout << "save_name " << savename << std::endl;
    cv::imwrite(savename, boardImage);
  }

  return 0;
}
