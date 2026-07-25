#include <algorithm>

#include <boost/test/unit_test.hpp>

#include <utilities.hpp>

namespace {

std::vector<int> getBoardIds(const cv::Ptr<cv::aruco::CharucoBoard> &board) {
#if MC_CALIB_USE_LEGACY_ARUCO_API
  return board->ids;
#else
  return board->getIds();
#endif
}

} // namespace

BOOST_AUTO_TEST_SUITE(CheckUtilities)

BOOST_AUTO_TEST_CASE(CheckConvertStrToPath) {
  const std::filesystem::path path =
      McCalib::convertStrToPath("data/Blender_Images/Scenario_1");

  BOOST_CHECK_EQUAL(path.filename().string(), "Scenario_1");
}

BOOST_AUTO_TEST_CASE(CheckConvertVecStrToVecPath) {
  const std::vector<std::string> input = {"alpha/file1.txt", "beta/file2.txt"};
  const std::vector<std::filesystem::path> output =
      McCalib::convertVecStrToVecPath(input);

  BOOST_REQUIRE_EQUAL(output.size(), input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    BOOST_CHECK_EQUAL(output[i].string(), input[i]);
  }
}

BOOST_AUTO_TEST_CASE(CheckConvertVecStrToVecPathEmptyInput) {
  const std::vector<std::string> input;
  const std::vector<std::filesystem::path> output =
      McCalib::convertVecStrToVecPath(input);

  BOOST_CHECK_EQUAL(output.size(), 0);
}

BOOST_AUTO_TEST_CASE(CheckCreateCharucoBoardsCountAndIdOffset) {
#if MC_CALIB_USE_LEGACY_ARUCO_API
  cv::Ptr<cv::aruco::Dictionary> dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
#else
  const cv::aruco::Dictionary dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
#endif

  const unsigned int num_board = 2;
  const std::vector<int> x_squares = {5, 6};
  const std::vector<int> y_squares = {7, 7};

  const auto boards = McCalib::createCharucoBoards(
      num_board, x_squares, y_squares, 0.03f, 0.02f, dict);

  BOOST_REQUIRE_EQUAL(boards.size(), num_board);
  BOOST_REQUIRE(boards.count(0) == 1);
  BOOST_REQUIRE(boards.count(1) == 1);

  const std::vector<int> ids_board_0 = getBoardIds(boards.at(0));
  const std::vector<int> ids_board_1 = getBoardIds(boards.at(1));
  BOOST_REQUIRE(!ids_board_0.empty());
  BOOST_REQUIRE(!ids_board_1.empty());

  const int max_board_0 =
      *std::max_element(ids_board_0.begin(), ids_board_0.end());
  const int min_board_1 =
      *std::min_element(ids_board_1.begin(), ids_board_1.end());
  BOOST_CHECK(min_board_1 > max_board_0);
}

BOOST_AUTO_TEST_CASE(CheckCreateCharucoBoardsImagesDimensions) {
  const unsigned int num_board = 2;
  const std::vector<int> x_squares = {5, 5};
  const std::vector<int> y_squares = {7, 7};
  const std::vector<int> resolution_x = {640, 800};
  const std::vector<int> resolution_y = {480, 600};

  const std::vector<cv::Mat> images =
      McCalib::createCharucoBoardsImages(num_board, x_squares, y_squares, 0.03f,
                                         0.02f, resolution_x, resolution_y);

  BOOST_REQUIRE_EQUAL(images.size(), num_board);
  BOOST_REQUIRE(!images[0].empty());
  BOOST_REQUIRE(!images[1].empty());

  BOOST_CHECK_EQUAL(images[0].cols, resolution_x[0]);
  BOOST_CHECK_EQUAL(images[0].rows, resolution_y[0]);
  BOOST_CHECK_EQUAL(images[1].cols, resolution_x[1]);
  BOOST_CHECK_EQUAL(images[1].rows, resolution_y[1]);
}

BOOST_AUTO_TEST_SUITE_END()
