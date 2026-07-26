#pragma once

#include <filesystem>
#include <iostream>

#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv_compat.hpp>
#include <stdio.h>

namespace McCalib {

class BoardObs;
class Frame;

/**
 * @class Board
 * @brief Represents one Charuco board geometry used for calibration.
 *
 * Stores static board geometry and links to all associated observations/frames.
 */

class Board final {
public:
  // Parameters
  int nb_x_square_, nb_y_square_, res_x_, res_y_;
  float length_square_, length_marker_;
  float square_size_;        // size of the squares in the calibration board
  unsigned int nb_pts_;      // Number of points on the board
  std::array<int, 3> color_; // color to display the board

  // 3D points
  std::vector<cv::Point3f> pts_3d_;
  std::vector<int> pts_idx_; // indexing 0 to N
  int board_id_;             // index of the board

  // List of board observation for this board
  std::map<int, std::weak_ptr<BoardObs>> board_observations_;

  // List of frames where this board is visible
  std::map<int, std::weak_ptr<Frame>> frames_;

  // Charuco board
  cv::Ptr<cv::aruco::CharucoBoard> charuco_board_;

  // Functions
  Board() = delete;

  /**
   * @brief Destroy the board object.
   */
  ~Board(){};

  /**
   * @brief Construct board geometry from YAML configuration.
   *
   * @param config Path to calibration configuration file.
   * @param board_idx Board id to construct.
   * @param charuco_board OpenCV Charuco board descriptor.
   */
  Board(const std::filesystem::path &config, const int board_idx,
        const cv::Ptr<cv::aruco::CharucoBoard> charuco_board);

  /**
   * @brief Register one board observation.
   *
   * @param new_board Observation to insert.
   */
  void insertNewBoard(const std::shared_ptr<BoardObs> new_board);

  /**
   * @brief Register one frame where the board is visible.
   *
   * @param new_frame Frame to insert.
   */
  void insertNewFrame(const std::shared_ptr<Frame> new_frame);
};

} // namespace McCalib
