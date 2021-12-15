#pragma once

#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

class BoardObs;
class Frame;

/**
 * @class Board
 * @brief This class contains the 3D board information
 *
 *  - 3D points
 *  - local 3D index (0-N)
 *  - observations of these boards
 *  - frames where these 3D boards are observed
 */

class Board final {
public:
  // Parameters
  int nb_x_square_, nb_y_square_, res_x_, res_y_;
  float length_square_, length_marker_;
  float square_size_;         // size of the squares in the calibration board
  int nb_pts_;                // Number of points on the board
  std::vector<double> color_; // color to display the board

  // 3D points
  std::vector<cv::Point3f> pts_3d_;
  std::vector<int> pts_idx_; // indexing 0 to N
  int board_id_;             // index of the board

  // List of board observation for this board
  std::map<int, std::weak_ptr<BoardObs>> board_observations_;

  // List of frames where this board is visible
  std::map<int, std::weak_ptr<Frame>> frames_;

  // Charuco board
  cv::Ptr<cv::aruco::CharucoBoard> charuco_board_; // vector of charuco boards

  // Functions
  Board() = delete;
  ~Board(){};
  Board(const std::string config, const int board_idx);
  void insertNewBoard(std::shared_ptr<BoardObs> new_board);
  void insertNewFrame(std::shared_ptr<Frame> new_frame);
};
