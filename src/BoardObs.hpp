#pragma once

#include "Board.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

class Camera;
class Board;

/**
 * @class BoardObs
 *
 * @brief This class contains information related to this board observation
 *
 * The board observation contains the 2D points and corresponding 3D board of an
 * observed board. Additionally, it contains the pose of the board w.r.t. the
 * camera observing the board.
 *
 */
class BoardObs {
public:
  // Indexing
  int frame_id_;
  int camera_id_;
  int board_id_;

  // Pose
  double *pose_ = new double[6];

  // points
  std::vector<cv::Point2f> pts_2d_;
  std::vector<int> charuco_id_;

  // Camera corresponding to this Observation
  std::weak_ptr<Camera> cam_;

  // Board3D corresponding to this Observation
  std::weak_ptr<Board> board_3d_;

  // Validity : An observation is judged not valid when the RANSAC pose
  // estimation return too few pts
  bool valid_ = true;

  // Functions
  BoardObs();
  ~BoardObs() { delete[] pose_; };
  void init(int camera_id, int frame_id, int board_id,
            std::vector<cv::Point2f> pts_2d, std::vector<int> charuco_id,
            std::shared_ptr<Camera> cam, std::shared_ptr<Board> board_3d);
  void getPoseVec(cv::Mat &R, cv::Mat &T);
  cv::Mat getPoseMat();
  void setPoseMat(cv::Mat Pose);
  void setPoseVec(cv::Mat Rvec, cv::Mat T);
  void estimatePose(double ransac_thresh);
  float computeReprojectionError();
  cv::Mat getRotVec();
  cv::Mat getTransVec();
};
