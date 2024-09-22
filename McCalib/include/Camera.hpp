#pragma once

#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "Board.hpp"
#include "BoardObs.hpp"
#include "Frame.hpp"
#include "Object3DObs.hpp"

namespace McCalib {

/**
 * @class Camera
 *
 * @brief This class contains camera information
 *
 * - intrinsics (K + distortion + image size)
 * - frames
 * - boards observation
 * - object observation
 */
class Camera final {

private:
  // fisheye border margin to exclude invalid boards during initialization
  const float border_marging = 0.05f; // border margin tolerance

public:
  // datastructure for this camera
  std::map<int, std::weak_ptr<BoardObs>>
      board_observations_; // Observation of the boards (2d points)
  std::map<int, std::weak_ptr<Object3DObs>>
      object_observations_; // Observation of the 3D object (2d points)
  std::map<int, std::weak_ptr<Frame>>
      frames_;                      // Frames containing boards for this cameras
  std::vector<int> vis_board_idx_;  // vector of index of the 3D boards
  std::vector<int> vis_object_idx_; // vector of index of the 3D object

  // intrinsics
  // fx,fy,u0,v0,r1,r2,t1,t2,r3 (perspective)
  // fx,fy,u0,v0,k1,k2,k3,k4 (Kannala)
  std::array<double, 9> intrinsics_;

  int cam_idx_ = 0; // camera index
  int distortion_model_ = 0;
  int im_cols_, im_rows_;

  // Functions
  Camera() = delete;
  Camera(const int cam_idx, const int distortion_model);
  ~Camera(){};
  void insertNewBoard(std::shared_ptr<BoardObs> newBoard);
  void insertNewFrame(std::shared_ptr<Frame> newFrame);
  void insertNewObject(std::shared_ptr<Object3DObs> new_object);
  void initializeCalibration();
  void refineIntrinsicCalibration(const int nb_iterations);
  cv::Mat getCameraMat() const;
  void setCameraMat(const cv::Mat K);
  void setDistortionVector(const cv::Mat distortion_vector);
  cv::Mat getDistortionVectorVector() const;
  void getIntrinsics(cv::Mat &K, cv::Mat &distortion_vector);
  void setIntrinsics(const cv::Mat K, const cv::Mat distortion_vector);
  bool checkBorderToleranceFisheye(std::shared_ptr<BoardObs> board_obs);
};

} // namespace McCalib