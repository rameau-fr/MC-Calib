#pragma once

#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv_compat.hpp>
#include <stdio.h>

#include "Board.hpp"
#include "BoardObs.hpp"
#include "Frame.hpp"
#include "Object3DObs.hpp"

namespace McCalib {

/**
 * @class Camera
 *
 * @brief Stores camera intrinsics and all board/object observations.
 *
 * The class handles intrinsic initialization/refinement and provides
 * conversion helpers between internal array storage and OpenCV matrices.
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

  /**
   * @brief Construct a camera model.
   *
   * @param cam_idx Camera id.
   * @param distortion_model Distortion model id (0: Brown, 1: Kannala).
   */
  Camera(const int cam_idx, const int distortion_model);

  /**
   * @brief Destroy the camera object.
   */
  ~Camera(){};

  /**
   * @brief Register one board observation for this camera.
   *
   * @param newBoard Board observation to insert.
   */
  void insertNewBoard(const std::shared_ptr<BoardObs> newBoard);

  /**
   * @brief Register one frame where this camera has valid observations.
   *
   * @param newFrame Frame to insert.
   */
  void insertNewFrame(const std::shared_ptr<Frame> newFrame);

  /**
   * @brief Register one object observation for this camera.
   *
   * @param new_object Object observation to insert.
   */
  void insertNewObject(const std::shared_ptr<Object3DObs> new_object);

  /**
   * @brief Estimate initial intrinsic parameters from observed boards.
   */
  void initializeCalibration();

  /**
   * @brief Refine intrinsic parameters with non-linear optimization.
   *
   * @param nb_iterations Number of optimization iterations.
   */
  void refineIntrinsicCalibration(const int nb_iterations);

  /**
   * @brief Get intrinsic matrix K.
   *
   * @return 3x3 camera matrix.
   */
  cv::Mat getCameraMat() const;

  /**
   * @brief Set intrinsic matrix K.
   *
   * @param K 3x3 camera matrix.
   */
  void setCameraMat(const cv::Mat &K);

  /**
   * @brief Set distortion coefficients from OpenCV-style vector.
   *
   * @param distortion_vector 1x5 (Brown) or 1x4 (Kannala) vector.
   */
  void setDistortionVector(const cv::Mat &distortion_vector);

  /**
   * @brief Get distortion coefficients as OpenCV-style row vector.
   *
   * @return 1x5 (Brown) or 1x4 (Kannala) distortion vector.
   */
  cv::Mat getDistortionVectorVector() const;

  /**
   * @brief Get camera matrix and distortion coefficients.
   *
   * @param K Output 3x3 camera matrix.
   * @param distortion_vector Output distortion vector.
   */
  void getIntrinsics(cv::Mat &K, cv::Mat &distortion_vector);

  /**
   * @brief Set camera matrix and distortion coefficients.
   *
   * @param K 3x3 camera matrix.
   * @param distortion_vector Distortion vector.
   */
  void setIntrinsics(const cv::Mat &K, const cv::Mat &distortion_vector);

  /**
   * @brief Check if a board observation is sufficiently inside fisheye image.
   *
   * @param board_obs Board observation to validate.
   * @return True if all points satisfy the configured border tolerance.
   */
  bool checkBorderToleranceFisheye(const std::shared_ptr<BoardObs> board_obs);
};

} // namespace McCalib