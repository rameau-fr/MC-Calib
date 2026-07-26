#pragma once

#include "Board.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv_compat.hpp>
#include <stdio.h>

namespace McCalib {

class Camera;
class Board;

/**
 * @class BoardObs
 *
 * @brief Observation of one board by one camera in one frame.
 *
 * It stores matched Charuco detections, links to the corresponding board/camera
 * entities, and an estimated board-to-camera pose.
 *
 */
class BoardObs final {
public:
  // Indexing
  int frame_id_;
  int camera_id_;
  int board_id_;

  // Pose
  std::array<double, 6> pose_;

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
  BoardObs() = delete;

  /**
   * @brief Destroy the board observation object.
   */
  ~BoardObs(){};

  /**
   * @brief Construct one board observation.
   *
   * @param camera_id Id of the observing camera.
   * @param frame_id Frame id where the board is detected.
   * @param board_id Id of the observed board.
   * @param pts_2d Detected 2D Charuco corners.
   * @param charuco_id Charuco corner ids matching pts_2d.
   * @param cam Camera object owning this observation.
   * @param board_3d Board object corresponding to board_id.
   */
  BoardObs(const int camera_id, const int frame_id, const int board_id,
           const std::vector<cv::Point2f> &pts_2d,
           const std::vector<int> &charuco_id,
           const std::shared_ptr<Camera> cam,
           const std::shared_ptr<Board> board_3d);

  /**
   * @brief Get board pose as Rodrigues rotation and translation vectors.
   *
   * @param R Output rotation vector (3x1).
   * @param T Output translation vector (3x1).
   */
  void getPoseVec(cv::Mat &R, cv::Mat &T) const;

  /**
   * @brief Get board pose as a 4x4 homogeneous transform.
   *
   * @return 4x4 transform from board frame to camera frame.
   */
  cv::Mat getPoseMat() const;

  /**
   * @brief Set board pose from a 4x4 homogeneous transform.
   *
   * @param Pose 4x4 board-to-camera transform.
   */
  void setPoseMat(const cv::Mat &Pose);

  /**
   * @brief Set board pose from Rodrigues and translation vectors.
   *
   * @param Rvec Rotation vector (3x1).
   * @param T Translation vector (3x1).
   */
  void setPoseVec(const cv::Mat &Rvec, const cv::Mat &T);

  /**
   * @brief Estimate pose with a robust PnP pipeline.
   *
   * @param ransac_thresh Reprojection threshold in pixels.
   * @param ransac_iterations Number of RANSAC iterations.
   */
  void estimatePose(const float ransac_thresh, const int ransac_iterations);

  /**
   * @brief Compute mean reprojection error for this observation.
   *
   * @return Mean per-point reprojection error in pixels.
   */
  float computeReprojectionError();

  /**
   * @brief Get only the rotation component of the board pose.
   *
   * @return Rotation vector (3x1).
   */
  cv::Mat getRotVec() const;

  /**
   * @brief Get only the translation component of the board pose.
   *
   * @return Translation vector (3x1).
   */
  cv::Mat getTransVec() const;
};

} // namespace McCalib