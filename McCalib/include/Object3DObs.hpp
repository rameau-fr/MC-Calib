#pragma once

#include "Board.hpp"
#include "Object3D.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv_compat.hpp>
#include <stdio.h>

namespace McCalib {

class Camera;
class Board;
class BoardObs;
class Object3D;

/**
 * @class Object3DObs
 *
 * @brief Observation of one 3D object from one camera/frame viewpoint.
 *
 * Aggregates board observations belonging to the same object and exposes the
 * estimated object pose in camera coordinates and camera-group coordinates.
 */
class Object3DObs final {
public:
  // Indexing
  int frame_id_;
  int camera_id_;
  std::vector<int> board_id_;

  // Pose
  std::array<double, 6> pose_;

  // Pose of the object expressed in camera group referential
  std::array<double, 6> group_pose_;

  // points
  std::vector<cv::Point2f> pts_2d_;
  std::vector<int> pts_id_; // index in the object

  // Camera corresponding to this Observation
  std::weak_ptr<Camera> cam_;

  // Board3D corresponding to this Observation
  std::map<int, std::weak_ptr<Board>> board_3d_;

  // BoardObs corresponding to this Observation
  std::map<int, std::weak_ptr<BoardObs>> board_observations_;

  // Object3DObs corresponding to this Observation
  std::weak_ptr<Object3D> object_3d_;

  int object_3d_id_;

  // Validity : An observation is judged not valid when the RANSAC pose
  // estimation return too few pts
  bool valid_ = true;

  // Functions
  Object3DObs() = delete;

  /**
   * @brief Destroy the object observation.
   */
  ~Object3DObs(){};

  /**
   * @brief Construct an object observation for one object id.
   *
   * @param obj_obs Parent object this observation belongs to.
   * @param object_idx Object id.
   */
  Object3DObs(const std::shared_ptr<Object3D> obj_obs, const int object_idx);

  /**
   * @brief Insert one board observation into this object observation.
   *
   * Also appends 2D points and remapped point ids in the object index space.
   *
   * @param new_board_obs Board observation to add.
   */
  void insertNewBoardObs(const std::shared_ptr<BoardObs> new_board_obs);

  /**
   * @brief Get object pose in camera coordinates as vector form.
   *
   * @param R Output rotation vector (3x1).
   * @param T Output translation vector (3x1).
   */
  void getPoseVec(cv::Mat &R, cv::Mat &T) const;

  /**
   * @brief Get object pose in camera coordinates as 4x4 transform.
   *
   * @return 4x4 object pose in camera frame.
   */
  cv::Mat getPoseMat() const;

  /**
   * @brief Set object pose in camera coordinates from 4x4 transform.
   *
   * @param Pose 4x4 object pose in camera frame.
   */
  void setPoseMat(const cv::Mat &Pose);

  /**
   * @brief Set object pose in camera coordinates from vector form.
   *
   * @param Rvec Rotation vector (3x1).
   * @param T Translation vector (3x1).
   */
  void setPoseVec(const cv::Mat &Rvec, const cv::Mat &T);

  /**
   * @brief Get only the rotation component of the camera-frame pose.
   *
   * @return Rotation vector (3x1).
   */
  cv::Mat getRotVec() const;

  /**
   * @brief Get only the translation component of the camera-frame pose.
   *
   * @return Translation vector (3x1).
   */
  cv::Mat getTransVec() const;

  /**
   * @brief Estimate object pose from 2D-3D correspondences.
   *
   * @param ransac_thresh Reprojection threshold in pixels.
   * @param ransac_iterations Number of RANSAC iterations.
   */
  void estimatePose(const float ransac_thresh, const int ransac_iterations);

  /**
   * @brief Compute mean reprojection error for this object observation.
   *
   * @return Mean per-point reprojection error in pixels.
   */
  float computeReprojectionError() const;

  /**
   * @brief Set object pose in camera-group coordinates from 4x4 transform.
   *
   * @param pose 4x4 object pose in camera-group frame.
   */
  void setPoseInGroupMat(const cv::Mat &pose);

  /**
   * @brief Set object pose in camera-group coordinates from vector form.
   *
   * @param r_vec Rotation vector (3x1).
   * @param t_vec Translation vector (3x1).
   */
  void setPoseInGroupVec(const cv::Mat &r_vec, const cv::Mat &t_vec);

  /**
   * @brief Get object pose in camera-group coordinates as vector form.
   *
   * @param r_vec Output rotation vector (3x1).
   * @param t_vec Output translation vector (3x1).
   */
  void getPoseInGroupVec(cv::Mat &r_vec, cv::Mat &t_vec) const;

  /**
   * @brief Get object pose in camera-group coordinates as 4x4 transform.
   *
   * @return 4x4 object pose in camera-group frame.
   */
  cv::Mat getPoseInGroupMat() const;

  /**
   * @brief Get rotation component of the camera-group pose.
   *
   * @return Rotation vector (3x1).
   */
  cv::Mat getRotInGroupVec() const;

  /**
   * @brief Get translation component of the camera-group pose.
   *
   * @return Translation vector (3x1).
   */
  cv::Mat getTransInGroupVec() const;
};

} // namespace McCalib