#pragma once

#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv_compat.hpp>
#include <stdio.h>

#include "Board.hpp"
#include "BoardObs.hpp"
#include "Camera.hpp"
#include "Frame.hpp"
#include "Object3DObs.hpp"

namespace McCalib {

/**
 * @class CameraGroup
 *
 * @brief Represents one connected group of cameras and their relative poses.
 *
 * A camera group is built from overlapping observations. Poses of all cameras
 * in this group are expressed relative to the reference camera id_ref_cam_.
 */
class CameraGroup final {
public:
  // datastructure for this camera group
  std::map<int, std::weak_ptr<Object3DObs>>
      object_observations_; // Observation of the 3D object (2d points)
  std::map<int, std::weak_ptr<Frame>>
      frames_; // Frames containing boards for this cameras
  std::map<int, std::weak_ptr<Camera>> cameras_; // cameras in the camera group
  int nb_cams_ = 0;                 // number of cameras in the group
  std::vector<int> vis_object_idx_; // vector of index of the 3D object

  // extrinsic
  std::map<int, std::array<double, 6>>
      relative_camera_pose_; // camera pose wrt. the ref. cam
  int id_ref_cam_;
  std::vector<int> cam_idx; // index of the cameras in the group

  // camera group index
  int cam_group_idx_;

  // Functions
  CameraGroup() = delete;

  /**
   * @brief Destroy the camera group.
   */
  ~CameraGroup();

  /**
   * @brief Construct a camera group.
   *
   * @param id_ref_cam Id of the reference camera for group coordinates.
   * @param cam_group_idx Unique index of this camera group.
   */
  CameraGroup(const int id_ref_cam, const int cam_group_idx);

  /**
   * @brief Add a camera to the group.
   *
   * @param new_camera Camera to insert.
   */
  void insertCamera(const std::shared_ptr<Camera> new_camera);

  /**
   * @brief Add one object observation to this group.
   *
   * @param new_object_observation Object observation to insert.
   */
  void insertNewObjectObservation(
      const std::shared_ptr<Object3DObs> new_object_observation);

  /**
   * @brief Register a frame that contributes to this group.
   *
   * @param new_frame Frame to insert.
   */
  void insertNewFrame(const std::shared_ptr<Frame> new_frame);

  /**
   * @brief Get camera pose as Rodrigues rotation and translation vectors.
   *
   * @param id_cam Camera id in the group.
   * @param r_vec Output rotation vector (3x1).
   * @param t_vec Output translation vector (3x1).
   */
  void getCameraPoseVec(const int id_cam, cv::Mat &r_vec, cv::Mat &t_vec);

  /**
   * @brief Get camera pose as a homogeneous transform.
   *
   * @param id_cam Camera id in the group.
   * @return 4x4 transform from camera frame to group reference frame.
   */
  cv::Mat getCameraPoseMat(const int id_cam);

  /**
   * @brief Set camera pose from a homogeneous transform.
   *
   * @param pose 4x4 camera pose in group reference frame.
   * @param id_cam Camera id in the group.
   */
  void setCameraPoseMat(const cv::Mat &pose, const int id_cam);

  /**
   * @brief Set camera pose from Rodrigues and translation vectors.
   *
   * @param r_vec Rotation vector (3x1).
   * @param t_vec Translation vector (3x1).
   * @param id_cam Camera id in the group.
   */
  void setCameraPoseVec(const cv::Mat &r_vec, const cv::Mat &t_vec,
                        const int id_cam);

  /**
   * @brief Get only the rotation component of a camera pose.
   *
   * @param id_cam Camera id in the group.
   * @return Rotation vector (3x1).
   */
  cv::Mat getCameraRotVec(const int id_cam);

  /**
   * @brief Get only the translation component of a camera pose.
   *
   * @param id_cam Camera id in the group.
   * @return Translation vector (3x1).
   */
  cv::Mat getCameraTransVec(const int id_cam);

  /**
   * @brief Estimate one pose per visible object in this group.
   */
  void computeObjPoseInCameraGroup();

  /**
   * @brief Refine relative camera poses using fixed object structure.
   *
   * @param nb_iterations Number of non-linear optimization iterations.
   */
  void refineCameraGroup(const int nb_iterations);

  /**
   * @brief Compute and log per-group reprojection statistics.
   */
  void reproErrorCameraGroup();

  /**
   * @brief Jointly refine camera poses and object poses.
   *
   * @param nb_iterations Number of non-linear optimization iterations.
   */
  void refineCameraGroupAndObjects(const int nb_iterations);

  /**
   * @brief Jointly refine camera poses, object poses and camera intrinsics.
   *
   * @param nb_iterations Number of non-linear optimization iterations.
   */
  void refineCameraGroupAndObjectsAndIntrinsics(const int nb_iterations);
};

} // namespace McCalib