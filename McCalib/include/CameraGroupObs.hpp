#pragma once

#include "BoardObs.hpp"
#include "Object3DObs.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv_compat.hpp>
#include <stdio.h>

namespace McCalib {

class CameraGroup;

/**
 * @class CameraGroupObs
 *
 * @brief Aggregates object observations for one camera-group/frame pair.
 *
 * Stores all object observations associated with a camera group in a specific
 * frame and maintains one representative object pose per object id, expressed
 * in the camera-group reference frame.
 */
class CameraGroupObs final {
public:
  // Objects
  std::vector<int> object_idx_; // index of the visible 3d objects
  std::map<int, std::weak_ptr<Object3DObs>>
      object_observations_; // Objects stored
  std::map<int, std::array<double, 6>>
      object_pose_; // object pose wrt. the ref. cam of the group

  // Camera group
  int cam_group_idx_;
  std::weak_ptr<CameraGroup> cam_group_;

  bool quaternion_averaging_ =
      true; // use Quaternion Averaging or median for average rotation

  // Functions
  CameraGroupObs() = delete;

  /**
   * @brief Destroy the camera-group observation object.
   */
  ~CameraGroupObs();

  /**
   * @brief Construct a camera-group observation container.
   *
   * @param new_cam_group Camera group associated with this observation.
   * @param quaternion_averaging If true, average rotations with quaternion
   * averaging; otherwise use component-wise median fallback.
   */
  CameraGroupObs(const std::shared_ptr<CameraGroup> new_cam_group,
                 const bool quaternion_averaging);

  /**
   * @brief Insert one object observation into this camera-group observation.
   *
   * @param new_object_observation Object observation to register.
   */
  void insertObjectObservation(
      const std::shared_ptr<Object3DObs> new_object_observation);

  /**
   * @brief Compute one representative group pose per observed object id.
   *
   * If the reference camera observed the object, that pose is used.
   * Otherwise, the pose is estimated by averaging all available observations
   * for this object inside the group.
   */
  void computeObjectsPose();

  /**
   * @brief Get the pose of an object as Rodrigues rotation and translation.
   *
   * @param object_id Object id in this camera-group observation.
   * @param r_vec Output rotation vector (3x1, Rodrigues).
   * @param t_vec Output translation vector (3x1).
   */
  void getObjectPoseVec(const int object_id, cv::Mat &r_vec, cv::Mat &t_vec);

  /**
   * @brief Get the pose of an object as a 4x4 homogeneous transform.
   *
   * @param object_id Object id in this camera-group observation.
   * @return 4x4 transform from object frame to camera-group reference frame.
   */
  cv::Mat getObjectPoseMat(const int object_id);

  /**
   * @brief Set the pose of an object from a 4x4 homogeneous transform.
   *
   * @param pose 4x4 object pose in camera-group reference frame.
   * @param object_id Object id in this camera-group observation.
   */
  void setObjectPoseMat(const cv::Mat &pose, const int object_id);

  /**
   * @brief Set the pose of an object from Rodrigues and translation vectors.
   *
   * @param r_vec Rotation vector (3x1, Rodrigues).
   * @param t_vec Translation vector (3x1).
   * @param object_id Object id in this camera-group observation.
   */
  void setObjectPoseVec(const cv::Mat &r_vec, const cv::Mat &t_vec,
                        const int object_id);

  /**
   * @brief Get only the rotation component of an object pose.
   *
   * @param object_id Object id in this camera-group observation.
   * @return Rotation vector (3x1, Rodrigues).
   */
  cv::Mat getObjectRotVec(const int object_id);

  /**
   * @brief Get only the translation component of an object pose.
   *
   * @param object_id Object id in this camera-group observation.
   * @return Translation vector (3x1).
   */
  cv::Mat getObjectTransVec(const int object_id);

  /**
   * @brief Propagate group-level object poses back to contained observations.
   */
  void updateObjObsPose();
};

} // namespace McCalib