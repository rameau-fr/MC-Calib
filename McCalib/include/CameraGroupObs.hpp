#pragma once

#include "BoardObs.hpp"
#include "Object3DObs.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

namespace McCalib {

class CameraGroup;

/**
 * @class CameraGroupObs
 *
 * @brief Contains information related to the camera group observation
 *
 * - 3D object observations from this camera group observation
 * - camera group corresponding to this observation
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
  ~CameraGroupObs();
  CameraGroupObs(const std::shared_ptr<CameraGroup> new_cam_group,
                 const bool quaternion_averaging);
  void insertObjectObservation(
      const std::shared_ptr<Object3DObs> new_object_observation);
  void computeObjectsPose();
  void getObjectPoseVec(const int object_id, cv::Mat &r_vec, cv::Mat &t_vec);
  cv::Mat getObjectPoseMat(const int object_id);
  void setObjectPoseMat(const cv::Mat &pose, const int object_id);
  void setObjectPoseVec(const cv::Mat &r_vec, const cv::Mat &t_vec,
                        const int object_id);
  cv::Mat getObjectRotVec(const int object_id);
  cv::Mat getObjectTransVec(const int object_id);
  void updateObjObsPose();
};

} // namespace McCalib