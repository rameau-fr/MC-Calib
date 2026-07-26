#include "CameraGroup.hpp"
#include "geometrytools.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv_compat.hpp>
#include <stdio.h>

#include "CameraGroupObs.hpp"

namespace McCalib {

/**
 * @brief Construct a camera-group observation container.
 *
 * @param new_cam_group Camera group associated with this observation.
 * @param quaternion_averaging If true, use quaternion averaging for rotations.
 */
CameraGroupObs::CameraGroupObs(const std::shared_ptr<CameraGroup> new_cam_group,
                               const bool quaternion_averaging) {
  cam_group_ = new_cam_group;
  cam_group_idx_ = new_cam_group->cam_group_idx_;
  quaternion_averaging_ = quaternion_averaging;
}

/**
 * @brief Insert one object observation into this camera-group observation.
 *
 * @param new_object_observation Observation to register.
 */
void CameraGroupObs::insertObjectObservation(
    const std::shared_ptr<Object3DObs> new_object_observation) {
  object_idx_.push_back(new_object_observation->object_3d_id_);
  object_observations_[object_observations_.size()] = new_object_observation;
}

CameraGroupObs::~CameraGroupObs() {}

/**
 * @brief Compute one representative group-frame pose per observed object id.
 *
 * If the reference camera sees the object, its pose is used directly.
 * Otherwise, the pose is estimated from the average of all available
 * observations in the group.
 */
void CameraGroupObs::computeObjectsPose() {

  // Find group of observation of the same object
  std::vector<int> object_unique_ind = object_idx_;
  std::vector<int>::iterator newEnd =
      std::unique(object_unique_ind.begin(), object_unique_ind.end());
  object_unique_ind.erase(newEnd, object_unique_ind.end());

  // Find indexes of object 3D obs with common index
  std::map<int, std::vector<int>>
      obj_obs_group; // key: object index // value: index in
                     // the vector of observation
  for (std::size_t i = 0; i < object_unique_ind.size(); i++) {
    for (std::size_t j = 0; j < object_idx_.size(); j++) {
      if (object_idx_[j] == object_unique_ind[i]) {
        obj_obs_group[object_unique_ind[i]].push_back(j);
      }
    }
  }

  // Compute the pose of the objects:
  for (const auto &it_obj_obs : obj_obs_group) {
    // if the reference camera has an observation, take its pose as initial
    // value
    bool flag_ref_cam = false;
    cv::Mat group_pose_r, group_pose_t;
    for (std::size_t i = 0; i < it_obj_obs.second.size(); i++) {
      auto cam_group_ptr = cam_group_.lock();
      auto obj_obs_ptr = object_observations_[it_obj_obs.second[i]].lock();
      if (cam_group_ptr && obj_obs_ptr &&
          cam_group_ptr->id_ref_cam_ == obj_obs_ptr->camera_id_) {
        flag_ref_cam = true;
        group_pose_r = obj_obs_ptr->getRotInGroupVec();
        group_pose_t = obj_obs_ptr->getTransInGroupVec();
      }
    }

    // if the reference camera has no visible observation, then take
    // the average of other observations
    if (flag_ref_cam == false) {
      std::vector<double> r1, r2, r3;
      cv::Mat average_translation = cv::Mat::zeros(3, 1, CV_64F);
      for (const auto &obj_obs_idx : it_obj_obs.second) {
        auto obj_obs_ptr = object_observations_[obj_obs_idx].lock();
        if (obj_obs_ptr) {
          const cv::Mat &R = obj_obs_ptr->getRotInGroupVec();
          r1.push_back(R.at<double>(0));
          r2.push_back(R.at<double>(1));
          r3.push_back(R.at<double>(2));
          average_translation += obj_obs_ptr->getTransInGroupVec();
        }
      }
      // Average version
      group_pose_t = average_translation / it_obj_obs.second.size();
      group_pose_r = getAverageRotation(r1, r2, r3, quaternion_averaging_);
    }

    // set the pose and update the observation
    setObjectPoseVec(group_pose_r, group_pose_t, it_obj_obs.first);
    // update the object observations
    for (std::size_t i = 0; i < it_obj_obs.second.size(); i++) {
      auto obj_obs_ptr = object_observations_[it_obj_obs.second[i]].lock();
      if (obj_obs_ptr)
        obj_obs_ptr->setPoseInGroupVec(group_pose_r, group_pose_t);
    }
  }
}

/**
 * @brief Get the "object_id" object pose (vector)
 *
 * @param r_vec return (by reference) Rodrigues rotation vector
 * @param t_vec return (by reference) translation vector
 * @param object_id object index of interest in the cam group
 */
void CameraGroupObs::getObjectPoseVec(const int object_id, cv::Mat &r_vec,
                                      cv::Mat &t_vec) {
  cv::Mat rot_v = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat trans_v = cv::Mat::zeros(3, 1, CV_64F);
  rot_v.at<double>(0) = object_pose_[object_id][0];
  rot_v.at<double>(1) = object_pose_[object_id][1];
  rot_v.at<double>(2) = object_pose_[object_id][2];
  trans_v.at<double>(0) = object_pose_[object_id][3];
  trans_v.at<double>(1) = object_pose_[object_id][4];
  trans_v.at<double>(2) = object_pose_[object_id][5];
  rot_v.copyTo(r_vec);
  trans_v.copyTo(t_vec);
}

/**
 * @brief Get the "object_id" object pose
 *
 * @param object_id index of the camera of interest in the group
 *
 * @return Pose matrix of the object in the group
 */
cv::Mat CameraGroupObs::getObjectPoseMat(const int object_id) {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getObjectPoseVec(object_id, r_vec, t_vec);
  cv::Mat pose = RVecT2Proj(r_vec, t_vec);
  return pose;
}

/**
 * @brief Set "object_id" object pose in the data structure
 *
 * @param pose 4x4 pose to set
 * @param object_id object index of interest in the group
 */
void CameraGroupObs::setObjectPoseMat(const cv::Mat &pose,
                                      const int object_id) {
  cv::Mat r_vec, t_vec;
  Proj2RT(pose, r_vec, t_vec);
  object_pose_[object_id] = {r_vec.at<double>(0), r_vec.at<double>(1),
                             r_vec.at<double>(2), t_vec.at<double>(0),
                             t_vec.at<double>(1), t_vec.at<double>(2)};
}

/**
 * @brief Set "object_id" object pose (Vector) in the data structure
 *
 * @param r_vec Rodrigues rotation vector to set
 * @param t_vec translation vector to set
 * @param object_id object index of interest in the group
 */
void CameraGroupObs::setObjectPoseVec(const cv::Mat &r_vec,
                                      const cv::Mat &t_vec,
                                      const int object_id) {
  object_pose_[object_id] = {r_vec.at<double>(0), r_vec.at<double>(1),
                             r_vec.at<double>(2), t_vec.at<double>(0),
                             t_vec.at<double>(1), t_vec.at<double>(2)};
}

/**
 * @brief Get rotation vector of the object "object_id" in the group
 *
 * @param object_id object index of interest in the group
 *
 * @return 1x3 Rodrigues vector of the camera "object_id"
 */
cv::Mat CameraGroupObs::getObjectRotVec(const int object_id) {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getObjectPoseVec(object_id, r_vec, t_vec);
  return r_vec;
}

/**
 * @brief Get translation vector of the object "object_id" in the group
 *
 * @param object_id object index of interest in the group
 *
 * @return 1x3 translation vector of the camera "object_id"
 */
cv::Mat CameraGroupObs::getObjectTransVec(const int object_id) {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getObjectPoseVec(object_id, r_vec, t_vec);
  return t_vec;
}

/**
 * @brief Propagate stored group-frame object poses back to each observation.
 *
 */
void CameraGroupObs::updateObjObsPose() {
  for (const auto &obj_obs : object_observations_) {
    auto obj_obs_ptr = obj_obs.second.lock();
    if (obj_obs_ptr) {
      obj_obs_ptr->setPoseInGroupVec(
          getObjectRotVec(obj_obs_ptr->object_3d_id_),
          getObjectTransVec(obj_obs_ptr->object_3d_id_));
    }
  }
}

} // namespace McCalib