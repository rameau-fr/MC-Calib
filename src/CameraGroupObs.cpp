#include "CameraGroup.hpp"
#include "geometrytools.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "CameraGroupObs.hpp"

CameraGroupObs::CameraGroupObs() {}

/**
 * @brief Associate this observation with its respective camera group
 *
 * @param new_cam_group camera group to be added
 */
void CameraGroupObs::insertCameraGroup(
    std::shared_ptr<CameraGroup> new_cam_group) {
  cam_group_ = new_cam_group;
  cam_group_idx_ = new_cam_group->cam_group_idx_;
}

/**
 * @brief Insert a new object observation into the camera group obs
 *
 * @param new_object_observation observation to be added
 */
void CameraGroupObs::insertObjectObservation(
    std::shared_ptr<Object3DObs> new_object_observation) {
  object_idx_.push_back(new_object_observation->object_3d_id_);
  object_observations_[object_observations_.size()] = new_object_observation;
}

CameraGroupObs::~CameraGroupObs() {
  for (auto const &item : object_pose_)
    delete[] item.second;
  object_pose_.clear();
}

/**
 * @brief Compute pose of object in the camera obs
 *
 * @todo remove dead code
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
  for (int i = 0; i < object_unique_ind.size(); i++) {
    for (int j = 0; j < object_idx_.size(); j++) {
      if (object_idx_[j] == object_unique_ind[i]) {
        obj_obs_group[object_unique_ind[i]].push_back(j);
      }
    }
  }

  // Compute the pose of the objects:
  for (std::map<int, std::vector<int>>::iterator it_obj_obs =
           obj_obs_group.begin();
       it_obj_obs != obj_obs_group.end(); it_obj_obs++) {

    // if the reference camera has an observation, take its pose as initial
    // value
    bool flag_ref_cam = false;
    cv::Mat group_pose_r, group_pose_t;
    for (int i = 0; i < it_obj_obs->second.size(); i++) {
      if (cam_group_.lock()->id_ref_cam_ ==
          object_observations_[it_obj_obs->second[i]].lock()->camera_id_) {
        flag_ref_cam = true;
        group_pose_r = object_observations_[it_obj_obs->second[i]]
                           .lock()
                           ->getRotInGroupVec();
        group_pose_t = object_observations_[it_obj_obs->second[i]]
                           .lock()
                           ->getTransInGroupVec();
      }
    }

    if (flag_ref_cam ==
        false) // if the reference camera has no visible observation, then take
               // the average of other observations
    {
      cv::Mat average_rotation = cv::Mat::zeros(3, 1, CV_64F);
      cv::Mat average_translation = cv::Mat::zeros(3, 1, CV_64F);
      for (int i = 0; i < it_obj_obs->second.size(); i++) {
        average_rotation += object_observations_[it_obj_obs->second[i]]
                                .lock()
                                ->getRotInGroupVec();
        average_translation += object_observations_[it_obj_obs->second[i]]
                                   .lock()
                                   ->getTransInGroupVec();
      }
      // Average version
      group_pose_t = average_translation / it_obj_obs->second.size();
      group_pose_r = average_rotation / it_obj_obs->second.size();

      // One board version (arbitrary)
      // group_pose_r =
      // object_observations_[it_obj_obs->second[0]]->getRotInGroupVec();
      // group_pose_t =
      // object_observations_[it_obj_obs->second[0]]->getTransInGroupVec();
    }

    // set the pose and update the observation
    setObjectPoseVec(group_pose_r, group_pose_t, it_obj_obs->first);
    // update the object observations
    for (int i = 0; i < it_obj_obs->second.size(); i++) {
      object_observations_[it_obj_obs->second[i]].lock()->setPoseInGroupVec(
          group_pose_r, group_pose_t);
      // Get the pose in the camera referential (no need here, maybe...)
      /*cv::Mat pose_cam_in_group =
      cam_group_->getCameraPoseMat(object_observations_[it_obj_obs->second[i]]->camera_id_);
      cv::Mat pose_object_in_group =
      object_observations_[it_obj_obs->second[i]]->getPoseInGroupMat(); Mat
      trans = pose_cam_in_group.inv()*pose_object_in_group;
      object_observations_[it_obj_obs->second[i]]->setPoseMat(trans);*/
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
void CameraGroupObs::getObjectPoseVec(cv::Mat &r_vec, cv::Mat &t_vec,
                                      int object_id) {
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
cv::Mat CameraGroupObs::getObjectPoseMat(int object_id) {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getObjectPoseVec(r_vec, t_vec, object_id);
  cv::Mat pose = RVecT2Proj(r_vec, t_vec);
  return pose;
}

/**
 * @brief Set "object_id" object pose in the data structure
 *
 * @param pose 4x4 pose to set
 * @param object_id object index of interest in the group
 */
void CameraGroupObs::setObjectPoseMat(cv::Mat pose, int object_id) {
  object_pose_[object_id] = new double[6];
  cv::Mat r_vec, t_vec;
  Proj2RT(pose, r_vec, t_vec);
  object_pose_[object_id][0] = r_vec.at<double>(0);
  object_pose_[object_id][1] = r_vec.at<double>(1);
  object_pose_[object_id][2] = r_vec.at<double>(2);
  object_pose_[object_id][3] = t_vec.at<double>(0);
  object_pose_[object_id][4] = t_vec.at<double>(1);
  object_pose_[object_id][5] = t_vec.at<double>(2);
}

/**
 * @brief Set "object_id" object pose (Vector) in the data structure
 *
 * @param r_vec Rodrigues rotation vector to set
 * @param t_vec translation vector to set
 * @param object_id object index of interest in the group
 */
void CameraGroupObs::setObjectPoseVec(cv::Mat r_vec, cv::Mat t_vec,
                                      int object_id) {
  object_pose_[object_id] = new double[6];
  object_pose_[object_id][0] = r_vec.at<double>(0);
  object_pose_[object_id][1] = r_vec.at<double>(1);
  object_pose_[object_id][2] = r_vec.at<double>(2);
  object_pose_[object_id][3] = t_vec.at<double>(0);
  object_pose_[object_id][4] = t_vec.at<double>(1);
  object_pose_[object_id][5] = t_vec.at<double>(2);
}

/**
 * @brief Get rotation vector of the object "object_id" in the group
 *
 * @param object_id object index of interest in the group
 *
 * @return 1x3 Rodrigues vector of the camera "object_id"
 */
cv::Mat CameraGroupObs::getObjectRotVec(int object_id) {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getObjectPoseVec(r_vec, t_vec, object_id);
  return r_vec;
}

/**
 * @brief Get translation vector of the object "object_id" in the group
 *
 * @param object_id object index of interest in the group
 *
 * @return 1x3 translation vector of the camera "object_id"
 */
cv::Mat CameraGroupObs::getObjectTransVec(int object_id) {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getObjectPoseVec(r_vec, t_vec, object_id);
  return t_vec;
}

/**
 * @brief Update the object observation pose
 *
 */
void CameraGroupObs::updateObjObsPose() {
  for (int i = 0; i < object_idx_.size(); i++) {
    object_observations_[i].lock()->setPoseInGroupVec(
        getObjectRotVec(object_observations_[i].lock()->object_3d_id_),
        getObjectTransVec(object_observations_[i].lock()->object_3d_id_));
  }
}
