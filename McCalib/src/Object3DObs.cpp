#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "Camera.hpp"
#include "Object3DObs.hpp"
#include "geometrytools.hpp"
#include "logger.h"

namespace McCalib {

/**
 * @brief initialize the object in the observation (what board is observed)
 *
 * @param new_obj_obs object to be added
 * @param object_idx object index
 */
Object3DObs::Object3DObs(std::shared_ptr<Object3D> obj_obs,
                         const int object_idx)
    : object_3d_(obj_obs), object_3d_id_(object_idx) {}

/**
 * @brief Insert a new board in the object
 *
 * @note Do not forget to initialize the object_3d_ first.
 *
 * @param new_board pointer to the new board observation
 */
void Object3DObs::insertNewBoardObs(std::shared_ptr<BoardObs> new_board_obs) {
  board_id_.push_back(new_board_obs->board_id_);
  cam_ = new_board_obs->cam_;
  camera_id_ = new_board_obs->camera_id_;
  frame_id_ = new_board_obs->frame_id_;
  board_observations_[board_observations_.size()] = new_board_obs;

  // push the 2d pts and index
  const size_t num_points = new_board_obs->pts_2d_.size();
  pts_2d_.reserve(num_points);
  pts_id_.reserve(num_points);
  for (std::size_t i = 0; i < num_points; i++) {
    // Convert the index from the board to the object
    std::pair<int, int> board_id_pts_id =
        std::make_pair(new_board_obs->board_id_, new_board_obs->charuco_id_[i]);
    auto object_3d_ptr = object_3d_.lock();
    if (object_3d_ptr) {
      pts_2d_.emplace_back(new_board_obs->pts_2d_[i]);
      pts_id_.emplace_back(object_3d_ptr->pts_board_2_obj_[board_id_pts_id]);
    }
  }
}

/**
 * @brief Get the pose of the object w.r.t. the camera
 *
 * @param r_vec return by reference Rodrigues rotation vector
 * @param t_vec return by reference the translation vector
 */
void Object3DObs::getPoseVec(cv::Mat &r_vec, cv::Mat &t_vec) const {
  cv::Mat rot_v = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat trans_v = cv::Mat::zeros(3, 1, CV_64F);
  rot_v.at<double>(0) = pose_[0];
  rot_v.at<double>(1) = pose_[1];
  rot_v.at<double>(2) = pose_[2];
  trans_v.at<double>(0) = pose_[3];
  trans_v.at<double>(1) = pose_[4];
  trans_v.at<double>(2) = pose_[5];
  rot_v.copyTo(r_vec);
  trans_v.copyTo(t_vec);
}

/**
 * @brief Get the matrix pose of the object w.r.t. the camera
 *
 * @return 4x4 pose matrix
 */
cv::Mat Object3DObs::getPoseMat() const {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getPoseVec(r_vec, t_vec);
  cv::Mat pose = RVecT2Proj(r_vec, t_vec);
  return pose;
}

/**
 * @brief Get rotation vector of the object w.r.t. the camera
 *
 * @return 1x3 Rodrigues vector
 */
cv::Mat Object3DObs::getRotVec() const {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getPoseVec(r_vec, t_vec);
  return r_vec;
}

/**
 * @brief Get translation vector of the object w.r.t. the camera
 *
 * @return 1x3 translation vector
 */
cv::Mat Object3DObs::getTransVec() const {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getPoseVec(r_vec, t_vec);
  return t_vec;
}

/**
 * @brief Set pose of the object w.r.t. the camera from pose matrix
 *
 * @param pose 4x4 pose matrix
 */
void Object3DObs::setPoseMat(cv::Mat pose) {
  cv::Mat r_vec, t_vec;
  Proj2RT(pose, r_vec, t_vec);
  pose_ = {r_vec.at<double>(0), r_vec.at<double>(1), r_vec.at<double>(2),
           t_vec.at<double>(0), t_vec.at<double>(1), t_vec.at<double>(2)};
}

/**
 * @brief Set pose of the object w.r.t. the cam from rotation and translation
 * vectors
 *
 * @param r_vec Rodrigues rotation vector
 * @param t_vec translation vector
 */
void Object3DObs::setPoseVec(const cv::Mat r_vec, const cv::Mat t_vec) {
  pose_ = {r_vec.at<double>(0), r_vec.at<double>(1), r_vec.at<double>(2),
           t_vec.at<double>(0), t_vec.at<double>(1), t_vec.at<double>(2)};
}

/**
 * @brief Set the pose of the object in the referential of the group observing
 * the object
 *
 * @param pose 4x4 pose matrix
 */
void Object3DObs::setPoseInGroupMat(cv::Mat pose) {
  cv::Mat r_vec, t_vec;
  Proj2RT(pose, r_vec, t_vec);
  group_pose_ = {r_vec.at<double>(0), r_vec.at<double>(1), r_vec.at<double>(2),
                 t_vec.at<double>(0), t_vec.at<double>(1), t_vec.at<double>(2)};
}

/**
 * @brief Set the pose of the object in the referential of the group observing
 * the object (from rotation / translation vectors)
 *
 * @param r_vec Rodrigues rotation vector
 * @param t_vec translation vector
 */
void Object3DObs::setPoseInGroupVec(const cv::Mat r_vec, const cv::Mat t_vec) {
  group_pose_ = {r_vec.at<double>(0), r_vec.at<double>(1), r_vec.at<double>(2),
                 t_vec.at<double>(0), t_vec.at<double>(1), t_vec.at<double>(2)};
}

/**
 * @brief Get the pose of the object w.r.t. the camera group
 *
 * @param r_vec return by reference Rodrigues rotation vector
 * @param t_vec return by reference the translation vector
 */
void Object3DObs::getPoseInGroupVec(cv::Mat &r_vec, cv::Mat &t_vec) const {
  cv::Mat rot_v = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat trans_v = cv::Mat::zeros(3, 1, CV_64F);
  rot_v.at<double>(0) = group_pose_[0];
  rot_v.at<double>(1) = group_pose_[1];
  rot_v.at<double>(2) = group_pose_[2];
  trans_v.at<double>(0) = group_pose_[3];
  trans_v.at<double>(1) = group_pose_[4];
  trans_v.at<double>(2) = group_pose_[5];
  rot_v.copyTo(r_vec);
  trans_v.copyTo(t_vec);
}

/**
 * @brief Get the matrix pose of the object w.r.t. the group of camera
 *
 * @return 4x4 pose matrix
 */
cv::Mat Object3DObs::getPoseInGroupMat() const {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getPoseInGroupVec(r_vec, t_vec);
  cv::Mat pose = RVecT2Proj(r_vec, t_vec);
  return pose;
}

/**
 * @brief Get rotation vector of the object w.r.t. the camera group
 *
 * @return 1x3 Rodrigues vector
 */
cv::Mat Object3DObs::getRotInGroupVec() const {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getPoseInGroupVec(r_vec, t_vec);
  return r_vec;
}

/**
 * @brief Get translation vector of the object w.r.t. the camera
 *
 * @return 1x3 translation vector
 */
cv::Mat Object3DObs::getTransInGroupVec() const {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getPoseInGroupVec(r_vec, t_vec);
  return t_vec;
}

/**
 * @brief Estimate the pose of the 3D object w.r.t. to the camera
 *
 * It using a PnP RANSAC algorithm.
 *
 */
void Object3DObs::estimatePose(const float ransac_thresh,
                               const int ransac_iterations) {
  std::vector<cv::Point3f> object_pts_temp;
  object_pts_temp.reserve(pts_id_.size());
  for (const auto &pt_id : pts_id_) {
    auto object_3d_ptr = object_3d_.lock();
    if (object_3d_ptr)
      object_pts_temp.emplace_back(object_3d_ptr->pts_3d_[pt_id]);
  }

  // Estimate the pose using a RANSAC
  cv::Mat r_vec(1, 3, CV_64F);
  cv::Mat t_vec(1, 3, CV_64F);
  std::shared_ptr<Camera> cam_ptr = cam_.lock();
  if (cam_ptr) {
    cv::Mat inliers = ransacP3PDistortion(
        object_pts_temp, pts_2d_, cam_ptr->getCameraMat(),
        cam_ptr->getDistortionVectorVector(), r_vec, t_vec, ransac_thresh,
        ransac_iterations, cam_ptr->distortion_model_);
    LOG_DEBUG << "Trans :: " << t_vec << "       Rot :: " << r_vec;
    LOG_DEBUG << "input pts 3D :: " << object_pts_temp.size();
    LOG_DEBUG << "Inliers :: " << inliers.rows;
    r_vec.convertTo(r_vec, CV_64F);
    t_vec.convertTo(t_vec, CV_64F);
    setPoseVec(r_vec, t_vec);
  }
}

/**
 * @brief Compute the reprojection error for this object observation
 *
 * @return mean reprojection error for this observation
 */
float Object3DObs::computeReprojectionError() const {
  float sum_err_object = 0.0;
  std::vector<cv::Point3f> object_pts_temp;
  object_pts_temp.reserve(pts_id_.size());
  for (const auto &pt_id : pts_id_) {
    auto object_3d_ptr = object_3d_.lock();
    if (object_3d_ptr)
      object_pts_temp.emplace_back(object_3d_ptr->pts_3d_[pt_id]);
  }

  // Project the 3D pts on the image
  std::vector<cv::Point2f> repro_pts;
  std::vector<float> error_object_vec;
  std::shared_ptr<Camera> cam_ptr = cam_.lock();
  if (cam_ptr) {
    projectPointsWithDistortion(object_pts_temp, getRotVec(), getTransVec(),
                                cam_ptr->getCameraMat(),
                                cam_ptr->getDistortionVectorVector(), repro_pts,
                                cam_ptr->distortion_model_);
    for (std::size_t j = 0; j < repro_pts.size(); j++) {
      float rep_err = std::sqrt(std::pow((pts_2d_[j].x - repro_pts[j].x), 2) +
                                std::pow((pts_2d_[j].y - repro_pts[j].y), 2));
      error_object_vec.push_back(rep_err);
      sum_err_object += rep_err;
      // if (rep_err > 6.0)
      // LOG_WARNING << "LARGE REPROJECTION ERROR ::: " << rep_err  ;
    }
    LOG_DEBUG << "Frame :: " << this->frame_id_
              << "  object :: " << this->object_3d_id_ << "  --- Mean Error ::"
              << sum_err_object / error_object_vec.size()
              << "  Nb pts :: " << error_object_vec.size();
  }
  // return mean error for the board
  return error_object_vec.size() > 0 ? sum_err_object / error_object_vec.size()
                                     : sum_err_object;
}

} // namespace McCalib