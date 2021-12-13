#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "BoardObs.hpp"
#include "Camera.hpp"
#include "geometrytools.hpp"
#include "logger.h"

BoardObs::BoardObs() {}

/**
 * @brief Initialize the board observation
 *
 * @param camera_id camera index of the camera observing the board
 * @param frame_id frame in which the board is visible
 * @param board_id which board is concerned
 * @param pts_2d 2D points observed
 * @param charuco_id 3D points index
 * @param cam camera observing the board
 * @param board_3d 3D board corresponding to the observation
 */
void BoardObs::init(const int camera_id, const int frame_id, const int board_id,
                    const std::vector<cv::Point2f> pts_2d,
                    const std::vector<int> charuco_id,
                    std::shared_ptr<Camera> cam,
                    std::shared_ptr<Board> board_3d) {
  frame_id_ = frame_id;
  camera_id_ = camera_id;
  board_id_ = board_id;
  pts_2d_ = pts_2d;
  charuco_id_ = charuco_id;
  cam_ = cam;
  board_3d_ = board_3d;
}

/**
 * @brief Get pose vector of the observed board
 *
 * @param r_vec Rodrigues rotation vector passed by reference
 * @param t_vec translation vector passed by reference
 */
void BoardObs::getPoseVec(cv::Mat &r_vec, cv::Mat &t_vec) const {
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
 * @brief Get pose of the observed board
 *
 * @return 4x4 board pose w.r.t. to the camera
 */
cv::Mat BoardObs::getPoseMat() const {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getPoseVec(r_vec, t_vec);
  cv::Mat pose = RVecT2Proj(r_vec, t_vec);
  return pose;
}

/**
 * @brief Get the rotation Rodrigues vector of this observation
 *
 * @return 1x3 rotation vector w.r.t. the camera observing the board
 */
cv::Mat BoardObs::getRotVec() const {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getPoseVec(r_vec, t_vec);
  return r_vec;
}

/**
 * @brief Get the translation vector of this observation
 *
 * @return 1x3 translation vector w.r.t. the camera observing the board
 */
cv::Mat BoardObs::getTransVec() const {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getPoseVec(r_vec, t_vec);
  return t_vec;
}

/**
 * @brief Set the board pose w.r.t. the camera from a pose matrix
 *
 * @param pose 4x4 pose matrix
 */
void BoardObs::setPoseMat(const cv::Mat pose) {
  cv::Mat r_vec, t_vec;
  Proj2RT(pose, r_vec, t_vec);
  pose_[0] = r_vec.at<double>(0);
  pose_[1] = r_vec.at<double>(1);
  pose_[2] = r_vec.at<double>(2);
  pose_[3] = t_vec.at<double>(0);
  pose_[4] = t_vec.at<double>(1);
  pose_[5] = t_vec.at<double>(2);
}

/**
 * @brief Set the board pose wrt. the camera from rotation and translation
 * vector
 *
 * @param r_vec Rodrigues rotation vector
 * @param t_vec translation vector
 */
void BoardObs::setPoseVec(const cv::Mat r_vec, const cv::Mat t_vec) {
  pose_[0] = r_vec.at<double>(0);
  pose_[1] = r_vec.at<double>(1);
  pose_[2] = r_vec.at<double>(2);
  pose_[3] = t_vec.at<double>(0);
  pose_[4] = t_vec.at<double>(1);
  pose_[5] = t_vec.at<double>(2);
}

/**
 * @brief Estimate the pose of this board w.r.t. the camera observing it.
 *
 * It uses PnP RANSAC under the hood.
 *
 * @param ransac_thresh RANSAC threshold in pixels to remove strong outliers
 *
 * @todo possible division by zero on the return
 */
void BoardObs::estimatePose(const float ransac_thresh) {
  std::vector<cv::Point3f> board_pts_temp;
  for (int i = 0; i < charuco_id_.size(); i++) {
    std::shared_ptr<Board> board_3d_ptr = board_3d_.lock();
    if (board_3d_ptr)
      board_pts_temp.push_back(board_3d_ptr->pts_3d_[charuco_id_[i]]);
  }

  // Estimate the pose using a RANSAC
  cv::Mat r_vec, t_vec;
  std::shared_ptr<Camera> cam_ptr = cam_.lock();
  if (cam_ptr) {
    cv::Mat inliers = ransacP3PDistortion(
        board_pts_temp, pts_2d_, cam_ptr->getCameraMat(),
        cam_ptr->getDistortionVectorVector(), r_vec, t_vec, ransac_thresh, 0.99,
        1000, true, cam_ptr->distortion_model_);
    LOG_DEBUG << "Trans :: " << t_vec << "       Rot :: " << r_vec;
    LOG_DEBUG << "input pts 3D :: " << board_pts_temp.size();
    LOG_DEBUG << "Inliers :: " << inliers.rows;
    r_vec.convertTo(r_vec, CV_64F);
    t_vec.convertTo(t_vec, CV_64F);
    setPoseVec(r_vec, t_vec);

    // if the number of inlier is too low the board is not valid
    std::shared_ptr<Board> board_3d_ptr = board_3d_.lock();
    if (board_3d_ptr && inliers.rows < 4) {
      valid_ = false;
    }

    // remove outliers
    std::vector<cv::Point2f> new_pts_vec;
    std::vector<int> new_charuco_id;
    for (int i = 0; i < inliers.rows; i++) {
      new_pts_vec.push_back(pts_2d_[inliers.at<int>(i)]);
      new_charuco_id.push_back(charuco_id_[inliers.at<int>(i)]);
    }
    pts_2d_ = new_pts_vec;
    charuco_id_ = new_charuco_id;
  }
}

float BoardObs::computeReprojectionError() {
  float sum_err_board = 0;
  std::vector<cv::Point3f> board_pts_temp;
  for (int i = 0; i < charuco_id_.size(); i++) {
    std::shared_ptr<Board> board_3d_ptr = board_3d_.lock();
    if (board_3d_ptr)
      board_pts_temp.push_back(board_3d_ptr->pts_3d_[charuco_id_[i]]);
  }
  // Project the 3D pts on the image
  std::vector<cv::Point2f> repro_pts;
  std::vector<float> error_board_vec;
  std::shared_ptr<Camera> cam_ptr = cam_.lock();
  if (cam_ptr) {
    projectPointsWithDistortion(board_pts_temp, getRotVec(), getTransVec(),
                                cam_ptr->getCameraMat(),
                                cam_ptr->getDistortionVectorVector(), repro_pts,
                                cam_ptr->distortion_model_);
    for (int j = 0; j < repro_pts.size(); j++) {
      float rep_err = sqrt(pow((pts_2d_[j].x - repro_pts[j].x), 2) +
                           pow((pts_2d_[j].y - repro_pts[j].y), 2));
      error_board_vec.push_back(rep_err);
      sum_err_board += rep_err;
      // if (rep_err > 6.0)
      // LOG_WARNING << "LARGE REPROJECTION ERROR ::: " << rep_err  ;
    }

    if ((sum_err_board / error_board_vec.size()) > 10) {
      LOG_INFO << "High error detected in this board pose estimation";
      LOG_INFO << "Frame :: " << this->frame_id_
               << "  Boards :: " << this->board_id_ << "  --- Mean Error ::"
               << sum_err_board / error_board_vec.size()
               << "  Nb pts :: " << error_board_vec.size();
    }
  }

  // return mean error for the board
  return sum_err_board / error_board_vec.size();
}
