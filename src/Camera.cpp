#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "Camera.hpp"
#include "OptimizationCeres.h"
#include "logger.h"

Camera::Camera(const int cam_idx, const int distortion_model)
{
  cam_idx_ = cam_idx;
  distortion_model_ = distortion_model;
}

/**
 * @brief Get camera matrix (K)
 *
 * @return 3x3 camera matrix
 */
cv::Mat Camera::getCameraMat() const {
  cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
  camera_matrix.at<double>(0, 0) = intrinsics_[0];
  camera_matrix.at<double>(1, 1) = intrinsics_[1]; // focal
  camera_matrix.at<double>(0, 2) = intrinsics_[2];
  camera_matrix.at<double>(1, 2) = intrinsics_[3]; // principal pt
  camera_matrix.at<double>(2, 2) = 1;
  return camera_matrix;
}

/**
 * @brief Set the camera matrix in the intrinsics vector
 *
 * @param camera_matrix 3x3 camera matrix (K)
 */
void Camera::setCameraMat(const cv::Mat camera_matrix) {
  intrinsics_[0] = camera_matrix.at<double>(0, 0);
  intrinsics_[1] = camera_matrix.at<double>(1, 1); // focal
  intrinsics_[2] = camera_matrix.at<double>(0, 2);
  intrinsics_[3] = camera_matrix.at<double>(1, 2); // principal point
}

/**
 * @brief Set the distortion parameters in the intrinsics vector
 *
 * Two distortion models supported:
 *  - Brown: 1x5 distortion vector (radial_1, radial_2, tangential_1,
 * tangential_2, radial_3)
 *  - Kannala: 1x4 distortion vector
 *
 * @param distortion_vector
 */
void Camera::setDistortionVector(const cv::Mat distortion_vector) {
  if (distortion_model_ == 0) {
    intrinsics_[4] = distortion_vector.at<double>(0);
    intrinsics_[5] = distortion_vector.at<double>(1);
    intrinsics_[8] = distortion_vector.at<double>(4); // radial
    intrinsics_[6] = distortion_vector.at<double>(2);
    intrinsics_[7] = distortion_vector.at<double>(3); // tangential
  }
  if (distortion_model_ == 1) {
    intrinsics_[4] = distortion_vector.at<double>(0);
    intrinsics_[5] = distortion_vector.at<double>(1);
    intrinsics_[8] = distortion_vector.at<double>(2);
    intrinsics_[6] = distortion_vector.at<double>(3);
  }
}

/**
 * @brief Get the distortion parameters
 *
 * @return distortion vector (Brown or Kannala) following OpenCV conventions
 *
 * @todo early exit, possible memory leak due to unsupported distorion model
 */
cv::Mat Camera::getDistortionVectorVector() const {
  if (distortion_model_ == 0) {
    cv::Mat distortion_vector = cv::Mat(1, 5, CV_64F, cv::Scalar(0));
    distortion_vector.at<double>(0) = intrinsics_[4];
    distortion_vector.at<double>(1) = intrinsics_[5];
    distortion_vector.at<double>(4) = intrinsics_[8];
    distortion_vector.at<double>(2) = intrinsics_[6];
    distortion_vector.at<double>(3) = intrinsics_[7];
    return distortion_vector;
  }

  else if (distortion_model_ == 1) {
    cv::Mat distortion_vector = cv::Mat(1, 4, CV_64F, cv::Scalar(0));
    distortion_vector.at<double>(0) = intrinsics_[4];
    distortion_vector.at<double>(1) = intrinsics_[5];
    distortion_vector.at<double>(4) = intrinsics_[8];
    distortion_vector.at<double>(2) = intrinsics_[6];
    return distortion_vector;
  }

  else {
    LOG_FATAL << "Unsupported distortion model";
    std::exit(0);
  }
}

/**
 * @brief Get intrinsics
 *
 * Parameters are passed as reference and set within the function.
 *
 * @param camera_matrix camera matrix 3x3
 * @param distortion_vector distortion parameters 1x[5 or 4]
 */
void Camera::getIntrinsics(cv::Mat &camera_matrix, cv::Mat &distortion_vector) {
  camera_matrix = getCameraMat();
  distortion_vector = getDistortionVectorVector();
}

/**
 * @brief Set the camera matrix and distorsion in the intrinsics vector
 *
 * @param camera_matrix 3x3 K matrix
 * @param distortion_vector 1x5 distortion vector (following OpenCV)
 */
void Camera::setIntrinsics(const cv::Mat camera_matrix,
                           const cv::Mat distortion_vector) {
  setCameraMat(camera_matrix);
  setDistortionVector(distortion_vector);
}

/**
 * @brief Insert new board observation in the data structure
 *
 * @param new_board pointer to the new board
 */
void Camera::insertNewBoard(std::shared_ptr<BoardObs> new_board) {
  board_observations_[board_observations_.size()] = new_board;
  vis_board_idx_.push_back(new_board->board_id_);
}

/**
 * @brief Insert new frame in the data structure
 *
 * @param new_frame pointer to the new frame to include
 */
void Camera::insertNewFrame(std::shared_ptr<Frame> new_frame) {
  frames_[new_frame->frame_idx_] = new_frame;
}

/**
 * @brief Insert new object observation in the data structure
 *
 * @param new_object pointer to the new object to be inserted
 */
void Camera::insertNewObject(std::shared_ptr<Object3DObs> new_object) {
  object_observations_[object_observations_.size()] = new_object;
  vis_object_idx_.push_back(new_object->object_3d_id_);
}

/**
 * @brief Initialize the calibration parameters using a subset of images
 *
 * A subset of images is used since the OpenCV function is slow.
 *
 * @todo remove dead code
 */
void Camera::initializeCalibration() {
  LOG_INFO << "NB of board available in this camera :: "
           << board_observations_.size();
  LOG_INFO << "NB of frames where this camera saw a board :: "
           << frames_.size();

  // Subsample the total number of images (because the OpenCV function is
  // significantly too slow...)
  std::vector<int> indbv;
  for (int i = 0; i < board_observations_.size(); i++) {
    indbv.push_back(i);
  }
  std::srand(unsigned(std::time(0)));
  std::vector<int> shuffled_board_ind;
  for (unsigned int i = 0; i < indbv.size(); ++i)
    shuffled_board_ind.push_back(i);
  random_shuffle(shuffled_board_ind.begin(), shuffled_board_ind.end());

  // Prepare list of 2D-3D correspondences
  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;
  // nb of boards used for the initial estimation of intrinsic parameters
  //(at least 50 boards for perspective)
  int nb_board_est = 50;
  if (distortion_model_ == 1) {
    nb_board_est =
        500; // fisheye is more sensitive and require more img (but faster)
  }
  if (indbv.size() < nb_board_est)
    nb_board_est = indbv.size();

  for (int i = 0; i < nb_board_est; i++) {
    std::shared_ptr<BoardObs> board_obs_temp =
        board_observations_[indbv[shuffled_board_ind[i]]].lock();
    if (board_obs_temp) {
      img_points.push_back(board_obs_temp->pts_2d_);
      std::vector<int> corners_idx_temp = board_obs_temp->charuco_id_;
      std::vector<cv::Point3f> pts_3d_temp;
      std::shared_ptr<Board> board_3d_ptr = board_obs_temp->board_3d_.lock();
      if (board_3d_ptr) {
        for (int k = 0; k < corners_idx_temp.size(); k++)
          pts_3d_temp.push_back(board_3d_ptr->pts_3d_[corners_idx_temp[k]]);
      }
      obj_points.push_back(pts_3d_temp);
    }
  }

  // Calibrate using OpenCV
  cv::Mat camera_matrix, distortion_coeffs, r_vec, t_vec;
  if (distortion_model_ == 0) {
    cv::calibrateCamera(obj_points, img_points, cv::Size(im_cols_, im_rows_),
                        camera_matrix, distortion_coeffs, r_vec, t_vec);
    LOG_INFO << "cameraMatrix : " << camera_matrix;
    LOG_INFO << "distCoeffs : " << distortion_coeffs;
  }
  if (distortion_model_ == 1) {
    // cv::fisheye::calibrate(obj_points, img_points, cv::Size(im_cols_,
    // im_rows_),
    //                    camera_matrix, distortion_coeffs, r_vec, t_vec,
    //                    cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC  |
    //                    cv::fisheye::CALIB_FIX_SKEW  |
    //                    cv::fisheye::CALIB_FIX_K2 | cv::fisheye::CALIB_FIX_K3
    //                    | cv::fisheye::CALIB_FIX_K4);
    cv::fisheye::calibrate(obj_points, img_points, cv::Size(im_cols_, im_rows_),
                           camera_matrix, distortion_coeffs, r_vec, t_vec,
                           cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
                               cv::fisheye::CALIB_FIX_SKEW);

    LOG_INFO << "cameraMatrix : " << camera_matrix;
    LOG_INFO << "distCoeffs : " << distortion_coeffs;
  }

  // Save data in the structure
  setIntrinsics(camera_matrix, distortion_coeffs);
}

/**
 * @brief Compute the reprojection error for each boards for this camera
 *
 * @todo double-check whether it is used, duplication in
 * Calibration::computeReproErrAllBoard()
 */
void Camera::computeReproErrAllBoard() {
  std::vector<float> err_vec;
  float sum_err = 0;
  for (std::map<int, std::weak_ptr<BoardObs>>::iterator it =
           board_observations_.begin();
       it != board_observations_.end(); ++it) {
    std::shared_ptr<BoardObs> board_obs_ptr = it->second.lock();
    if (board_obs_ptr)
      float err = board_obs_ptr->computeReprojectionError();
  }
}

/**
 * @brief Refinement of the camera parameters of the current camera
 *
 */
void Camera::refineIntrinsicCalibration(const int nb_iterations) {
  ceres::Problem problem;
  double loss = 1.0;
  LOG_INFO << "Parameters before optimization :: " << this->getCameraMat();
  LOG_INFO << "distortion vector :: " << getDistortionVectorVector();
  for (std::map<int, std::weak_ptr<BoardObs>>::iterator it =
           board_observations_.begin();
       it != board_observations_.end(); ++it) {
    std::shared_ptr<BoardObs> board_obs_ptr = it->second.lock();
    if (board_obs_ptr && board_obs_ptr->valid_ == true) {
      std::shared_ptr<Board> board_3d_ptr = board_obs_ptr->board_3d_.lock();
      if (board_3d_ptr) {
        std::vector<cv::Point3f> board_pts_3d = board_3d_ptr->pts_3d_;
        std::vector<int> board_pts_idx = board_obs_ptr->charuco_id_;
        std::vector<cv::Point2f> board_pts_2d = board_obs_ptr->pts_2d_;
        for (int i = 0; i < board_pts_idx.size(); i++) {
          cv::Point3f current_pts_3d =
              board_pts_3d[board_pts_idx[i]];           // Current 3D pts
          cv::Point2f current_pts_2d = board_pts_2d[i]; // Current 2D pts
          ceres::CostFunction *reprojection_error = ReprojectionError::Create(
              double(current_pts_2d.x), double(current_pts_2d.y),
              double(current_pts_3d.x), double(current_pts_3d.y),
              double(current_pts_3d.z), distortion_model_);
          // problem.AddResidualBlock(ReprojectionError, new
          // ceres::ArctanLoss(loss), poses[i], Intrinsics);
          problem.AddResidualBlock(reprojection_error,
                                   new ceres::HuberLoss(1.0),
                                   board_obs_ptr->pose_, intrinsics_);
        }
      }
    }
  }
  // Run the optimization
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.max_num_iterations = nb_iterations;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  LOG_INFO << "Parameters after optimization :: " << this->getCameraMat();
  LOG_INFO << "distortion vector after optimization :: "
           << getDistortionVectorVector();
}
