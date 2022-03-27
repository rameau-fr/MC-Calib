#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "Camera.hpp"
#include "Frame.hpp"
#include "Object3D.hpp"
#include "OptimizationCeres.h"
#include "geometrytools.hpp"
#include "logger.h"

/**
 * @brief Insert a new object observation for this 3D object
 *
 * @param new_object pointer to the new object
 */
void Object3D::insertNewObject(std::shared_ptr<Object3DObs> new_object) {
  object_observations_[object_observations_.size()] = new_object;
}

/**
 * @brief Initialize the 3D object
 *
 * @param nb_boards number of boards used for calibration
 * @param ref_board_id reference board id
 * @param obj_id object id
 * @param color color of the 3D object
 */
Object3D::Object3D(const int nb_boards, const int ref_board_id,
                   const int obj_id, const std::vector<double> color)
    : nb_boards_(nb_boards), ref_board_id_(ref_board_id), obj_id_(obj_id),
      nb_pts_(0), color_(color){};

/**
 * @brief Insert a new board in the object
 *
 * @param new_board pointer to the new board
 * @param board_id board index inserted in the object
 */
void Object3D::insertBoardInObject(std::shared_ptr<Board> new_board) {
  boards_[new_board->board_id_] = new_board;
  nb_pts_ += new_board->nb_pts_;
}

/**
 * @brief Insert a new frame associated to this 3D object
 *
 * @param new_frame pointer to the new frame to be added
 */
void Object3D::insertNewFrame(std::shared_ptr<Frame> new_frame) {
  frames_[new_frame->frame_idx_] = new_frame;
}

Object3D::~Object3D() {
  for (auto const &item : relative_board_pose_)
    delete[] item.second;
  relative_board_pose_.clear();
}

/**
 * @brief Return (by reference) the pose vector of the board "board_id" in the
 * object
 *
 * @param r_vec rotation vector (Rodrigues)
 * @param t_vec translation vector
 * @param board_id id of the board of interest in the object
 */
void Object3D::getBoardPoseVec(cv::Mat &r_vec, cv::Mat &t_vec, int board_id) {
  cv::Mat rot_v = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat trans_v = cv::Mat::zeros(3, 1, CV_64F);
  rot_v.at<double>(0) = relative_board_pose_[board_id][0];
  rot_v.at<double>(1) = relative_board_pose_[board_id][1];
  rot_v.at<double>(2) = relative_board_pose_[board_id][2];
  trans_v.at<double>(0) = relative_board_pose_[board_id][3];
  trans_v.at<double>(1) = relative_board_pose_[board_id][4];
  trans_v.at<double>(2) = relative_board_pose_[board_id][5];
  rot_v.copyTo(r_vec);
  trans_v.copyTo(t_vec);
}

/**
 * @brief Return the pose matrix (4x4) of "board_id" in the object
 *
 * @param board_id id of the board of interest in the object
 *
 * @return 4x4 pose matrix of the board in the object
 */
cv::Mat Object3D::getBoardPoseMat(int board_id) {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getBoardPoseVec(r_vec, t_vec, board_id);
  cv::Mat pose = RVecT2Proj(r_vec, t_vec);
  return pose;
}

/**
 * @brief Set pose of the board "board_id" in the object (using 4x4 matrix)
 *
 * @param pose 4x4 pose matrix of the board in the object
 * @param board_id board index of interest
 */
void Object3D::setBoardPoseMat(cv::Mat pose, int board_id) {
  relative_board_pose_[board_id] = new double[6];
  cv::Mat r_vec, t_vec;
  Proj2RT(pose, r_vec, t_vec);
  relative_board_pose_[board_id][0] = r_vec.at<double>(0);
  relative_board_pose_[board_id][1] = r_vec.at<double>(1);
  relative_board_pose_[board_id][2] = r_vec.at<double>(2);
  relative_board_pose_[board_id][3] = t_vec.at<double>(0);
  relative_board_pose_[board_id][4] = t_vec.at<double>(1);
  relative_board_pose_[board_id][5] = t_vec.at<double>(2);
}

/**
 * @brief Set pose of the board "board_id" in the object (using 6 double vector)
 *
 * @param r_vec rotation vector (Rodrigues)
 * @param t_vec translation vector
 * @param board_id board index of interest
 */
void Object3D::setBoardPoseVec(cv::Mat r_vec, cv::Mat t_vec, int board_id) {
  relative_board_pose_[board_id] = new double[6];
  relative_board_pose_[board_id][0] = r_vec.at<double>(0);
  relative_board_pose_[board_id][1] = r_vec.at<double>(1);
  relative_board_pose_[board_id][2] = r_vec.at<double>(2);
  relative_board_pose_[board_id][3] = t_vec.at<double>(0);
  relative_board_pose_[board_id][4] = t_vec.at<double>(1);
  relative_board_pose_[board_id][5] = t_vec.at<double>(2);
}

/**
 * @brief Get the rotation vector of the board "board_id"
 *
 * @param board_id board index of interest
 *
 * @return 1x3 Rodrigues rotation vector
 */
cv::Mat Object3D::getBoardRotVec(int board_id) {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getBoardPoseVec(r_vec, t_vec, board_id);
  return r_vec;
}

/**
 * @brief Get the translation vector of the board "board_id"
 *
 * @param board_id board index of interest
 *
 * @return 1x3 translation vector
 */
cv::Mat Object3D::getBoardTransVec(int board_id) {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getBoardPoseVec(r_vec, t_vec, board_id);
  return t_vec;
}

/**
 * @brief Refine the 3D object (board absolute pose) and pose of the object
 *
 * @param nb_iterations number of iterations of non-linear refinement
 *
 * @todo The current 3D object refinement refines all frames even when a single
 * board of the object is visible. We might need to include yet another
 * objective function
 */
void Object3D::refineObject(const int nb_iterations) {

  ceres::Problem problem;

  // Iterate through the object obs
  for (const auto &it_obj_obs : object_observations_) {
    std::shared_ptr<Object3DObs> current_object_obs = it_obj_obs.second.lock();
    if (current_object_obs) {
      for (const auto &it_board_obs : current_object_obs->board_observations_) {
        auto board_obs_ptr = it_board_obs.second.lock();
        if (board_obs_ptr && board_obs_ptr->valid_ == true) {
          std::shared_ptr<Board> board_3d_ptr = board_obs_ptr->board_3d_.lock();
          if (board_3d_ptr) {
            std::vector<cv::Point3f> board_pts_3d = board_3d_ptr->pts_3d_;
            std::vector<int> board_pts_idx = board_obs_ptr->charuco_id_;
            std::vector<cv::Point2f> board_pts_2d = board_obs_ptr->pts_2d_;
            std::shared_ptr<Camera> cam_ptr = board_obs_ptr->cam_.lock();
            if (cam_ptr) {
              double fx = cam_ptr->intrinsics_[0];
              double fy = cam_ptr->intrinsics_[1];
              double u0 = cam_ptr->intrinsics_[2];
              double v0 = cam_ptr->intrinsics_[3];
              double r1 = cam_ptr->intrinsics_[4];
              double r2 = cam_ptr->intrinsics_[5];
              double t1 = cam_ptr->intrinsics_[6];
              double t2 = cam_ptr->intrinsics_[7];
              double r3 = cam_ptr->intrinsics_[8];
              bool refine_board = true;
              // We do not refine the board pose if only one board is visible or
              // if it is the ref board
              if (ref_board_id_ == board_obs_ptr->board_id_)
                refine_board = false;

              for (int i = 0; i < board_pts_idx.size(); i++) {
                cv::Point3f current_pts_3d =
                    board_pts_3d[board_pts_idx[i]];           // Current 3D pts
                cv::Point2f current_pts_2d = board_pts_2d[i]; // Current 2D pts
                ceres::CostFunction *reprojection_error =
                    ReprojectionError_3DObjRef::Create(
                        double(current_pts_2d.x), double(current_pts_2d.y),
                        double(current_pts_3d.x), double(current_pts_3d.y),
                        double(current_pts_3d.z), fx, fy, u0, v0, r1, r2, r3,
                        t1, t2, refine_board, cam_ptr->distortion_model_);
                problem.AddResidualBlock(
                    reprojection_error, new ceres::HuberLoss(1.0),
                    current_object_obs->pose_,
                    relative_board_pose_[board_obs_ptr->board_id_]);
              }
            }
          }
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

  // Update the pts3d in the object
  for (const auto &it_board : boards_) {
    auto board_ptr = it_board.second.lock();
    if (board_ptr) {
      int board_idx = board_ptr->board_id_;

      // Transform the 3D pts to push in the object 3D
      std::vector<cv::Point3f> trans_pts =
          transform3DPts(board_ptr->pts_3d_, getBoardRotVec(board_idx),
                         getBoardTransVec(board_idx));

      // Replace the keypoints
      for (int i = 0; i < trans_pts.size(); i++) {
        std::pair<int, int> board_id_pts_id = std::make_pair(board_idx, i);
        pts_3d_[pts_board_2_obj_[board_id_pts_id]] = trans_pts[i];
      }
    }
  }
}

/**
 * @brief Update the 3D pts of the object after refinement
 *
 */
void Object3D::updateObjectPts() {
  // Update the pts3d in the object
  for (const auto &it_board : boards_) {
    auto board_ptr = it_board.second.lock();
    if (board_ptr) {
      int board_idx = board_ptr->board_id_;

      // Transform the 3D pts to push in the object 3D
      std::vector<cv::Point3f> trans_pts =
          transform3DPts(board_ptr->pts_3d_, getBoardRotVec(board_idx),
                         getBoardTransVec(board_idx));

      // Replace the keypoints
      for (int i = 0; i < trans_pts.size(); i++) {
        std::pair<int, int> board_id_pts_id = std::make_pair(board_idx, i);
        pts_3d_[pts_board_2_obj_[board_id_pts_id]] = trans_pts[i];
      }
    }
  }
}
