#include "OptimizationCeres.h"
#include "geometrytools.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "CameraGroup.hpp"
#include "logger.h"

/**
 * @brief Initialize the camera group object
 *
 * Set the index of the reference camera and the index of the camera group.
 *
 * @param id_ref_cam index of the reference camera
 * @param cam_group_idx index of the camera group
 */
CameraGroup::CameraGroup(const int id_ref_cam, const int cam_group_idx)
    : id_ref_cam_(id_ref_cam), cam_group_idx_(cam_group_idx){};

/**
 * @brief Insert a new camera in the group
 *
 * @param new_camera pointer to the new camera
 */
void CameraGroup::insertCamera(std::shared_ptr<Camera> new_camera) {
  cameras_[new_camera->cam_idx_] = new_camera;
  cam_idx.push_back(new_camera->cam_idx_);
}

/**
 * @brief Insert a new object observation in the data structure
 *
 * @param new_object_observation pointer to the object observation
 */
void CameraGroup::insertNewObjectObservation(
    std::shared_ptr<Object3DObs> new_object_observation) {
  object_observations_[object_observations_.size()] = new_object_observation;
}

/**
 * @brief Insert new frame in the data structure
 *
 * @param new_frame pointer to the new frame
 */
void CameraGroup::insertNewFrame(std::shared_ptr<Frame> new_frame) {
  frames_[new_frame->frame_idx_] = new_frame;
}

CameraGroup::~CameraGroup() {}

/**
 * @brief Get camera pose (vector) of the camera "id_cam" in the group
 *
 * The rotation and translation vector are passed by reference and set within
 * the function.
 *
 * @param r_vec return (by reference) Rodrigues rotation vector
 * @param t_vec return (by reference) translation vector
 * @param id_cam index of the camera of interest in the group
 */
void CameraGroup::getCameraPoseVec(cv::Mat &r_vec, cv::Mat &t_vec, int id_cam) {
  cv::Mat rot_v = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat trans_v = cv::Mat::zeros(3, 1, CV_64F);
  rot_v.at<double>(0) = relative_camera_pose_[id_cam][0];
  rot_v.at<double>(1) = relative_camera_pose_[id_cam][1];
  rot_v.at<double>(2) = relative_camera_pose_[id_cam][2];
  trans_v.at<double>(0) = relative_camera_pose_[id_cam][3];
  trans_v.at<double>(1) = relative_camera_pose_[id_cam][4];
  trans_v.at<double>(2) = relative_camera_pose_[id_cam][5];
  rot_v.copyTo(r_vec);
  trans_v.copyTo(t_vec);
}

/**
 * @brief Get camera pose (cv::Mat) of the camera "id_cam" in the group
 *
 * @param id_cam index of the camera of interest in the group
 *
 * @return pose matrix of the camera in the group
 */
cv::Mat CameraGroup::getCameraPoseMat(int id_cam) {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getCameraPoseVec(r_vec, t_vec, id_cam);
  cv::Mat pose = RVecT2Proj(r_vec, t_vec);
  return pose;
}

/**
 * @brief Set "id_cam" camera pose (cv::Matrix) in the data structure
 *
 * @param pose 4x4 pose to set
 * @param id_cam index of the camera of interest in the group
 */
void CameraGroup::setCameraPoseMat(cv::Mat pose, int id_cam) {
  relative_camera_pose_[id_cam] = std::vector<double>(6);
  cv::Mat r_vec, t_vec;
  Proj2RT(pose, r_vec, t_vec);
  relative_camera_pose_[id_cam][0] = r_vec.at<double>(0);
  relative_camera_pose_[id_cam][1] = r_vec.at<double>(1);
  relative_camera_pose_[id_cam][2] = r_vec.at<double>(2);
  relative_camera_pose_[id_cam][3] = t_vec.at<double>(0);
  relative_camera_pose_[id_cam][4] = t_vec.at<double>(1);
  relative_camera_pose_[id_cam][5] = t_vec.at<double>(2);
}

/**
 * @brief Set "id_cam" camera pose (Vector) in the data structure
 *
 * @param r_vec Rodrigues rotation vector to set
 * @param t_vec translation vector to set
 * @param id_cam index of the camera of interest in the group
 */
void CameraGroup::setCameraPoseVec(cv::Mat r_vec, cv::Mat t_vec, int id_cam) {
  relative_camera_pose_[id_cam] = std::vector<double>(6);
  relative_camera_pose_[id_cam][0] = r_vec.at<double>(0);
  relative_camera_pose_[id_cam][1] = r_vec.at<double>(1);
  relative_camera_pose_[id_cam][2] = r_vec.at<double>(2);
  relative_camera_pose_[id_cam][3] = t_vec.at<double>(0);
  relative_camera_pose_[id_cam][4] = t_vec.at<double>(1);
  relative_camera_pose_[id_cam][5] = t_vec.at<double>(2);
}

/**
 * @brief Get rotation vector of the camera "id_cam" in the group
 *
 * @param id_cam index of the camera of interest in the group
 * @return 1x3 Rodrigues vector of the camera "id_cam"
 */
cv::Mat CameraGroup::getCameraRotVec(int id_cam) {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getCameraPoseVec(r_vec, t_vec, id_cam);
  return r_vec;
}

/**
 * @brief Get translation vector of the camera "id_cam" in the group
 *
 * @param id_cam index of the camera of interest in the group
 * @return 1x3 translation vector of the camera "id_cam"
 */
cv::Mat CameraGroup::getCameraTransVec(int id_cam) {
  cv::Mat r_vec;
  cv::Mat t_vec;
  getCameraPoseVec(r_vec, t_vec, id_cam);
  return t_vec;
}

/**
 * @brief Compute the pose of each visible 3D object for the camera group
 *
 */
void CameraGroup::computeObjPoseInCameraGroup() {
  for (std::map<int, std::weak_ptr<Frame>>::iterator it_frame = frames_.begin();
       it_frame != frames_.end(); ++it_frame) {

    auto frame_ptr = it_frame->second.lock();
    if (frame_ptr) {
      // Iterate through cameraGroupObs
      std::map<int, std::weak_ptr<CameraGroupObs>> current_cam_group_obs_vec =
          frame_ptr->cam_group_observations_;
      for (std::map<int, std::weak_ptr<CameraGroupObs>>::iterator
               it_cam_group_obs = current_cam_group_obs_vec.begin();
           it_cam_group_obs != current_cam_group_obs_vec.end();
           ++it_cam_group_obs) {

        auto cam_group_obs_ptr = it_cam_group_obs->second.lock();
        if (cam_group_obs_ptr &&
            cam_group_idx_ == cam_group_obs_ptr->cam_group_idx_) {
          // iterate through 3D object obs
          std::map<int, std::weak_ptr<Object3DObs>> current_obj3d_obs_vec =
              cam_group_obs_ptr->object_observations_;
          for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_obj3d =
                   current_obj3d_obs_vec.begin();
               it_obj3d != current_obj3d_obs_vec.end(); ++it_obj3d) {
            // Transform the 3D object in the referential of the group
            std::shared_ptr<Object3DObs> it_obj3d_ptr = it_obj3d->second.lock();
            if (it_obj3d_ptr) {
              int current_cam_id = it_obj3d_ptr->camera_id_;
              cv::Mat pose_cam_mat = getCameraPoseMat(current_cam_id);
              cv::Mat pose_obj_mat = it_obj3d_ptr->getPoseMat();
              cv::Mat pose_in_ref_mat = pose_cam_mat.inv() * pose_obj_mat;
              // set in the observation
              it_obj3d_ptr->setPoseInGroupMat(pose_in_ref_mat);
            }
          }
        }
      }
    }
  }
}

/**
 * @brief Refine the objects pose and camera pose in the group
 *
 * @param nb_iterations number of iterations for non-linear refinement
 *
 */
void CameraGroup::refineCameraGroup(const int nb_iterations) {
  ceres::Problem problem;
  LOG_INFO << "Number of frames for camera group optimization  :: "
           << frames_.size();
  // Iterate through frames
  for (std::map<int, std::weak_ptr<Frame>>::iterator it_frame = frames_.begin();
       it_frame != frames_.end(); ++it_frame) {

    auto frame_ptr = it_frame->second.lock();
    if (frame_ptr) {
      // Iterate through cameraGroupObs
      std::map<int, std::weak_ptr<CameraGroupObs>> current_cam_group_obs_vec =
          frame_ptr->cam_group_observations_;
      for (std::map<int, std::weak_ptr<CameraGroupObs>>::iterator
               it_cam_group_obs = current_cam_group_obs_vec.begin();
           it_cam_group_obs != current_cam_group_obs_vec.end();
           ++it_cam_group_obs) {

        auto cam_group_obs_ptr = it_cam_group_obs->second.lock();
        if (cam_group_obs_ptr) {
          // Check if the current group the group we refine (be careful)
          if (cam_group_idx_ == cam_group_obs_ptr->cam_group_idx_) {
            // iterate through 3D object obs
            std::map<int, std::weak_ptr<Object3DObs>> current_obj3d_obs_vec =
                cam_group_obs_ptr->object_observations_;
            for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_obj3d =
                     current_obj3d_obs_vec.begin();
                 it_obj3d != current_obj3d_obs_vec.end(); ++it_obj3d) {
              std::shared_ptr<Object3DObs> it_obj3d_ptr =
                  it_obj3d->second.lock();
              if (it_obj3d_ptr) {
                int current_cam_id = it_obj3d_ptr->camera_id_;
                auto object_3d_ptr = it_obj3d_ptr->object_3d_.lock();
                if (object_3d_ptr) {
                  std::vector<cv::Point3f> obj_pts_3d = object_3d_ptr->pts_3d_;
                  std::vector<int> obj_pts_idx = it_obj3d_ptr->pts_id_;
                  std::vector<cv::Point2f> obj_pts_2d = it_obj3d_ptr->pts_2d_;
                  std::shared_ptr<Camera> cam_ptr = it_obj3d_ptr->cam_.lock();
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
                    bool refine_cam = true;
                    // We do not refine the camera pose if it is the ref camera
                    if (this->id_ref_cam_ == cam_ptr->cam_idx_)
                      refine_cam = false;
                    for (int i = 0; i < obj_pts_idx.size(); i++) {
                      cv::Point3f current_pts_3d =
                          obj_pts_3d[obj_pts_idx[i]]; // Current 3D pts
                      cv::Point2f current_pts_2d =
                          obj_pts_2d[i]; // Current 2D pts
                      ceres::CostFunction *reprojection_error =
                          ReprojectionError_CameraGroupRef::Create(
                              double(current_pts_2d.x),
                              double(current_pts_2d.y),
                              double(current_pts_3d.x),
                              double(current_pts_3d.y),
                              double(current_pts_3d.z), fx, fy, u0, v0, r1, r2,
                              r3, t1, t2, refine_cam,
                              cam_ptr->distortion_model_);
                      problem.AddResidualBlock(
                          reprojection_error, new ceres::HuberLoss(1.0),
                          relative_camera_pose_[current_cam_id].data(),
                          cam_group_obs_ptr
                              ->object_pose_[it_obj3d_ptr->object_3d_id_]
                              .data());
                      // it_obj3d->second->group_pose_);
                      // it_cam_group_obs->second->object_pose_[it_obj3d->second->object_3d_id_]
                    }
                  }
                }
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

  // Display poses in the group
  for (std::map<int, std::vector<double>>::iterator it =
           relative_camera_pose_.begin();
       it != relative_camera_pose_.end(); ++it) {
    LOG_INFO << "Camera  " << it->first
             << "  :: " << getCameraPoseMat(it->first);
  }
}

/**
 * @brief Compute the reprojection error for this camera group
 *
 */
void CameraGroup::reproErrorCameraGroup() {

  // Iterate through frames
  for (std::map<int, std::weak_ptr<Frame>>::iterator it_frame = frames_.begin();
       it_frame != frames_.end(); ++it_frame) {

    auto frame_ptr = it_frame->second.lock();
    if (frame_ptr) {
      // Iterate through cameraGroupObs
      std::map<int, std::weak_ptr<CameraGroupObs>> current_cam_group_obs_vec =
          frame_ptr->cam_group_observations_;
      for (std::map<int, std::weak_ptr<CameraGroupObs>>::iterator
               it_cam_group_obs = current_cam_group_obs_vec.begin();
           it_cam_group_obs != current_cam_group_obs_vec.end();
           ++it_cam_group_obs) {

        auto cam_group_obs_ptr = it_cam_group_obs->second.lock();
        // Check if the current group is the camera group of interest
        if (cam_group_obs_ptr &&
            cam_group_idx_ == cam_group_obs_ptr->cam_group_idx_) {
          // iterate through 3D object obs
          std::map<int, std::weak_ptr<Object3DObs>> current_obj3d_obs_vec =
              cam_group_obs_ptr->object_observations_;
          for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_obj3d =
                   current_obj3d_obs_vec.begin();
               it_obj3d != current_obj3d_obs_vec.end(); ++it_obj3d) {
            std::shared_ptr<Object3DObs> it_obj3d_ptr = it_obj3d->second.lock();
            if (it_obj3d_ptr) {
              int current_cam_id = it_obj3d_ptr->camera_id_;
              auto object_3d_ptr = it_obj3d_ptr->object_3d_.lock();
              if (object_3d_ptr) {
                std::vector<cv::Point3f> obj_pts_3d = object_3d_ptr->pts_3d_;
                std::vector<int> obj_pts_idx = it_obj3d_ptr->pts_id_;
                std::vector<cv::Point2f> obj_pts_2d = it_obj3d_ptr->pts_2d_;

                // Compute the reprojection error
                std::vector<cv::Point3f> object_pts;
                for (int i = 0; i < obj_pts_idx.size(); i++)
                  object_pts.push_back(obj_pts_3d[obj_pts_idx[i]]);

                // Apply object pose transform
                std::vector<cv::Point3f> object_pts_trans1 =
                    transform3DPts(object_pts,
                                   cam_group_obs_ptr->getObjectRotVec(
                                       it_obj3d_ptr->object_3d_id_),
                                   cam_group_obs_ptr->getObjectTransVec(
                                       it_obj3d_ptr->object_3d_id_));
                // Reproject pts
                std::vector<cv::Point2f> repro_pts;
                std::shared_ptr<Camera> cam_ptr = it_obj3d_ptr->cam_.lock();
                if (cam_ptr) {
                  projectPointsWithDistortion(
                      object_pts_trans1, getCameraRotVec(current_cam_id),
                      getCameraTransVec(current_cam_id),
                      cam_ptr->getCameraMat(),
                      cam_ptr->getDistortionVectorVector(), repro_pts,
                      cam_ptr->distortion_model_);
                  float sum_error = 0;
                  // Compute error
                  for (int i = 0; i < repro_pts.size(); i++) {
                    float rep_err = std::sqrt(
                        std::pow((obj_pts_2d[i].x - repro_pts[i].x), 2) +
                        std::pow((obj_pts_2d[i].y - repro_pts[i].y), 2));
                    sum_error += rep_err;
                  }
                  float mean_error = sum_error / repro_pts.size();
                  LOG_DEBUG << "cam_id :: " << current_cam_id
                            << "   object_id :: " << it_obj3d_ptr->object_3d_id_
                            << "  mean error :: " << mean_error
                            << "  nb pts :: " << repro_pts.size();
                }
              }
            }
          }
        }
      }
    }
  }
}

/**
 * @brief Refine the objects pose, camera pose in the group and board poses
 *
 * @param nb_iterations number of iterations for non-linear refinement
 *
 */
void CameraGroup::refineCameraGroupAndObjects(const int nb_iterations) {
  ceres::Problem problem;
  LOG_INFO << "Number of frames for camera group optimization  :: "
           << frames_.size();
  // Iterate through frames
  for (std::map<int, std::weak_ptr<Frame>>::iterator it_frame = frames_.begin();
       it_frame != frames_.end(); ++it_frame) {

    auto frame_ptr = it_frame->second.lock();
    if (frame_ptr) {
      // Iterate through cameraGroupObs
      std::map<int, std::weak_ptr<CameraGroupObs>> current_cam_group_obs_vec =
          frame_ptr->cam_group_observations_;
      for (std::map<int, std::weak_ptr<CameraGroupObs>>::iterator
               it_cam_group_obs = current_cam_group_obs_vec.begin();
           it_cam_group_obs != current_cam_group_obs_vec.end();
           ++it_cam_group_obs) {

        auto cam_group_obs_ptr = it_cam_group_obs->second.lock();
        // Check if the current group the group we refine (be careful)
        if (cam_group_obs_ptr &&
            cam_group_idx_ == cam_group_obs_ptr->cam_group_idx_) {
          // iterate through 3D object obs
          std::map<int, std::weak_ptr<Object3DObs>> current_obj3d_obs_vec =
              cam_group_obs_ptr->object_observations_;
          for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_obj3d =
                   current_obj3d_obs_vec.begin();
               it_obj3d != current_obj3d_obs_vec.end(); ++it_obj3d) {
            std::shared_ptr<Object3DObs> it_obj3d_ptr = it_obj3d->second.lock();
            if (it_obj3d_ptr) {
              int current_cam_id = it_obj3d_ptr->camera_id_;
              std::shared_ptr<Object3D> object_3d_ptr =
                  it_obj3d_ptr->object_3d_.lock();
              if (object_3d_ptr) {
                std::vector<cv::Point3f> obj_pts_3d = object_3d_ptr->pts_3d_;
                std::vector<int> obj_pts_idx = it_obj3d_ptr->pts_id_;
                std::vector<cv::Point2f> obj_pts_2d = it_obj3d_ptr->pts_2d_;
                std::shared_ptr<Camera> cam_ptr = it_obj3d_ptr->cam_.lock();
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
                  bool refine_cam = true;
                  // We do not refine the camera pose if it is the ref camera
                  if (this->id_ref_cam_ == cam_ptr->cam_idx_) {
                    refine_cam = false;
                  }
                  for (int i = 0; i < obj_pts_idx.size(); i++) {
                    cv::Point3f current_pts_3d =
                        obj_pts_3d[obj_pts_idx[i]]; // Current 3D pts
                    cv::Point2f current_pts_2d =
                        obj_pts_2d[i]; // Current 2D pts

                    // find the board and pts corresponding to the 3D point
                    // object
                    std::pair<int, int> board_id_pts_id =
                        object_3d_ptr->pts_obj_2_board_[obj_pts_idx[i]];

                    auto cur_board_pts_idx_ptr =
                        object_3d_ptr->boards_[board_id_pts_id.first].lock();
                    if (cur_board_pts_idx_ptr) {
                      cv::Point3f current_pts3D_board =
                          cur_board_pts_idx_ptr
                              ->pts_3d_[board_id_pts_id.second];
                      int ref_board_id = object_3d_ptr->ref_board_id_;
                      bool refine_board = true;
                      if (ref_board_id == board_id_pts_id.first) {
                        refine_board = false;
                      }

                      // key(boardid//ptsid)-->pts_ind_board
                      ceres::CostFunction *reprojection_error =
                          ReprojectionError_CameraGroupAndObjectRef::Create(
                              double(current_pts_2d.x),
                              double(current_pts_2d.y),
                              double(current_pts3D_board.x),
                              double(current_pts3D_board.y),
                              double(current_pts3D_board.z), fx, fy, u0, v0, r1,
                              r2, r3, t1, t2, refine_cam, refine_board,
                              cam_ptr->distortion_model_);
                      problem.AddResidualBlock(
                          reprojection_error,
                          new ceres::HuberLoss(1.0), // nullptr,
                          relative_camera_pose_[current_cam_id].data(),
                          cam_group_obs_ptr
                              ->object_pose_[it_obj3d_ptr->object_3d_id_]
                              .data(),
                          object_3d_ptr
                              ->relative_board_pose_[board_id_pts_id.first]);
                    }
                  }
                }
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

  // Display poses in the group
  for (std::map<int, std::vector<double>>::iterator it =
           relative_camera_pose_.begin();
       it != relative_camera_pose_.end(); ++it) {
    LOG_INFO << "Camera  " << it->first
             << "  :: " << getCameraPoseMat(it->first);
  }
}

/**
 * @brief Refine the objects pose, camera pose in the group and board poses
 * and cameras intrinsic parameters
 *
 * @param nb_iterations number of iterations for non-linear refinement
 *
 */
void CameraGroup::refineCameraGroupAndObjectsAndIntrinsics(
    const int nb_iterations) {
  ceres::Problem problem;
  LOG_INFO << "Number of frames for camera group optimization  :: "
           << frames_.size();
  // Iterate through frames
  for (std::map<int, std::weak_ptr<Frame>>::iterator it_frame = frames_.begin();
       it_frame != frames_.end(); ++it_frame) {

    auto frame_ptr = it_frame->second.lock();
    if (frame_ptr) {
      // Iterate through cameraGroupObs
      std::map<int, std::weak_ptr<CameraGroupObs>> current_cam_group_obs_vec =
          frame_ptr->cam_group_observations_;
      for (std::map<int, std::weak_ptr<CameraGroupObs>>::iterator
               it_cam_group_obs = current_cam_group_obs_vec.begin();
           it_cam_group_obs != current_cam_group_obs_vec.end();
           ++it_cam_group_obs) {

        auto cam_group_obs_ptr = it_cam_group_obs->second.lock();
        // Check if the current group the group we refine (be careful)
        if (cam_group_obs_ptr &&
            cam_group_idx_ == cam_group_obs_ptr->cam_group_idx_) {
          // iterate through 3D object obs
          std::map<int, std::weak_ptr<Object3DObs>> current_obj3d_obs_vec =
              cam_group_obs_ptr->object_observations_;
          for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_obj3d =
                   current_obj3d_obs_vec.begin();
               it_obj3d != current_obj3d_obs_vec.end(); ++it_obj3d) {
            std::shared_ptr<Object3DObs> it_obj3d_ptr = it_obj3d->second.lock();
            if (it_obj3d_ptr) {
              int current_cam_id = it_obj3d_ptr->camera_id_;
              std::shared_ptr<Object3D> object_3d_ptr =
                  it_obj3d_ptr->object_3d_.lock();
              if (object_3d_ptr) {
                std::vector<cv::Point3f> obj_pts_3d = object_3d_ptr->pts_3d_;
                std::vector<int> obj_pts_idx = it_obj3d_ptr->pts_id_;
                std::vector<cv::Point2f> obj_pts_2d = it_obj3d_ptr->pts_2d_;
                std::shared_ptr<Camera> cam_ptr = it_obj3d_ptr->cam_.lock();
                if (cam_ptr) {
                  bool refine_cam = true;
                  // We do not refine the camera pose if it is the ref camera
                  if (this->id_ref_cam_ == cam_ptr->cam_idx_) {
                    refine_cam = false;
                  }
                  for (int i = 0; i < obj_pts_idx.size(); i++) {
                    cv::Point3f current_pts_3d =
                        obj_pts_3d[obj_pts_idx[i]]; // Current 3D pts
                    cv::Point2f current_pts_2d =
                        obj_pts_2d[i]; // Current 2D pts

                    // find the board and pts corresponding to the 3D point
                    // object
                    std::pair<int, int> board_id_pts_id =
                        object_3d_ptr->pts_obj_2_board_[obj_pts_idx[i]];

                    auto cur_board_pts_idx_ptr =
                        object_3d_ptr->boards_[board_id_pts_id.first].lock();
                    if (cur_board_pts_idx_ptr) {
                      cv::Point3f current_pts3D_board =
                          cur_board_pts_idx_ptr
                              ->pts_3d_[board_id_pts_id.second];
                      int ref_board_id = object_3d_ptr->ref_board_id_;
                      bool refine_board = true;
                      if (ref_board_id == board_id_pts_id.first) {
                        refine_board = false;
                      }

                      // key(boardid//ptsid)-->pts_ind_board
                      ceres::CostFunction *reprojection_error =
                          ReprojectionError_CameraGroupAndObjectRefAndIntrinsics::
                              Create(double(current_pts_2d.x),
                                     double(current_pts_2d.y),
                                     double(current_pts3D_board.x),
                                     double(current_pts3D_board.y),
                                     double(current_pts3D_board.z), refine_cam,
                                     refine_board, cam_ptr->distortion_model_);
                      problem.AddResidualBlock(
                          reprojection_error,
                          new ceres::HuberLoss(1.0), // nullptr,
                          relative_camera_pose_[current_cam_id].data(),
                          cam_group_obs_ptr
                              ->object_pose_[it_obj3d_ptr->object_3d_id_]
                              .data(),
                          object_3d_ptr
                              ->relative_board_pose_[board_id_pts_id.first],
                          cam_ptr->intrinsics_);
                    }
                  }
                }
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

  // Display poses in the group
  for (std::map<int, std::vector<double>>::iterator it =
           relative_camera_pose_.begin();
       it != relative_camera_pose_.end(); ++it) {
    LOG_INFO << "Camera  " << it->first
             << "  :: " << getCameraPoseMat(it->first);
  }
}
