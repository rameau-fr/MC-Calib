#pragma once

#include "Board.hpp"
#include "Object3D.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

class Camera;
class Board;
class BoardObs;
class Object3D;

/**
 * @class Object3DObs
 *
 * @brief This class contains the object 3D observation
 *
 * Observation of one 3D object by a camera/camera group.
 * - object 3D pose w.r.t. to the camera observing the object
 * - camera observing the object
 * - object 3D pose w.r.t. to the camera group observing the object
 * - board 3D in the in the object
 * - board observations in this object obs
 * - frame index
 * - object 3D index
 */
class Object3DObs {
public:
  // Indexing
  int frame_id_;
  int camera_id_;
  std::vector<int> board_id_;
  int object_3d_id_;

  // Pose
  double *pose_ = new double[6];

  // Pose in camera group
  double *group_pose_ =
      new double[6]; // pose of the object expressed in camera group referential
  // int cam_group_id_;

  // points
  std::vector<cv::Point2f> pts_2d_;
  std::vector<int> pts_id_; // index in the object

  // Camera corresponding to this Observation
  std::weak_ptr<Camera> cam_;

  // Board3D corresponding to this Observation
  std::map<int, std::weak_ptr<Board>> board_3d_;

  // BoardObs corresponding to this Observation
  std::map<int, std::weak_ptr<BoardObs>> board_observations_;

  // Object3DObs corresponding to this Observation
  std::weak_ptr<Object3D> object_3d_;

  // Validity : An observation is judged not valid when the RANSAC pose
  // estimation return too few pts
  bool valid_ = true;

  // Functions
  Object3DObs();
  ~Object3DObs();
  void initializeObject(std::shared_ptr<Object3D> obj_obs, const int object_idx);
  void insertNewBoardObs(std::shared_ptr<BoardObs> new_board_obs);
  void getPoseVec(cv::Mat &R, cv::Mat &T) const;
  cv::Mat getPoseMat() const;
  void setPoseMat(const cv::Mat Pose);
  void setPoseVec(const cv::Mat Rvec, const cv::Mat T);
  cv::Mat getRotVec() const;
  cv::Mat getTransVec() const;
  void estimatePose(const float ransac_thresh);
  float computeReprojectionError() const;
  void setPoseInGroupMat(const cv::Mat pose);
  void setPoseInGroupVec(const cv::Mat r_vec, const cv::Mat t_vec);
  void getPoseInGroupVec(cv::Mat &r_vec, cv::Mat &t_vec) const;
  cv::Mat getPoseInGroupMat() const;
  cv::Mat getRotInGroupVec() const;
  cv::Mat getTransInGroupVec() const;
};
