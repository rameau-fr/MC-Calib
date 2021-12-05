#pragma once

#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "Board.hpp"
#include "BoardObs.hpp"
#include "Camera.hpp"
#include "Frame.hpp"
#include "Object3DObs.hpp"

/**
 * @class CameraGroup
 *
 * @brief This class contains the CameraGroup information
 *
 * - list of cameras forming the group
 * - frames
 * - boards observation
 * - object observation
 * - reference camera in the group
 */
class CameraGroup {
public:
  // datastructure for this camera group
  std::map<int, std::weak_ptr<Object3DObs>>
      object_observations_; // Observation of the 3D object (2d points)
  std::map<int, std::weak_ptr<Frame>>
      frames_; // Frames containing boards for this cameras
  std::map<int, std::weak_ptr<Camera>> cameras_; // cameras in the camera group
  int nb_cams_ = 0;                 // number of cameras in the group
  std::vector<int> vis_object_idx_; // vector of index of the 3D object

  // extrinsic
  std::map<int, std::vector<double>>
      relative_camera_pose_; // camera pose wrt. the ref. cam
  int id_ref_cam_;
  std::vector<int> cam_idx; // index of the cameras in the group

  // camera group index
  int cam_group_idx_;

  // Functions
  CameraGroup();
  ~CameraGroup();
  void initializeCameraGroup(int id_ref_cam, int cam_group_idx);
  void insertCamera(std::shared_ptr<Camera> new_camera);
  void insertNewObjectObservation(
      std::shared_ptr<Object3DObs> new_object_observation);
  void insertNewFrame(std::shared_ptr<Frame> new_frame);
  void getCameraPoseVec(cv::Mat &r_vec, cv::Mat &t_vec, int id_cam);
  cv::Mat getCameraPoseMat(int id_cam);
  void setCameraPoseMat(cv::Mat pose, int id_cam);
  void setCameraPoseVec(cv::Mat r_vec, cv::Mat t_vec, int id_cam);
  cv::Mat getCameraRotVec(int id_cam);
  cv::Mat getCameraTransVec(int id_cam);
  void computeObjPoseInCameraGroup();
  void refineCameraGroup(int nb_iterations);
  void reproErrorCameraGroup();
  void refineCameraGroupAndObjects(int nb_iterations);
  void refineCameraGroupAndObjectsAndIntrinsics(int nb_iterations);
};
