#pragma once

#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "BoardObs.hpp"
#include "CameraGroupObs.hpp"
#include "CameraObs.hpp"
#include "Object3DObs.hpp"

/**
 * @class Frame
 *
 * @brief Contains information related to the each frame
 *
 * A frame is all the observation from all the synchronized cameras at a given
 * time.
 */
class Frame final
{
public:
  int frame_idx_;

  // Cameras
  std::vector<int> cam_idx_; // the camera index which can see at least one
                             // board in the frame
  std::map<int, std::weak_ptr<CameraObs>>
      cam_obs_; // List of Camera Observation

  // Boards
  std::vector<int> boards_idx_; // index of the visible boards
  std::map<int, std::weak_ptr<BoardObs>> board_observations_; // Boards stored

  // Objects
  std::vector<int> objects_idx_; // index of the visible object
  std::map<int, std::weak_ptr<Object3DObs>>
      object_observations_; // object stored

  // Camera Group obs
  std::vector<int> cam_group_idx_; // index of the cam group for this frame
  std::map<int, std::weak_ptr<CameraGroupObs>>
      cam_group_observations_; // cam group stored

  // Image
  std::map<int, std::string> frame_path_; // camera_id // path

  // Functions
  Frame();
  ~Frame(){};
  void insertNewBoard(std::shared_ptr<BoardObs> newBoard);
  void insertNewCamObs(std::shared_ptr<CameraObs> newCamObs);
  void insertNewObject(std::shared_ptr<Object3DObs> new_object);
  void
  insertNewCameraGroupObs(std::shared_ptr<CameraGroupObs> new_cam_group_obs,
                          const int camera_group_idx);
};
