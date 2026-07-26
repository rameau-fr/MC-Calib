#pragma once

#include <filesystem>
#include <iostream>
#include <stdio.h>

#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv_compat.hpp>

#include "BoardObs.hpp"
#include "CameraGroupObs.hpp"
#include "CameraObs.hpp"
#include "Object3DObs.hpp"

namespace McCalib {

/**
 * @class Frame
 *
 * @brief Stores all synchronized observations for one time index.
 *
 * A frame is all the observation from all the synchronized cameras at a given
 * time.
 */
class Frame final {
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
  std::map<int, std::filesystem::path> frame_path_; // camera_id // path

  // Functions
  Frame() = delete;

  /**
   * @brief Destroy the frame container.
   */
  ~Frame(){};

  /**
   * @brief Construct a frame initialized with one camera image path.
   *
   * @param frame_idx Global frame index.
   * @param cam_idx Camera id associated with frame_path.
   * @param frame_path Path of the image for this camera/frame.
   */
  Frame(const int frame_idx, const int cam_idx,
        const std::filesystem::path &frame_path);

  /**
   * @brief Insert a board observation into this frame.
   *
   * @param newBoard Board observation to insert.
   */
  void insertNewBoard(const std::shared_ptr<BoardObs> newBoard);

  /**
   * @brief Insert a camera-level observation into this frame.
   *
   * @param newCamObs Camera observation to insert.
   */
  void insertNewCamObs(const std::shared_ptr<CameraObs> newCamObs);

  /**
   * @brief Insert an object observation into this frame.
   *
   * @param new_object Object observation to insert.
   */
  void insertNewObject(const std::shared_ptr<Object3DObs> new_object);

  /**
   * @brief Insert a camera-group observation into this frame.
   *
   * @param new_cam_group_obs Camera-group observation to insert.
   * @param camera_group_idx Camera-group id for this observation.
   */
  void insertNewCameraGroupObs(
      const std::shared_ptr<CameraGroupObs> new_cam_group_obs,
      const int camera_group_idx);
};

} // namespace McCalib