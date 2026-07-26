#pragma once

#include "BoardObs.hpp"
#include "Object3DObs.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv_compat.hpp>
#include <stdio.h>

namespace McCalib {

class Camera;

/**
 * @class CameraObs
 *
 * @brief Collects all board/object observations for one camera in one frame.
 *
 * This container groups per-frame detections already associated to a specific
 * camera id.
 */
class CameraObs final {
public:
  // Boards
  std::vector<int> board_idx_; // index of the visible 3D boards
  std::map<int, std::weak_ptr<BoardObs>> board_observations_; // Boards stored
  int cam_idx_ = 0;
  // std::shared_ptr<Camera>cam_;

  // Objects
  std::vector<int> object_idx_; // index of the visible 3D objects
  std::map<int, std::weak_ptr<Object3DObs>>
      object_observations_; // Objects stored

  // Functions
  CameraObs() = delete;

  /**
   * @brief Destroy the camera observation.
   */
  ~CameraObs(){};

  /**
   * @brief Build a camera observation initialized with one board observation.
   *
   * @param new_board First board observation seen by this camera.
   */
  CameraObs(const std::shared_ptr<BoardObs> new_board);

  /**
   * @brief Insert a board observation.
   *
   * @param new_board Board observation to insert.
   */
  void insertNewBoard(const std::shared_ptr<BoardObs> new_board);

  /**
   * @brief Insert an object observation.
   *
   * @param new_object Object observation to insert.
   */
  void insertNewObject(const std::shared_ptr<Object3DObs> new_object);
};

} // namespace McCalib