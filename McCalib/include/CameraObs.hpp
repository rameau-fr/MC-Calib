#pragma once

#include "BoardObs.hpp"
#include "Object3DObs.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

namespace McCalib {

class Camera;

/**
 * @class CameraObs
 *
 * @brief Information related to camera observation
 *
 * A camera observation contains all the board/object observed by the camera
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
  ~CameraObs(){};
  CameraObs(std::shared_ptr<BoardObs> new_board);
  void insertNewBoard(std::shared_ptr<BoardObs> new_board);
  void insertNewObject(std::shared_ptr<Object3DObs> new_object);
};

} // namespace McCalib