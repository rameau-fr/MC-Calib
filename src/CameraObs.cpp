#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "CameraObs.hpp"

CameraObs::CameraObs() {}

/**
 * @brief Insert new board observation in the camera observation
 *
 * @param new_board pointer to the board to be inserted
 */
void CameraObs::insertNewBoard(std::shared_ptr<BoardObs> new_board) {
  board_observations_[board_observations_.size()] = new_board;
  board_idx_.push_back(new_board->board_id_);
  cam_idx_ = new_board->camera_id_;
}

/**
 * @brief Insert new object observation in the camera observation
 *
 * @param new_object pointer to the new object
 */
void CameraObs::insertNewObject(std::shared_ptr<Object3DObs> new_object) {
  object_observations_[object_observations_.size()] = new_object;
  object_idx_.push_back(new_object->object_3d_id_);
}
