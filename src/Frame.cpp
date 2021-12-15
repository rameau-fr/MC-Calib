#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "Frame.hpp"

Frame::Frame(const int frame_idx, const int cam_idx, const std::string frame_path) {
  frame_idx_ = frame_idx;
  frame_path_[cam_idx] = frame_path;
}

/**
 * @brief Insert new board observation in this frame
 *
 * @param new_board pointer to the new board
 */
void Frame::insertNewBoard(std::shared_ptr<BoardObs> new_board) {
  boards_idx_.push_back(new_board->board_id_);
  board_observations_[board_observations_.size()] = new_board;
}

/**
 * @brief Insert new object observation in this frame
 *
 * @param new_object pointer to the new obj. observation
 */
void Frame::insertNewObject(std::shared_ptr<Object3DObs> new_object) {
  objects_idx_.push_back(new_object->object_3d_id_);
  object_observations_[object_observations_.size()] = new_object;
}

/**
 * @brief Insert a new camera observation in this frame
 *
 * @param new_cam_obs pointer to new camera observation
 */
void Frame::insertNewCamObs(std::shared_ptr<CameraObs> new_cam_obs) {
  cam_idx_.push_back(new_cam_obs->cam_idx_);
  cam_obs_[cam_obs_.size()] = new_cam_obs;
}

/**
 * @brief Insert new camera group observation
 *
 * @param new_cam_group_obs pointer to the new camera group observation
 * @param camera_group_idx index of the group to be inserted
 */
void Frame::insertNewCameraGroupObs(
    std::shared_ptr<CameraGroupObs> new_cam_group_obs,
    const int camera_group_idx) {
  cam_group_idx_.push_back(camera_group_idx);
  cam_group_observations_[cam_group_observations_.size()] = new_cam_group_obs;
}
