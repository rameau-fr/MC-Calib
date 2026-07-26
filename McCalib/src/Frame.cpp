#include <iostream>
#include <stdio.h>

#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv_compat.hpp>

#include "Frame.hpp"

namespace McCalib {

/**
 * @brief Construct a frame initialized with one camera image path.
 *
 * @param frame_idx Global frame index.
 * @param cam_idx Camera id associated with frame_path.
 * @param frame_path Image path for this camera/frame pair.
 */
Frame::Frame(const int frame_idx, const int cam_idx,
             const std::filesystem::path &frame_path) {
  frame_idx_ = frame_idx;
  frame_path_[cam_idx] = frame_path;
}

/**
 * @brief Insert a board observation into this frame.
 *
 * @param new_board Board observation to insert.
 */
void Frame::insertNewBoard(const std::shared_ptr<BoardObs> new_board) {
  boards_idx_.push_back(new_board->board_id_);
  board_observations_[board_observations_.size()] = new_board;
}

/**
 * @brief Insert an object observation into this frame.
 *
 * @param new_object Object observation to insert.
 */
void Frame::insertNewObject(const std::shared_ptr<Object3DObs> new_object) {
  objects_idx_.push_back(new_object->object_3d_id_);
  object_observations_[object_observations_.size()] = new_object;
}

/**
 * @brief Insert a camera-level observation into this frame.
 *
 * @param new_cam_obs Camera observation to insert.
 */
void Frame::insertNewCamObs(const std::shared_ptr<CameraObs> new_cam_obs) {
  cam_idx_.push_back(new_cam_obs->cam_idx_);
  cam_obs_[cam_obs_.size()] = new_cam_obs;
}

/**
 * @brief Insert a camera-group observation into this frame.
 *
 * @param new_cam_group_obs Camera-group observation to insert.
 * @param camera_group_idx Index of the associated camera group.
 */
void Frame::insertNewCameraGroupObs(
    const std::shared_ptr<CameraGroupObs> new_cam_group_obs,
    const int camera_group_idx) {
  cam_group_idx_.push_back(camera_group_idx);
  cam_group_observations_[cam_group_observations_.size()] = new_cam_group_obs;
}

} // namespace McCalib