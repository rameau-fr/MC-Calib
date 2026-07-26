#pragma once

#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv_compat.hpp>
#include <stdio.h>

namespace McCalib {

class Board;
class BoardObs;
class Object3DObs;
class Frame;
class Camera;

/**
 * @class Object3D
 *
 * @brief Represents a rigid 3D object composed of multiple boards.
 *
 * It stores merged object geometry, board-to-object relative poses and all
 * related observations across frames.
 */
class Object3D final {
public:
  // Parameters
  int nb_boards_;    // number of boards constituting the 3D object
  int ref_board_id_; // the id of the reference board (which will take the pose
                     // // I|0)
  int obj_id_;       // id of the 3D object
  unsigned int nb_pts_;      // Number of points in the 3D object
  std::array<int, 3> color_; // color of the 3D object

  // 3D points
  std::vector<cv::Point3f> pts_3d_; // 3D points in the object

  // Indexing (from pts board to pts 3D obj and vice-versa)
  std::map<std::pair<int, int>, int>
      pts_board_2_obj_; // key(board_ind,pts_ind) --> pts_ind_obj
  std::vector<std::pair<int, int>>
      pts_obj_2_board_; // key(boardid//ptsid)-->pts_ind_board

  // Boards composing the object
  std::map<int, std::weak_ptr<Board>> boards_;
  std::map<int, std::array<double, 6>> relative_board_pose_;

  // List of object observation for this 3D object
  std::map<int, std::weak_ptr<Object3DObs>> object_observations_;

  // List of frames where this board is visible
  std::map<int, std::weak_ptr<Frame>> frames_;

  // Functions
  Object3D() = delete;

  /**
   * @brief Destroy the 3D object.
   */
  ~Object3D(){};

  /**
   * @brief Construct a 3D object descriptor.
   *
   * @param nb_boards Number of boards used to define the object.
   * @param ref_board_id Board id used as object reference.
   * @param obj_id Object id.
   * @param color Display color.
   */
  Object3D(const int nb_boards, const int ref_board_id, const int obj_id,
           const std::array<int, 3> &color);

  /**
   * @brief Add a board to this object.
   *
   * @param new_board Board to insert.
   */
  void insertBoardInObject(const std::shared_ptr<Board> new_board);

  /**
   * @brief Add an object observation.
   *
   * @param new_object Observation to insert.
   */
  void insertNewObject(const std::shared_ptr<Object3DObs> new_object);

  /**
   * @brief Add a frame where this object is observed.
   *
   * @param new_frame Frame to insert.
   */
  void insertNewFrame(const std::shared_ptr<Frame> new_frame);

  /**
   * @brief Get board pose in object coordinates as Rodrigues + translation.
   *
   * @param board_id Board id in this object.
   * @param r_vec Output rotation vector (3x1).
   * @param t_vec Output translation vector (3x1).
   */
  void getBoardPoseVec(const int board_id, cv::Mat &r_vec, cv::Mat &t_vec);

  /**
   * @brief Get board pose in object coordinates as a 4x4 transform.
   *
   * @param board_id Board id in this object.
   * @return 4x4 board pose in object frame.
   */
  cv::Mat getBoardPoseMat(const int board_id);

  /**
   * @brief Set board pose in object coordinates from a 4x4 transform.
   *
   * @param board_id Board id in this object.
   * @param pose 4x4 board pose in object frame.
   */
  void setBoardPoseMat(const int board_id, const cv::Mat &pose);

  /**
   * @brief Set board pose in object coordinates from vector form.
   *
   * @param board_id Board id in this object.
   * @param r_vec Rotation vector (3x1).
   * @param t_vec Translation vector (3x1).
   */
  void setBoardPoseVec(const int board_id, const cv::Mat &r_vec,
                       const cv::Mat &t_vec);

  /**
   * @brief Get only the rotation component of a board pose.
   *
   * @param board_id Board id in this object.
   * @return Rotation vector (3x1).
   */
  cv::Mat getBoardRotVec(const int board_id);

  /**
   * @brief Get only the translation component of a board pose.
   *
   * @param board_id Board id in this object.
   * @return Translation vector (3x1).
   */
  cv::Mat getBoardTransVec(const int board_id);

  /**
   * @brief Refine object structure and associated observation poses.
   *
   * @param nb_iterations Number of optimization iterations.
   */
  void refineObject(const int nb_iterations);

  /**
   * @brief Rebuild merged object points from current board poses.
   */
  void updateObjectPts();
};

} // namespace McCalib