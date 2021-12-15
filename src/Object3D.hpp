#pragma once

#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

class Board;
class BoardObs;
class Object3DObs;
class Frame;
class Camera;

/**
 * @class Object3D
 *
 * @brief This class contains the 3D object information
 *
 * - 3D points
 * - local 3D index (0-N)
 * - observation of these objects
 * - frames where these 3D objects are observed
 */
class Object3D final
{
public:
  // Parameters
  int nb_boards_;             // number of boards constituting the 3D object
  int nb_pts_;                // Number of points in the 3D object
  std::vector<double> color_; // color of the 3D object
  int obj_id_;                // id of the 3D object
  int ref_board_id_; // the id of the reference board (which will take the pose
                     // I|0)

  // 3D points
  std::vector<cv::Point3f> pts_3d_; // 3D points in the object

  // Indexing (from pts board to pts 3D obj and vice-versa)
  std::map<std::pair<int, int>, int>
      pts_board_2_obj_; // key(board_ind,pts_ind) --> pts_ind_obj
  std::vector<std::pair<int, int>>
      pts_obj_2_board_; // key(boardid//ptsid)-->pts_ind_board

  // Boards composing the object
  std::map<int, std::weak_ptr<Board>> boards_;
  std::map<int, double *> relative_board_pose_;

  // List of object observation for this 3D object
  std::map<int, std::weak_ptr<Object3DObs>> object_observations_;

  // List of frames where this board is visible
  std::map<int, std::weak_ptr<Frame>> frames_;

  // Functions
  Object3D() = delete;
  ~Object3D();
  Object3D(const int nb_boards, const int ref_board_id, const int obj_id, const std::vector<double> color);
  void insertBoardInObject(std::shared_ptr<Board> new_board);
  void insertNewObject(std::shared_ptr<Object3DObs> new_object);
  void insertNewFrame(std::shared_ptr<Frame> new_frame);
  void getBoardPoseVec(cv::Mat &r_vec, cv::Mat &t_vec, int board_id);
  cv::Mat getBoardPoseMat(int board_id);
  void setBoardPoseMat(cv::Mat pose, int board_id);
  void setBoardPoseVec(cv::Mat r_vec, cv::Mat t_vec, int board_id);
  cv::Mat getBoardRotVec(int board_id);
  cv::Mat getBoardTransVec(int board_id);
  void refineObject(const int nb_iterations);
  void updateObjectPts();
};
