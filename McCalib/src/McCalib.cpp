#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <stdio.h>

#include "McCalib.hpp"
#include "logger.h"
#include "point_refinement.h"
#include "utilities.hpp"

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>
#include <thread>

namespace McCalib {

/**
 * @brief Initialize the number of cameras and the 3D Boards
 *
 * @param config_path path to the configuration file
 */
Calibration::Calibration(const std::filesystem::path &config_path) {
  cv::FileStorage fs; // cv::FileStorage to read calibration params from file
  int distortion_model;
  std::vector<int> distortion_per_camera;
  std::vector<int> boards_index;
  int nb_x_square, nb_y_square;
  float length_square, length_marker;
  const bool is_file_available = std::filesystem::exists(config_path) &&
                                 config_path.has_filename() &&
                                 config_path.extension() == ".yml";

  if (!is_file_available) {
    LOG_FATAL << "Config path '" << config_path << "' doesn't exist.";
    return;
  }
  fs.open(config_path, cv::FileStorage::READ);

  int nb_camera;
  fs["number_camera"] >> nb_camera;
  assert(nb_camera > 0);
  nb_camera_ = static_cast<unsigned int>(nb_camera);

  int nb_board;
  fs["number_board"] >> nb_board;
  assert(nb_board > 0);
  nb_board_ = static_cast<unsigned int>(nb_board);

  fs["refine_corner"] >> refine_corner_;
  fs["min_perc_pts"] >> min_perc_pts_;
  fs["number_x_square"] >> nb_x_square;
  fs["number_y_square"] >> nb_y_square;
  root_path_ = convertStrToPath(fs["root_path"]);
  fs["cam_prefix"] >> cam_prefix_;
  fs["quaternion_averaging:"] >> quaternion_averaging_;
  fs["ransac_threshold"] >> ransac_thresh_;
  fs["number_iterations"] >> nb_iterations_;
  fs["distortion_model"] >> distortion_model;
  fs["distortion_per_camera"] >> distortion_per_camera;
  fs["boards_index"] >> boards_index;
  fs["length_square"] >> length_square;
  fs["length_marker"] >> length_marker;
  save_path_ = convertStrToPath(fs["save_path"]);
  camera_params_file_name_ = convertStrToPath(fs["camera_params_file_name"]);
  cam_params_path_ = convertStrToPath(fs["cam_params_path"]);
  keypoints_path_ = convertStrToPath(fs["keypoints_path"]);
  fs["save_reprojection"] >> save_repro_;
  fs["save_detection"] >> save_detect_;
  fs["square_size_per_board"] >> square_size_per_board_;
  fs["number_x_square_per_board"] >> number_x_square_per_board_;
  fs["number_y_square_per_board"] >> number_y_square_per_board_;
  fs["resolution_x_per_board"] >> resolution_x_per_board_;
  fs["resolution_y_per_board"] >> resolution_y_per_board_;
  fs["he_approach"] >> he_approach_;
  fs["fix_intrinsic"] >> fix_intrinsic_;

  fs.release(); // close the input file

  // Check if multi-size boards are used or not
  if (boards_index.size() != 0) {
    nb_board_ = boards_index.size();
  }
  int max_board_idx = nb_board_ - 1;
  if (boards_index.size() != 0) {
    max_board_idx = *max_element(boards_index.begin(), boards_index.end());
  }
  if (number_x_square_per_board_.size() == 0) {
    number_x_square_per_board_.assign(max_board_idx + 1, nb_x_square);
    number_y_square_per_board_.assign(max_board_idx + 1, nb_y_square);
  }

  LOG_INFO << "Nb of cameras : " << nb_camera_
           << "   Nb of Boards : " << nb_board_
           << "   Refined Corners : " << refine_corner_
           << "   Distortion mode : " << distortion_model;

  // check if the save dir exist and create it if it does not
  if (!std::filesystem::exists(save_path_) && save_path_.has_filename()) {
    std::filesystem::create_directories(save_path_);
  }

  // prepare the distortion type per camera
  if (distortion_per_camera.size() == 0)
    distortion_per_camera.assign(nb_camera_, distortion_model);

  // Initialize Cameras
  for (std::size_t i = 0; i < nb_camera_; i++) {
    std::shared_ptr<Camera> new_cam =
        std::make_shared<Camera>(i, distortion_per_camera[i]);
    cams_[i] = new_cam;
  }

  // Prepare the charuco patterns
  if (boards_index.size() == 0) {
    boards_index.resize(nb_board_);
    std::iota(boards_index.begin(), boards_index.end(), 0);
  }
  std::map<int, cv::Ptr<cv::aruco::CharucoBoard>> charuco_boards;
  int offset_count = 0;
  for (int i = 0; i <= max_board_idx; i++) {
    cv::Ptr<cv::aruco::CharucoBoard> charuco = cv::aruco::CharucoBoard::create(
        number_x_square_per_board_[i], number_y_square_per_board_[i],
        length_square, length_marker, dict_);
    if (i != 0) // If it is the first board then just use the standard idx
    {
      int id_offset = charuco_boards[i - 1]->ids.size() + offset_count;
      offset_count = id_offset;
      for (auto &id : charuco->ids)
        id += id_offset;
    }
    charuco_boards[i] = charuco;
  }

  // Initialize the 3D boards
  for (std::size_t i = 0; i < nb_board_; i++) {
    // Initialize board
    std::shared_ptr<Board> new_board = std::make_shared<Board>(config_path, i);
    boards_3d_[i] = new_board;
    boards_3d_[i]->nb_pts_ =
        (boards_3d_[i]->nb_x_square_ - 1) * (boards_3d_[i]->nb_y_square_ - 1);
    boards_3d_[i]->charuco_board_ = charuco_boards[boards_index[i]];
    boards_3d_[i]->pts_3d_.reserve(boards_3d_[i]->nb_pts_);
    // Prepare the 3D pts of the boards
    for (int y = 0; y < boards_3d_[i]->nb_y_square_ - 1; y++) {
      for (int x = 0; x < boards_3d_[i]->nb_x_square_ - 1; x++) {
        boards_3d_[i]->pts_3d_.emplace_back(x * boards_3d_[i]->square_size_,
                                            y * boards_3d_[i]->square_size_, 0);
      }
    }
  }
}

/**
 * @brief Detect boards on images with all cameras
 */
void Calibration::detectBoards() {
  const std::unordered_set<cv::String> allowed_exts = {"jpg",  "png", "bmp",
                                                       "jpeg", "jp2", "tiff"};

  // iterate through the cameras
  for (std::size_t cam = 0; cam < nb_camera_; cam++) {
    // prepare the folder's name
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << cam + 1;
    const std::string cam_nb = ss.str();
    const std::filesystem::path cam_path = root_path_ / (cam_prefix_ + cam_nb);
    LOG_INFO << "Extraction camera " << cam_nb;

    // iterate through the images for corner extraction
    std::vector<cv::String> fn;
    cv::glob(cam_path / "*.*", fn, true);

    // filter based on allowed extensions
    std::vector<cv::String> fn_filtered;
    for (const cv::String &cur_path : fn) {
      std::size_t ext_idx = cur_path.find_last_of(".");
      cv::String cur_ext = cur_path.substr(ext_idx + 1);
      if (allowed_exts.find(cur_ext) != allowed_exts.end()) {
        fn_filtered.push_back(cur_path);
      }
    }
    fn = fn_filtered;

    detectBoardsWithCamera(fn, cam);
  }
}

void Calibration::loadDetectedKeypoints() {
  cv::FileStorage fs;
  fs.open(keypoints_path_, cv::FileStorage::READ);
  LOG_INFO << "Loading keypoints from " << keypoints_path_;
  for (unsigned int cam_idx = 0u; cam_idx < nb_camera_; ++cam_idx) {
    // extract camera matrix and distortion coefficients from the file
    cv::FileNode data_per_camera = fs["camera_" + std::to_string(cam_idx)];

    int img_cols;
    int img_rows;
    std::vector<int> frame_idxs;
    std::vector<std::filesystem::path> frame_paths;
    std::vector<int> board_idxs;
    std::vector<std::vector<cv::Point2f>> points;
    std::vector<std::vector<int>> charuco_idxs;

    data_per_camera["img_width"] >> img_cols;
    data_per_camera["img_height"] >> img_rows;
    data_per_camera["frame_idxs"] >> frame_idxs;

    std::vector<std::string> frame_paths_str;
    data_per_camera["frame_paths"] >> frame_paths_str;
    frame_paths = convertVecStrToVecPath(frame_paths_str);

    data_per_camera["board_idxs"] >> board_idxs;
    data_per_camera["pts_2d"] >> points;
    data_per_camera["charuco_idxs"] >> charuco_idxs;

    cams_[cam_idx]->im_cols_ = img_cols;
    cams_[cam_idx]->im_rows_ = img_rows;
    assert(frame_idxs.size() != 0 && frame_idxs.size() == frame_paths.size() &&
           frame_paths.size() == board_idxs.size() &&
           board_idxs.size() == points.size() &&
           points.size() == charuco_idxs.size());

    for (unsigned int i = 0u; i < frame_idxs.size(); ++i) {
      insertNewBoard(cam_idx, frame_idxs[i], board_idxs[i], points[i],
                     charuco_idxs[i], frame_paths[i]);
    }
  }
  fs.release();
}

/**
 * @brief Extract necessary boards info from initialized paths
 *
 */
void Calibration::boardExtraction() {
  if (!keypoints_path_.empty() && keypoints_path_ != "None" &&
      std::filesystem::exists(keypoints_path_)) {
    loadDetectedKeypoints();
  } else {
    detectBoards();
  }
}

/**
 * @brief Detect boards in images with a camera
 *
 * @param fn images paths
 * @param cam_idx camera index which acquire the frame
 */
void Calibration::detectBoardsWithCamera(const std::vector<cv::String> &fn,
                                         const int cam_idx) {
  const unsigned int num_threads = std::thread::hardware_concurrency();
  boost::asio::thread_pool pool(num_threads);
  LOG_INFO << "Number of threads for board detection :: " << num_threads;

  std::size_t num_frames = fn.size();
  if (num_frames > 0u) {
    cv::Mat image = cv::imread(fn[0]);
    if (image.empty()) {
      LOG_ERROR << "Could not read the image :: " << fn[0];
      assert((!image.empty()) && "Calibration cannot be done");
    }
    cams_[cam_idx]->im_cols_ = image.cols;
    cams_[cam_idx]->im_rows_ = image.rows;
  }
  for (std::size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
    LOG_DEBUG << "Frame index :: " << frame_idx;

    // detect the checkerboard on this image
    const std::string frame_path = fn[frame_idx];
    boost::asio::post(pool,
                      std::bind(&Calibration::detectBoardsInImageWithCamera,
                                this, frame_path, cam_idx, frame_idx));
    // displayBoards(currentIm, cam, frameind); // Display frame
  }
  pool.join();
}

/**
 * @brief Detect boards on an image
 *
 * @param frame_path image path on which we would like to detect the board
 * @param cam_idx camera index which acquire the frame
 * @param frame_idx frame index
 */
void Calibration::detectBoardsInImageWithCamera(const std::string &frame_path,
                                                const int cam_idx,
                                                const int frame_idx) {
  cv::Mat image = cv::imread(frame_path);
  if (image.empty()) {
    LOG_ERROR << "Could not read the image :: " << frame_path;
    return;
  }

  // Greyscale image for subpixel refinement
  cv::Mat graymat;
  cv::cvtColor(image, graymat, cv::COLOR_BGR2GRAY);

  // Datastructure to save the checkerboard corners
  // key == board id, value == markersIDs on MARKERS markerIds
  std::map<int, std::vector<int>> marker_idx;
  // key == board id, value == 2d points visualized on MARKERS
  std::map<int, std::vector<std::vector<cv::Point2f>>> marker_corners;
  // key == board id, value == 2d points on checkerboard
  std::map<int, std::vector<cv::Point2f>> charuco_corners;
  // key == board id, value == ID corners on checkerboard
  std::map<int, std::vector<int>> charuco_idx;

  charuco_params_->adaptiveThreshConstant = 1;

  for (std::size_t i = 0; i < nb_board_; i++) {
    cv::aruco::detectMarkers(image, boards_3d_[i]->charuco_board_->dictionary,
                             marker_corners[i], marker_idx[i], charuco_params_);

    if (marker_corners[i].size() > 0) {
      cv::aruco::interpolateCornersCharuco(marker_corners[i], marker_idx[i],
                                           image, boards_3d_[i]->charuco_board_,
                                           charuco_corners[i], charuco_idx[i]);
    }

    if (charuco_corners[i].size() >
        static_cast<std::size_t>(
            std::round(min_perc_pts_ * boards_3d_[i]->nb_pts_))) {
      LOG_INFO << "Number of detected corners :: " << charuco_corners[i].size();
      // Refine the detected corners
      if (refine_corner_ == true) {
        std::vector<SaddlePoint> refined;
        saddleSubpixelRefinement(graymat, charuco_corners[i], refined,
                                 corner_ref_window_, corner_ref_max_iter_);
        for (std::size_t j = 0; j < charuco_corners[i].size(); j++) {
          if (std::isinf(refined[j].x) || std::isinf(refined[j].y)) {
            break;
          }
          charuco_corners[i][j].x = refined[j].x;
          charuco_corners[i][j].y = refined[j].y;
        }
      }

      // Check for colinnerarity
      std::vector<cv::Point2f> pts_on_board_2d;
      pts_on_board_2d.reserve(charuco_idx[i].size());
      for (const auto &charuco_idx_at_board_id : charuco_idx[i]) {
        pts_on_board_2d.emplace_back(
            boards_3d_[i]->pts_3d_[charuco_idx_at_board_id].x,
            boards_3d_[i]->pts_3d_[charuco_idx_at_board_id].y);
      }
      double dum_a = 0.0;
      double dum_b = 0.0;
      double dum_c = 0.0;
      double residual = 0.0;
      calcLinePara(pts_on_board_2d, dum_a, dum_b, dum_c, residual);

      // Add the board if it passes the collinearity check
      if ((residual > boards_3d_[i]->square_size_ * 0.1) &&
          (charuco_corners[i].size() > 4)) {
        int board_idx = i;
        {
          std::unique_lock<std::mutex> lock(insert_new_board_lock_);
          insertNewBoard(cam_idx, frame_idx, board_idx,
                         charuco_corners[board_idx], charuco_idx[board_idx],
                         frame_path);
        }
      }
    }
  }
}

/**
 * @brief Save all cameras parameters
 *
 * The exports include camera intrinsic matrix, distortion vector,
 * img resolution, pose matrix with respect to the reference camera
 *
 */
void Calibration::saveCamerasParams() {

  const std::filesystem::path save_path_camera_params =
      (!camera_params_file_name_.empty())
          ? save_path_ / camera_params_file_name_
          : save_path_ / "calibrated_cameras_data.yml";
  cv::FileStorage fs(save_path_camera_params, cv::FileStorage::WRITE);
  for (const auto &it_cam_group : cam_group_) {
    std::shared_ptr<CameraGroup> cur_cam_group = it_cam_group.second;
    fs << "nb_camera" << static_cast<int>(nb_camera_);
    for (const auto &it_cam : cur_cam_group->cameras_) {
      std::shared_ptr<Camera> cur_cam = it_cam.second.lock();
      if (cur_cam) {
        fs << "camera_" + std::to_string(cur_cam->cam_idx_);
        cv::Mat cam_matrix;
        cv::Mat distortion_vector;
        cur_cam->getIntrinsics(cam_matrix, distortion_vector);
        fs << "{"
           << "camera_matrix" << cam_matrix;
        fs << "distortion_vector" << distortion_vector;
        fs << "distortion_type" << cur_cam->distortion_model_;
        fs << "camera_group" << it_cam_group.first;
        fs << "img_width" << cur_cam->im_cols_;
        fs << "img_height" << cur_cam->im_rows_;
        fs << "camera_pose_matrix"
           << cur_cam_group->getCameraPoseMat(cur_cam->cam_idx_).inv() << "}";
      }
    }

    fs.release();
  }
}

/**
 * @brief Save all the 3D object
 *
 * Export 3D points constituting the objects.
 * Format: X Y Z board_id pts_id
 */
void Calibration::save3DObj() {
  const std::filesystem::path save_path_object =
      save_path_ / "calibrated_objects_data.yml";
  cv::FileStorage fs(save_path_object, cv::FileStorage::WRITE);

  for (const auto &it_obj : object_3d_) {
    unsigned int obj_pts_count = 0;
    std::shared_ptr<Object3D> cur_object = it_obj.second;
    fs << "object_" + std::to_string(cur_object->obj_id_);
    int obj_nb_pts = cur_object->nb_pts_;
    cv::Mat pts_mat(5, obj_nb_pts, CV_32FC1);
    for (const auto &it_board : cur_object->boards_) {
      auto board_ptr = it_board.second.lock();
      if (board_ptr) {
        const int board_idx = board_ptr->board_id_;
        // Replace the keypoints
        for (std::size_t i = 0; i < board_ptr->nb_pts_; i++) {
          std::pair<int, int> board_id_pts_id = std::make_pair(board_idx, i);
          cv::Point3f curr_pts =
              cur_object
                  ->pts_3d_[cur_object->pts_board_2_obj_[board_id_pts_id]];
          pts_mat.at<float>(0, obj_pts_count) = curr_pts.x;
          pts_mat.at<float>(1, obj_pts_count) = curr_pts.y;
          pts_mat.at<float>(2, obj_pts_count) = curr_pts.z;
          pts_mat.at<float>(3, obj_pts_count) = static_cast<float>(board_idx);
          pts_mat.at<float>(4, obj_pts_count) =
              static_cast<float>(board_id_pts_id.second);
          obj_pts_count++;
        }
      }
    }
    fs << "{"
       << "points" << pts_mat;
    fs << "}";
  }
  fs.release();
}

/**
 * @brief Save 3D object poses for each frame where the object is visible
 *
 */
void Calibration::save3DObjPose() {
  const std::filesystem::path save_path_object_pose =
      save_path_ / "calibrated_objects_pose_data.yml";
  cv::FileStorage fs(save_path_object_pose, cv::FileStorage::WRITE);
  for (const auto &it_obj : object_3d_) {
    std::shared_ptr<Object3D> cur_object = it_obj.second;
    fs << "object_" + std::to_string(cur_object->obj_id_);
    fs << "{";
    cv::Mat pose_mat(6, cur_object->object_observations_.size(), CV_64FC1);
    int a = 0;
    for (const auto &it_obj_obs : cur_object->object_observations_) {
      std::shared_ptr<Object3DObs> cur_object_obs = it_obj_obs.second.lock();
      if (cur_object_obs) {
        cv::Mat rot, trans;
        cur_object_obs->getPoseVec(rot, trans);
        pose_mat.at<double>(0, a) = rot.at<double>(0);
        pose_mat.at<double>(1, a) = rot.at<double>(1);
        pose_mat.at<double>(2, a) = rot.at<double>(2);
        pose_mat.at<double>(3, a) = trans.at<double>(0);
        pose_mat.at<double>(4, a) = trans.at<double>(1);
        pose_mat.at<double>(5, a) = trans.at<double>(2);
        a = a + 1;
      }
    }
    fs << "poses" << pose_mat;
    fs << "}";
  }
  fs.release();
}

/**
 * @brief Save detected keypoints
 *
 * The saved keypoints could be re-used during to save the detection time.
 * This could be useful when several calibration experiments are to be done
 * with different parameters with the same detected keypoints.
 *
 */
void Calibration::saveDetectedKeypoints() const {
  const std::filesystem::path save_keypoint_path =
      save_path_ / "detected_keypoints_data.yml";
  cv::FileStorage fs(save_keypoint_path, cv::FileStorage::WRITE);

  fs << "nb_camera" << static_cast<int>(cams_.size());
  for (const auto &it_cam : cams_) {
    const int cam_idx = it_cam.second->cam_idx_;
    fs << "camera_" + std::to_string(cam_idx) << "{";

    const int image_cols = it_cam.second->im_cols_;
    const int image_rows = it_cam.second->im_rows_;
    fs << "img_width" << image_cols;
    fs << "img_height" << image_rows;

    std::vector<int> frame_idxs;
    std::vector<std::string> frame_paths;
    std::vector<int> board_idxs;
    std::vector<std::vector<cv::Point2f>> points;
    std::vector<std::vector<int>> charuco_idxs;

    for (const auto &it_frame : frames_) {
      const int frame_idx = it_frame.second->frame_idx_;
      const std::string &frame_path = it_frame.second->frame_path_[cam_idx];
      for (const auto &it_board : boards_3d_) {
        const int board_idx = it_board.first;

        for (const auto &it_board_obs : board_observations_) {
          if ((cam_idx == it_board_obs.second->camera_id_) &&
              (frame_idx == it_board_obs.second->frame_id_) &&
              (board_idx == it_board_obs.second->board_id_)) {
            const std::vector<cv::Point2f> &pts_2d =
                it_board_obs.second->pts_2d_;
            const std::vector<int> &charuco_idx =
                it_board_obs.second->charuco_id_;

            frame_idxs.push_back(frame_idx);
            frame_paths.push_back(frame_path);
            board_idxs.push_back(board_idx);
            points.push_back(pts_2d);
            charuco_idxs.push_back(charuco_idx);
          }
        }
      }
    }

    fs << "frame_idxs" << frame_idxs;
    fs << "frame_paths" << frame_paths;
    fs << "board_idxs" << board_idxs;
    fs << "pts_2d" << points;
    fs << "charuco_idxs" << charuco_idxs;
    fs << "}";
  }

  fs.release();
}

/**
 * @brief Display the board of cam "cam_idx" at frame "frame_idx"
 *
 * @param image image on which to display the detection
 * @param cam_idx camera index
 * @param frame_idx frame index
 */
void Calibration::displayBoards(const cv::Mat &image, const int cam_idx,
                                const int frame_idx) {
  std::pair<int, int> cam_frame = std::make_pair(cam_idx, frame_idx);
  std::map<std::pair<int, int>, std::shared_ptr<CameraObs>>::iterator it =
      cams_obs_.find(cam_frame); // Check if a frame exist
  if (it != cams_obs_.end()) {
    for (const auto &it : cams_obs_[cam_frame]->board_observations_) {
      auto board_obs_ptr = it.second.lock();
      if (board_obs_ptr) {
        const std::vector<cv::Point2f> &current_pts = board_obs_ptr->pts_2d_;
        std::shared_ptr<Board> board_3d_ptr = board_obs_ptr->board_3d_.lock();
        if (board_3d_ptr) {
          std::array<int, 3> &color_temp = board_3d_ptr->color_;
          for (const auto &current_pt : current_pts) {
            LOG_DEBUG << "Pts x :: " << current_pt.x
                      << "   y :: " << current_pt.y;
            cv::circle(image, cv::Point(current_pt.x, current_pt.y), 4,
                       cv::Scalar(color_temp[0], color_temp[1], color_temp[2]),
                       cv::FILLED, 8, 0);
          }
        }
      }
    }
  }
  // cv::imshow("detected board", image);
  // cv::waitKey(1);
}

/**
 * @brief Update the data structure with a new board to be inserted
 *
 * @param cam_idx camera index in which the board was detected
 * @param frame_idx frame index in which the board was detected
 * @param board_idx board index of the detected board
 * @param pts_2d detected 2D points
 * @param charuco_idx index of the detected points in the board
 */
void Calibration::insertNewBoard(const int cam_idx, const int frame_idx,
                                 const int board_idx,
                                 const std::vector<cv::Point2f> &pts_2d,
                                 const std::vector<int> &charuco_idx,
                                 const std::filesystem::path &frame_path) {
  std::shared_ptr<BoardObs> new_board = std::make_shared<BoardObs>(
      cam_idx, frame_idx, board_idx, pts_2d, charuco_idx, cams_[cam_idx],
      boards_3d_[board_idx]);

  // Add new board in the board list
  board_observations_[board_observations_.size()] = new_board;

  // Add board in the camera
  cams_[cam_idx]->insertNewBoard(new_board);

  // Add board in the Board3D
  boards_3d_[board_idx]->insertNewBoard(new_board);

  // Add board in the Frames list
  std::map<int, std::shared_ptr<Frame>>::iterator it = frames_.find(
      frame_idx); // Check if a frame has already been initialize at this key?
  if (it != frames_.end()) {
    frames_[frame_idx]->insertNewBoard(
        new_board); // If the key already exist just push a new board in there
    frames_[frame_idx]->frame_path_[cam_idx] = frame_path;
  } else {
    std::shared_ptr<Frame> newFrame =
        std::make_shared<Frame>(frame_idx, cam_idx, frame_path);
    frames_[frame_idx] = newFrame; // Initialize the Frame if key does not exist
    frames_[frame_idx]->insertNewBoard(new_board);
    cams_[cam_idx]->insertNewFrame(newFrame);
    boards_3d_[board_idx]->insertNewFrame(newFrame);
  }

  // Add CamObs in the CamobsList and frame
  std::pair<int, int> cam_frame_idx = std::make_pair(cam_idx, frame_idx);
  std::map<std::pair<int, int>, std::shared_ptr<CameraObs>>::iterator itCamObs =
      cams_obs_.find(cam_frame_idx); // Check if a Camobs has already been
                                     // initialize at this key?
  if (itCamObs != cams_obs_.end()) {
    // insert in list
    cams_obs_[cam_frame_idx]->insertNewBoard(new_board);
  } else {
    std::shared_ptr<CameraObs> new_cam_obs =
        std::make_shared<CameraObs>(new_board);
    cams_obs_[cam_frame_idx] = new_cam_obs;
    frames_[frame_idx]->insertNewCamObs(new_cam_obs);
  }
}

/**
 * @brief Insert a new 3D object observation in the data structure
 *
 * @param new_obj_obs pointer to the new object to be inserted
 */
void Calibration::insertNewObjectObservation(
    std::shared_ptr<Object3DObs> new_obj_obs) {
  object_observations_[object_observations_.size()] = new_obj_obs;
}

/**
 * @brief Initialize the calibration of all the cameras individually
 *
 * If cam_params_path_ is set and the file exists the initialization is
 * done using the precalibrated information. Otherwise it is done
 * with a subset of images.
 *
 */
void Calibration::initializeCalibrationAllCam() {
  if (!cam_params_path_.empty() && cam_params_path_ != "None") {
    cv::FileStorage fs;
    const bool is_file_available = std::filesystem::exists(cam_params_path_) &&
                                   cam_params_path_.has_filename() &&
                                   cam_params_path_.extension() == ".yml";
    if (!is_file_available) {
      LOG_FATAL << "Camera parameters path '" << cam_params_path_
                << "' doesn't exist.";
      return;
    }
    fs.open(cam_params_path_, cv::FileStorage::READ);

    LOG_INFO << "Initializing camera calibration from " << cam_params_path_;

    for (const auto &it : cams_) {
      // extract camera matrix and distortion coefficients from the file
      cv::FileNode loaded_cam_params = fs["camera_" + std::to_string(it.first)];

      cv::Mat camera_matrix;
      cv::Mat distortion_coeffs;
      loaded_cam_params["camera_matrix"] >> camera_matrix;
      loaded_cam_params["distortion_vector"] >> distortion_coeffs;

      it.second->setIntrinsics(camera_matrix, distortion_coeffs);
    }

  } else {
    LOG_INFO << "Initializing camera calibration using images";

    for (const auto &it : cams_)
      it.second->initializeCalibration();
  }
}

/**
 * @brief Estimate the boards' pose w.r.t. cameras
 *
 * It is based on a PnP algorithm.
 *
 */
void Calibration::estimatePoseAllBoards() {
  for (const auto &it : board_observations_)
    it.second->estimatePose(ransac_thresh_, nb_iterations_);
}

/**
 * @brief Non-linear refinement of all the cameras intrinsic parameters
 * individually
 *
 */
void Calibration::refineIntrinsicAndPoseAllCam() {
  for (const auto &it : cams_)
    it.second->refineIntrinsicCalibration(nb_iterations_);
}

/**
 * @brief Compute the reprojection error for each boards
 *
 */
void Calibration::computeReproErrAllBoard() {
  for (const auto &it : board_observations_) {
    std::ignore = it.second->computeReprojectionError();
  }
}

/**
 * @brief Find all views where multiple boards are visible and store their
 * relative transformation
 *
 * If two boards appears in a single image their interpose can be computed and
 * stored.
 */
void Calibration::computeBoardsPairPose() {
  board_pose_pairs_.clear();
  for (const auto &it : cams_obs_) {
    std::shared_ptr<CameraObs> current_board = it.second;
    const std::vector<int> &BoardIdx = current_board->board_idx_;

    if (BoardIdx.size() > 1) // if more than one board is visible
    {
      for (const auto &it1 : current_board->board_observations_) {
        auto board1_obs_ptr = it1.second.lock();
        if (board1_obs_ptr) {
          int boardid1 = board1_obs_ptr->board_id_;
          for (const auto &it2 : current_board->board_observations_) {
            auto board2_obs_ptr = it2.second.lock();
            if (board2_obs_ptr) {
              int boardid2 = board2_obs_ptr->board_id_;
              cv::Mat proj_1 = board1_obs_ptr->getPoseMat();
              if (boardid1 != boardid2) // We do not care about the
                                        // transformation with itself ...
              {
                cv::Mat proj_2 = board2_obs_ptr->getPoseMat();
                cv::Mat inter_board_pose = proj_2.inv() * proj_1;
                std::pair<int, int> cam_idx_pair =
                    std::make_pair(boardid1, boardid2);
                board_pose_pairs_[cam_idx_pair].push_back(inter_board_pose);
                LOG_DEBUG << "Multiple boards detected";
              }
            }
          }
        }
      }
    }
  }
}

/**
 * @brief Compute the mean transformation between pose pairs
 *
 * Multiple poses can be computed per frames, these measurements are then
 * averaged.
 *
 * @param pose_pairs from {board_pose_pairs_, camera_pose_pairs_,
 * object_pose_pairs_}
 * @param inter_transform from {inter_board_transform_, inter_camera_transform_,
 * inter_object_transform_}
 */
void Calibration::initInterTransform(
    const std::map<std::pair<int, int>, std::vector<cv::Mat>> &pose_pairs,
    std::map<std::pair<int, int>, cv::Mat> &inter_transform) {
  inter_transform.clear();
  for (const auto &it : pose_pairs) {
    const std::pair<int, int> &pair_idx = it.first;
    const std::vector<cv::Mat> &poses_temp = it.second;
    cv::Mat average_translation = cv::Mat::zeros(3, 1, CV_64F);

    // Median
    const size_t num_poses = poses_temp.size();
    std::vector<double> r1, r2, r3;
    std::vector<double> t1, t2, t3;
    r1.reserve(num_poses);
    r2.reserve(num_poses);
    r3.reserve(num_poses);
    t1.reserve(num_poses);
    t2.reserve(num_poses);
    t3.reserve(num_poses);
    for (const auto &pose_temp : poses_temp) {
      cv::Mat R, T;
      Proj2RT(pose_temp, R, T);
      r1.push_back(R.at<double>(0));
      r2.push_back(R.at<double>(1));
      r3.push_back(R.at<double>(2));
      t1.push_back(T.at<double>(0));
      t2.push_back(T.at<double>(1));
      t3.push_back(T.at<double>(2));
    }

    cv::Mat average_rotation =
        getAverageRotation(r1, r2, r3, quaternion_averaging_);
    average_translation.at<double>(0) = median(t1);
    average_translation.at<double>(1) = median(t2);
    average_translation.at<double>(2) = median(t3);

    inter_transform[pair_idx] =
        RVecT2Proj(average_rotation, average_translation);
    LOG_DEBUG << "Average Rot :: " << average_rotation
              << "    Average Trans :: " << average_translation;
  }
}

/**
 * @brief Initialize the graph with the poses between boards
 *
 */
void Calibration::initInterBoardsGraph() {

  covis_boards_graph_.clearGraph();

  // Each board is a vertex if it has been observed at least once
  for (const auto &it : boards_3d_) {
    if (it.second->board_observations_.size() > 0) {
      covis_boards_graph_.addVertex(it.second->board_id_);
    }
  }

  for (const auto &it : board_pose_pairs_) {
    const std::pair<int, int> &board_pair_idx = it.first;
    const std::vector<cv::Mat> &board_poses_temp = it.second;
    covis_boards_graph_.addEdge(board_pair_idx.first, board_pair_idx.second,
                                ((double)1 / board_poses_temp.size()));
  }
}

/**
 * @brief Initialize 3D objects using mean inter board pose estimation
 *
 * The connected components in the inter-board graph forms an object. An object
 * is formed of multiple board which are not necessarily physically connected
 * together
 */
void Calibration::init3DObjects() {
  // Find the connected components in the graph (each connected components is a
  // new 3D object)
  std::vector<std::vector<int>> connect_comp =
      covis_boards_graph_.connectedComponents();
  LOG_DEBUG << "Number of 3D objects detected :: " << connect_comp.size();

  // Declare a new 3D object for each connected component
  for (std::size_t i = 0; i < connect_comp.size(); i++) {
    LOG_DEBUG << "Obj Id :: " << i;
    LOG_DEBUG << "Number of boards in this object :: "
              << connect_comp[i].size();

    // Find the reference board in this object
    int ref_board_id =
        *min_element(connect_comp[i].begin(), connect_comp[i].end());

    // Declare a new 3D object
    std::shared_ptr<Object3D> newObject3D =
        std::make_shared<Object3D>(connect_comp[i].size(), ref_board_id, i,
                                   boards_3d_[ref_board_id]->color_);
    int pts_count = 0;

    // Compute the shortest path between the reference and the other board
    for (std::size_t j = 0; j < connect_comp[i].size(); j++) {
      int current_board_id = connect_comp[i][j];
      newObject3D->insertBoardInObject(boards_3d_[current_board_id]);

      // Compute the transformation between the reference board and the other
      // boards in the object if the board is not the referential board compute
      // the path
      std::vector<int> short_path = covis_boards_graph_.shortestPathBetween(
          ref_board_id, current_board_id);
      // Compute the transformation wrt. the reference board
      cv::Mat transform = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0,
                           0, 1, 0, 0, 0, 0,
                           1); // initialize the transformation to identity

      if (short_path.size() >= 1u) {
        for (std::size_t k = 0; k < short_path.size() - 1; k++) {
          int current_board = short_path[k];
          int next_board = short_path[k + 1];
          std::pair<int, int> board_pair_idx =
              std::make_pair(current_board, next_board);
          cv::Mat current_trans = inter_board_transform_[board_pair_idx];
          transform = transform * current_trans.inv();
        }
      }

      // Store the relative board transformation in the object
      newObject3D->setBoardPoseMat(current_board_id, transform);

      // Transform the 3D pts to push in the object 3D
      std::vector<cv::Point3f> trans_pts =
          transform3DPts(boards_3d_[current_board_id]->pts_3d_,
                         newObject3D->getBoardRotVec(current_board_id),
                         newObject3D->getBoardTransVec(current_board_id));
      // Make a indexing between board to object
      for (std::size_t k = 0; k < trans_pts.size(); k++) {
        int char_id = k;
        std::pair<int, int> boardid_charid =
            std::make_pair(current_board_id, char_id);
        newObject3D->pts_board_2_obj_[boardid_charid] = pts_count;
        newObject3D->pts_obj_2_board_.push_back(boardid_charid);
        newObject3D->pts_3d_.push_back(trans_pts[k]);
        pts_count++;
      }
      LOG_DEBUG << "Board ID :: " << current_board_id;
    }
    // Add the 3D object into the structure
    object_3d_[i] = newObject3D;
  }
}

/**
 * @brief Initialize 3D objects observation
 *
 * After the "physical" 3D object have been initialized, we gather all the
 * boards observation (belonging to the object) into the 3D object observation.
 *
 * @param object_idx index of the 3D object observed
 */
void Calibration::init3DObjectObs(const int object_idx) {

  // Iterate through cameraobs
  for (const auto &it_cam_obs : cams_obs_) {
    std::pair<int, int> cam_id_frame_id = it_cam_obs.first; // Cam ind/Frame ind
    std::shared_ptr<CameraObs> current_camobs = it_cam_obs.second;

    // Declare the 3D object observed in this camera observation
    // Keep in mind that a single object can be observed in one image
    std::shared_ptr<Object3DObs> object_obs =
        std::make_shared<Object3DObs>(object_3d_[object_idx], object_idx);

    // Check the boards observing this camera
    std::map<int, std::weak_ptr<BoardObs>> current_board_obs =
        current_camobs->board_observations_;
    for (const auto &it_board_obs : current_board_obs) {
      auto board_obs_ptr = it_board_obs.second.lock();
      if (board_obs_ptr) {
        // Check if this board correspond to the object of interest
        std::map<int, std::weak_ptr<Board>>::iterator it =
            object_3d_[object_idx]->boards_.find(board_obs_ptr->board_id_);
        if (it != object_3d_[object_idx]
                      ->boards_.end()) // if the board belong to the object
        {
          object_obs->insertNewBoardObs(board_obs_ptr);
        }
      }
    }

    if (object_obs->pts_id_.size() > 0) {
      // Update the camobs//frame//camera//3DObject
      cams_obs_[cam_id_frame_id]->insertNewObject(object_obs);
      frames_[cam_id_frame_id.second]->insertNewObject(object_obs);
      cams_[it_cam_obs.first.first]->insertNewObject(object_obs);
      object_3d_[object_idx]->insertNewObject(object_obs);
      object_3d_[object_idx]->insertNewFrame(frames_[cam_id_frame_id.second]);
      insertNewObjectObservation(object_obs);
    }
  }
}

/**
 * @brief Initialize all 3D object observations
 *
 */
void Calibration::initAll3DObjectObs() {
  for (const auto &it : object_3d_)
    this->init3DObjectObs(it.first);
}

/**
 * @brief Estimate all the pose of 3D object observation using a PnP algo
 *
 */
void Calibration::estimatePoseAllObjects() {
  for (const auto &it : object_observations_)
    it.second->estimatePose(ransac_thresh_, nb_iterations_);
}

/**
 * @brief Compute the reprojection error for each object
 *
 */
void Calibration::computeReproErrAllObject() {
  std::vector<float> err_vec;
  err_vec.reserve(object_observations_.size());
  for (const auto &it : object_observations_)
    err_vec.push_back(it.second->computeReprojectionError());

  LOG_INFO << "Mean Error "
           << std::accumulate(err_vec.begin(), err_vec.end(), 0.0) /
                  err_vec.size();
}

/**
 * @brief Refine the structure of all the 3D objects
 *
 */
void Calibration::refineAllObject3D() {
  for (const auto &it : object_3d_)
    it.second->refineObject(nb_iterations_);
}

/**
 * @brief Find all view where multiple camera share visible objects and store
 * their relative transformation
 *
 */
void Calibration::computeCamerasPairPose() {
  camera_pose_pairs_.clear();
  // Iterate through frames
  for (const auto &it_frame : frames_) {
    // if more than one observation is available
    if (it_frame.second->board_observations_.size() > 1) {
      // Iterate through the object observation
      std::map<int, std::weak_ptr<Object3DObs>> frame_obj_obs =
          it_frame.second->object_observations_;
      for (const auto &it_objectobs1 : frame_obj_obs) {
        auto obj_obs_1_ptr = it_objectobs1.second.lock();
        if (obj_obs_1_ptr) {
          int cam_id_1 = obj_obs_1_ptr->camera_id_;
          int obj_id_1 = obj_obs_1_ptr->object_3d_id_;
          cv::Mat pose_cam_1 = obj_obs_1_ptr->getPoseMat();
          for (const auto &it_objectobs2 : frame_obj_obs) {
            auto obj_obs_2_ptr = it_objectobs2.second.lock();
            if (obj_obs_2_ptr) {
              int cam_id_2 = obj_obs_2_ptr->camera_id_;
              int obj_id_2 = obj_obs_2_ptr->object_3d_id_;
              cv::Mat pose_cam_2 = obj_obs_2_ptr->getPoseMat();
              if (cam_id_1 != cam_id_2) // if the camera is not the same
              {
                // if the same object is visible from the two cameras
                if (obj_id_1 == obj_id_2) {
                  // Compute the relative pose between the cameras
                  cv::Mat inter_cam_pose =
                      pose_cam_2 * pose_cam_1.inv(); // not sure here ...

                  // Store in a database
                  camera_pose_pairs_[std::make_pair(cam_id_1, cam_id_2)]
                      .push_back(inter_cam_pose);
                }
              }
            }
          }
        }
      }
    }
  }
}

/**
 * @brief Initialize the relationship graph between cameras to form groups
 *
 */
void Calibration::initInterCamerasGraph() {
  covis_camera_graph_.clearGraph();
  // Each camera is a vertex if it has observed at least one object
  for (const auto &it : this->cams_) {
    if (it.second->board_observations_.size() > 0) {
      covis_camera_graph_.addVertex(it.second->cam_idx_);
    }
  }
  // Build the graph with cameras' pairs
  for (const auto &it : camera_pose_pairs_) {
    const std::pair<int, int> &camera_pair_idx = it.first;
    const std::vector<cv::Mat> &camera_poses_temp = it.second;
    covis_camera_graph_.addEdge(camera_pair_idx.first, camera_pair_idx.second,
                                ((double)1 / camera_poses_temp.size()));
  }
}

/**
 * @brief Initialize camera group based on co-visibility pair between cameras
 *
 */
void Calibration::initCameraGroup() {
  // Find the connected components in the graph (each connected components is a
  // new camera group)
  std::vector<std::vector<int>> connect_comp =
      covis_camera_graph_.connectedComponents();
  LOG_DEBUG << "Number of camera group detected :: " << connect_comp.size();

  // Declare a new camera group for each connected component
  for (std::size_t i = 0; i < connect_comp.size(); i++) {
    LOG_DEBUG << "camera group id :: " << i;
    LOG_DEBUG << "Number of cameras in the group :: " << connect_comp.size();

    // Find the reference camera in this group
    int id_ref_cam =
        *min_element(connect_comp[i].begin(), connect_comp[i].end());

    // Declare a new camera group
    std::shared_ptr<CameraGroup> new_camera_group =
        std::make_shared<CameraGroup>(id_ref_cam, i);

    // Compute the shortest path between the reference and the other cams
    for (std::size_t j = 0; j < connect_comp[i].size(); j++) {
      int current_camera_id = connect_comp[i][j];
      new_camera_group->insertCamera(cams_[current_camera_id]);

      // Compute the transformation between the reference cam and the other
      // cam in the group
      std::vector<int> short_path = covis_camera_graph_.shortestPathBetween(
          id_ref_cam, current_camera_id);
      // Compute the transformation wrt. the reference camera
      cv::Mat transform =
          (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
           0, 1); // initialize the transformation to identity

      if (short_path.size() >= 1u) {
        for (std::size_t k = 0; k < short_path.size() - 1; k++) {
          int current_cam = short_path[k];
          int next_cam = short_path[k + 1];
          std::pair<int, int> cam_pair_idx =
              std::make_pair(current_cam, next_cam);
          cv::Mat current_trans = inter_camera_transform_[cam_pair_idx];
          // transform = transform * current_trans.inv();
          transform = transform * current_trans;
        }
      }
      // Store the relative camera transformation in the object
      new_camera_group->setCameraPoseMat(transform, current_camera_id);
    }
    // Add the 3D camera group into the structure
    cam_group_[i] = new_camera_group;
  }
}

/**
 * @brief Initialize camera group observation
 *
 * @param camera_group_idx camera group index
 */
void Calibration::initCameraGroupObs(const int camera_group_idx) {
  // List of camera idx in the group
  const std::vector<int> &cam_in_group = cam_group_[camera_group_idx]->cam_idx;

  // Iterate through frame
  for (const auto &it_frame : frames_) {
    int current_frame_id = it_frame.second->frame_idx_;
    std::shared_ptr<CameraGroupObs> new_cam_group_obs =
        std::make_shared<CameraGroupObs>(
            cam_group_[camera_group_idx],
            quaternion_averaging_); // declare a new observation

    std::map<int, std::weak_ptr<Object3DObs>> current_object_obs =
        it_frame.second->object_observations_;
    for (const auto &it_obj_obs : current_object_obs) {
      auto obj_obs_ptr = it_obj_obs.second.lock();
      if (obj_obs_ptr) {
        int current_cam_id = obj_obs_ptr->camera_id_;

        // Check if this camera id belongs to the group
        if (std::find(cam_in_group.begin(), cam_in_group.end(),
                      current_cam_id) != cam_in_group.end()) {
          // the camera is in the group so this object is visible in the cam
          // group udpate the observation
          new_cam_group_obs->insertObjectObservation(obj_obs_ptr);

          // push the object observation in the camera group
          cam_group_[camera_group_idx]->insertNewObjectObservation(obj_obs_ptr);
        }
      }
    }
    if (new_cam_group_obs->object_observations_.size() > 0) {
      // add the new cam group observation to the database
      cams_group_obs_[std::make_pair(camera_group_idx, current_frame_id)] =
          new_cam_group_obs;

      // update frame
      frames_[current_frame_id]->insertNewCameraGroupObs(new_cam_group_obs,
                                                         camera_group_idx);

      // update the cam_group
      cam_group_[camera_group_idx]->insertNewFrame(frames_[current_frame_id]);
    }
  }
}

/**
 * @brief Initialize all the camera groups observations
 *
 */
void Calibration::initAllCameraGroupObs() {
  for (const auto &it : cam_group_) {
    int camera_group_idx = it.second->cam_group_idx_;
    initCameraGroupObs(camera_group_idx);
  }
}

/**
 * @brief Non-linear optimization of the camera pose in the groups and the pose
 * of the observed objects
 *
 */
void Calibration::refineAllCameraGroup() {
  for (const auto &it : cam_group_) {
    // it->second->computeObjPoseInCameraGroup();
    it.second->refineCameraGroup(nb_iterations_);
  }

  // Update the object3D observation
  for (const auto &it : cams_group_obs_)
    it.second->updateObjObsPose();
}

/**
 * @brief Find the pair of objects to use to calibrate the pairs of camera
 * groups
 *
 * The pair of object with the highest number of occurrences are used for
 * calibration.
 */
void Calibration::findPairObjectForNonOverlap() {
  no_overlap_object_pair_.clear();
  // Iterate through the camera groups
  for (const auto &it_groups_1 : cam_group_) {
    int group_idx1 = it_groups_1.first;
    for (const auto &it_groups_2 : cam_group_) {
      int group_idx2 = it_groups_2.first;
      if (group_idx1 != group_idx2) // if the two groups are different
      {
        // Prepare the list of possible objects pairs
        std::map<std::pair<int, int>, unsigned int> count_pair_obs;
        for (const auto &it_obj1 : object_3d_) {
          for (const auto &it_obj2 : object_3d_) {
            int obj_id1 = it_obj1.second->obj_id_;
            int obj_id2 = it_obj2.second->obj_id_;
            if (obj_id1 != obj_id2)
              count_pair_obs[std::make_pair(obj_id1, obj_id2)] = 0u;
          }
        }

        // move to shared_ptr cause there is no = for weak_ptr
        std::map<int, std::shared_ptr<Frame>> it_groups_1_frames;
        std::map<int, std::shared_ptr<Frame>> it_groups_2_frames;
        for (const auto &item : it_groups_1.second->frames_) {
          if (auto frame_ptr = item.second.lock())
            it_groups_1_frames[item.first] = frame_ptr;
        }
        for (const auto &item : it_groups_2.second->frames_) {
          if (auto frame_ptr = item.second.lock())
            it_groups_2_frames[item.first] = frame_ptr;
        }

        // Find frames in common
        std::map<int, std::shared_ptr<Frame>> common_frames;
        std::map<int, std::shared_ptr<Frame>>::iterator it_frames(
            common_frames.begin());
        std::set_intersection(
            it_groups_1_frames.begin(), it_groups_1_frames.end(),
            it_groups_2_frames.begin(), it_groups_2_frames.end(),
            std::inserter(common_frames, it_frames));

        // Iterate through the frames and count the occurrence of object pair
        // (to select the pair of object appearing the most)
        for (const auto &it_common_frames : common_frames) {
          // Find the index of the observation corresponding to the groups in
          // cam group obs
          auto it_camgroupid_1 =
              find(it_common_frames.second->cam_group_idx_.begin(),
                   it_common_frames.second->cam_group_idx_.end(), group_idx1);
          auto it_camgroupid_2 =
              find(it_common_frames.second->cam_group_idx_.begin(),
                   it_common_frames.second->cam_group_idx_.end(), group_idx2);
          int index_camgroup_1 =
              it_camgroupid_1 - it_common_frames.second->cam_group_idx_.begin();
          int index_camgroup_2 =
              it_camgroupid_2 - it_common_frames.second->cam_group_idx_.begin();

          // Access the objects 3D index for both groups
          auto common_frames_cam_group1_obs_ptr =
              it_common_frames.second->cam_group_observations_[index_camgroup_1]
                  .lock();
          auto common_frames_cam_group2_obs_ptr =
              it_common_frames.second->cam_group_observations_[index_camgroup_2]
                  .lock();
          if (common_frames_cam_group1_obs_ptr &&
              common_frames_cam_group2_obs_ptr) {
            std::map<int, std::weak_ptr<Object3DObs>> object_obs_1 =
                common_frames_cam_group1_obs_ptr->object_observations_;
            std::map<int, std::weak_ptr<Object3DObs>> object_obs_2 =
                common_frames_cam_group2_obs_ptr->object_observations_;
            for (const auto &it_object_obs_1 : object_obs_1) {
              auto object_obs_1_ptr = it_object_obs_1.second.lock();
              if (object_obs_1_ptr) {
                int obj_ind_1 = object_obs_1_ptr->object_3d_id_;
                for (const auto &it_object_obs_2 : object_obs_2) {
                  auto object_obs_2_ptr = it_object_obs_2.second.lock();
                  if (object_obs_2_ptr) {
                    int obj_ind_2 = object_obs_2_ptr->object_3d_id_;
                    count_pair_obs[std::make_pair(obj_ind_1, obj_ind_2)]++;
                  }
                }
              }
            }
          }
        }

        // find the pair of object with the maximum shared frames
        unsigned int currentMax = 0u;
        std::pair<int, int> arg_max = std::make_pair(0, 0);
        for (const auto &it : count_pair_obs) {
          if (it.second > currentMax) {
            arg_max = it.first;
            currentMax = it.second;
          }
        }

        // Save in the data structure
        LOG_DEBUG << "max visibility, Object 1 :: " << arg_max.first
                  << "Object 2 :: " << arg_max.second;
        LOG_DEBUG << "Number of occurrence :: " << currentMax;
        no_overlap_object_pair_[std::make_pair(group_idx1, group_idx2)] =
            arg_max;
      }
    }
  }
}

/**
 * @brief Handeye calibration of a pair of non overlapping pair of group of
 * cameras
 *
 */
void Calibration::initNonOverlapPair(const int cam_group_id1,
                                     const int cam_group_id2) {
  // Prepare the group of interest
  std::shared_ptr<CameraGroup> cam_group1 = cam_group_[cam_group_id1];
  std::shared_ptr<CameraGroup> cam_group2 = cam_group_[cam_group_id2];

  // Check the object per camera
  std::pair<int, int> object_pair =
      no_overlap_object_pair_[std::make_pair(cam_group_id1, cam_group_id2)];
  int object_cam_1 = object_pair.first;
  int object_cam_2 = object_pair.second;

  // Prepare the 3D objects
  std::shared_ptr<Object3D> object_3D_1 = object_3d_[object_cam_1];
  std::shared_ptr<Object3D> object_3D_2 = object_3d_[object_cam_2];

  // std::vector to store data for non-overlapping calibration
  std::vector<cv::Mat> pose_abs_1,
      pose_abs_2; // absolute pose stored to compute relative displacements
  cv::Mat repo_obj_1_2; // reprojected pts for clustering

  // move to shared_ptr cause there is no = for weak_ptr
  std::map<int, std::shared_ptr<Frame>> cam_group1_frames;
  std::map<int, std::shared_ptr<Frame>> cam_group2_frames;
  for (const auto &item : cam_group1->frames_) {
    if (auto frames_ptr = item.second.lock())
      cam_group1_frames[item.first] = frames_ptr;
  }
  for (const auto &item : cam_group2->frames_) {
    if (auto frames_ptr = item.second.lock())
      cam_group2_frames[item.first] = frames_ptr;
  }

  // Find frames in common
  std::map<int, std::shared_ptr<Frame>> common_frames;
  std::map<int, std::shared_ptr<Frame>>::iterator it_frames(
      common_frames.begin());
  std::set_intersection(cam_group1_frames.begin(), cam_group1_frames.end(),
                        cam_group2_frames.begin(), cam_group2_frames.end(),
                        std::inserter(common_frames, it_frames));

  // Iterate through common frames and reproject the objects in the images to
  // cluster
  for (const auto &it_common_frames : common_frames) {
    // Find the index of the observation corresponding to the groups in cam
    // group obs
    auto it_camgroupid_1 =
        find(it_common_frames.second->cam_group_idx_.begin(),
             it_common_frames.second->cam_group_idx_.end(), cam_group_id1);
    auto it_camgroupid_2 =
        find(it_common_frames.second->cam_group_idx_.begin(),
             it_common_frames.second->cam_group_idx_.end(), cam_group_id2);
    int index_camgroup_1 =
        it_camgroupid_1 - it_common_frames.second->cam_group_idx_.begin();
    int index_camgroup_2 =
        it_camgroupid_2 - it_common_frames.second->cam_group_idx_.begin();

    // check if both objects of interest are in the frame
    std::weak_ptr<CameraGroupObs> cam_group_obs1 =
        it_common_frames.second->cam_group_observations_[index_camgroup_1];
    std::weak_ptr<CameraGroupObs> cam_group_obs2 =
        it_common_frames.second->cam_group_observations_[index_camgroup_2];

    auto cam_group_obs1_ptr = cam_group_obs1.lock();
    auto cam_group_obs2_ptr = cam_group_obs2.lock();
    if (cam_group_obs1_ptr && cam_group_obs2_ptr) {
      const std::vector<int> &cam_group_obs_obj1 =
          cam_group_obs1_ptr->object_idx_;
      const std::vector<int> &cam_group_obs_obj2 =
          cam_group_obs2_ptr->object_idx_;
      auto it1 = find(cam_group_obs_obj1.begin(), cam_group_obs_obj1.end(),
                      object_cam_1);
      auto it2 = find(cam_group_obs_obj2.begin(), cam_group_obs_obj2.end(),
                      object_cam_2);
      bool obj_vis1 = it1 != cam_group_obs_obj1.end();
      bool obj_vis2 = it2 != cam_group_obs_obj2.end();
      int index_objobs_1 = it1 - cam_group_obs_obj1.begin();
      int index_objobs_2 = it2 - cam_group_obs_obj2.begin();

      // if both objects are visible
      if (obj_vis1 & obj_vis2) {
        // Reproject 3D objects in ref camera group
        auto cam_group_obs1_cam_group_ptr =
            cam_group_obs1_ptr->cam_group_.lock();
        auto cam_group_obs2_cam_group_ptr =
            cam_group_obs2_ptr->cam_group_.lock();
        if (cam_group_obs1_cam_group_ptr && cam_group_obs2_cam_group_ptr) {
          std::weak_ptr<Camera> ref_cam_1 =
              cam_group_obs1_cam_group_ptr
                  ->cameras_[cam_group_obs1_cam_group_ptr->id_ref_cam_];
          std::weak_ptr<Camera> ref_cam_2 =
              cam_group_obs2_cam_group_ptr
                  ->cameras_[cam_group_obs2_cam_group_ptr->id_ref_cam_];

          auto obj_obs1_ptr =
              cam_group_obs1_ptr->object_observations_[index_objobs_1].lock();
          auto obj_obs2_ptr =
              cam_group_obs2_ptr->object_observations_[index_objobs_2].lock();
          if (obj_obs1_ptr && obj_obs2_ptr) {
            int object_id1 = obj_obs1_ptr->object_3d_id_;
            int object_id2 = obj_obs2_ptr->object_3d_id_;
            cv::Mat pose_obj_1 =
                cam_group_obs1_ptr->getObjectPoseMat(object_id1);
            cv::Mat pose_obj_2 =
                cam_group_obs2_ptr->getObjectPoseMat(object_id2);
            pose_abs_1.push_back(pose_obj_1);
            pose_abs_2.push_back(pose_obj_2);
          }
        }
      }
    }
  }

  // Check if enough common poses are available:
  if (pose_abs_1.size() <= 3 || pose_abs_2.size() <= 3) {
    return;
  }

  // HANDEYE CALIBRATION
  cv::Mat pose_g1_g2;
  if (he_approach_ == 0) {
    // Boot strapping technique
    int nb_cluster = 20;
    int nb_it_he = 200; // Nb of time we apply the handeye calibration
    pose_g1_g2 = handeyeBootstratpTranslationCalibration(
        nb_cluster, nb_it_he, pose_abs_1, pose_abs_2);
  } else {
    pose_g1_g2 = handeyeCalibration(pose_abs_1, pose_abs_2);
  }

  pose_g1_g2 = pose_g1_g2.inv();
  // Save the parameter in the datastructure
  no_overlap_camgroup_pair_pose_[std::make_pair(cam_group_id1, cam_group_id2)] =
      pose_g1_g2;
  no_overlap__camgroup_pair_common_cnt_[std::make_pair(
      cam_group_id1, cam_group_id2)] = pose_abs_1.size();
}

/**
 * @brief Initialize the pose between all groups of non-overlaping camera group
 */
void Calibration::findPoseNoOverlapAllCamGroup() {
  no_overlap_camgroup_pair_pose_.clear();
  no_overlap_camgroup_pair_pose_.clear();

  // Iterate through the camera groups
  for (const auto &it_groups_1 : cam_group_) {
    int group_idx1 = it_groups_1.first;
    for (const auto &it_groups_2 : cam_group_) {
      int group_idx2 = it_groups_2.first;
      if (group_idx1 != group_idx2) // if the two groups are different
      {
        initNonOverlapPair(group_idx1, group_idx2);
      }
    }
  }
}

/**
 * @brief Create graph between nonoverlap camera groups
 *
 */
void Calibration::initInterCamGroupGraph() {
  no_overlap_camgroup_graph_.clearGraph();
  // All the existing groups form a vertex
  for (const auto &it : cam_group_) {
    if (it.second->object_observations_.size() > 0) {
      no_overlap_camgroup_graph_.addVertex(it.second->cam_group_idx_);
    }
  }

  // Create the graph
  for (const auto &it : no_overlap_camgroup_pair_pose_) {
    const std::pair<int, int> &camgroup_pair_idx = it.first;
    int nb_common_frame =
        no_overlap__camgroup_pair_common_cnt_[camgroup_pair_idx];
    no_overlap_camgroup_graph_.addEdge(camgroup_pair_idx.first,
                                       camgroup_pair_idx.second,
                                       ((double)1 / nb_common_frame));
  }
}

/**
 * @brief Merge all camera groups using non-overlaping pose estimation
 *
 */
void Calibration::mergeCameraGroup() {
  // Find the connected components in the graph
  std::vector<std::vector<int>> connect_comp =
      no_overlap_camgroup_graph_.connectedComponents();
  std::map<int, std::shared_ptr<CameraGroup>> cam_group; // list of camera group

  for (std::size_t i = 0; i < connect_comp.size(); i++) {
    // find the reference camera group reference and the camera reference among
    // all the groups
    int id_ref_cam_group =
        *std::min_element(connect_comp[i].begin(), connect_comp[i].end());
    int id_ref_cam = cam_group_[id_ref_cam_group]->id_ref_cam_;

    // Recompute the camera pose in the referential of the reference group
    std::map<int, cv::Mat>
        cam_group_pose_to_ref; // pose of the cam group in the cam group

    // Used the graph to find the transformations of camera groups to the
    // reference group
    for (std::size_t j = 0; j < connect_comp[i].size(); j++) {
      int current_cam_group_id = connect_comp[i][j];
      // Compute the transformation between the reference group and the current
      // group
      std::vector<int> short_path =
          no_overlap_camgroup_graph_.shortestPathBetween(id_ref_cam_group,
                                                         current_cam_group_id);
      // Compute the transformation wrt. the reference camera
      cv::Mat transform =
          (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
           0, 1); // initialize the transformation to identity

      if (short_path.size() >= 1u) {
        for (std::size_t k = 0; k < short_path.size() - 1; k++) {
          int current_group = short_path[k];
          int next_group = short_path[k + 1];
          std::pair<int, int> group_pair_idx =
              std::make_pair(current_group, next_group);
          cv::Mat current_trans =
              no_overlap_camgroup_pair_pose_[group_pair_idx];
          // transform = transform * current_trans.inv();
          transform = transform * current_trans;
        }
      }
      // Store the poses
      cam_group_pose_to_ref[current_cam_group_id] = transform;
    }

    // initialize the camera group
    std::shared_ptr<CameraGroup> new_camera_group =
        std::make_shared<CameraGroup>(id_ref_cam, i);
    // Iterate through the camera groups and add all the cameras individually in
    // the new group
    for (const auto &it_group : cam_group_) {
      // Check if the current camera group belong to the final group
      int current_cam_group_idx = it_group.second->cam_group_idx_;
      if (std::find(connect_comp[i].begin(), connect_comp[i].end(),
                    current_cam_group_idx) != connect_comp[i].end()) {
        // Prepare the current group pose in the referential of the final group
        std::shared_ptr<CameraGroup> current_group = it_group.second;
        cv::Mat pose_in_final =
            cam_group_pose_to_ref[current_group->cam_group_idx_];
        // the camera group is in the final group so we include its cameras
        for (const auto &it_cam : current_group->cameras_) {
          std::shared_ptr<Camera> current_camera = it_cam.second.lock();
          if (current_camera) {
            // Update the pose in the referential of the final group
            cv::Mat pose_cam_in_current_group =
                current_group->getCameraPoseMat(current_camera->cam_idx_);
            cv::Mat transform = pose_cam_in_current_group * pose_in_final;
            new_camera_group->insertCamera(current_camera);
            new_camera_group->setCameraPoseMat(transform,
                                               current_camera->cam_idx_);
          }
        }
      }
    }
    cam_group[i] = new_camera_group;
  }
  // Erase previous camera group and replace it with the merged one
  cam_group_.clear();
  cam_group_ = cam_group;
}

/**
 * @brief Merge the camera groups observation
 *
 */
void Calibration::mergeAllCameraGroupObs() {
  // First we erase all the Cameragroups observation in the entire datastructure
  for (const auto &it_frame : frames_) {
    it_frame.second->cam_group_idx_.clear();
    it_frame.second->cam_group_observations_.clear();
  }
  cams_group_obs_.clear();

  // Reinitialize all camera obserations
  for (const auto &it : cam_group_) {
    int camera_group_idx = it.second->cam_group_idx_;
    initCameraGroupObs(camera_group_idx);
  }
}

/**
 * @brief Compute the 3D object position in the camera group
 *
 */
void Calibration::computeAllObjPoseInCameraGroup() {
  for (const auto &it : cam_group_)
    it.second->computeObjPoseInCameraGroup();
  // Compute the pose of each object in the camera groups obs
  for (const auto &it_cam_group_obs : cams_group_obs_)
    it_cam_group_obs.second->computeObjectsPose();
}

/**
 * @brief Find all frames where multiple objects are visible and store their
 * relative transformation.
 *
 * If two object appears in a single frames their
 * interpose can be computed and stored.
 */
void Calibration::computeObjectsPairPose() {
  object_pose_pairs_.clear();
  // Iterate through camera group obs
  for (const auto &it_cam_group_obs : cams_group_obs_) {
    if (it_cam_group_obs.second->object_idx_.size() > 1) {
      std::map<int, std::weak_ptr<Object3DObs>> obj_obs =
          it_cam_group_obs.second->object_observations_;
      for (const auto &it_object1 : obj_obs) {
        auto it_object1_ptr = it_object1.second.lock();
        if (it_object1_ptr) {
          int object_3d_id_1 = it_object1_ptr->object_3d_id_;
          cv::Mat obj_pose_1 =
              it_cam_group_obs.second->getObjectPoseMat(object_3d_id_1);
          // cv::Mat obj_pose_1 = it_object1->second->getPoseInGroupMat();
          for (const auto &it_object2 : obj_obs) {
            auto it_object2_ptr = it_object2.second.lock();
            if (it_object2_ptr) {
              int object_3d_id_2 = it_object2_ptr->object_3d_id_;
              cv::Mat obj_pose_2 =
                  it_cam_group_obs.second->getObjectPoseMat(object_3d_id_2);
              // cv::Mat obj_pose_2 = it_object2->second->getPoseInGroupMat();
              if (object_3d_id_1 != object_3d_id_2) {
                cv::Mat inter_object_pose = obj_pose_2.inv() * obj_pose_1;
                std::pair<int, int> object_idx_pair =
                    std::make_pair(object_3d_id_1, object_3d_id_2);
                object_pose_pairs_[object_idx_pair].push_back(
                    inter_object_pose);
              }
            }
          }
        }
      }
    }
  }
}

/**
 * @brief Initialize the graph with the poses between objects
 *
 */
void Calibration::initInterObjectsGraph() {

  covis_objects_graph_.clearGraph();
  // Each object is a vertex if it has been observed at least once
  for (const auto &it : object_3d_) {
    if (it.second->object_observations_.size() > 0) {
      covis_objects_graph_.addVertex(it.second->obj_id_);
    }
  }

  for (const auto &it : object_pose_pairs_) {
    const std::pair<int, int> &object_pair_idx = it.first;
    const std::vector<cv::Mat> &object_poses_temp = it.second;
    covis_objects_graph_.addEdge(object_pair_idx.first, object_pair_idx.second,
                                 ((double)1 / (object_poses_temp.size())));
  }
  LOG_DEBUG << "GRAPH INTER OBJECT DONE";
}

/**
 * @brief Merge all objects groups which have been visible in same camera groups
 *
 */
void Calibration::mergeObjects() {
  // find the connected objects in the graph
  std::vector<std::vector<int>> connect_comp =
      covis_objects_graph_.connectedComponents();
  std::map<int, std::shared_ptr<Object3D>> object_3d; // list of object 3D

  for (std::size_t i = 0; i < connect_comp.size(); i++) {
    // find the reference camera group reference and the camera reference among
    // all the groups
    int id_ref_object =
        *std::min_element(connect_comp[i].begin(), connect_comp[i].end());
    int ref_board_id = object_3d_[id_ref_object]->ref_board_id_;

    // recompute the board poses in the referential of the reference object
    std::map<int, cv::Mat>
        object_pose_to_ref; // pose of the object in the ref object
    int nb_board_in_obj = 0;

    // Used the graph to find the transformations of objects to the reference
    // object
    for (std::size_t j = 0; j < connect_comp[i].size(); j++) {
      nb_board_in_obj += object_3d_[connect_comp[i][j]]->boards_.size();
      int current_object_id = connect_comp[i][j];
      // Compute the transformation between the reference object and the current
      // object
      std::vector<int> short_path = covis_objects_graph_.shortestPathBetween(
          id_ref_object, current_object_id);
      // Compute the transformation wrt. the reference object
      cv::Mat transform =
          (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
           0, 1); // initialize the transformation to identity

      if (short_path.size() >= 1u) {
        for (std::size_t k = 0; k < short_path.size() - 1; k++) {
          int current_object = short_path[k];
          int next_object = short_path[k + 1];
          std::pair<int, int> object_pair_idx =
              std::make_pair(current_object, next_object);
          cv::Mat current_trans = inter_object_transform_[object_pair_idx];
          transform = transform * current_trans.inv(); // original
          // transform = transform * current_trans;
        }
      }
      // Store the poses
      object_pose_to_ref[current_object_id] = transform;
    }
    // initialize the object
    std::shared_ptr<Object3D> newObject3D = std::make_shared<Object3D>(
        nb_board_in_obj, ref_board_id, i, boards_3d_[ref_board_id]->color_);
    int pts_count = 0;
    // Iterate through the objects and add all of them individually in the new
    // object
    for (const auto &it_object : object_3d_) {
      // Check if the current object belong to the new object
      int current_object_idx = it_object.second->obj_id_;
      if (std::find(connect_comp[i].begin(), connect_comp[i].end(),
                    current_object_idx) != connect_comp[i].end()) {
        // Prepare the current object pose in the referential of the merged
        // object
        std::shared_ptr<Object3D> current_object = it_object.second;
        cv::Mat pose_in_merged = object_pose_to_ref[current_object->obj_id_];
        // the object is in the merged group so we include its boards
        for (const auto &it_board : current_object->boards_) {
          std::shared_ptr<Board> current_board = it_board.second.lock();
          if (current_board) {
            // Update the pose to be in the referential of the merged object
            cv::Mat pose_board_in_current_obj =
                current_object->getBoardPoseMat(current_board->board_id_);
            cv::Mat transform = pose_in_merged * pose_board_in_current_obj;

            // insert new board
            newObject3D->insertBoardInObject(current_board);
            // Store the relative board transformation in the object
            newObject3D->setBoardPoseMat(current_board->board_id_, transform);
            // Transform the 3D pts to push in the object 3D
            std::vector<cv::Point3f> trans_pts = transform3DPts(
                current_board->pts_3d_,
                newObject3D->getBoardRotVec(current_board->board_id_),
                newObject3D->getBoardTransVec(current_board->board_id_));
            // Make a indexing between board to object
            for (std::size_t k = 0; k < trans_pts.size(); k++) {
              int char_id = k;
              std::pair<int, int> boardid_charid =
                  std::make_pair(current_board->board_id_, char_id);
              newObject3D->pts_board_2_obj_[boardid_charid] = pts_count;
              newObject3D->pts_obj_2_board_.push_back(boardid_charid);
              newObject3D->pts_3d_.push_back(trans_pts[k]);
              pts_count++;
            }
          }
        }
      }
    }

    // Add the 3D object into the structure
    object_3d[i] = newObject3D;
  }

  // Update the 3D object
  object_3d_.clear();
  object_3d_ = object_3d;
}

/**
 * @brief Merge the object observation
 *
 */
void Calibration::mergeAllObjectObs() {

  // First we erase all the object observation in the entire datastructure
  for (const auto &it : cams_) {
    it.second->vis_object_idx_.clear();
    it.second->object_observations_.clear();
  }

  for (const auto &it : cam_group_) {
    it.second->vis_object_idx_.clear();
    it.second->object_observations_.clear();
  }

  for (const auto &it : cams_group_obs_) {
    it.second->object_idx_.clear();
    it.second->object_observations_.clear();
  }

  for (const auto &it : cams_obs_) {
    it.second->object_observations_.clear();
    it.second->object_idx_.clear();
  }

  for (const auto &it : frames_) {
    it.second->object_observations_.clear();
    it.second->objects_idx_.clear();
  }

  for (const auto &it : object_3d_)
    it.second->object_observations_.clear();

  object_observations_.clear();

  // Reinitialize all the 3D object
  for (const auto &it_object : object_3d_) {
    (void)it_object;
    // Reinitialize all object obserations
    for (const auto &it : object_3d_)
      this->init3DObjectObs(it.first);
  }
}

/**
 * @brief Compute the reprojection error for each camera group
 *
 */
void Calibration::reproErrorAllCamGroup() {
  for (const auto &it : cam_group_)
    it.second->reproErrorCameraGroup();
}

/**
 * @brief Non-linear optimization of the camera pose in the groups, the pose
 * of the observed objects, and the pose of the boards in the 3D objects
 *
 */
void Calibration::refineAllCameraGroupAndObjects() {
  for (const auto &it : cam_group_)
    it.second->refineCameraGroupAndObjects(nb_iterations_);

  // Update the 3D objects
  for (const auto &it : object_3d_)
    it.second->updateObjectPts();

  // Update the object3D observation
  for (const auto &it : cams_group_obs_)
    it.second->updateObjObsPose();
}

/**
 * @brief Save reprojection results images for a given camera.
 *
 */
void Calibration::saveReprojectionImages(const int cam_id) {
  // Prepare the path to save the images
  const std::filesystem::path path_root = save_path_ / "Reprojection";
  std::stringstream ss;
  ss << std::setw(3) << std::setfill('0') << cam_id;
  const std::string cam_folder = ss.str();
  const std::filesystem::path path_save = path_root / cam_folder;

  // check if the file exist and create it if it does not
  if (!std::filesystem::exists(path_root) && path_root.has_filename()) {
    std::filesystem::create_directories(path_root);
  }
  if (!std::filesystem::exists(path_save) && path_root.has_filename()) {
    std::filesystem::create_directory(path_save);
  }

  std::shared_ptr<Camera> cam = cams_[cam_id];

  // Iterate through the frames where this camera has visibility
  for (const auto &it_frame : frames_) {
    // Open the image
    const std::filesystem::path im_path = it_frame.second->frame_path_[cam_id];
    cv::Mat image = cv::imread(im_path);

    // Iterate through the camera group observations
    std::map<int, std::weak_ptr<CameraGroupObs>> cam_group_obs =
        it_frame.second->cam_group_observations_;
    for (const auto &it_cam_group_obs : cam_group_obs) {
      auto it_cam_group_obs_ptr = it_cam_group_obs.second.lock();
      if (it_cam_group_obs_ptr) {
        // Iterate through the object observation
        std::map<int, std::weak_ptr<Object3DObs>> object_observations =
            it_cam_group_obs_ptr->object_observations_;
        for (const auto &it_obj_obs : object_observations) {
          auto it_obj_obs_ptr = it_obj_obs.second.lock();
          auto it_obj_obs_cam_group_ptr =
              it_cam_group_obs_ptr->cam_group_.lock();
          auto it_obj_obs_object_3d_ptr = it_obj_obs_ptr->object_3d_.lock();
          if (it_obj_obs_ptr && it_obj_obs_cam_group_ptr &&
              it_obj_obs_object_3d_ptr &&
              it_obj_obs_ptr->camera_id_ == cam_id) {
            // Prepare the transformation matrix
            cv::Mat cam_pose =
                it_obj_obs_cam_group_ptr->getCameraPoseMat(cam_id) *
                it_obj_obs_ptr->getPoseInGroupMat();
            cv::Mat rot_vec, trans_vec;
            Proj2RT(cam_pose, rot_vec, trans_vec);

            // Get the 2d and 3d pts
            const std::vector<cv::Point2f> &pts_2d = it_obj_obs_ptr->pts_2d_;
            const std::vector<int> &pts_ind = it_obj_obs_ptr->pts_id_;
            const std::vector<cv::Point3f> &pts_3d_obj =
                it_obj_obs_object_3d_ptr->pts_3d_;
            std::vector<cv::Point2f> pts_repro;
            std::vector<cv::Point3f> pts_3d;
            pts_3d.reserve(pts_ind.size());
            for (const auto &pt_ind : pts_ind)
              pts_3d.emplace_back(pts_3d_obj[pt_ind]);

            // Reproject the pts
            cv::Mat rr, tt;
            rot_vec.copyTo(rr);
            trans_vec.copyTo(tt);
            projectPointsWithDistortion(pts_3d, rr, tt, cam->getCameraMat(),
                                        cam->getDistortionVectorVector(),
                                        cam->distortion_model_, pts_repro);

            // plot the keypoints on the image (red project // green detected)
            std::vector<double> color_repro{0, 0, 255};
            std::vector<double> color_detect{0, 255, 0};
            for (std::size_t i = 0; i < pts_2d.size(); i++) {
              cv::circle(
                  image, cv::Point(pts_repro[i].x, pts_repro[i].y), 4,
                  cv::Scalar(color_repro[0], color_repro[1], color_repro[2]),
                  cv::FILLED, 8, 0);
              cv::circle(
                  image, cv::Point(pts_2d[i].x, pts_2d[i].y), 4,
                  cv::Scalar(color_detect[0], color_detect[1], color_detect[2]),
                  cv::FILLED, 8, 0);
            }
          }
        }
      }
    }

    if (!image.empty()) {
      // display image
      // cv::imshow("reprojection_error", image);
      // cv::waitKey(1);

      // Save image
      std::stringstream ss1;
      ss1 << std::setw(6) << std::setfill('0') << it_frame.second->frame_idx_;
      const std::string image_name = ss1.str() + ".jpg";
      cv::imwrite(path_save / image_name, image);
    }
  }
}

/**
 * @brief Save reprojection images for all camera
 *
 */
void Calibration::saveReprojectionImagesAllCam() {
  for (const auto &it : cams_)
    saveReprojectionImages(it.second->cam_idx_);
}

/**
 * @brief Save detection results images for a given camera
 *
 */
void Calibration::saveDetectionImages(const int cam_id) {
  // Prepare the path to save the images
  const std::filesystem::path path_root = save_path_ / "Detection";
  std::stringstream ss;
  ss << std::setw(3) << std::setfill('0') << cam_id;
  std::string cam_folder = ss.str();
  const std::filesystem::path path_save = path_root / cam_folder;

  // check if the file exist and create it if it does not
  if (!std::filesystem::exists(path_root) && path_root.has_filename()) {
    std::filesystem::create_directories(path_root);
  }
  if (!std::filesystem::exists(path_save) && path_root.has_filename()) {
    std::filesystem::create_directory(path_save);
  }

  std::shared_ptr<Camera> cam = cams_[cam_id];

  // Iterate through the frames where this camera has visibility
  for (const auto &it_frame : frames_) {
    // Open the image
    const std::filesystem::path im_path = it_frame.second->frame_path_[cam_id];
    cv::Mat image = cv::imread(im_path);

    // Iterate through the camera group observations
    std::map<int, std::weak_ptr<CameraGroupObs>> cam_group_obs =
        it_frame.second->cam_group_observations_;
    for (const auto &it_cam_group_obs : cam_group_obs) {
      auto it_cam_group_obs_ptr = it_cam_group_obs.second.lock();
      if (it_cam_group_obs_ptr) {
        // Iterate through the object observation
        std::map<int, std::weak_ptr<Object3DObs>> object_observations =
            it_cam_group_obs_ptr->object_observations_;
        for (const auto &it_obj_obs : object_observations) {
          auto it_obj_obs_ptr = it_obj_obs.second.lock();
          auto it_obj_obs_object_3d_ptr = it_obj_obs_ptr->object_3d_.lock();
          if (it_obj_obs_ptr && it_obj_obs_ptr->camera_id_ == cam_id &&
              it_obj_obs_object_3d_ptr) {
            // Get the 2d and 3d pts
            const std::vector<cv::Point2f> &pts_2d = it_obj_obs_ptr->pts_2d_;
            // plot the keypoints on the image (red project // green detected)
            std::array<int, 3> &color = it_obj_obs_object_3d_ptr->color_;
            for (const auto &pt_2d : pts_2d) {
              cv::circle(image, cv::Point(pt_2d.x, pt_2d.y), 4,
                         cv::Scalar(color[0], color[1], color[2]), cv::FILLED,
                         8, 0);
            }
          }
        }
      }
    }

    if (!image.empty()) {
      // display image
      // cv::imshow("detection results", image);
      // cv::waitKey(1);

      // Save image
      std::stringstream ss1;
      ss1 << std::setw(6) << std::setfill('0') << it_frame.second->frame_idx_;
      std::string image_name = ss1.str() + ".jpg";
      cv::imwrite(path_save / image_name, image);
    }
  }
}

/**
 * @brief Save detection images for all camera
 *
 */
void Calibration::saveDetectionImagesAllCam() {
  for (const auto &it : cams_)
    saveDetectionImages(it.second->cam_idx_);
}

/**
 * @brief Initialize the intrinsic parameters and board pose of the entire
 * system
 *
 */
void Calibration::initIntrinsic() {
  initializeCalibrationAllCam();
  estimatePoseAllBoards();
  if (fix_intrinsic_ == 0) {
    refineIntrinsicAndPoseAllCam();
  }
  computeReproErrAllBoard();
}

/**
 * @brief Calibrate 3D objects
 *
 */
void Calibration::calibrate3DObjects() {
  computeBoardsPairPose();
  initInterTransform(board_pose_pairs_, inter_board_transform_);
  initInterBoardsGraph();
  init3DObjects();
  initAll3DObjectObs();
  estimatePoseAllObjects();
  computeReproErrAllObject();
  refineAllObject3D();
  computeReproErrAllObject();
}

/**
 * @brief Calibrate Camera groups
 *
 */
void Calibration::calibrateCameraGroup() {
  computeCamerasPairPose();
  initInterTransform(camera_pose_pairs_, inter_camera_transform_);
  initInterCamerasGraph();
  initCameraGroup();
  initAllCameraGroupObs();
  computeAllObjPoseInCameraGroup();
  refineAllCameraGroupAndObjects();
}

/**
 * @brief Merge objects
 *
 */
void Calibration::merge3DObjects() {
  initInterCamGroupGraph();
  estimatePoseAllObjects();
  computeAllObjPoseInCameraGroup();
  computeObjectsPairPose();
  initInterTransform(object_pose_pairs_, inter_object_transform_);
  initInterObjectsGraph();
  this->reproErrorAllCamGroup();
  mergeObjects();
  mergeAllObjectObs();
  mergeAllCameraGroupObs();
  estimatePoseAllObjects();
  computeAllObjPoseInCameraGroup();
  refineAllCameraGroupAndObjects();
  this->reproErrorAllCamGroup();
}

/**
 * @brief Compute distance between std::vectors of 2D points
 *
 * @return list of distances between points
 */
cv::Mat Calibration::computeDistanceBetweenPoints(
    const std::vector<cv::Point2f> &obj_pts_2d,
    const std::vector<cv::Point2f> &repro_pts) {
  cv::Mat error_list;
  for (std::size_t i = 0; i < repro_pts.size(); i++) {
    float rep_err = std::sqrt(std::pow((obj_pts_2d[i].x - repro_pts[i].x), 2) +
                              std::pow((obj_pts_2d[i].y - repro_pts[i].y), 2));
    error_list.push_back(rep_err);
  }
  return error_list;
}

/**
 * @brief Compute average reprojection error per camera group, cameras, frames,
 * observations
 *
 * @return average reprojection error
 */
double Calibration::computeAvgReprojectionError() {
  cv::Mat frame_list;
  cv::Scalar total_avg_error_sum;
  int number_of_adds = 0;

  for (const auto &it : cam_group_) {
    int cam_group_idx = it.second->cam_group_idx_;
    std::shared_ptr<CameraGroup> cur_cam_group = it.second;

    // iterate through frames
    for (const auto &it_frame : cur_cam_group->frames_) {
      auto it_frame_ptr = it_frame.second.lock();
      if (it_frame_ptr) {
        cv::Mat camera_list;
        frame_list.push_back(it_frame_ptr->frame_idx_);

        // iterate through cameraGroupObs
        std::map<int, std::weak_ptr<CameraGroupObs>> current_cam_group_obs_vec =
            it_frame_ptr->cam_group_observations_;
        for (const auto &it_cam_group_obs : current_cam_group_obs_vec) {
          // check if the current group is the camera group of interest
          auto it_cam_group_obs_ptr = it_cam_group_obs.second.lock();
          if (it_cam_group_obs_ptr &&
              cam_group_idx == it_cam_group_obs_ptr->cam_group_idx_) {
            std::map<int, std::weak_ptr<Object3DObs>> current_obj3d_obs_vec =
                it_cam_group_obs_ptr->object_observations_;

            // iterate through 3D object obs
            for (const auto &it_obj3d : current_obj3d_obs_vec) {
              auto it_obj3d_ptr = it_obj3d.second.lock();
              auto it_obj3d_object_3d_ptr = it_obj3d_ptr->object_3d_.lock();
              auto it_obj3d_cam_ptr = it_obj3d_ptr->cam_.lock();
              if (it_obj3d_ptr && it_obj3d_object_3d_ptr && it_obj3d_cam_ptr) {
                int current_cam_id = it_obj3d_ptr->camera_id_;
                const std::vector<cv::Point3f> &obj_pts_3d =
                    it_obj3d_object_3d_ptr->pts_3d_;
                const std::vector<int> &obj_pts_idx = it_obj3d_ptr->pts_id_;
                const std::vector<cv::Point2f> &obj_pts_2d =
                    it_obj3d_ptr->pts_2d_;
                camera_list.push_back(current_cam_id);

                // compute the reprojection error
                std::vector<cv::Point3f> object_pts;
                for (const auto &obj_pt_idx : obj_pts_idx)
                  object_pts.push_back(obj_pts_3d[obj_pt_idx]);

                // apply object pose transform
                std::vector<cv::Point3f> object_pts_trans1 =
                    transform3DPts(object_pts,
                                   it_cam_group_obs_ptr->getObjectRotVec(
                                       it_obj3d_ptr->object_3d_id_),
                                   it_cam_group_obs_ptr->getObjectTransVec(
                                       it_obj3d_ptr->object_3d_id_));
                // reproject pts
                std::vector<cv::Point2f> repro_pts;
                std::shared_ptr<Camera> cam_ptr = it_obj3d_cam_ptr;
                projectPointsWithDistortion(
                    object_pts_trans1,
                    it.second->getCameraRotVec(current_cam_id),
                    it.second->getCameraTransVec(current_cam_id),
                    cam_ptr->getCameraMat(),
                    cam_ptr->getDistortionVectorVector(),
                    cam_ptr->distortion_model_, repro_pts);

                cv::Mat error_list =
                    computeDistanceBetweenPoints(obj_pts_2d, repro_pts);
                total_avg_error_sum += cv::mean(error_list);
                number_of_adds++;
              }
            }
          }
        }
      }
    }
  }

  return total_avg_error_sum.val[0] / number_of_adds;
}

/**
 * @brief Save reprojection error in a file for analysis
 *
 */
void Calibration::saveReprojectionErrorToFile() {
  const std::filesystem::path save_reprojection_error =
      save_path_ / "reprojection_error_data.yml";
  cv::FileStorage fs(save_reprojection_error, cv::FileStorage::WRITE);
  cv::Mat frame_list;
  int nb_cam_group = cam_group_.size();
  fs << "nb_camera_group" << nb_cam_group;

  for (const auto &it : cam_group_) {
    int cam_group_idx = it.second->cam_group_idx_;
    fs << "camera_group_" + std::to_string(cam_group_idx);
    fs << "{";
    std::shared_ptr<CameraGroup> cur_cam_group = it.second;
    // iterate through frames
    for (const auto &it_frame : cur_cam_group->frames_) {
      std::shared_ptr<Frame> it_frame_ptr = it_frame.second.lock();
      if (it_frame_ptr) {
        cv::Mat camera_list;
        fs << "frame_" + std::to_string(it_frame_ptr->frame_idx_);
        fs << "{";
        frame_list.push_back(it_frame_ptr->frame_idx_);

        // iterate through cameraGroupObs
        std::map<int, std::weak_ptr<CameraGroupObs>> current_cam_group_obs_vec =
            it_frame_ptr->cam_group_observations_;
        for (const auto &it_cam_group_obs : current_cam_group_obs_vec) {
          auto it_cam_group_obs_ptr = it_cam_group_obs.second.lock();
          // check if the current group is the camera group of interest
          if (it_cam_group_obs_ptr &&
              cam_group_idx == it_cam_group_obs_ptr->cam_group_idx_) {
            std::map<int, std::weak_ptr<Object3DObs>> current_obj3d_obs_vec =
                it_cam_group_obs_ptr->object_observations_;

            // iterate through 3D object obs
            for (const auto &it_obj3d : current_obj3d_obs_vec) {
              auto it_obj3d_ptr = it_obj3d.second.lock();
              auto it_obj3d_object_3d_ptr = it_obj3d_ptr->object_3d_.lock();
              auto it_cam_group_obs_ptr = it_cam_group_obs.second.lock();
              auto it_obj3d_cam_ptr = it_obj3d_ptr->cam_.lock();
              if (it_obj3d_ptr && it_obj3d_object_3d_ptr &&
                  it_cam_group_obs_ptr && it_obj3d_cam_ptr) {
                int current_cam_id = it_obj3d_ptr->camera_id_;
                const std::vector<cv::Point3f> &obj_pts_3d =
                    it_obj3d_object_3d_ptr->pts_3d_;
                const std::vector<int> &obj_pts_idx = it_obj3d_ptr->pts_id_;
                const std::vector<cv::Point2f> &obj_pts_2d =
                    it_obj3d_ptr->pts_2d_;
                camera_list.push_back(current_cam_id);
                fs << "camera_" + std::to_string(current_cam_id);
                fs << "{";

                // compute the reprojection error
                std::vector<cv::Point3f> object_pts;
                for (const auto &obj_pt_idx : obj_pts_idx)
                  object_pts.push_back(obj_pts_3d[obj_pt_idx]);

                int nb_pts = obj_pts_idx.size();
                fs << "nb_pts" << nb_pts;

                // apply object pose transform
                std::vector<cv::Point3f> object_pts_trans1 =
                    transform3DPts(object_pts,
                                   it_cam_group_obs_ptr->getObjectRotVec(
                                       it_obj3d_ptr->object_3d_id_),
                                   it_cam_group_obs_ptr->getObjectTransVec(
                                       it_obj3d_ptr->object_3d_id_));
                // reproject pts
                std::vector<cv::Point2f> repro_pts;
                std::shared_ptr<Camera> cam_ptr = it_obj3d_cam_ptr;
                projectPointsWithDistortion(
                    object_pts_trans1,
                    it.second->getCameraRotVec(current_cam_id),
                    it.second->getCameraTransVec(current_cam_id),
                    cam_ptr->getCameraMat(),
                    cam_ptr->getDistortionVectorVector(),
                    cam_ptr->distortion_model_, repro_pts);

                cv::Mat error_list =
                    computeDistanceBetweenPoints(obj_pts_2d, repro_pts);
                fs << "error_list" << error_list << "}";
              }
            }
          }
        }
        fs << "camera_list" << camera_list << "}";
      }
    }
    fs << "frame_list" << frame_list << "}";
  }
  fs.release();
}

/**
 * @brief Non-linear optimization of the camera pose in the groups, the pose
 * of the observed objects, and the pose of the boards in the 3D objects
 *
 */
void Calibration::refineAllCameraGroupAndObjectsAndIntrinsic() {
  for (const auto &it : cam_group_)
    it.second->refineCameraGroupAndObjectsAndIntrinsics(nb_iterations_);

  // Update the 3D objects
  for (const auto &it : object_3d_)
    it.second->updateObjectPts();

  // Update the object3D observation
  for (const auto &it : cams_group_obs_)
    it.second->updateObjObsPose();
}

} // namespace McCalib
