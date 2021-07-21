#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <stdio.h>

#include "Calibration.hpp"
#include "logger.h"
#include "point_refinement.h"

Calibration::Calibration() {}

/**
 * @brief Initialize the number of cameras and the 3D Boards
 *
 * @param config_path path to the configuration file
 */
void Calibration::initialization(std::string config_path) {
  cv::FileStorage fs; // cv::FileStorage to read calibration params from file
  int distortion_model;
  std::vector<int> distortion_per_camera;
  std::vector<int> boards_index;
  int nb_x_square, nb_y_square;
  float length_square, length_marker;
  fs.open(config_path, cv::FileStorage::READ);
  fs["number_camera"] >> nb_camera_;
  fs["number_board"] >> nb_board_;
  fs["refine_corner"] >> refine_corner_;
  fs["min_perc_pts"] >> min_perc_pts_;
  fs["number_x_square"] >> nb_x_square;
  fs["number_y_square"] >> nb_y_square;
  fs["root_path"] >> root_dir_;
  fs["cam_prefix"] >> cam_prefix_;
  fs["ransac_threshold"] >> ransac_thresh_;
  fs["number_iterations"] >> nb_iterations_;
  fs["distortion_model"] >> distortion_model;
  fs["distortion_per_camera"] >> distortion_per_camera;
  fs["boards_index"] >> boards_index;
  fs["length_square"] >> length_square;
  fs["length_marker"] >> length_marker;
  fs["save_path"] >> save_path_;
  fs["camera_params_file_name"] >> camera_params_file_name_;
  fs["cam_params_path"] >> cam_params_path_;
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
  if (square_size_per_board_.size() == 0) {
    for (int i = 0; i <= max_board_idx; i++) {
      number_x_square_per_board_.push_back(nb_x_square);
      number_y_square_per_board_.push_back(nb_y_square);
    }
  }

  LOG_INFO << "Nb of cameras : " << nb_camera_
           << "   Nb of Boards : " << nb_board_
           << "   Refined Corners : " << refine_corner_
           << "   Distortion mode : " << distortion_model;

  // check if the save dir exist and create it if it does not
  if (!boost::filesystem::exists(save_path_)) {
    boost::filesystem::create_directories(save_path_);
  }

  // prepare the distortion type per camera
  if (distortion_per_camera.size() == 0) {
    for (int i = 0; i < nb_camera_; i++)
      distortion_per_camera.push_back(distortion_model);
  }

  // Initialize Cameras
  for (int i = 0; i < nb_camera_; i++) {
    std::shared_ptr<Camera> new_cam = std::make_shared<Camera>();
    new_cam->cam_idx_ = i;
    new_cam->distortion_model_ = distortion_per_camera[i];
    cams_[i] = new_cam;
  }

  // Prepare the charuco patterns
  if (boards_index.size() == 0) {
    for (int i = 0; i < nb_board_; i++)
      boards_index.push_back(i);
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
  for (int i = 0; i < nb_board_; i++) {
    // Initialize board
    std::shared_ptr<Board> new_board = std::make_shared<Board>();
    boards_3d_[i] = new_board;
    boards_3d_[i]->initParams(config_path, i);
    LOG_DEBUG << "Here1";
    // Prepare the 3D pts of the boards
    for (int y = 0; y < boards_3d_[i]->nb_y_square_ - 1; y++) {
      for (int x = 0; x < boards_3d_[i]->nb_x_square_ - 1; x++) {
        float X = x * boards_3d_[i]->square_size_;
        float Y = y * boards_3d_[i]->square_size_;
        cv::Point3f pts_3d_temp;
        pts_3d_temp.x = X;
        pts_3d_temp.y = Y;
        pts_3d_temp.z = 0;
        boards_3d_[i]->pts_3d_.push_back(pts_3d_temp);
      }
    }
    LOG_DEBUG << "Here2";
    boards_3d_[i]->nb_pts_ =
        (boards_3d_[i]->nb_x_square_ - 1) * (boards_3d_[i]->nb_y_square_ - 1);
    boards_3d_[i]->charuco_board_ = charuco_boards[boards_index[i]];
  }
}

/**
 * @brief Extract necessary boards info from initialized paths
 *
 */
void Calibration::boardExtraction() {
  std::unordered_set<cv::String> allowed_exts = {"jpg",  "png", "bmp",
                                                 "jpeg", "jp2", "tiff"};

  // iterate through the cameras
  for (int cam = 0; cam < nb_camera_; cam++) {
    // prepare the folder's name
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << cam + 1;
    std::string cam_nb = ss.str();
    std::string cam_path = root_dir_ + cam_prefix_ + cam_nb;
    LOG_INFO << "Extraction camera " << cam_nb;

    // iterate through the images for corner extraction
    std::vector<cv::String> fn;
    cv::glob(cam_path + "/*.*", fn, true);

    // filter based on allowed extensions
    std::vector<cv::String> fn_filtered;
    for (cv::String cur_path : fn) {
      std::size_t ext_idx = cur_path.find_last_of(".");
      cv::String cur_ext = cur_path.substr(ext_idx + 1);
      if (allowed_exts.find(cur_ext) != allowed_exts.end()) {
        fn_filtered.push_back(cur_path);
      }
    }
    fn = fn_filtered;

    size_t count_frame =
        fn.size(); // number of allowed image files in images folder
    for (size_t frameind = 0; frameind < count_frame; frameind = frameind + 1) {
      // open Image
      cv::Mat currentIm = cv::imread(fn[frameind]);
      std::string frame_path = fn[frameind];
      // detect the checkerboard on this image
      LOG_DEBUG << "Frame index :: " << frameind;
      detectBoards(currentIm, cam, frameind, frame_path);
      LOG_DEBUG << frameind;
      // displayBoards(currentIm, cam, frameind); // Display frame
    }
  }
}

/**
 * @brief Detect boards on an image
 *
 * @param image Image on which we would like to detect the board
 * @param cam_idx camera index which acquire the frame
 * @param frame_idx frame index
 */
void Calibration::detectBoards(cv::Mat image, int cam_idx, int frame_idx,
                               std::string frame_path) {
  // Greyscale image for subpixel refinement
  cv::Mat graymat;
  cv::cvtColor(image, graymat, cv::COLOR_BGR2GRAY);

  // Initialize image size
  cams_[cam_idx]->im_cols_ = graymat.cols;
  cams_[cam_idx]->im_rows_ = graymat.rows;

  // Datastructure to save the checkerboard corners
  std::map<int, std::vector<int>>
      marker_idx; // key == board id, value == markersIDs on MARKERS markerIds
  std::map<int, std::vector<std::vector<cv::Point2f>>>
      marker_corners; // key == board id, value == 2d points visualized on
                      // MARKERS
  std::map<int, std::vector<cv::Point2f>>
      charuco_corners; // key == board id, value == 2d points on checkerboard
  std::map<int, std::vector<int>>
      charuco_idx; // key == board id, value == ID corners on checkerboard

  charuco_params_->adaptiveThreshConstant = 1;

  for (int i = 0; i < nb_board_; i++) {
    cv::aruco::detectMarkers(image, boards_3d_[i]->charuco_board_->dictionary,
                             marker_corners[i], marker_idx[i],
                             charuco_params_); // detect markers

    if (marker_corners[i].size() > 0) {
      cv::aruco::interpolateCornersCharuco(marker_corners[i], marker_idx[i],
                                           image, boards_3d_[i]->charuco_board_,
                                           charuco_corners[i], charuco_idx[i]);
    }

    if (charuco_corners[i].size() >
        (int)round(min_perc_pts_ * boards_3d_[i]->nb_pts_)) {
      LOG_INFO << "Number of detected corners :: " << charuco_corners[i].size();
      // Refine the detected corners
      if (refine_corner_ == true) {
        std::vector<SaddlePoint> refined;
        saddleSubpixelRefinement(graymat, charuco_corners[i], refined,
                                 corner_ref_window_, corner_ref_max_iter_);
        for (int j = 0; j < charuco_corners[i].size(); j++) {
          if (isinf(refined[j].x) || isinf(refined[j].y)) {
            break;
          }
          charuco_corners[i][j].x = refined[j].x;
          charuco_corners[i][j].y = refined[j].y;
        }
      }
      // Add the board to the datastructures
      int board_idx = i;
      insertNewBoard(cam_idx, frame_idx, board_idx, charuco_corners[board_idx],
                     charuco_idx[board_idx], frame_path);
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

  std::string save_path_camera_params =
      (!camera_params_file_name_.empty())
          ? save_path_ + camera_params_file_name_
          : save_path_ + "calibrated_cameras_data.yml";
  cv::FileStorage fs(save_path_camera_params, cv::FileStorage::WRITE);
  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it_cam_group =
           cam_group_.begin();
       it_cam_group != cam_group_.end(); ++it_cam_group) {

    std::shared_ptr<CameraGroup> cur_cam_group = it_cam_group->second;
    fs << "nb_camera" << nb_camera_;
    for (std::map<int, std::weak_ptr<Camera>>::iterator it_cam =
             cur_cam_group->cameras_.begin();
         it_cam != cur_cam_group->cameras_.end(); ++it_cam) {
      std::shared_ptr<Camera> cur_cam = it_cam->second.lock();
      fs << "camera_" + std::to_string(cur_cam->cam_idx_);
      cv::Mat cam_matrix;
      cv::Mat distortion_vector;
      cur_cam->getIntrinsics(cam_matrix, distortion_vector);
      fs << "{"
         << "camera_matrix" << cam_matrix;
      fs << "distortion_vector" << distortion_vector;
      fs << "distortion_type" << cur_cam->distortion_model_;
      fs << "camera_group" << it_cam_group->first;
      fs << "img_width" << cur_cam->im_cols_;
      fs << "img_height" << cur_cam->im_rows_;
      fs << "camera_pose_matrix"
         << cur_cam_group->getCameraPoseMat(cur_cam->cam_idx_).inv() << "}";
    }

    fs.release();
  }
}

/**
 * @brief Save all the 3D object
 *
 * Export 3D points constituting the objects.
 *
 */
void Calibration::save3DObj() {
  std::string save_path_object = save_path_ + "calibrated_objects_data.yml";
  cv::FileStorage fs(save_path_object, cv::FileStorage::WRITE);

  for (std::map<int, std::shared_ptr<Object3D>>::iterator it_obj =
           object_3d_.begin();
       it_obj != object_3d_.end(); ++it_obj) {
    std::shared_ptr<Object3D> cur_object = it_obj->second;
    fs << "object_" + std::to_string(cur_object->obj_id_);
    int obj_nb_pts = cur_object->nb_pts_;
    cv::Mat pts_mat(3, obj_nb_pts, CV_32FC1);
    for (int i = 0; i < obj_nb_pts; i++) {
      cv::Point3f curr_pts = cur_object->pts_3d_[i];
      pts_mat.at<float>(0, i) = curr_pts.x;
      pts_mat.at<float>(1, i) = curr_pts.y;
      pts_mat.at<float>(2, i) = curr_pts.z;
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
  std::string save_path_object_pose =
      save_path_ + "calibrated_objects_pose_data.yml";
  cv::FileStorage fs(save_path_object_pose, cv::FileStorage::WRITE);
  for (std::map<int, std::shared_ptr<Object3D>>::iterator it_obj =
           object_3d_.begin();
       it_obj != object_3d_.end(); ++it_obj) {
    std::shared_ptr<Object3D> cur_object = it_obj->second;
    fs << "object_" + std::to_string(cur_object->obj_id_);
    fs << "{";
    cv::Mat pose_mat(6, cur_object->object_observations_.size(), CV_64FC1);
    int a = 0;
    for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_obj_obs =
             cur_object->object_observations_.begin();
         it_obj_obs != cur_object->object_observations_.end(); ++it_obj_obs) {
      std::shared_ptr<Object3DObs> cur_object_obs = it_obj_obs->second.lock();
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
    fs << "poses" << pose_mat;
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
void Calibration::displayBoards(cv::Mat image, int cam_idx, int frame_idx) {
  std::pair<int, int> cam_frame = std::make_pair(cam_idx, frame_idx);
  std::map<std::pair<int, int>, std::shared_ptr<CameraObs>>::iterator it =
      cams_obs_.find(cam_frame); // Check if a frame exist
  if (it != cams_obs_.end()) {
    for (std::map<int, std::weak_ptr<BoardObs>>::iterator it =
             cams_obs_[cam_frame]->board_observations_.begin();
         it != cams_obs_[cam_frame]->board_observations_.end(); ++it) {
      std::vector<cv::Point2f> current_pts = it->second.lock()->pts_2d_;
      std::shared_ptr<Board> board_3d_ptr = it->second.lock()->board_3d_.lock();
      std::vector<double> color_temp = board_3d_ptr->color_;
      for (int j = 0; j < current_pts.size(); j++) {
        LOG_DEBUG << "Pts x :: " << current_pts[j].x << "   y :: ";
        // Current_pts[j].y  ;
        circle(image, cv::Point(current_pts[j].x, current_pts[j].y), 4,
               cv::Scalar(color_temp[0], color_temp[1], color_temp[2]),
               cv::FILLED, 8, 0);
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
void Calibration::insertNewBoard(int cam_idx, int frame_idx, int board_idx,
                                 std::vector<cv::Point2f> pts_2d,
                                 std::vector<int> charuco_idx,
                                 std::string frame_path) {
  std::shared_ptr<BoardObs> new_board = std::make_shared<BoardObs>();
  new_board->init(cam_idx, frame_idx, board_idx, pts_2d, charuco_idx,
                  cams_[cam_idx], boards_3d_[board_idx]);

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
        std::make_shared<Frame>(); // declare new frame in heap
    newFrame->frame_idx_ = frame_idx;
    newFrame->frame_path_[cam_idx] = frame_path;
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
    std::shared_ptr<CameraObs> new_cam_obs = std::make_shared<CameraObs>();
    ;
    cams_obs_[cam_frame_idx] = new_cam_obs;
    cams_obs_[cam_frame_idx]->insertNewBoard(new_board); // add the observation
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
  if (!cam_params_path_.empty() & cam_params_path_ != "None") {
    cv::FileStorage fs;
    fs.open(cam_params_path_, cv::FileStorage::READ);

    LOG_INFO << "Initializing camera calibration from " << cam_params_path_;

    for (std::map<int, std::shared_ptr<Camera>>::iterator it = cams_.begin();
         it != cams_.end(); ++it) {
      // extract camera matrix and distortion coefficients from the file
      cv::FileNode loaded_cam_params =
          fs["camera_" + std::to_string(it->first)];

      cv::Mat camera_matrix;
      cv::Mat distortion_coeffs;
      loaded_cam_params["camera_matrix"] >> camera_matrix;
      loaded_cam_params["distortion_vector"] >> distortion_coeffs;

      it->second->setIntrinsics(camera_matrix, distortion_coeffs);
    }

  } else {
    LOG_INFO << "Initializing camera calibration using images";

    for (std::map<int, std::shared_ptr<Camera>>::iterator it = cams_.begin();
         it != cams_.end(); ++it)
      it->second->initializeCalibration();
  }
}

/**
 * @brief Estimate the boards' pose w.r.t. cameras
 *
 * It is based on a PnP algorithm.
 *
 */
void Calibration::estimatePoseAllBoards() {
  for (std::map<int, std::shared_ptr<BoardObs>>::iterator it =
           board_observations_.begin();
       it != board_observations_.end(); ++it)
    it->second->estimatePose(ransac_thresh_);
}

/**
 * @brief Non-linear refinement of all the cameras intrinsic parameters
 * individually
 *
 */
void Calibration::refineIntrinsicAndPoseAllCam() {
  for (std::map<int, std::shared_ptr<Camera>>::iterator it = cams_.begin();
       it != cams_.end(); ++it)
    it->second->refineIntrinsicCalibration(nb_iterations_);
}

/**
 * @brief Compute the reprojection error for each boards
 *
 */
void Calibration::computeReproErrAllBoard() {
  std::vector<float> err_vec;
  float sum_err = 0;
  for (std::map<int, std::shared_ptr<BoardObs>>::iterator it =
           board_observations_.begin();
       it != board_observations_.end(); ++it) {
    float err = it->second->computeReprojectionError();
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
  for (std::map<std::pair<int, int>, std::shared_ptr<CameraObs>>::iterator it =
           cams_obs_.begin();
       it != cams_obs_.end(); ++it) {
    std::shared_ptr<CameraObs> current_board = it->second;
    std::vector<int> BoardIdx = current_board->board_idx_;

    if (BoardIdx.size() > 1) // if more than one board is visible
    {
      for (std::map<int, std::weak_ptr<BoardObs>>::iterator it1 =
               current_board->board_observations_.begin();
           it1 != current_board->board_observations_.end(); ++it1) {
        int boardid1 = it1->second.lock()->board_id_;
        for (std::map<int, std::weak_ptr<BoardObs>>::iterator it2 =
                 current_board->board_observations_.begin();
             it2 != current_board->board_observations_.end(); ++it2) {
          int boardid2 = it2->second.lock()->board_id_;
          cv::Mat proj_1 = it1->second.lock()->getPoseMat();
          if (boardid1 != boardid2) // We do not care about the transformation
                                    // with itself ...
          {
            cv::Mat proj_2 = it2->second.lock()->getPoseMat();
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

/**
 * @brief Compute the mean transformation between all boards' observations
 *
 * Multiple interboard pose can be computed per frames, these measurements are
 * averaged in this function.
 *
 * @todo remove dead code
 */
void Calibration::initInterBoardsTransform() {
  inter_board_transform_.clear();
  for (std::map<std::pair<int, int>, std::vector<cv::Mat>>::iterator it =
           board_pose_pairs_.begin();
       it != board_pose_pairs_.end(); ++it) {
    std::pair<int, int> board_pair_idx = it->first;
    std::vector<cv::Mat> board_poses_temp = it->second;
    cv::Mat average_rotation = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat average_translation = cv::Mat::zeros(3, 1, CV_64F);

    // Averaging technique
    /*for (int i = 0; i < board_poses_temp.size(); i++) {
      cv::Mat R, T;
      Proj2RT(board_poses_temp[i], R, T);
      average_rotation += R;
      average_translation += T;
    }
    LOG_DEBUG << "Number images where the 2 boards appear :: "
             << board_poses_temp.size()  ;
    average_translation = average_translation / board_poses_temp.size();
    average_rotation = average_rotation / board_poses_temp.size();*/

    // Median
    std::vector<double> r1, r2, r3;
    std::vector<double> t1, t2, t3;
    for (int i = 0; i < board_poses_temp.size(); i++) {
      cv::Mat R, T;
      Proj2RT(board_poses_temp[i], R, T);
      r1.push_back(R.at<double>(0));
      r2.push_back(R.at<double>(1));
      r3.push_back(R.at<double>(2));
      t1.push_back(T.at<double>(0));
      t2.push_back(T.at<double>(1));
      t3.push_back(T.at<double>(2));
    }
    average_rotation.at<double>(0) = median(r1);
    average_rotation.at<double>(1) = median(r2);
    average_rotation.at<double>(2) = median(r3);
    average_translation.at<double>(0) = median(t1);
    average_translation.at<double>(1) = median(t2);
    average_translation.at<double>(2) = median(t3);
    // TEST
    /* cv::Mat R, T;
     Proj2RT(board_poses_temp[0], R, T);
     average_translation = T;
     average_rotation = R;*/
    //
    inter_board_transform_[board_pair_idx] =
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
  for (std::map<int, std::shared_ptr<Board>>::iterator it = boards_3d_.begin();
       it != boards_3d_.end(); ++it) {
    if (it->second->board_observations_.size() > 0) {
      covis_boards_graph_.addVertex(it->second->board_id_);
    }
  }

  for (std::map<std::pair<int, int>, std::vector<cv::Mat>>::iterator it =
           board_pose_pairs_.begin();
       it != board_pose_pairs_.end(); ++it) {
    std::pair<int, int> board_pair_idx = it->first;
    std::vector<cv::Mat> board_poses_temp = it->second;
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
  for (int i = 0; i < connect_comp.size(); i++) {
    LOG_DEBUG << "Obj Id :: " << i;
    LOG_DEBUG << "Number of boards in this object :: "
              << connect_comp[i].size();

    // Find the reference board in this object
    int ref_board_id =
        *min_element(connect_comp[i].begin(), connect_comp[i].end());

    // Declare a new 3D object
    std::shared_ptr<Object3D> newObject3D =
        std::make_shared<Object3D>(); // declare new object 3D in heap
    newObject3D->initializeObject3D(connect_comp[i].size(), ref_board_id, i,
                                    boards_3d_[ref_board_id]->color_);
    int pts_count = 0;

    // Compute the shortest path between the reference and the other board
    for (int j = 0; j < connect_comp[i].size(); j++) {
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
      for (int k = 0; k < short_path.size() - 1; k++) {
        int current_board = short_path[k];
        int next_board = short_path[k + 1];
        std::pair<int, int> board_pair_idx =
            std::make_pair(current_board, next_board);
        cv::Mat current_trans = inter_board_transform_[board_pair_idx];
        transform = transform * current_trans.inv();
      }

      // Store the relative board transformation in the object
      newObject3D->setBoardPoseMat(transform, current_board_id);

      // Transform the 3D pts to push in the object 3D
      std::vector<cv::Point3f> trans_pts =
          transform3DPts(boards_3d_[current_board_id]->pts_3d_,
                         newObject3D->getBoardRotVec(current_board_id),
                         newObject3D->getBoardTransVec(current_board_id));
      // Make a indexing between board to object
      for (int k = 0; k < trans_pts.size(); k++) {
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
void Calibration::init3DObjectObs(int object_idx) {

  // Iterate through cameraobs
  for (std::map<std::pair<int, int>, std::shared_ptr<CameraObs>>::iterator
           it_cam_obs = cams_obs_.begin();
       it_cam_obs != cams_obs_.end(); ++it_cam_obs) {
    std::pair<int, int> cam_id_frame_id =
        it_cam_obs->first; // Cam ind/Frame ind
    std::shared_ptr<CameraObs> current_camobs = it_cam_obs->second;

    // Declare the 3D object observed in this camera observation
    // Keep in mind that a single object can be observed in one image
    std::shared_ptr<Object3DObs> object_obs = std::make_shared<Object3DObs>();
    object_obs->initializeObject(object_3d_[object_idx], object_idx);

    // Check the boards observing this camera
    std::map<int, std::weak_ptr<BoardObs>> current_board_obs =
        current_camobs->board_observations_;
    for (std::map<int, std::weak_ptr<BoardObs>>::iterator it_board_obs =
             current_board_obs.begin();
         it_board_obs != current_board_obs.end(); ++it_board_obs) {
      // Check if this board correspond to the object of interest
      std::map<int, std::weak_ptr<Board>>::iterator it =
          object_3d_[object_idx]->boards_.find(
              it_board_obs->second.lock()->board_id_);
      if (it != object_3d_[object_idx]
                    ->boards_.end()) // if the board belong to the object
      {
        object_obs->insertNewBoardObs(it_board_obs->second.lock());
      }
    }

    if (object_obs->pts_id_.size() > 0) {
      // Update the camobs//frame//camera//3DObject
      cams_obs_[cam_id_frame_id]->insertNewObject(object_obs);
      frames_[cam_id_frame_id.second]->insertNewObject(object_obs);
      cams_[it_cam_obs->first.first]->insertNewObject(object_obs);
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
  for (std::map<int, std::shared_ptr<Object3D>>::iterator it =
           object_3d_.begin();
       it != object_3d_.end(); ++it) {
    this->init3DObjectObs(it->first);
  }
}

/**
 * @brief Estimate all the pose of 3D object observation using a PnP algo
 *
 */
void Calibration::estimatePoseAllObjects() {
  for (std::map<int, std::shared_ptr<Object3DObs>>::iterator it =
           object_observations_.begin();
       it != object_observations_.end(); ++it)
    it->second->estimatePose(ransac_thresh_);
}

/**
 * @brief Compute the reprojection error for each object
 *
 */
void Calibration::computeReproErrAllObject() {
  std::vector<float> err_vec;
  for (std::map<int, std::shared_ptr<Object3DObs>>::iterator it =
           object_observations_.begin();
       it != object_observations_.end(); ++it) {
    err_vec.push_back(it->second->computeReprojectionError());
  }

  LOG_INFO << "Mean Error "
           << std::accumulate(err_vec.begin(), err_vec.end(), 0.0) /
                  err_vec.size();
}

/**
 * @brief Refine the structure of all the 3D objects
 *
 */
void Calibration::refineAllObject3D() {
  for (std::map<int, std::shared_ptr<Object3D>>::iterator it =
           object_3d_.begin();
       it != object_3d_.end(); ++it)
    it->second->refineObject(nb_iterations_);
}

/**
 * @brief Find all view where multiple camera share visible objects and store
 * their relative transformation
 *
 */
void Calibration::computeCamerasPairPose() {
  camera_pose_pairs_.clear();
  // Iterate through frames
  for (std::map<int, std::shared_ptr<Frame>>::iterator it_frame =
           frames_.begin();
       it_frame != frames_.end(); ++it_frame) {
    if (it_frame->second->board_observations_.size() >
        1) // if more than one observation is available
    {
      // Iterate through the object observation
      std::map<int, std::weak_ptr<Object3DObs>> frame_obj_obs =
          it_frame->second->object_observations_;
      for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_objectobs1 =
               frame_obj_obs.begin();
           it_objectobs1 != frame_obj_obs.end(); ++it_objectobs1) {
        int cam_id_1 = it_objectobs1->second.lock()->camera_id_;
        int obj_id_1 = it_objectobs1->second.lock()->object_3d_id_;
        cv::Mat pose_cam_1 = it_objectobs1->second.lock()->getPoseMat();
        for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_objectobs2 =
                 frame_obj_obs.begin();
             it_objectobs2 != frame_obj_obs.end(); ++it_objectobs2) {
          int cam_id_2 = it_objectobs2->second.lock()->camera_id_;
          int obj_id_2 = it_objectobs2->second.lock()->object_3d_id_;
          cv::Mat pose_cam_2 = it_objectobs2->second.lock()->getPoseMat();
          if (cam_id_1 != cam_id_2) // if the camera is not the same
          {
            // if the same object is visible from the two cameras
            if (obj_id_1 == obj_id_2) {
              // Compute the relative pose between the cameras
              cv::Mat inter_cam_pose =
                  pose_cam_2 * pose_cam_1.inv(); // not sure here ...

              // Store in a database
              camera_pose_pairs_[std::make_pair(cam_id_1, cam_id_2)].push_back(
                  inter_cam_pose);
            }
          }
        }
      }
    }
  }
}

/**
 * @brief Find average transformation between pairs of cameras to form groups
 *
 * @todo remove dead code
 */
void Calibration::initInterCamerasTransform() {
  inter_camera_transform_.clear();
  for (std::map<std::pair<int, int>, std::vector<cv::Mat>>::iterator it =
           camera_pose_pairs_.begin();
       it != camera_pose_pairs_.end(); ++it) {
    std::pair<int, int> camera_pair_idx = it->first;
    std::vector<cv::Mat> camera_poses_temp = it->second;
    cv::Mat average_rotation = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat average_translation = cv::Mat::zeros(3, 1, CV_64F);

    // Averaging technique
    /*for (int i = 0; i < camera_poses_temp.size(); i++) {
      cv::Mat R, T;
      Proj2RT(camera_poses_temp[i], R, T);
      average_rotation += R;
      average_translation += T;
    }
    LOG_DEBUG << "Number images where the 2 camera see the same object :: "
             << camera_poses_temp.size()  ;
    average_translation = average_translation / camera_poses_temp.size();
    average_rotation = average_rotation / camera_poses_temp.size();*/

    // Median technique
    // Median
    std::vector<double> r1, r2, r3;
    std::vector<double> t1, t2, t3;
    for (int i = 0; i < camera_poses_temp.size(); i++) {
      cv::Mat R, T;
      Proj2RT(camera_poses_temp[i], R, T);
      r1.push_back(R.at<double>(0));
      r2.push_back(R.at<double>(1));
      r3.push_back(R.at<double>(2));
      t1.push_back(T.at<double>(0));
      t2.push_back(T.at<double>(1));
      t3.push_back(T.at<double>(2));
    }
    average_rotation.at<double>(0) = median(r1);
    average_rotation.at<double>(1) = median(r2);
    average_rotation.at<double>(2) = median(r3);
    average_translation.at<double>(0) = median(t1);
    average_translation.at<double>(1) = median(t2);
    average_translation.at<double>(2) = median(t3);

    inter_camera_transform_[camera_pair_idx] =
        RVecT2Proj(average_rotation, average_translation);
    LOG_DEBUG << "Average Rot :: " << average_rotation
              << "    Average Trans :: " << average_translation;
  }
}

/**
 * @brief Initialize the relationship graph between cameras to form groups
 *
 */
void Calibration::initInterCamerasGraph() {
  covis_camera_graph_.clearGraph();
  // Each camera is a vertex if it has observed at least one object
  for (std::map<int, std::shared_ptr<Camera>>::iterator it =
           this->cams_.begin();
       it != cams_.end(); ++it) {
    if (it->second->board_observations_.size() > 0) {
      covis_camera_graph_.addVertex(it->second->cam_idx_);
    }
  }
  // Build the graph with cameras' pairs
  for (std::map<std::pair<int, int>, std::vector<cv::Mat>>::iterator it =
           camera_pose_pairs_.begin();
       it != camera_pose_pairs_.end(); ++it) {
    std::pair<int, int> camera_pair_idx = it->first;
    std::vector<cv::Mat> camera_poses_temp = it->second;
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
  for (int i = 0; i < connect_comp.size(); i++) {
    LOG_DEBUG << "camera group id :: " << i;
    LOG_DEBUG << "Number of cameras in the group :: " << connect_comp.size();

    // Find the reference camera in this group
    int id_ref_cam =
        *min_element(connect_comp[i].begin(), connect_comp[i].end());

    // Declare a new camera group
    std::shared_ptr<CameraGroup> new_camera_group =
        std::make_shared<CameraGroup>();
    new_camera_group->initializeCameraGroup(id_ref_cam, i);

    // Compute the shortest path between the reference and the other cams
    for (int j = 0; j < connect_comp[i].size(); j++) {
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
      for (int k = 0; k < short_path.size() - 1; k++) {
        int current_cam = short_path[k];
        int next_cam = short_path[k + 1];
        std::pair<int, int> cam_pair_idx =
            std::make_pair(current_cam, next_cam);
        cv::Mat current_trans = inter_camera_transform_[cam_pair_idx];
        // transform = transform * current_trans.inv();
        transform = transform * current_trans;
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
void Calibration::initCameraGroupObs(int camera_group_idx) {
  // List of camera idx in the group
  std::vector<int> cam_in_group = cam_group_[camera_group_idx]->cam_idx;

  // Iterate through frame
  for (std::map<int, std::shared_ptr<Frame>>::iterator it_frame =
           frames_.begin();
       it_frame != frames_.end(); ++it_frame) {
    int current_frame_id = it_frame->second->frame_idx_;
    std::shared_ptr<CameraGroupObs> new_cam_group_obs =
        std::make_shared<CameraGroupObs>(); // declare a new observation
    new_cam_group_obs->insertCameraGroup(cam_group_[camera_group_idx]);

    std::map<int, std::weak_ptr<Object3DObs>> current_object_obs =
        it_frame->second->object_observations_;
    for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_obj_obs =
             current_object_obs.begin();
         it_obj_obs != current_object_obs.end(); ++it_obj_obs) {
      int current_cam_id = it_obj_obs->second.lock()->camera_id_;
      int current_obj_id = it_obj_obs->second.lock()->object_3d_id_;

      // Check if this camera id belongs to the group
      if (std::find(cam_in_group.begin(), cam_in_group.end(), current_cam_id) !=
          cam_in_group.end()) {
        // if (count(cam_in_group.begin(), cam_in_group.end(), current_cam_id))
        // {
        // the camera is in the group so this object is visible in the cam group
        // udpate the observation
        new_cam_group_obs->insertObjectObservation(it_obj_obs->second.lock());

        // push the object observation in the camera group
        cam_group_[camera_group_idx]->insertNewObjectObservation(
            it_obj_obs->second.lock());
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
  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it =
           cam_group_.begin();
       it != cam_group_.end(); ++it) {
    int camera_group_idx = it->second->cam_group_idx_;
    initCameraGroupObs(camera_group_idx);
  }
}

/**
 * @brief Non-linear optimization of the camera pose in the groups and the pose
 * of the observed objects
 *
 */
void Calibration::refineAllCameraGroup() {
  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it =
           cam_group_.begin();
       it != cam_group_.end(); ++it) {
    // it->second->computeObjPoseInCameraGroup();
    it->second->refineCameraGroup(nb_iterations_);
  }

  // Update the object3D observation
  for (std::map<std::pair<int, int>, std::shared_ptr<CameraGroupObs>>::iterator
           it = cams_group_obs_.begin();
       it != cams_group_obs_.end(); it++) {
    it->second->updateObjObsPose();
  }
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
  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it_groups_1 =
           cam_group_.begin();
       it_groups_1 != cam_group_.end(); ++it_groups_1) {
    int group_idx1 = it_groups_1->first;
    for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it_groups_2 =
             cam_group_.begin();
         it_groups_2 != cam_group_.end(); ++it_groups_2) {
      int group_idx2 = it_groups_2->first;
      if (group_idx1 != group_idx2) // if the two groups are different
      {
        // Prepare the list of possible objects pairs
        std::map<std::pair<int, int>, int> count_pair_obs;
        for (std::map<int, std::shared_ptr<Object3D>>::iterator it_obj1 =
                 object_3d_.begin();
             it_obj1 != object_3d_.end(); ++it_obj1) {
          for (std::map<int, std::shared_ptr<Object3D>>::iterator it_obj2 =
                   object_3d_.begin();
               it_obj2 != object_3d_.end(); ++it_obj2) {
            int obj_id1 = it_obj1->second->obj_id_;
            int obj_id2 = it_obj2->second->obj_id_;
            if (obj_id1 != obj_id2) {
              count_pair_obs[std::make_pair(obj_id1, obj_id2)] =
                  0; // initialize the count to zero
            }
          }
        }

        // move to shared_ptr cause there is no = for weak_ptr
        std::map<int, std::shared_ptr<Frame>> it_groups_1_frames;
        std::map<int, std::shared_ptr<Frame>> it_groups_2_frames;
        for (auto item : it_groups_1->second->frames_)
          it_groups_1_frames[item.first] = item.second.lock();
        for (auto item : it_groups_2->second->frames_)
          it_groups_2_frames[item.first] = item.second.lock();

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
        for (std::map<int, std::shared_ptr<Frame>>::iterator it_common_frames =
                 common_frames.begin();
             it_common_frames != common_frames.end(); ++it_common_frames) {
          // Find the index of the observation corresponding to the groups in
          // cam group obs
          auto it_camgroupid_1 =
              find(it_common_frames->second->cam_group_idx_.begin(),
                   it_common_frames->second->cam_group_idx_.end(), group_idx1);
          auto it_camgroupid_2 =
              find(it_common_frames->second->cam_group_idx_.begin(),
                   it_common_frames->second->cam_group_idx_.end(), group_idx2);
          int index_camgroup_1 =
              it_camgroupid_1 -
              it_common_frames->second->cam_group_idx_.begin();
          int index_camgroup_2 =
              it_camgroupid_2 -
              it_common_frames->second->cam_group_idx_.begin();

          // Access the objects 3D index for both groups
          std::map<int, std::weak_ptr<Object3DObs>> object_obs_1 =
              it_common_frames->second
                  ->cam_group_observations_[index_camgroup_1]
                  .lock()
                  ->object_observations_;
          std::map<int, std::weak_ptr<Object3DObs>> object_obs_2 =
              it_common_frames->second
                  ->cam_group_observations_[index_camgroup_2]
                  .lock()
                  ->object_observations_;
          for (std::map<int, std::weak_ptr<Object3DObs>>::iterator
                   it_object_obs_1 = object_obs_1.begin();
               it_object_obs_1 != object_obs_1.end(); ++it_object_obs_1) {
            int obj_ind_1 = it_object_obs_1->second.lock()->object_3d_id_;
            for (std::map<int, std::weak_ptr<Object3DObs>>::iterator
                     it_object_obs_2 = object_obs_2.begin();
                 it_object_obs_2 != object_obs_2.end(); ++it_object_obs_2) {
              int obj_ind_2 = it_object_obs_2->second.lock()->object_3d_id_;
              count_pair_obs[std::make_pair(obj_ind_1, obj_ind_2)]++;
            }
          }
        }

        // find the pair of object with the maximum shared frames
        unsigned currentMax = 0;
        std::pair<int, int> arg_max = std::make_pair(0, 0);
        for (auto it = count_pair_obs.cbegin(); it != count_pair_obs.cend();
             ++it) {
          if (it->second > currentMax) {
            arg_max = it->first;
            currentMax = it->second;
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
 * @todo remove dead code
 */
void Calibration::initNonOverlapPair(int cam_group_id1, int cam_group_id2) {
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
  std::vector<cv::Point3f> pts_3d_obj_1 = object_3D_1->pts_3d_;
  std::vector<cv::Point3f> pts_3d_obj_2 = object_3D_2->pts_3d_;

  // std::vector to store data for non-overlapping calibration
  std::vector<cv::Mat> pose_abs_1,
      pose_abs_2; // absolute pose stored to compute relative displacements
  cv::Mat repo_obj_1_2; // reprojected pts for clustering

  // move to shared_ptr cause there is no = for weak_ptr
  std::map<int, std::shared_ptr<Frame>> cam_group1_frames;
  std::map<int, std::shared_ptr<Frame>> cam_group2_frames;
  for (auto item : cam_group1->frames_)
    cam_group1_frames[item.first] = item.second.lock();
  for (auto item : cam_group2->frames_)
    cam_group2_frames[item.first] = item.second.lock();

  // Find frames in common
  std::map<int, std::shared_ptr<Frame>> common_frames;
  std::map<int, std::shared_ptr<Frame>>::iterator it_frames(
      common_frames.begin());
  std::set_intersection(cam_group1_frames.begin(), cam_group1_frames.end(),
                        cam_group2_frames.begin(), cam_group2_frames.end(),
                        std::inserter(common_frames, it_frames));

  // Iterate through common frames and reproject the objects in the images to
  // cluster
  for (std::map<int, std::shared_ptr<Frame>>::iterator it_common_frames =
           common_frames.begin();
       it_common_frames != common_frames.end(); ++it_common_frames) {
    // Find the index of the observation corresponding to the groups in cam
    // group obs
    auto it_camgroupid_1 =
        find(it_common_frames->second->cam_group_idx_.begin(),
             it_common_frames->second->cam_group_idx_.end(), cam_group_id1);
    auto it_camgroupid_2 =
        find(it_common_frames->second->cam_group_idx_.begin(),
             it_common_frames->second->cam_group_idx_.end(), cam_group_id2);
    int index_camgroup_1 =
        it_camgroupid_1 - it_common_frames->second->cam_group_idx_.begin();
    int index_camgroup_2 =
        it_camgroupid_2 - it_common_frames->second->cam_group_idx_.begin();

    // check if both objects of interest are in the frame
    std::weak_ptr<CameraGroupObs> cam_group_obs1 =
        it_common_frames->second->cam_group_observations_[index_camgroup_1];
    std::weak_ptr<CameraGroupObs> cam_group_obs2 =
        it_common_frames->second->cam_group_observations_[index_camgroup_2];
    std::vector<int> cam_group_obs_obj1 = cam_group_obs1.lock()->object_idx_;
    std::vector<int> cam_group_obs_obj2 = cam_group_obs2.lock()->object_idx_;
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
      std::weak_ptr<Camera> ref_cam_1 =
          cam_group_obs1.lock()
              ->cam_group_.lock()
              ->cameras_[cam_group_obs1.lock()->cam_group_.lock()->id_ref_cam_];
      std::weak_ptr<Camera> ref_cam_2 =
          cam_group_obs2.lock()
              ->cam_group_.lock()
              ->cameras_[cam_group_obs2.lock()->cam_group_.lock()->id_ref_cam_];
      cv::Mat cam_mat_1, dist_1;
      cv::Mat cam_mat_2, dist_2;
      ref_cam_1.lock()->getIntrinsics(cam_mat_1, dist_1);
      ref_cam_2.lock()->getIntrinsics(cam_mat_2, dist_2);

      int object_id1 = cam_group_obs1.lock()
                           ->object_observations_[index_objobs_1]
                           .lock()
                           ->object_3d_id_;
      int object_id2 = cam_group_obs2.lock()
                           ->object_observations_[index_objobs_2]
                           .lock()
                           ->object_3d_id_;
      cv::Mat pose_obj_1 = cam_group_obs1.lock()->getObjectPoseMat(object_id1);
      cv::Mat pose_obj_2 = cam_group_obs2.lock()->getObjectPoseMat(object_id2);

      pose_abs_1.push_back(pose_obj_1);
      pose_abs_2.push_back(pose_obj_2);
    }
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
  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it_groups_1 =
           cam_group_.begin();
       it_groups_1 != cam_group_.end(); ++it_groups_1) {
    int group_idx1 = it_groups_1->first;
    for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it_groups_2 =
             cam_group_.begin();
         it_groups_2 != cam_group_.end(); ++it_groups_2) {
      int group_idx2 = it_groups_2->first;
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
  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it =
           cam_group_.begin();
       it != cam_group_.end(); ++it) {
    if (it->second->object_observations_.size() > 0) {
      no_overlap_camgroup_graph_.addVertex(it->second->cam_group_idx_);
    }
  }

  // Create the graph
  for (std::map<std::pair<int, int>, cv::Mat>::iterator it =
           no_overlap_camgroup_pair_pose_.begin();
       it != no_overlap_camgroup_pair_pose_.end(); ++it) {
    std::pair<int, int> camgroup_pair_idx = it->first;
    cv::Mat camgroup_poses_temp = it->second;
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

  for (int i = 0; i < connect_comp.size(); i++) {
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
    for (int j = 0; j < connect_comp[i].size(); j++) {
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
      for (int k = 0; k < short_path.size() - 1; k++) {
        int current_group = short_path[k];
        int next_group = short_path[k + 1];
        std::pair<int, int> group_pair_idx =
            std::make_pair(current_group, next_group);
        cv::Mat current_trans = no_overlap_camgroup_pair_pose_[group_pair_idx];
        // transform = transform * current_trans.inv();
        transform = transform * current_trans;
      }
      // Store the poses
      cam_group_pose_to_ref[current_cam_group_id] = transform;
    }

    // initialize the camera group
    std::shared_ptr<CameraGroup> new_camera_group =
        std::make_shared<CameraGroup>();
    new_camera_group->initializeCameraGroup(id_ref_cam, i);
    // Iterate through the camera groups and add all the cameras individually in
    // the new group
    for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it_group =
             cam_group_.begin();
         it_group != cam_group_.end(); ++it_group) {
      // Check if the current camera group belong to the final group
      int current_cam_group_idx = it_group->second->cam_group_idx_;
      if (std::find(connect_comp[i].begin(), connect_comp[i].end(),
                    current_cam_group_idx) != connect_comp[i].end()) {
        // Prepare the current group pose in the referential of the final group
        std::weak_ptr<CameraGroup> current_group = it_group->second;
        cv::Mat pose_in_final =
            cam_group_pose_to_ref[current_group.lock()->cam_group_idx_];
        // the camera group is in the final group so we include its cameras
        for (std::map<int, std::weak_ptr<Camera>>::iterator it_cam =
                 current_group.lock()->cameras_.begin();
             it_cam != current_group.lock()->cameras_.end(); ++it_cam) {
          std::weak_ptr<Camera> current_camera = it_cam->second;
          // Update the pose in the referential of the final group
          cv::Mat pose_cam_in_current_group =
              current_group.lock()->getCameraPoseMat(
                  current_camera.lock()->cam_idx_);
          cv::Mat transform = pose_cam_in_current_group * pose_in_final;
          new_camera_group->insertCamera(it_cam->second.lock());
          new_camera_group->setCameraPoseMat(transform,
                                             current_camera.lock()->cam_idx_);
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
  for (std::map<int, std::shared_ptr<Frame>>::iterator it_frame =
           frames_.begin();
       it_frame != frames_.end(); ++it_frame) {
    it_frame->second->cam_group_idx_.clear();
    it_frame->second->cam_group_observations_.clear();
  }
  cams_group_obs_.clear();

  // Reinitialize all camera obserations
  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it =
           cam_group_.begin();
       it != cam_group_.end(); ++it) {
    int camera_group_idx = it->second->cam_group_idx_;
    initCameraGroupObs(camera_group_idx);
  }
}

/**
 * @brief Compute the 3D object position in the camera group
 *
 */
void Calibration::computeAllObjPoseInCameraGroup() {

  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it =
           cam_group_.begin();
       it != cam_group_.end(); ++it) {
    it->second->computeObjPoseInCameraGroup();
  }
  // Compute the pose of each object in the camera groups obs
  for (std::map<std::pair<int, int>, std::shared_ptr<CameraGroupObs>>::iterator
           it_cam_group_obs = cams_group_obs_.begin();
       it_cam_group_obs != cams_group_obs_.end(); it_cam_group_obs++) {
    it_cam_group_obs->second->computeObjectsPose();
  }
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
  for (std::map<std::pair<int, int>, std::shared_ptr<CameraGroupObs>>::iterator
           it_cam_group_obs = cams_group_obs_.begin();
       it_cam_group_obs != cams_group_obs_.end(); it_cam_group_obs++) {
    if (it_cam_group_obs->second->object_idx_.size() > 1) {
      std::map<int, std::weak_ptr<Object3DObs>> obj_obs =
          it_cam_group_obs->second->object_observations_;
      for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_object1 =
               obj_obs.begin();
           it_object1 != obj_obs.end(); it_object1++) {
        int object_3d_id_1 = it_object1->second.lock()->object_3d_id_;
        cv::Mat obj_pose_1 =
            it_cam_group_obs->second->getObjectPoseMat(object_3d_id_1);
        // cv::Mat obj_pose_1 = it_object1->second->getPoseInGroupMat();
        for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_object2 =
                 obj_obs.begin();
             it_object2 != obj_obs.end(); it_object2++) {
          int object_3d_id_2 = it_object2->second.lock()->object_3d_id_;
          cv::Mat obj_pose_2 =
              it_cam_group_obs->second->getObjectPoseMat(object_3d_id_2);
          // cv::Mat obj_pose_2 = it_object2->second->getPoseInGroupMat();
          if (object_3d_id_1 != object_3d_id_2) {
            cv::Mat inter_object_pose = obj_pose_2.inv() * obj_pose_1;
            std::pair<int, int> object_idx_pair =
                std::make_pair(object_3d_id_1, object_3d_id_2);
            object_pose_pairs_[object_idx_pair].push_back(inter_object_pose);
          }
        }
      }
    }
  }
}

/**
 * @brief Compute the mean transformation between all objects' observations
 *
 * Multiple interobject pose can be computed per frames, these measurements are
 * averaged in this function.
 *
 * @todo remove dead code
 */
void Calibration::initInterObjectsTransform() {
  inter_object_transform_.clear();
  for (std::map<std::pair<int, int>, std::vector<cv::Mat>>::iterator it =
           object_pose_pairs_.begin();
       it != object_pose_pairs_.end(); ++it) {
    std::pair<int, int> object_pair_idx = it->first;
    std::vector<cv::Mat> object_poses_temp = it->second;
    cv::Mat average_rotation = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat average_translation = cv::Mat::zeros(3, 1, CV_64F);

    // Average technique
    /*for (int i = 0; i < object_poses_temp.size(); i++) {
      cv::Mat R, T;
      Proj2RT(object_poses_temp[i], R, T);
      average_rotation += R;
      average_translation += T;
    }
    LOG_DEBUG << "Number images where the 2 object appear :: "
             << object_poses_temp.size()  ;
    average_translation = average_translation / object_poses_temp.size();
    average_rotation = average_rotation / object_poses_temp.size();*/

    // Median
    std::vector<double> r1, r2, r3;
    std::vector<double> t1, t2, t3;
    for (int i = 0; i < object_poses_temp.size(); i++) {
      cv::Mat R, T;
      Proj2RT(object_poses_temp[i], R, T);
      r1.push_back(R.at<double>(0));
      r2.push_back(R.at<double>(1));
      r3.push_back(R.at<double>(2));
      t1.push_back(T.at<double>(0));
      t2.push_back(T.at<double>(1));
      t3.push_back(T.at<double>(2));
    }
    average_rotation.at<double>(0) = median(r1);
    average_rotation.at<double>(1) = median(r2);
    average_rotation.at<double>(2) = median(r3);
    average_translation.at<double>(0) = median(t1);
    average_translation.at<double>(1) = median(t2);
    average_translation.at<double>(2) = median(t3);

    inter_object_transform_[object_pair_idx] =
        RVecT2Proj(average_rotation, average_translation);
    LOG_DEBUG << "Average Rot :: " << average_rotation
              << "    Average Trans :: " << average_translation;
  }
}

/**
 * @brief Initialize the graph with the poses between objects
 *
 */
void Calibration::initInterObjectsGraph() {

  covis_objects_graph_.clearGraph();
  // Each object is a vertex if it has been observed at least once
  for (std::map<int, std::shared_ptr<Object3D>>::iterator it =
           object_3d_.begin();
       it != object_3d_.end(); ++it) {
    if (it->second->object_observations_.size() > 0) {
      covis_objects_graph_.addVertex(it->second->obj_id_);
    }
  }

  for (std::map<std::pair<int, int>, std::vector<cv::Mat>>::iterator it =
           object_pose_pairs_.begin();
       it != object_pose_pairs_.end(); ++it) {
    std::pair<int, int> object_pair_idx = it->first;
    std::vector<cv::Mat> object_poses_temp = it->second;
    covis_objects_graph_.addEdge(object_pair_idx.first, object_pair_idx.second,
                                 ((double)1 / (object_poses_temp.size())));
  }
  LOG_DEBUG << "GRAPH INTER OBJECT DONE";
}

/**
 * @brief Merge all objects groups which have been visible in same camera groups
 *
 * @todo remove dead code
 */
void Calibration::mergeObjects() {
  // find the connected objects in the graph
  std::vector<std::vector<int>> connect_comp =
      covis_objects_graph_.connectedComponents();
  std::map<int, std::shared_ptr<Object3D>> object_3d; // list of object 3D

  for (int i = 0; i < connect_comp.size(); i++) {
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
    for (int j = 0; j < connect_comp[i].size(); j++) {
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
      for (int k = 0; k < short_path.size() - 1; k++) {
        int current_object = short_path[k];
        int next_object = short_path[k + 1];
        std::pair<int, int> object_pair_idx =
            std::make_pair(current_object, next_object);
        cv::Mat current_trans = inter_object_transform_[object_pair_idx];
        transform = transform * current_trans.inv(); // original
        // transform = transform * current_trans;
      }
      // Store the poses
      object_pose_to_ref[current_object_id] = transform;
    }
    // initialize the object
    std::shared_ptr<Object3D> newObject3D =
        std::make_shared<Object3D>(); // declare new object 3D in heap
    newObject3D->initializeObject3D(nb_board_in_obj, ref_board_id, i,
                                    boards_3d_[ref_board_id]->color_);
    int pts_count = 0;
    // Iterate through the objects and add all of them individually in the new
    // object
    for (std::map<int, std::shared_ptr<Object3D>>::iterator it_object =
             object_3d_.begin();
         it_object != object_3d_.end(); ++it_object) {
      // Check if the current object belong to the new object
      int current_object_idx = it_object->second->obj_id_;
      if (std::find(connect_comp[i].begin(), connect_comp[i].end(),
                    current_object_idx) != connect_comp[i].end()) {
        // Prepare the current object pose in the referential of the merged
        // object
        std::shared_ptr<Object3D> current_object = it_object->second;
        cv::Mat pose_in_merged = object_pose_to_ref[current_object->obj_id_];
        // the object is in the merged group so we include its boards
        for (std::map<int, std::weak_ptr<Board>>::iterator it_board =
                 current_object->boards_.begin();
             it_board != current_object->boards_.end(); ++it_board) {
          std::weak_ptr<Board> current_board = it_board->second;
          // Update the pose to be in the referential of the merged object
          cv::Mat pose_board_in_current_obj =
              current_object->getBoardPoseMat(current_board.lock()->board_id_);
          // cv::Mat transform = pose_board_in_current_obj*pose_in_merged; //
          // previous wrong version cv::Mat transform =
          // pose_in_merged*pose_board_in_current_obj.inv(); // second version
          // that failed cv::Mat transform =
          // pose_board_in_current_obj.inv()*pose_in_merged; // Does not work at
          // all
          cv::Mat transform = pose_in_merged * pose_board_in_current_obj;

          // insert new board
          newObject3D->insertBoardInObject(current_board.lock());
          // Store the relative board transformation in the object
          newObject3D->setBoardPoseMat(transform,
                                       current_board.lock()->board_id_);
          // Transform the 3D pts to push in the object 3D
          std::vector<cv::Point3f> trans_pts = transform3DPts(
              current_board.lock()->pts_3d_,
              newObject3D->getBoardRotVec(current_board.lock()->board_id_),
              newObject3D->getBoardTransVec(current_board.lock()->board_id_));
          // Make a indexing between board to object
          for (int k = 0; k < trans_pts.size(); k++) {
            int char_id = k;
            std::pair<int, int> boardid_charid =
                std::make_pair(current_board.lock()->board_id_, char_id);
            newObject3D->pts_board_2_obj_[boardid_charid] = pts_count;
            newObject3D->pts_obj_2_board_.push_back(boardid_charid);
            newObject3D->pts_3d_.push_back(trans_pts[k]);
            pts_count++;
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
  for (std::map<int, std::shared_ptr<Camera>>::iterator it = cams_.begin();
       it != cams_.end(); ++it) {
    it->second->vis_object_idx_.clear();
    it->second->object_observations_.clear();
  }

  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it =
           cam_group_.begin();
       it != cam_group_.end(); ++it) {
    it->second->vis_object_idx_.clear();
    it->second->object_observations_.clear();
  }

  for (std::map<std::pair<int, int>, std::shared_ptr<CameraGroupObs>>::iterator
           it = cams_group_obs_.begin();
       it != cams_group_obs_.end(); ++it) {
    it->second->object_idx_.clear();
    it->second->object_observations_.clear();
  }

  for (std::map<std::pair<int, int>, std::shared_ptr<CameraObs>>::iterator it =
           cams_obs_.begin();
       it != cams_obs_.end(); ++it) {
    it->second->object_observations_.clear();
    it->second->object_idx_.clear();
  }

  for (std::map<int, std::shared_ptr<Frame>>::iterator it = frames_.begin();
       it != frames_.end(); ++it) {
    it->second->object_observations_.clear();
    it->second->objects_idx_.clear();
  }

  for (std::map<int, std::shared_ptr<Object3D>>::iterator it =
           object_3d_.begin();
       it != object_3d_.end(); ++it) {
    it->second->object_observations_.clear();
  }

  object_observations_.clear();

  // Reinitialize all the 3D object
  for (std::map<int, std::shared_ptr<Object3D>>::iterator it_object =
           object_3d_.begin();
       it_object != object_3d_.end(); ++it_object) {
    // Reinitialize all object obserations
    for (std::map<int, std::shared_ptr<Object3D>>::iterator it =
             object_3d_.begin();
         it != object_3d_.end(); ++it) {
      this->init3DObjectObs(it->first);
    }
  }
}

/**
 * @brief Compute the reprojection error for each camera group
 *
 */
void Calibration::reproErrorAllCamGroup() {
  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it =
           cam_group_.begin();
       it != cam_group_.end(); it++) {
    it->second->reproErrorCameraGroup();
  }
}

/**
 * @brief Non-linear optimization of the camera pose in the groups, the pose
 * of the observed objects, and the pose of the boards in the 3D objects
 *
 */
void Calibration::refineAllCameraGroupAndObjects() {
  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it =
           cam_group_.begin();
       it != cam_group_.end(); ++it) {
    it->second->refineCameraGroupAndObjects(nb_iterations_);
  }

  // Update the 3D objects
  for (std::map<int, std::shared_ptr<Object3D>>::iterator it =
           object_3d_.begin();
       it != object_3d_.end(); it++) {
    it->second->updateObjectPts();
  }

  // Update the object3D observation
  for (std::map<std::pair<int, int>, std::shared_ptr<CameraGroupObs>>::iterator
           it = cams_group_obs_.begin();
       it != cams_group_obs_.end(); it++) {
    it->second->updateObjObsPose();
  }
}

/**
 * @brief Save reprojection results images for a given camera.
 *
 */
void Calibration::saveReprojection(int cam_id) {
  // Prepare the path to save the images
  std::string path_root = save_path_ + "Reprojection/";
  std::stringstream ss;
  ss << std::setw(3) << std::setfill('0') << cam_id;
  std::string cam_folder = ss.str();
  std::string path_save = path_root + cam_folder + "/";

  // check if the file exist and create it if it does not
  if (!boost::filesystem::exists(path_root)) {
    boost::filesystem::create_directories(path_root);
  }
  if (!boost::filesystem::exists(path_save)) {
    boost::filesystem::create_directory(path_save);
  }

  std::shared_ptr<Camera> cam = cams_[cam_id];

  // Iterate through the frames where this camera has visibility
  for (std::map<int, std::shared_ptr<Frame>>::iterator it_frame =
           frames_.begin();
       it_frame != frames_.end(); it_frame++) {
    // Open the image
    std::string im_path = it_frame->second->frame_path_[cam_id];
    cv::Mat image = cv::imread(im_path);

    // Iterate through the camera group observations
    std::map<int, std::weak_ptr<CameraGroupObs>> cam_group_obs =
        it_frame->second->cam_group_observations_;
    for (std::map<int, std::weak_ptr<CameraGroupObs>>::iterator
             it_cam_group_obs = cam_group_obs.begin();
         it_cam_group_obs != cam_group_obs.end(); it_cam_group_obs++) {
      // Iterate through the object observation
      std::map<int, std::weak_ptr<Object3DObs>> object_observations =
          it_cam_group_obs->second.lock()->object_observations_;
      for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_obj_obs =
               object_observations.begin();
           it_obj_obs != object_observations.end(); it_obj_obs++) {
        if (it_obj_obs->second.lock()->camera_id_ == cam_id) {
          // Prepare the transformation matrix
          cv::Mat cam_pose = it_cam_group_obs->second.lock()
                                 ->cam_group_.lock()
                                 ->getCameraPoseMat(cam_id) *
                             it_obj_obs->second.lock()->getPoseInGroupMat();
          cv::Mat rot_vec, trans_vec;
          Proj2RT(cam_pose, rot_vec, trans_vec);

          // Get the 2d and 3d pts
          std::vector<cv::Point2f> pts_2d = it_obj_obs->second.lock()->pts_2d_;
          std::vector<int> pts_ind = it_obj_obs->second.lock()->pts_id_;
          std::vector<cv::Point3f> pts_3d_obj =
              it_obj_obs->second.lock()->object_3d_.lock()->pts_3d_;
          std::vector<cv::Point3f> pts_3d;
          std::vector<cv::Point2f> pts_repro;
          for (int i = 0; i < pts_ind.size(); i++)
            pts_3d.push_back(pts_3d_obj[pts_ind[i]]);

          // Reproject the pts
          cv::Mat rr, tt;
          rot_vec.copyTo(rr);
          trans_vec.copyTo(tt);
          projectPointsWithDistortion(pts_3d, rr, tt, cam->getCameraMat(),
                                      cam->getDistortionVectorVector(),
                                      pts_repro, cam->distortion_model_);

          // plot the keypoints on the image (red project // green detected)
          std::vector<double> color_repro{0, 0, 255};
          std::vector<double> color_detect{0, 255, 0};
          for (int i = 0; i < pts_2d.size(); i++) {
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

    if (!image.empty()) {
      // display image
      // cv::imshow("reprojection_error", image);
      // cv::waitKey(1);

      // Save image
      std::stringstream ss1;
      ss1 << std::setw(6) << std::setfill('0') << it_frame->second->frame_idx_;
      std::string image_name = ss1.str() + ".jpg";
      cv::imwrite(path_save + image_name, image);
    }
  }
}

/**
 * @brief Save reprojection images for all camera
 *
 */
void Calibration::saveReprojectionAllCam() {
  for (std::map<int, std::shared_ptr<Camera>>::iterator it = cams_.begin();
       it != cams_.end(); it++) {
    int cam_id = it->second->cam_idx_;
    saveReprojection(cam_id);
  }
}

/**
 * @brief Save detection results images for a given camera
 *
 */
void Calibration::saveDetection(int cam_id) {
  // Prepare the path to save the images
  std::string path_root = save_path_ + "Detection/";
  std::stringstream ss;
  ss << std::setw(3) << std::setfill('0') << cam_id;
  std::string cam_folder = ss.str();
  std::string path_save = path_root + cam_folder + "/";

  // check if the file exist and create it if it does not
  if (!boost::filesystem::exists(path_root)) {
    boost::filesystem::create_directories(path_root);
  }
  if (!boost::filesystem::exists(path_save)) {
    boost::filesystem::create_directory(path_save);
  }

  std::shared_ptr<Camera> cam = cams_[cam_id];

  // Iterate through the frames where this camera has visibility
  for (std::map<int, std::shared_ptr<Frame>>::iterator it_frame =
           frames_.begin();
       it_frame != frames_.end(); it_frame++) {
    // Open the image
    std::string im_path = it_frame->second->frame_path_[cam_id];
    cv::Mat image = cv::imread(im_path);

    // Iterate through the camera group observations
    std::map<int, std::weak_ptr<CameraGroupObs>> cam_group_obs =
        it_frame->second->cam_group_observations_;
    for (std::map<int, std::weak_ptr<CameraGroupObs>>::iterator
             it_cam_group_obs = cam_group_obs.begin();
         it_cam_group_obs != cam_group_obs.end(); it_cam_group_obs++) {
      // Iterate through the object observation
      std::map<int, std::weak_ptr<Object3DObs>> object_observations =
          it_cam_group_obs->second.lock()->object_observations_;
      for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_obj_obs =
               object_observations.begin();
           it_obj_obs != object_observations.end(); it_obj_obs++) {
        if (it_obj_obs->second.lock()->camera_id_ == cam_id) {
          // Get the 2d and 3d pts
          std::vector<cv::Point2f> pts_2d = it_obj_obs->second.lock()->pts_2d_;
          // plot the keypoints on the image (red project // green detected)
          std::vector<double> color =
              it_obj_obs->second.lock()->object_3d_.lock()->color_;
          for (int i = 0; i < pts_2d.size(); i++) {
            circle(image, cv::Point(pts_2d[i].x, pts_2d[i].y), 4,
                   cv::Scalar(color[0], color[1], color[2]), cv::FILLED, 8, 0);
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
      ss1 << std::setw(6) << std::setfill('0') << it_frame->second->frame_idx_;
      std::string image_name = ss1.str() + ".jpg";
      imwrite(path_save + image_name, image);
    }
  }
}

/**
 * @brief Save detection images for all camera
 *
 */
void Calibration::saveDetectionAllCam() {
  for (std::map<int, std::shared_ptr<Camera>>::iterator it = cams_.begin();
       it != cams_.end(); it++) {
    int cam_id = it->second->cam_idx_;
    saveDetection(cam_id);
  }
}

/**
 * @brief Initialize the intrinsic parameters and board pose of the entire
 * system
 *
 */
void Calibration::initIntrinsic() {
  initializeCalibrationAllCam();
  estimatePoseAllBoards();
  if (fix_intrinsic_ == 0)
  {
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
  initInterBoardsTransform();
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
 * @todo remove dead code
 */
void Calibration::calibrateCameraGroup() {
  computeCamerasPairPose();
  initInterCamerasTransform();
  initInterCamerasGraph();
  initCameraGroup();
  initAllCameraGroupObs();
  computeAllObjPoseInCameraGroup();
  // this->reproErrorAllCamGroup();
  refineAllCameraGroupAndObjects();
  // this->reproErrorAllCamGroup();
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
  initInterObjectsTransform();
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
cv::Mat
Calibration::computeDistanceBetweenPoints(std::vector<cv::Point2f> obj_pts_2d,
                                          std::vector<cv::Point2f> repro_pts) {
  cv::Mat error_list;
  for (int i = 0; i < repro_pts.size(); i++) {
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

  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it =
           cam_group_.begin();
       it != cam_group_.end(); it++) {
    int cam_group_idx = it->second->cam_group_idx_;
    std::shared_ptr<CameraGroup> cur_cam_group = it->second;

    // iterate through frames
    for (std::map<int, std::weak_ptr<Frame>>::iterator it_frame =
             cur_cam_group->frames_.begin();
         it_frame != cur_cam_group->frames_.end(); ++it_frame) {
      cv::Mat camera_list;
      frame_list.push_back(it_frame->second.lock()->frame_idx_);

      // iterate through cameraGroupObs
      std::map<int, std::weak_ptr<CameraGroupObs>> current_cam_group_obs_vec =
          it_frame->second.lock()->cam_group_observations_;
      for (std::map<int, std::weak_ptr<CameraGroupObs>>::iterator
               it_cam_group_obs = current_cam_group_obs_vec.begin();
           it_cam_group_obs != current_cam_group_obs_vec.end();
           ++it_cam_group_obs) {

        // check if the current group is the camera group of interest
        if (cam_group_idx == it_cam_group_obs->second.lock()->cam_group_idx_) {
          std::map<int, std::weak_ptr<Object3DObs>> current_obj3d_obs_vec =
              it_cam_group_obs->second.lock()->object_observations_;

          // iterate through 3D object obs
          for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_obj3d =
                   current_obj3d_obs_vec.begin();
               it_obj3d != current_obj3d_obs_vec.end(); ++it_obj3d) {
            int current_cam_id = it_obj3d->second.lock()->camera_id_;
            std::vector<cv::Point3f> obj_pts_3d =
                it_obj3d->second.lock()->object_3d_.lock()->pts_3d_;
            std::vector<int> obj_pts_idx = it_obj3d->second.lock()->pts_id_;
            std::vector<cv::Point2f> obj_pts_2d =
                it_obj3d->second.lock()->pts_2d_;
            camera_list.push_back(current_cam_id);

            // compute the reprojection error
            std::vector<cv::Point3f> object_pts;
            for (int i = 0; i < obj_pts_idx.size(); i++)
              object_pts.push_back(obj_pts_3d[obj_pts_idx[i]]);

            // apply object pose transform
            std::vector<cv::Point3f> object_pts_trans1 = transform3DPts(
                object_pts,
                it_cam_group_obs->second.lock()->getObjectRotVec(
                    it_obj3d->second.lock()->object_3d_id_),
                it_cam_group_obs->second.lock()->getObjectTransVec(
                    it_obj3d->second.lock()->object_3d_id_));
            // reproject pts
            std::vector<cv::Point2f> repro_pts;
            std::shared_ptr<Camera> cam_ptr =
                it_obj3d->second.lock()->cam_.lock();
            projectPointsWithDistortion(
                object_pts_trans1, it->second->getCameraRotVec(current_cam_id),
                it->second->getCameraTransVec(current_cam_id),
                cam_ptr->getCameraMat(), cam_ptr->getDistortionVectorVector(),
                repro_pts, cam_ptr->distortion_model_);

            cv::Mat error_list =
                computeDistanceBetweenPoints(obj_pts_2d, repro_pts);
            total_avg_error_sum += cv::mean(error_list);
            number_of_adds++;
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
  std::string save_reprojection_error =
      save_path_ + "reprojection_error_data.yml";
  cv::FileStorage fs(save_reprojection_error, cv::FileStorage::WRITE);
  cv::Mat frame_list;
  int nb_cam_group = cam_group_.size();
  fs << "nb_camera_group" << nb_cam_group;

  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it =
           cam_group_.begin();
       it != cam_group_.end(); it++) {
    int cam_group_idx = it->second->cam_group_idx_;
    fs << "camera_group_" + std::to_string(cam_group_idx);
    fs << "{";
    std::shared_ptr<CameraGroup> cur_cam_group = it->second;
    // iterate through frames
    for (std::map<int, std::weak_ptr<Frame>>::iterator it_frame =
             cur_cam_group->frames_.begin();
         it_frame != cur_cam_group->frames_.end(); ++it_frame) {
      std::shared_ptr<Frame> it_frame_ptr = it_frame->second.lock();
      cv::Mat camera_list;
      fs << "frame_" + std::to_string(it_frame_ptr->frame_idx_);
      fs << "{";
      frame_list.push_back(it_frame_ptr->frame_idx_);

      // iterate through cameraGroupObs
      std::map<int, std::weak_ptr<CameraGroupObs>> current_cam_group_obs_vec =
          it_frame_ptr->cam_group_observations_;
      for (std::map<int, std::weak_ptr<CameraGroupObs>>::iterator
               it_cam_group_obs = current_cam_group_obs_vec.begin();
           it_cam_group_obs != current_cam_group_obs_vec.end();
           ++it_cam_group_obs) {

        // check if the current group is the camera group of interest
        if (cam_group_idx == it_cam_group_obs->second.lock()->cam_group_idx_) {
          std::map<int, std::weak_ptr<Object3DObs>> current_obj3d_obs_vec =
              it_cam_group_obs->second.lock()->object_observations_;

          // iterate through 3D object obs
          for (std::map<int, std::weak_ptr<Object3DObs>>::iterator it_obj3d =
                   current_obj3d_obs_vec.begin();
               it_obj3d != current_obj3d_obs_vec.end(); ++it_obj3d) {
            int current_cam_id = it_obj3d->second.lock()->camera_id_;
            std::vector<cv::Point3f> obj_pts_3d =
                it_obj3d->second.lock()->object_3d_.lock()->pts_3d_;
            std::vector<int> obj_pts_idx = it_obj3d->second.lock()->pts_id_;
            std::vector<cv::Point2f> obj_pts_2d =
                it_obj3d->second.lock()->pts_2d_;
            camera_list.push_back(current_cam_id);
            fs << "camera_" + std::to_string(current_cam_id);
            fs << "{";

            // compute the reprojection error
            std::vector<cv::Point3f> object_pts;
            for (int i = 0; i < obj_pts_idx.size(); i++)
              object_pts.push_back(obj_pts_3d[obj_pts_idx[i]]);

            int nb_pts = obj_pts_idx.size();
            fs << "nb_pts" << nb_pts;

            // apply object pose transform
            std::vector<cv::Point3f> object_pts_trans1 = transform3DPts(
                object_pts,
                it_cam_group_obs->second.lock()->getObjectRotVec(
                    it_obj3d->second.lock()->object_3d_id_),
                it_cam_group_obs->second.lock()->getObjectTransVec(
                    it_obj3d->second.lock()->object_3d_id_));
            // reproject pts
            std::vector<cv::Point2f> repro_pts;
            std::shared_ptr<Camera> cam_ptr =
                it_obj3d->second.lock()->cam_.lock();
            projectPointsWithDistortion(
                object_pts_trans1, it->second->getCameraRotVec(current_cam_id),
                it->second->getCameraTransVec(current_cam_id),
                cam_ptr->getCameraMat(), cam_ptr->getDistortionVectorVector(),
                repro_pts, cam_ptr->distortion_model_);

            cv::Mat error_list =
                computeDistanceBetweenPoints(obj_pts_2d, repro_pts);
            fs << "error_list" << error_list << "}";
          }
        }
      }
      fs << "camera_list" << camera_list << "}";
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
  for (std::map<int, std::shared_ptr<CameraGroup>>::iterator it =
           cam_group_.begin();
       it != cam_group_.end(); ++it) {

    it->second->refineCameraGroupAndObjectsAndIntrinsics(nb_iterations_);
  }

  // Update the 3D objects
  for (std::map<int, std::shared_ptr<Object3D>>::iterator it =
           object_3d_.begin();
       it != object_3d_.end(); it++) {
    it->second->updateObjectPts();
  }

  // Update the object3D observation
  for (std::map<std::pair<int, int>, std::shared_ptr<CameraGroupObs>>::iterator
           it = cams_group_obs_.begin();
       it != cams_group_obs_.end(); it++) {
    it->second->updateObjObsPose();
  }
}
