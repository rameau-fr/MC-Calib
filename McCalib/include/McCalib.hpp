#pragma once

#include <filesystem>
#include <iostream>
#include <mutex>
#include <numeric>
#include <stdio.h>

#include "opencv2/core/core.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include "Board.hpp"
#include "BoardObs.hpp"
#include "Camera.hpp"
#include "CameraGroup.hpp"
#include "CameraGroupObs.hpp"
#include "CameraObs.hpp"
#include "Frame.hpp"
#include "Graph.hpp"
#include "Object3D.hpp"
#include "Object3DObs.hpp"
#include "geometrytools.hpp"
#include "opencv_compat.hpp"

namespace McCalib {

/**
 * @class Calibration
 *
 * @brief Main orchestration class for multi-camera calibration.
 *
 * Centralizes detections, graph construction, pose initialization, and
 * non-linear refinement for cameras, boards, objects, and camera groups.
 */
class Calibration final {
public:
  // Parameters
  unsigned int nb_camera_, nb_board_;

#if MC_CALIB_USE_LEGACY_ARUCO_API
  cv::Ptr<cv::aruco::Dictionary> dict_ = cv::aruco::getPredefinedDictionary(
      cv::aruco::DICT_6X6_1000); // load the dictionary that correspond to the
                                 // charuco board
  cv::Ptr<cv::aruco::DetectorParameters> charuco_params_ =
      cv::aruco::DetectorParameters::create(); // parameters for detection
#else
  cv::aruco::Dictionary dict_ = cv::aruco::getPredefinedDictionary(
      cv::aruco::DICT_6X6_1000); // load the dictionary that correspond to the
                                 // charuco board
  cv::aruco::DetectorParameters charuco_params_ =
      cv::aruco::DetectorParameters(); // parameters for detection
#endif

  float min_perc_pts_;

  // images path
  std::filesystem::path root_path_;
  std::string cam_prefix_;

  // intput/output path
  std::filesystem::path
      cam_params_path_; // path to precalibrated cameras intrinsics
  std::filesystem::path keypoints_path_; // path to predetected keypoints
  std::filesystem::path save_path_; // path to save calibrated cameras parameter
  std::filesystem::path
      camera_params_file_name_;  // file name with cameras params
  int save_repro_, save_detect_; // flag to save or not the images

  // various boards size parameters
  std::vector<int> number_x_square_per_board_, number_y_square_per_board_;
  std::vector<int> resolution_x_per_board_, resolution_y_per_board_;
  std::vector<double> square_size_per_board_;

  // parameters corner refinement
  bool refine_corner_ = true;
  int corner_ref_window_ = 5;    // half size window for corner ref
  int corner_ref_max_iter_ = 20; // max iterations for corner ref

  // Optimization parameters
  bool quaternion_averaging_ =
      true; // use Quaternion Averaging or median for average rotation
  float ransac_thresh_ = 10; // threshold in pixel
  int nb_iterations_ = 1000; // max number of iteration for refinements

  // hand-eye technique
  int he_approach_ = 0;

  // fix intrinsic parameters
  int fix_intrinsic_ = 0;

  // Data structures
  std::map<int, std::shared_ptr<BoardObs>>
      board_observations_; // Observation of the boards (2d points)
  std::map<int, std::shared_ptr<Camera>> cams_; // The cameras to be calibrated
  std::map<std::pair<int, int>, std::shared_ptr<CameraObs>>
      cams_obs_; // The cameras to be calibrated key=Cam ind/Frame ind
  std::map<int, std::shared_ptr<Board>>
      boards_3d_; // the 3D boards used for the calibration key=3D board ind
  std::map<int, std::shared_ptr<Frame>> frames_;       // list of Frames
  std::map<int, std::shared_ptr<Object3D>> object_3d_; // list of 3D objects
  std::map<int, std::shared_ptr<Object3DObs>>
      object_observations_; // Observation of the boards (2d points)
  std::map<int, std::shared_ptr<CameraGroup>>
      cam_group_; // list of camera group
  std::map<std::pair<int, int>, std::shared_ptr<CameraGroupObs>>
      cams_group_obs_; // The cameras group key=CamGroup ind/Frame ind

  // Relationship between boards seen in the same images
  std::map<std::pair<int, int>, std::vector<cv::Mat>>
      board_pose_pairs_; // key: (boardind1,boardind2) value: Vector of poses
  std::map<std::pair<int, int>, cv::Mat>
      inter_board_transform_; // key: (boardind1,boardind2) value: Pose between
                              // the two boards
  Graph covis_boards_graph_;  // graph of inter-boards relationship (vertex:
                              // boardId, edge: number of co-visibility)

  // Relationship between cameras seeing the same objects
  std::map<std::pair<int, int>, std::vector<cv::Mat>>
      camera_pose_pairs_; // key: (boardind1,boardind2) value: Vector of poses
  std::map<std::pair<int, int>, cv::Mat>
      inter_camera_transform_; // key: (cameraind1,cameraind2) value: Pose
                               // between the two cameras
  Graph covis_camera_graph_;   // graph of inter-cameras relationship (vertex:
                               // cameraID, edge: number of co-visibility)

  // Relationship between 3d object seeing in the same frame
  // Relationship between object seen in the same images
  std::map<std::pair<int, int>, std::vector<cv::Mat>>
      object_pose_pairs_; // key: (objectind1,objectind2) value: Vector of poses
  std::map<std::pair<int, int>, cv::Mat>
      inter_object_transform_; // key: (objectind1,objectind2) value: Pose
                               // between the two objects
  Graph covis_objects_graph_;  // graph of inter-objects relationship (vertex:
                               // ObjectId, edge: number of co-visibility)

  // Non-overlaping parameters and datastructure
  std::map<std::pair<int, int>, std::pair<int, int>>
      no_overlap_object_pair_; // key: (CamGroup1, CamGroup2) value:
                               // (object id1, object id2)
  std::map<std::pair<int, int>, cv::Mat>
      no_overlap_camgroup_pair_pose_; // key (CamGroup1, CamGroup2), value: 4x4
                                      // transformation matrix
  std::map<std::pair<int, int>, int>
      no_overlap__camgroup_pair_common_cnt_; // count number of frame in common
                                             // btw the two groups
  Graph no_overlap_camgroup_graph_;          // graph of inter-camgroup pose
                                             // determined without overlapping

  // Main functions
  /** @brief Initialize camera intrinsics for all cameras. */
  void initIntrinsic();

  /** @brief Build and refine 3D objects from inter-board relationships. */
  void calibrate3DObjects();

  /** @brief Build and refine camera groups from overlapping observations. */
  void calibrateCameraGroup();

  /** @brief Merge calibrated object and camera-group results into final output.
   */
  void merge3DObjects();

  // Functions
  Calibration() = delete;

  /** @brief Destroy calibration context. */
  ~Calibration(){};

  /**
   * @brief Construct calibration pipeline from YAML configuration.
   *
   * @param config_path Path to calibration configuration file.
   */
  Calibration(const std::filesystem::path
                  &config_path); // initialize the charuco pattern, nb
                                 // of cameras, nb of boards etc.
  Calibration(const Calibration &) = delete;
  Calibration &operator=(const Calibration &) = delete;

  /** @brief Parse image folders and create BoardObs/CameraObs/Frame entries. */
  void boardExtraction();

  /** @brief Load previously saved keypoint detections from disk. */
  void loadDetectedKeypoints();

  /** @brief Detect boards in all cameras and frames. */
  void detectBoards(); // detect the boards in all images with all cameras

  /**
   * @brief Detect boards for one camera over all its frames.
   *
   * @param fn Frame file paths for this camera.
   * @param cam Camera id.
   */
  void detectBoardsWithCamera(
      const std::vector<cv::String> &fn,
      const int cam); // detect the boards in all images with a camera
  /** @brief Save intrinsic and extrinsic parameters for all cameras. */
  void saveCamerasParams(); // Save all cameras params
  /** @brief Save reconstructed 3D objects to disk. */
  void save3DObj(); // Save 3D objects
  /** @brief Save estimated object poses to disk. */
  void save3DObjPose(); // Save 3D objects pose
  /** @brief Save detected keypoints for reuse in future runs. */
  void saveDetectedKeypoints() const; // save detection keypoints, can be
                                      // re-used to save time in detection stage

  /**
   * @brief Draw and display detected boards for one camera/frame.
   *
   * @param image Image to draw on.
   * @param cam_idx Camera id.
   * @param frame_idx Frame id.
   */
  void displayBoards(const cv::Mat &image, const int cam_idx,
                     const int frame_idx);

  /**
   * @brief Create and register one board observation across all structures.
   *
   * @param cam_idx Camera id.
   * @param frame_idx Frame id.
   * @param board_idx Board id.
   * @param pts_2d Detected 2D points.
   * @param charuco_idx Detected Charuco ids.
   * @param frame_path Path to source image.
   */
  void insertNewBoard(
      const int cam_idx, const int frame_idx, const int board_idx,
      const std::vector<cv::Point2f> &pts_2d,
      const std::vector<int> &charuco_idx,
      const std::filesystem::path &frame_path); // insert a new board in all the
                                                // different datastructure

  /**
   * @brief Register one object observation in global calibration containers.
   *
   * @param new_obj_obs Object observation to insert.
   */
  void
  insertNewObjectObservation(const std::shared_ptr<Object3DObs>
                                 new_obj_obs); // insert new object observation

  /** @brief Initialize intrinsics for every camera. */
  void initializeCalibrationAllCam(); // initialize the calibration of all the
                                      // cameras

  /** @brief Estimate pose of all board observations with PnP. */
  void estimatePoseAllBoards(); // Estimate the pose of all visible boards using
                                // a PnP

  /** @brief Refine camera intrinsics and board poses jointly. */
  void refineIntrinsicAndPoseAllCam(); // Refine all the cameras intrinsic and
                                       // pose wrt. the boards

  /** @brief Compute reprojection error statistics for all board observations.
   */
  void computeReproErrAllBoard(); // compute the reprojection error for al
                                  // the boards

  /** @brief Compute relative poses between co-visible board pairs. */
  void computeBoardsPairPose(); // compute the poses between all the pairs of
                                // boards visible simultaneous in an image

  /**
   * @brief Initialize mean pairwise transform map from raw pose-pair samples.
   *
   * @param pose_pairs Input pose samples per entity pair.
   * @param inter_transform Output averaged transform per pair.
   */
  void initInterTransform(
      const std::map<std::pair<int, int>, std::vector<cv::Mat>> &pose_pairs,
      std::map<std::pair<int, int>, cv::Mat>
          &inter_transform); // compute the mean transformation between pair of
                             // poses
  /** @brief Build board covisibility graph from estimated pair transforms. */
  void initInterBoardsGraph(); // Initialize the graph
  /** @brief Initialize 3D objects from connected components of board graph. */
  void init3DObjects(); // initialize the 3D objects with the board graph
  /** @brief Build object observations for one object id across all frames. */
  void init3DObjectObs(
      const int object_idx); // initialize the 3D objects observations
  /** @brief Build object observations for all object ids. */
  void initAll3DObjectObs(); // initialize all the 3D objects observations
  /** @brief Estimate pose of each object observation. */
  void estimatePoseAllObjects(); // Estimate the pose of all visible object
                                 // using a PnP
  /** @brief Compute reprojection error statistics for object observations. */
  void computeReproErrAllObject(); // compute the reprojection error for al
                                   // the objects
  /** @brief Refine all reconstructed 3D objects. */
  void refineAllObject3D(); // Refine all the 3D objects
  /** @brief Compute relative camera poses from co-visible object observations.
   */
  void computeCamerasPairPose(); // compute the poses between all the pairs of
                                 // cameras seeings objects simultaneously
  /** @brief Build camera covisibility graph from inter-camera transforms. */
  void initInterCamerasGraph(); // Initialize the graph for cameras
  /** @brief Initialize camera groups from camera graph connected components. */
  void initCameraGroup(); // Initialize camera group
  /** @brief Build camera-group observations for one camera-group id. */
  void initCameraGroupObs(
      const int camera_group_idx); // Initialize observation of cameraGroup
  /** @brief Build camera-group observations for all groups. */
  void initAllCameraGroupObs(); // initialize all camera groups
  /** @brief Refine all camera groups with fixed intrinsics. */
  void refineAllCameraGroup(); // Refine all camera group pose
  /** @brief Find anchor object pairs for non-overlapping camera groups. */
  void findPairObjectForNonOverlap();

  /**
   * @brief Initialize relative pose between two non-overlapping camera groups.
   *
   * @param cam_group_id1 First camera-group id.
   * @param cam_group_id2 Second camera-group id.
   */
  void initNonOverlapPair(
      const int cam_group_id1,
      const int cam_group_id2); // Initialize the pose between two non
                                // overlapping groups of cameras
  /** @brief Estimate poses for all non-overlapping camera-group pairs. */
  void findPoseNoOverlapAllCamGroup(); // initialize the pose between all non
                                       // overlapping camera groups
  /** @brief Build graph of non-overlapping camera-group constraints. */
  void
  initInterCamGroupGraph(); // Initialize camera group graph without overlaping
  /** @brief Merge camera groups into a unified global group. */
  void mergeCameraGroup(); // Merge the camera groups
  /** @brief Merge observations from all camera groups into final group. */
  void mergeAllCameraGroupObs(); // merge camera group observation in the final
                                 // camera group
  /** @brief Recompute all object poses in their corresponding camera groups. */
  void computeAllObjPoseInCameraGroup();
  /** @brief Compute pairwise object transforms from co-visibility. */
  void computeObjectsPairPose();

  /**
   * @brief Compute point-wise reprojection distances.
   *
   * @param obj_pts_2d Measured 2D points.
   * @param repro_pts Reprojected 2D points.
   * @return Column vector of Euclidean distances in pixels.
   */
  cv::Mat
  computeDistanceBetweenPoints(const std::vector<cv::Point2f> &obj_pts_2d,
                               const std::vector<cv::Point2f> &repro_pts);

  /** @brief Compute global mean reprojection error over all observations. */
  double computeAvgReprojectionError();

  /** @brief Build object covisibility graph from inter-object transforms. */
  void initInterObjectsGraph();

  /** @brief Merge objects into consolidated object models. */
  void mergeObjects();

  /** @brief Merge object observations into final merged objects. */
  void mergeAllObjectObs();

  /** @brief Report reprojection error per camera group. */
  void reproErrorAllCamGroup();

  /** @brief Jointly refine camera-group poses and object poses. */
  void refineAllCameraGroupAndObjects();

  /** @brief Jointly refine camera-group poses, object poses, and intrinsics. */
  void refineAllCameraGroupAndObjectsAndIntrinsic();

  /** @brief Save reprojection overlay images for one camera id. */
  void saveReprojectionImages(const int cam_id);

  /** @brief Save reprojection overlay images for all cameras. */
  void saveReprojectionImagesAllCam();

  /** @brief Save board detection overlay images for one camera id. */
  void saveDetectionImages(const int cam_id);

  /** @brief Save board detection overlay images for all cameras. */
  void saveDetectionImagesAllCam();

  /** @brief Export reprojection error statistics to disk. */
  void saveReprojectionErrorToFile();

private:
  /**
   * @brief Detect boards in one image for one camera/frame pair.
   *
   * @param frame_path Image path.
   * @param cam_idx Camera id.
   * @param frame_idx Frame id.
   */
  void detectBoardsInImageWithCamera(
      const std::string &frame_path, const int cam_idx,
      const int frame_idx); // detect the boards in the input frame

  std::mutex insert_new_board_lock_;
};

} // namespace McCalib
