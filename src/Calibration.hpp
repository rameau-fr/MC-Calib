#pragma once

#include "boost/filesystem.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <numeric>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

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

/**
 * @class Calibration
 *
 * @brief Core function of the toolbox
 *
 * Centralization of all the data to perform the calibration.
 */
class Calibration final
{
public:
  // Parameters
  int nb_camera_, nb_board_;
  cv::Ptr<cv::aruco::Dictionary> dict_ = cv::aruco::getPredefinedDictionary(
      cv::aruco::DICT_6X6_1000); // load the dictionary that correspond to the
                                 // charuco board
  cv::Ptr<cv::aruco::DetectorParameters> charuco_params_ =
      cv::aruco::DetectorParameters::create(); // parameters for detection
  float min_perc_pts_;

  // images path
  std::string root_dir_, cam_prefix_;

  // intput/output path
  std::string cam_params_path_; // path to precalibrated cameras intrinsics
  std::string save_path_;       // path to save calibrated cameras parameter
  std::string camera_params_file_name_; // file name with cameras params
  int save_repro_, save_detect_;        // flag to save or not the images

  // various boards size parameters
  std::vector<int> number_x_square_per_board_, number_y_square_per_board_;
  std::vector<int> resolution_x_per_board_, resolution_y_per_board_;
  std::vector<double> square_size_per_board_;

  // parameters corner refinement
  bool refine_corner_;
  int corner_ref_window_ = 5;    // half size window for corner ref
  int corner_ref_max_iter_ = 20; // max iterations for corner ref

  // Optimization parameters
  float ransac_thresh_; // threshold in pixel
  int nb_iterations_;   // max number of iteration for refinements

  // hand-eye technique
  int he_approach_;

  // fix intrinsic parameters
  int fix_intrinsic_;

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
  void initIntrinsic();
  void calibrate3DObjects();
  void calibrateCameraGroup();
  void merge3DObjects();

  // Functions
  Calibration();
  ~Calibration(){};
  void initialization(
      const std::string config_path); // initialize the charuco pattern, nb
                                      // of cameras, nb of boards etc.
  void boardExtraction();
  void detectBoards(
      const cv::Mat image, const int cam_idx, const int frame_idx,
      const std::string frame_path); // detect the board in the input frame
  void saveCamerasParams();          // Save all cameras params
  void save3DObj();                  // Save 3D objects
  void save3DObjPose();              // Save 3D objects pose
  void displayBoards(const cv::Mat image, const int cam_idx,
                     const int frame_idx);
  void
  insertNewBoard(const int cam_idx, const int frame_idx, const int board_idx,
                 const std::vector<cv::Point2f> pts_2d,
                 const std::vector<int> charuco_idx,
                 const std::string frame_path); // insert a new board in all the
                                                // different datastructure
  void
  insertNewObjectObservation(std::shared_ptr<Object3DObs>
                                 new_obj_obs); // insert new object observation
  void initializeCalibrationAllCam(); // initialize the calibration of all the
                                      // cameras
  void estimatePoseAllBoards(); // Estimate the pose of all visible boards using
                                // a PnP
  void refineIntrinsicAndPoseAllCam(); // Refine all the cameras intrinsic and
                                       // pose wrt. the boards
  void computeReproErrAllBoard();      // compute the reprojection error for al
                                       // the boards
  void computeBoardsPairPose();    // compute the poses between all the pairs of
                                   // boards visible simultaneous in an image
  void initInterBoardsTransform(); // compute the mean transformation between
                                   // the pairs of boards
  void initInterBoardsGraph();     // Initialize the graph
  void init3DObjects(); // initialize the 3D objects with the board graph
  void init3DObjectObs(
      const int object_idx);       // initialize the 3D objects observations
  void initAll3DObjectObs();       // initialize all the 3D objects observations
  void estimatePoseAllObjects();   // Estimate the pose of all visible object
                                   // using a PnP
  void computeReproErrAllObject(); // compute the reprojection error for al
                                   // the objects
  void refineAllObject3D();        // Refine all the 3D objects
  void computeCamerasPairPose();   // compute the poses between all the pairs of
                                   // cameras seeings objects simultaneously
  void initInterCamerasTransform(); // compute the mean transformation between
                                    // the pairs of cameras
  void initInterCamerasGraph();     // Initialize the graph for cameras
  void initCameraGroup();           // Initialize camera group
  void initCameraGroupObs(
      const int camera_group_idx); // Initialize observation of cameraGroup
  void initAllCameraGroupObs();    // initialize all camera groups
  void refineAllCameraGroup();     // Refine all camera group pose
  void findPairObjectForNonOverlap();
  void initNonOverlapPair(
      const int cam_group_id1,
      const int cam_group_id2);        // Initialize the pose between two non
                                       // overlapping groups of cameras
  void findPoseNoOverlapAllCamGroup(); // initialize the pose between all non
                                       // overlapping camera groups
  void
  initInterCamGroupGraph(); // Initialize camera group graph without overlaping
  void mergeCameraGroup();  // Merge the camera groups
  void mergeAllCameraGroupObs(); // merge camera group observation in the final
                                 // camera group
  void computeAllObjPoseInCameraGroup();
  void computeObjectsPairPose();
  cv::Mat
  computeDistanceBetweenPoints(const std::vector<cv::Point2f> obj_pts_2d,
                               const std::vector<cv::Point2f> repro_pts);
  double computeAvgReprojectionError();
  void initInterObjectsTransform();
  void initInterObjectsGraph();
  void mergeObjects();
  void mergeAllObjectObs();
  void reproErrorAllCamGroup();
  void refineAllCameraGroupAndObjects();
  void refineAllCameraGroupAndObjectsAndIntrinsic();
  void saveReprojection(const int cam_id);
  void saveReprojectionAllCam();
  void saveDetection(const int cam_id);
  void saveDetectionAllCam();
  void saveReprojectionErrorToFile();
};
