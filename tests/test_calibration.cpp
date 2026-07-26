#include <filesystem>
#include <iomanip>
#include <math.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <boost/test/unit_test.hpp>
#include <opencv2/opencv.hpp>
#include <opencv_compat.hpp>

#include <McCalib.hpp>

#define PI 3.14159265

constexpr double INTRINSICS_TOLERANCE = 4.0;     // in percentage
constexpr double ROTATION_ERROR_TOLERANCE = 1.0; // in degrees

#if MC_CALIB_USE_LEGACY_ARUCO_API
constexpr double TRANSLATION_ERROR_TOLERANCE = 0.005; // in meters
#else
constexpr double TRANSLATION_ERROR_TOLERANCE = 0.01; // in meters
#endif

double getTranslationError(cv::Mat a, cv::Mat b) {
  double dist = cv::norm(a, b, cv::NORM_L2);
  return dist;
}

double getRotationError(cv::Mat a, cv::Mat b) {
  cv::Mat a_transpose(3, 3, CV_64F);
  cv::transpose(a, a_transpose);
  double trace = cv::trace(a_transpose * b).val[0];
  double rot_error = std::acos(0.5 * (trace - 1.0)) * 180.0 / PI;
  return rot_error;
}

void calibrate(McCalib::Calibration &Calib) {
  // calibrate
  Calib.boardExtraction();
  Calib.initIntrinsic();
  Calib.calibrate3DObjects();
  Calib.calibrateCameraGroup();
  Calib.merge3DObjects();
  Calib.findPairObjectForNonOverlap();
  Calib.findPoseNoOverlapAllCamGroup();
  Calib.initInterCamGroupGraph();
  Calib.mergeCameraGroup();
  Calib.mergeAllCameraGroupObs();
  Calib.merge3DObjects();
  Calib.initInterCamGroupGraph();
  Calib.mergeCameraGroup();
  Calib.mergeAllCameraGroupObs();
  Calib.estimatePoseAllObjects();
  Calib.computeAllObjPoseInCameraGroup();
  Calib.refineAllCameraGroupAndObjects();
  if (Calib.fix_intrinsic_ == 0)
    Calib.refineAllCameraGroupAndObjectsAndIntrinsic();
  Calib.reproErrorAllCamGroup();

  // save calibration results (needed for Gitlab's CI jobs)
  Calib.saveCamerasParams();
  Calib.save3DObj();
  Calib.save3DObjPose();
  Calib.saveReprojectionErrorToFile();
  Calib.saveDetectedKeypoints();
}

void calibrateAndCheckGt(const std::filesystem::path &config_path,
                         const std::filesystem::path &gt_path,
                         const double intrinsics_tolerance,
                         const double translation_error_tolerance,
                         const double rotation_error_tolerance) {
  McCalib::Calibration Calib(config_path);
  calibrate(Calib);

  // read ground truth
  cv::FileStorage fs;
  fs.open(gt_path, cv::FileStorage::READ);
  int num_cameras;
  fs["nb_camera"] >> num_cameras;
  for (int camera_idx = 1; camera_idx <= num_cameras; ++camera_idx) {
    // get ground truth values
    cv::Mat camera_matrix_gt;
    cv::Mat camera_pose_matrix_gt;
    fs["K_" + std::to_string(camera_idx)] >> camera_matrix_gt;
    fs["P_" + std::to_string(camera_idx)] >> camera_pose_matrix_gt;

    // blender images has different axis orientation, correct to match opencv
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    transform.at<double>(1, 1) = -1.0;
    transform.at<double>(2, 2) = -1.0;
    camera_pose_matrix_gt = transform * camera_pose_matrix_gt * transform;

    double fx_gt = camera_matrix_gt.at<double>(0, 0);
    double fy_gt = camera_matrix_gt.at<double>(1, 1);
    double cx_gt = camera_matrix_gt.at<double>(0, 2);
    double cy_gt = camera_matrix_gt.at<double>(1, 2);
    cv::Mat rot_gt(3, 3, CV_64F);
    rot_gt = camera_pose_matrix_gt(cv::Range(0, 3), cv::Range(0, 3));
    cv::Mat tran_gt(3, 1, CV_64F);
    tran_gt = camera_pose_matrix_gt(cv::Range(0, 3), cv::Range(3, 4));

    // get calibrated values
    std::shared_ptr<McCalib::Camera> cur_cam = Calib.cams_[camera_idx - 1];
    int camera_group_idx = 0; // specific to the setup with single camera group
    cv::Mat camera_matrix_pred = cur_cam->getCameraMat();
    cv::Mat camera_pose_matrix_pred =
        Calib.cam_group_[camera_group_idx]->getCameraPoseMat(camera_idx - 1);

    double fx_pred = camera_matrix_pred.at<double>(0, 0);
    double fy_pred = camera_matrix_pred.at<double>(1, 1);
    double cx_pred = camera_matrix_pred.at<double>(0, 2);
    double cy_pred = camera_matrix_pred.at<double>(1, 2);
    cv::Mat rot_pred(3, 3, CV_64F);
    rot_pred = camera_pose_matrix_pred(cv::Range(0, 3), cv::Range(0, 3));
    cv::Mat tran_pred(3, 1, CV_64F);
    tran_pred = camera_pose_matrix_pred(cv::Range(0, 3), cv::Range(3, 4));

    double tran_error = getTranslationError(tran_pred, tran_gt);
    double rot_error = getRotationError(rot_pred, rot_gt);

    // perform verifications
    BOOST_CHECK_CLOSE(fx_pred, fx_gt, intrinsics_tolerance);
    BOOST_CHECK_CLOSE(fy_pred, fy_gt, intrinsics_tolerance);
    BOOST_CHECK_CLOSE(cx_pred, cx_gt, intrinsics_tolerance);
    BOOST_CHECK_CLOSE(cy_pred, cy_gt, intrinsics_tolerance);
    BOOST_CHECK_SMALL(tran_error, translation_error_tolerance);
    BOOST_CHECK_SMALL(rot_error, rotation_error_tolerance);
  }
}

void createDerivedScenario2Config(const std::filesystem::path &out_config_path,
                                  const std::filesystem::path &save_path,
                                  const std::string &keypoints_path,
                                  const std::string &root_path_override) {
  const std::filesystem::path base_config_path =
      "../tests/configs_for_end2end_tests/calib_param_synth_Scenario2.yml";
  BOOST_REQUIRE_EQUAL(std::filesystem::exists(base_config_path), true);

  cv::FileStorage fs_in(base_config_path.string(), cv::FileStorage::READ);

  int number_x_square, number_y_square, resolution_x, resolution_y;
  double length_square, length_marker, square_size;
  int number_board, distortion_model, number_camera;
  int refine_corner, fix_intrinsic, save_detection, save_reprojection;
  double min_perc_pts, ransac_threshold;
  int number_iterations, he_approach;
  std::string root_path, cam_prefix, cam_params_path, camera_params_file_name;
  std::vector<int> boards_index, distortion_per_camera;
  std::vector<int> number_x_square_per_board, number_y_square_per_board;
  std::vector<double> square_size_per_board;

  fs_in["number_x_square"] >> number_x_square;
  fs_in["number_y_square"] >> number_y_square;
  fs_in["resolution_x"] >> resolution_x;
  fs_in["resolution_y"] >> resolution_y;
  fs_in["length_square"] >> length_square;
  fs_in["length_marker"] >> length_marker;
  fs_in["number_board"] >> number_board;
  fs_in["boards_index"] >> boards_index;
  fs_in["square_size"] >> square_size;
  fs_in["number_x_square_per_board"] >> number_x_square_per_board;
  fs_in["number_y_square_per_board"] >> number_y_square_per_board;
  fs_in["square_size_per_board"] >> square_size_per_board;
  fs_in["distortion_model"] >> distortion_model;
  fs_in["distortion_per_camera"] >> distortion_per_camera;
  fs_in["number_camera"] >> number_camera;
  fs_in["refine_corner"] >> refine_corner;
  fs_in["min_perc_pts"] >> min_perc_pts;
  fs_in["cam_params_path"] >> cam_params_path;
  fs_in["fix_intrinsic"] >> fix_intrinsic;
  fs_in["root_path"] >> root_path;
  fs_in["cam_prefix"] >> cam_prefix;
  fs_in["ransac_threshold"] >> ransac_threshold;
  fs_in["number_iterations"] >> number_iterations;
  fs_in["he_approach"] >> he_approach;
  fs_in["save_detection"] >> save_detection;
  fs_in["save_reprojection"] >> save_reprojection;
  fs_in["camera_params_file_name"] >> camera_params_file_name;
  fs_in.release();

  std::filesystem::create_directories(out_config_path.parent_path());
  std::filesystem::create_directories(save_path);

  cv::FileStorage fs_out(out_config_path.string(), cv::FileStorage::WRITE);
  fs_out << "number_x_square" << number_x_square;
  fs_out << "number_y_square" << number_y_square;
  fs_out << "resolution_x" << resolution_x;
  fs_out << "resolution_y" << resolution_y;
  fs_out << "length_square" << length_square;
  fs_out << "length_marker" << length_marker;
  fs_out << "number_board" << number_board;
  fs_out << "boards_index" << boards_index;
  fs_out << "square_size" << square_size;
  fs_out << "number_x_square_per_board" << number_x_square_per_board;
  fs_out << "number_y_square_per_board" << number_y_square_per_board;
  fs_out << "square_size_per_board" << square_size_per_board;

  fs_out << "distortion_model" << distortion_model;
  fs_out << "distortion_per_camera" << distortion_per_camera;
  fs_out << "number_camera" << number_camera;
  fs_out << "refine_corner" << refine_corner;
  fs_out << "min_perc_pts" << min_perc_pts;
  fs_out << "cam_params_path" << cam_params_path;
  fs_out << "fix_intrinsic" << fix_intrinsic;

  const std::string root_path_for_test =
      root_path_override.empty() ? root_path : root_path_override;
  fs_out << "root_path" << root_path_for_test;
  fs_out << "cam_prefix" << cam_prefix;
  fs_out << "keypoints_path" << keypoints_path;

  fs_out << "ransac_threshold" << ransac_threshold;
  fs_out << "number_iterations" << number_iterations;
  fs_out << "he_approach" << he_approach;

  fs_out << "save_path" << save_path.string();
  fs_out << "save_detection" << save_detection;
  fs_out << "save_reprojection" << save_reprojection;
  fs_out << "camera_params_file_name" << camera_params_file_name;
  fs_out.release();
}

BOOST_AUTO_TEST_SUITE(CheckCalibration)

BOOST_AUTO_TEST_CASE(CheckBlenderDatasetIsPlacedCorrectly) {
  const std::filesystem::path blender_images_path = "../data/Blender_Images";
  bool is_path_existent = std::filesystem::exists(blender_images_path);
  BOOST_REQUIRE_EQUAL(is_path_existent, true);
}

BOOST_AUTO_TEST_CASE(CheckCalibrationSyntheticScenario1) {
  const std::filesystem::path config_path =
      "../tests/configs_for_end2end_tests/calib_param_synth_Scenario1.yml";
  const std::filesystem::path gt_path =
      "../data/Blender_Images/Scenario_1/GroundTruth.yml";
  BOOST_REQUIRE_EQUAL(std::filesystem::exists(config_path), true);
  BOOST_REQUIRE_EQUAL(std::filesystem::exists(gt_path), true);
  calibrateAndCheckGt(config_path, gt_path, INTRINSICS_TOLERANCE,
                      TRANSLATION_ERROR_TOLERANCE, ROTATION_ERROR_TOLERANCE);
}

BOOST_AUTO_TEST_CASE(CheckCalibrationSyntheticScenario2) {
  const std::filesystem::path config_path =
      "../tests/configs_for_end2end_tests/calib_param_synth_Scenario2.yml";
  const std::filesystem::path gt_path =
      "../data/Blender_Images/Scenario_2/GroundTruth.yml";
  BOOST_REQUIRE_EQUAL(std::filesystem::exists(config_path), true);
  BOOST_REQUIRE_EQUAL(std::filesystem::exists(gt_path), true);
  calibrateAndCheckGt(config_path, gt_path, INTRINSICS_TOLERANCE,
                      TRANSLATION_ERROR_TOLERANCE, ROTATION_ERROR_TOLERANCE);
}

BOOST_AUTO_TEST_CASE(CheckScenario2DetectSaveThenLoadKeypoints) {
  const std::filesystem::path temp_root =
      std::filesystem::temp_directory_path() /
      "mc_calib_scenario2_keypoints_roundtrip";
  std::filesystem::remove_all(temp_root);
  std::filesystem::create_directories(temp_root);

  const std::filesystem::path detect_save_path = temp_root / "results_detect";
  const std::filesystem::path load_save_path = temp_root / "results_load";
  const std::filesystem::path detect_cfg = temp_root / "scenario2_detect.yml";
  const std::filesystem::path load_cfg = temp_root / "scenario2_load.yml";
  const std::filesystem::path keypoints_file_path =
      detect_save_path / "detected_keypoints_data.yml";

  createDerivedScenario2Config(detect_cfg, detect_save_path, "None", "");

  McCalib::Calibration detect_calib(detect_cfg);
  calibrate(detect_calib);

  BOOST_REQUIRE_EQUAL(std::filesystem::exists(keypoints_file_path), true);
  BOOST_REQUIRE(detect_calib.board_observations_.size() > 0);
  BOOST_REQUIRE(detect_calib.frames_.size() > 0);

  createDerivedScenario2Config(
      load_cfg, load_save_path, keypoints_file_path.string(),
      "/tmp/mc_calib_non_existing_images_root_for_keypoint_loading");

  McCalib::Calibration load_calib(load_cfg);
  calibrate(load_calib);

  BOOST_REQUIRE(load_calib.board_observations_.size() > 0);
  BOOST_REQUIRE(load_calib.frames_.size() > 0);
  BOOST_CHECK_EQUAL(load_calib.board_observations_.size(),
                    detect_calib.board_observations_.size());
  BOOST_CHECK_EQUAL(load_calib.frames_.size(), detect_calib.frames_.size());

  const std::filesystem::path loaded_keypoints_saved_again =
      load_save_path / "detected_keypoints_data.yml";
  const std::filesystem::path reprojection_saved =
      load_save_path / "reprojection_error_data.yml";
  BOOST_CHECK_EQUAL(std::filesystem::exists(loaded_keypoints_saved_again),
                    true);
  BOOST_CHECK_EQUAL(std::filesystem::exists(reprojection_saved), true);
}

BOOST_AUTO_TEST_CASE(CheckCalibrationSyntheticScenario3) {
  const std::filesystem::path config_path =
      "../tests/configs_for_end2end_tests/calib_param_synth_Scenario3.yml";
  const std::filesystem::path gt_path =
      "../data/Blender_Images/Scenario_3/GroundTruth.yml";
  BOOST_REQUIRE_EQUAL(std::filesystem::exists(config_path), true);
  BOOST_REQUIRE_EQUAL(std::filesystem::exists(gt_path), true);
  calibrateAndCheckGt(config_path, gt_path, INTRINSICS_TOLERANCE,
                      TRANSLATION_ERROR_TOLERANCE, ROTATION_ERROR_TOLERANCE);
}

BOOST_AUTO_TEST_CASE(CheckCalibrationSyntheticScenario4) {
  const std::filesystem::path config_path =
      "../tests/configs_for_end2end_tests/calib_param_synth_Scenario4.yml";
  const std::filesystem::path gt_path =
      "../data/Blender_Images/Scenario_4/GroundTruth.yml";
  BOOST_REQUIRE_EQUAL(std::filesystem::exists(config_path), true);
  BOOST_REQUIRE_EQUAL(std::filesystem::exists(gt_path), true);
  calibrateAndCheckGt(config_path, gt_path, INTRINSICS_TOLERANCE,
                      TRANSLATION_ERROR_TOLERANCE, ROTATION_ERROR_TOLERANCE);
}

BOOST_AUTO_TEST_CASE(CheckCalibrationSyntheticScenario5) {
  const std::filesystem::path config_path =
      "../tests/configs_for_end2end_tests/calib_param_synth_Scenario5.yml";
  const std::filesystem::path gt_path =
      "../data/Blender_Images/Scenario_5/GroundTruth.yml";
  BOOST_REQUIRE_EQUAL(std::filesystem::exists(config_path), true);
  BOOST_REQUIRE_EQUAL(std::filesystem::exists(gt_path), true);
  calibrateAndCheckGt(config_path, gt_path, INTRINSICS_TOLERANCE,
                      3 * TRANSLATION_ERROR_TOLERANCE,
                      ROTATION_ERROR_TOLERANCE);
}

BOOST_AUTO_TEST_SUITE_END()