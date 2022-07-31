#include <boost/test/unit_test.hpp>

#include <iomanip>
#include <math.h>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include <../src/Board.hpp>
#include <../src/BoardObs.hpp>
#include <../src/Calibration.hpp>
#include <../src/Camera.hpp>
#include <../src/CameraObs.hpp>
#include <../src/Frame.hpp>

#define PI 3.14159265

double INTRINSICS_TOLERANCE = 4.0;          // in percentage
double TRANSLATION_ERROR_TOLERANCE = 0.005; // in meters
double ROTATION_ERROR_TOLERANCE = 1.0;      // in degrees

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

void calibrate(Calibration &Calib) {
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
}

void calibrateAndCheckGt(std::string config_path, std::string gt_path) {
  Calibration Calib(config_path);
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
    cv::Mat transform = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, -1, 0, 0, 0,
                         0, -1, 0, 0, 0, 0, 1);
    camera_pose_matrix_gt = transform * camera_pose_matrix_gt * transform;

    double fx_gt = camera_matrix_gt.at<double>(0, 0);
    double fy_gt = camera_matrix_gt.at<double>(1, 0);
    double cx_gt = camera_matrix_gt.at<double>(0, 2);
    double cy_gt = camera_matrix_gt.at<double>(1, 2);
    cv::Mat rot_gt(3, 3, CV_64F);
    rot_gt = camera_pose_matrix_gt(cv::Range(0, 3), cv::Range(0, 3));
    cv::Mat tran_gt(3, 1, CV_64F);
    tran_gt = camera_pose_matrix_gt(cv::Range(0, 3), cv::Range(3, 4));

    // get calibrated values
    std::shared_ptr<Camera> cur_cam = Calib.cams_[camera_idx - 1];
    int camera_group_idx = 0; // specific to the setup with single camera group
    cv::Mat camera_matrix_pred = cur_cam->getCameraMat();
    cv::Mat camera_pose_matrix_pred =
        Calib.cam_group_[camera_group_idx]->getCameraPoseMat(camera_idx - 1);

    double fx_pred = camera_matrix_pred.at<double>(0, 0);
    double fy_pred = camera_matrix_pred.at<double>(1, 0);
    double cx_pred = camera_matrix_pred.at<double>(0, 2);
    double cy_pred = camera_matrix_pred.at<double>(1, 2);
    cv::Mat rot_pred(3, 3, CV_64F);
    rot_pred = camera_pose_matrix_pred(cv::Range(0, 3), cv::Range(0, 3));
    cv::Mat tran_pred(3, 1, CV_64F);
    tran_pred = camera_pose_matrix_pred(cv::Range(0, 3), cv::Range(3, 4));

    double tran_error = getTranslationError(tran_pred, tran_gt);
    double rot_error = getRotationError(rot_pred, rot_gt);

    // perform verifications
    BOOST_CHECK_CLOSE(fx_pred, fx_gt, INTRINSICS_TOLERANCE);
    BOOST_CHECK_CLOSE(fy_pred, fy_gt, INTRINSICS_TOLERANCE);
    BOOST_CHECK_CLOSE(cx_pred, cx_gt, INTRINSICS_TOLERANCE);
    BOOST_CHECK_CLOSE(cy_pred, cy_gt, INTRINSICS_TOLERANCE);
    BOOST_CHECK_SMALL(tran_error, TRANSLATION_ERROR_TOLERANCE);
    BOOST_CHECK_SMALL(rot_error, ROTATION_ERROR_TOLERANCE);
  }
}

BOOST_AUTO_TEST_SUITE(CheckCalibration)

BOOST_AUTO_TEST_CASE(CheckBlenderDatasetIsPlacedCorrectly) {
  std::string blender_images_path = "../data/Blender_Images";
  bool is_path_existent = boost::filesystem::exists(blender_images_path);
  BOOST_REQUIRE_EQUAL(is_path_existent, true);
}

BOOST_AUTO_TEST_CASE(CheckCalibrationSyntheticScenario1) {
  std::string config_path =
      "../tests/configs_for_end2end_tests/calib_param_synth_Scenario1.yml";
  std::string gt_path = "../data/Blender_Images/Scenario_1/GroundTruth.yml";
  BOOST_REQUIRE_EQUAL(boost::filesystem::exists(config_path), true);
  BOOST_REQUIRE_EQUAL(boost::filesystem::exists(gt_path), true);
  calibrateAndCheckGt(config_path, gt_path);
}

BOOST_AUTO_TEST_CASE(CheckCalibrationSyntheticScenario2) {
  std::string config_path =
      "../tests/configs_for_end2end_tests/calib_param_synth_Scenario2.yml";
  std::string gt_path = "../data/Blender_Images/Scenario_2/GroundTruth.yml";
  BOOST_REQUIRE_EQUAL(boost::filesystem::exists(config_path), true);
  BOOST_REQUIRE_EQUAL(boost::filesystem::exists(gt_path), true);
  calibrateAndCheckGt(config_path, gt_path);
}

BOOST_AUTO_TEST_CASE(CheckCalibrationSyntheticScenario3) {
  std::string config_path =
      "../tests/configs_for_end2end_tests/calib_param_synth_Scenario3.yml";
  std::string gt_path = "../data/Blender_Images/Scenario_3/GroundTruth.yml";
  BOOST_REQUIRE_EQUAL(boost::filesystem::exists(config_path), true);
  BOOST_REQUIRE_EQUAL(boost::filesystem::exists(gt_path), true);
  calibrateAndCheckGt(config_path, gt_path);
}

BOOST_AUTO_TEST_CASE(CheckCalibrationSyntheticScenario4) {
  std::string config_path =
      "../tests/configs_for_end2end_tests/calib_param_synth_Scenario4.yml";
  std::string gt_path = "../data/Blender_Images/Scenario_4/GroundTruth.yml";
  BOOST_REQUIRE_EQUAL(boost::filesystem::exists(config_path), true);
  BOOST_REQUIRE_EQUAL(boost::filesystem::exists(gt_path), true);
  calibrateAndCheckGt(config_path, gt_path);
}

BOOST_AUTO_TEST_CASE(CheckCalibrationSyntheticScenario5) {
  std::string config_path =
      "../tests/configs_for_end2end_tests/calib_param_synth_Scenario5.yml";
  std::string gt_path = "../data/Blender_Images/Scenario_5/GroundTruth.yml";
  BOOST_REQUIRE_EQUAL(boost::filesystem::exists(config_path), true);
  BOOST_REQUIRE_EQUAL(boost::filesystem::exists(gt_path), true);
  calibrateAndCheckGt(config_path, gt_path);
}

BOOST_AUTO_TEST_SUITE_END()