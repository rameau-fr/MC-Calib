#include <boost/test/unit_test.hpp>

#include <iomanip>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include <../src/Board.hpp>
#include <../src/BoardObs.hpp>
#include <../src/Calibration.hpp>
#include <../src/Camera.hpp>
#include <../src/CameraObs.hpp>
#include <../src/Frame.hpp>

// Reference: https://stackoverflow.com/a/17503436
#define CHECK_CLOSE_COLLECTION(aa, bb, tolerance)                              \
  {                                                                            \
    using std::distance;                                                       \
    using std::begin;                                                          \
    using std::end;                                                            \
    auto a = begin(aa), ae = end(aa);                                          \
    auto b = begin(bb);                                                        \
    BOOST_REQUIRE_EQUAL(distance(a, ae), distance(b, end(bb)));                \
    for (; a != ae; ++a, ++b) {                                                \
      BOOST_CHECK_CLOSE(*a, *b, tolerance);                                    \
    }                                                                          \
  }

void calibrateAndCheckGt(std::string config_path, std::string gt_path) {
  Calibration Calib;
  Calib.initialization(config_path);
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
  Calib.reproErrorAllCamGroup();
  Calib.saveCamerasParams();

  cv::FileStorage fs;
  fs.open(gt_path, cv::FileStorage::READ);
  for (std::map<int, std::shared_ptr<Camera>>::iterator it =
           Calib.cams_.begin();
       it != Calib.cams_.end(); ++it) {
    cv::FileNode loaded_cam_params = fs["camera_" + std::to_string(it->first)];

    cv::Mat camera_matrix_gt;
    cv::Mat distortion_coeffs_gt;
    int distortion_type_gt;
    int camera_group_gt;
    int img_width_gt;
    int img_height_gt;
    cv::Mat camera_pose_matrix_gt;

    loaded_cam_params["camera_matrix"] >> camera_matrix_gt;
    loaded_cam_params["distortion_vector"] >> distortion_coeffs_gt;
    loaded_cam_params["distortion_type"] >> distortion_type_gt;
    loaded_cam_params["camera_group_gt"] >> camera_group_gt;
    loaded_cam_params["img_width"] >> img_width_gt;
    loaded_cam_params["img_height"] >> img_height_gt;
    loaded_cam_params["camera_pose_matrix"] >> camera_pose_matrix_gt;

    cv::Mat camera_matrix_pred;
    cv::Mat distortion_coeffs_pred;
    int distortion_type_pred;
    int img_width_pred;
    int img_height_pred;
    cv::Mat camera_pose_matrix_pred;

    std::shared_ptr<Camera> cur_cam = it->second;
    cur_cam->getIntrinsics(camera_matrix_pred, distortion_coeffs_pred);
    distortion_type_pred = cur_cam->distortion_model_;
    img_width_pred = cur_cam->im_cols_;
    img_height_pred = cur_cam->im_rows_;
    camera_pose_matrix_pred =
        Calib.cam_group_[camera_group_gt]->getCameraPoseMat(cur_cam->cam_idx_);

    BOOST_REQUIRE_EQUAL(img_width_gt, img_width_pred);
    BOOST_REQUIRE_EQUAL(img_height_gt, img_height_pred);
    BOOST_REQUIRE_EQUAL(distortion_type_gt, distortion_type_pred);

    // TODO: switch to proper error metrics
    // std::vector<double>
    // camera_matrix_gt_vec(camera_matrix_gt.begin<double>(),
    //                                          camera_matrix_gt.end<double>());
    // std::vector<double> camera_matrix_pred_vec(
    //     camera_matrix_pred.begin<double>(),
    //     camera_matrix_pred.end<double>());
    // std::vector<double>
    // distortion_coeffs_gt_vec(distortion_coeffs_gt.begin<double>(),
    //                                        distortion_coeffs_gt.end<double>());
    // std::vector<double>
    // distortion_coeffs_pred_vec(distortion_coeffs_pred.begin<double>(),
    //                                          distortion_coeffs_pred.end<double>());
    // std::vector<double> camera_pose_matrix_gt_vec(
    //     camera_pose_matrix_gt.begin<double>(),
    //     camera_pose_matrix_gt.end<double>());
    // std::vector<double> camera_pose_matrix_pred_vec(
    //     camera_pose_matrix_pred.begin<double>(),
    //     camera_pose_matrix_pred.end<double>());

    // CHECK_CLOSE_COLLECTION(camera_matrix_gt_vec, camera_matrix_pred_vec,
    //                        1); // tolerance in percentage
    // CHECK_CLOSE_COLLECTION(distortion_coeffs_gt_vec,
    // distortion_coeffs_pred_vec,
    //                        1); // tolerance in percentage
    // CHECK_CLOSE_COLLECTION(camera_pose_matrix_gt_vec,
    //                        camera_pose_matrix_pred_vec,
    //                        1); // tolerance in percentage

    BOOST_REQUIRE(Calib.computeAvgReprojectionError() <
                  1.0); // reprojection error less than 1 pixel
  }
}

BOOST_AUTO_TEST_SUITE(CheckCalibration)

BOOST_AUTO_TEST_CASE(CheckCalibrationSyntheticScenario1) {
  std::string config_path = "../configs/calib_param_synth_Scenario1.yml";
  std::string gt_path =
      "../tests/calibration_gts/synth_Scenario1_calibrated_cameras_data.yml";
  calibrateAndCheckGt(config_path, gt_path);
}

// BOOST_AUTO_TEST_CASE(CheckCalibrationSyntheticScenario3) {
//   std::string config_path = "../configs/calib_param_synth_Scenario3.yml";
//   std::string gt_path =
//       "../tests/calibration_gts/synth_Scenario3_calibrated_cameras_data.yml";
//   calibrateAndCheckGt(config_path, gt_path);
// }

// TODO: need more tests

BOOST_AUTO_TEST_SUITE_END()