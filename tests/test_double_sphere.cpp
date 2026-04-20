#include <boost/test/unit_test.hpp>
#include <cmath>

#include <Camera.hpp>
#include <geometrytools.hpp>

BOOST_AUTO_TEST_SUITE(CheckDoubleSphere)

// DS intrinsics: fx, fy, cx, cy, xi, alpha
static const double DS_FX = 711.574;
static const double DS_FY = 711.237;
static const double DS_CX = 960.0;
static const double DS_CY = 540.0;
static const double DS_XI = 0.183;
static const double DS_ALPHA = 0.809;

// Helper: create a Camera with DS intrinsics
static std::shared_ptr<McCalib::Camera> makeDSCamera() {
  auto cam = std::make_shared<McCalib::Camera>(0, 2);
  cam->im_cols_ = 1920;
  cam->im_rows_ = 1080;
  cam->intrinsics_[0] = DS_FX;
  cam->intrinsics_[1] = DS_FY;
  cam->intrinsics_[2] = DS_CX;
  cam->intrinsics_[3] = DS_CY;
  cam->intrinsics_[4] = DS_XI;
  cam->intrinsics_[5] = DS_ALPHA;
  return cam;
}

// Helper: build camera matrix and distortion vector from Camera object
static void getCameraMatAndDist(const std::shared_ptr<McCalib::Camera> &cam,
                                cv::Mat &cam_mat, cv::Mat &dist_vec) {
  cam_mat =
      (cv::Mat_<double>(3, 3) << cam->intrinsics_[0], 0, cam->intrinsics_[2], 0,
       cam->intrinsics_[1], cam->intrinsics_[3], 0, 0, 1);
  dist_vec =
      (cv::Mat_<double>(1, 2) << cam->intrinsics_[4], cam->intrinsics_[5]);
}

// ============================================================
// Test 1: Distortion storage round-trip
// ============================================================
BOOST_AUTO_TEST_CASE(DSDistortionStorageRoundTrip) {
  McCalib::Camera cam(0, 2);
  cv::Mat dist = (cv::Mat_<double>(1, 2) << 0.183, 0.809);
  cam.setDistortionVector(dist);
  cv::Mat dist_out = cam.getDistortionVectorVector();
  BOOST_CHECK_CLOSE(dist_out.at<double>(0, 0), 0.183, 1e-6);
  BOOST_CHECK_CLOSE(dist_out.at<double>(0, 1), 0.809, 1e-6);
}

// ============================================================
// Test 2: Unused intrinsics slots are zero
// ============================================================
BOOST_AUTO_TEST_CASE(DSIntrinsicsZeroInit) {
  McCalib::Camera cam(0, 2);
  BOOST_CHECK_EQUAL(cam.intrinsics_[6], 0.0);
  BOOST_CHECK_EQUAL(cam.intrinsics_[7], 0.0);
  BOOST_CHECK_EQUAL(cam.intrinsics_[8], 0.0);
}

// ============================================================
// Test 3: Projection/Unprojection round-trip for multiple points
// ============================================================
BOOST_AUTO_TEST_CASE(DSProjectUnprojectRoundTrip) {
  auto cam = makeDSCamera();
  cv::Mat cam_mat, dist_vec;
  getCameraMatAndDist(cam, cam_mat, dist_vec);

  std::vector<cv::Point3f> pts_3d;
  pts_3d.emplace_back(0.0f, 0.0f, 1.0f);
  pts_3d.emplace_back(0.1f, -0.05f, 0.8f);
  pts_3d.emplace_back(-0.2f, 0.15f, 1.2f);
  pts_3d.emplace_back(0.3f, 0.2f, 0.6f);

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  std::vector<cv::Point2f> pts_2d;
  McCalib::projectPointsWithDistortion(pts_3d, rvec, tvec, cam_mat, dist_vec, 2,
                                       pts_2d);

  std::vector<cv::Point3f> rays;
  cam->dsUnproject(pts_2d, rays);

  for (size_t i = 0; i < pts_3d.size(); i++) {
    float norm =
        std::sqrt(pts_3d[i].x * pts_3d[i].x + pts_3d[i].y * pts_3d[i].y +
                  pts_3d[i].z * pts_3d[i].z);
    float expected_x = pts_3d[i].x / norm;
    float expected_y = pts_3d[i].y / norm;
    float expected_z = pts_3d[i].z / norm;

    BOOST_CHECK_CLOSE(rays[i].x, expected_x, 0.1);
    BOOST_CHECK_CLOSE(rays[i].y, expected_y, 0.1);
    BOOST_CHECK_CLOSE(rays[i].z, expected_z, 0.1);
  }
}

// ============================================================
// Test 4: Center point projects to (cx, cy)
// ============================================================
BOOST_AUTO_TEST_CASE(DSProjectionCenterPoint) {
  auto cam = makeDSCamera();
  cv::Mat cam_mat, dist_vec;
  getCameraMatAndDist(cam, cam_mat, dist_vec);

  std::vector<cv::Point3f> pts_3d = {cv::Point3f(0.0f, 0.0f, 1.0f)};
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  std::vector<cv::Point2f> pts_2d;
  McCalib::projectPointsWithDistortion(pts_3d, rvec, tvec, cam_mat, dist_vec, 2,
                                       pts_2d);

  BOOST_CHECK_CLOSE(pts_2d[0].x, DS_CX, 0.01);
  BOOST_CHECK_CLOSE(pts_2d[0].y, DS_CY, 0.01);
}

// ============================================================
// Test 5: Unprojecting (cx, cy) gives optical axis [0, 0, 1]
// ============================================================
BOOST_AUTO_TEST_CASE(DSUnprojectCenterPoint) {
  auto cam = makeDSCamera();

  std::vector<cv::Point2f> pts_2d = {
      cv::Point2f(static_cast<float>(DS_CX), static_cast<float>(DS_CY))};
  std::vector<cv::Point3f> rays;
  cam->dsUnproject(pts_2d, rays);

  BOOST_CHECK_SMALL(double(rays[0].x), 1e-6);
  BOOST_CHECK_SMALL(double(rays[0].y), 1e-6);
  BOOST_CHECK_CLOSE(double(rays[0].z), 1.0, 0.01);
}

// ============================================================
// Test 6: DS projection with non-identity pose
// ============================================================
BOOST_AUTO_TEST_CASE(DSProjectionWithPose) {
  auto cam = makeDSCamera();
  cv::Mat cam_mat, dist_vec;
  getCameraMatAndDist(cam, cam_mat, dist_vec);

  // Board-like pattern
  std::vector<cv::Point3f> pts_3d;
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 6; c++) {
      pts_3d.emplace_back(c * 0.055f, r * 0.055f, 0.0f);
    }
  }

  // Pose: slight rotation + translation
  cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.1, -0.2, 0.15);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.05, -0.03, 0.8);

  std::vector<cv::Point2f> pts_2d;
  McCalib::projectPointsWithDistortion(pts_3d, rvec, tvec, cam_mat, dist_vec, 2,
                                       pts_2d);

  // All projected points should be within image bounds
  for (const auto &pt : pts_2d) {
    BOOST_CHECK_GE(pt.x, 0.0f);
    BOOST_CHECK_LE(pt.x, 1920.0f);
    BOOST_CHECK_GE(pt.y, 0.0f);
    BOOST_CHECK_LE(pt.y, 1080.0f);
  }

  // Unproject and verify round-trip
  std::vector<cv::Point3f> rays;
  cam->dsUnproject(pts_2d, rays);
  BOOST_CHECK_EQUAL(rays.size(), pts_2d.size());

  // Transform original points to camera frame for direction comparison
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  for (size_t i = 0; i < pts_3d.size(); i++) {
    double X = R.at<double>(0, 0) * pts_3d[i].x +
               R.at<double>(0, 1) * pts_3d[i].y +
               R.at<double>(0, 2) * pts_3d[i].z + tvec.at<double>(0);
    double Y = R.at<double>(1, 0) * pts_3d[i].x +
               R.at<double>(1, 1) * pts_3d[i].y +
               R.at<double>(1, 2) * pts_3d[i].z + tvec.at<double>(1);
    double Z = R.at<double>(2, 0) * pts_3d[i].x +
               R.at<double>(2, 1) * pts_3d[i].y +
               R.at<double>(2, 2) * pts_3d[i].z + tvec.at<double>(2);
    double norm = std::sqrt(X * X + Y * Y + Z * Z);

    BOOST_CHECK_CLOSE(double(rays[i].x), X / norm, 0.5);
    BOOST_CHECK_CLOSE(double(rays[i].y), Y / norm, 0.5);
    BOOST_CHECK_CLOSE(double(rays[i].z), Z / norm, 0.5);
  }
}

// ============================================================
// Test 7: Wide-angle points (edge of fisheye)
// ============================================================
BOOST_AUTO_TEST_CASE(DSWideAngleProjection) {
  auto cam = makeDSCamera();
  cv::Mat cam_mat, dist_vec;
  getCameraMatAndDist(cam, cam_mat, dist_vec);

  // Points at various angles from optical axis
  std::vector<cv::Point3f> pts_3d;
  pts_3d.emplace_back(0.0f, 0.0f, 1.0f);   // on-axis
  pts_3d.emplace_back(0.5f, 0.0f, 1.0f);   // ~26.5 deg off-axis
  pts_3d.emplace_back(0.0f, 0.5f, 1.0f);   // ~26.5 deg off-axis
  pts_3d.emplace_back(1.0f, 0.0f, 1.0f);   // ~45 deg off-axis
  pts_3d.emplace_back(-1.0f, -1.0f, 1.0f); // ~54.7 deg off-axis

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  std::vector<cv::Point2f> pts_2d;
  McCalib::projectPointsWithDistortion(pts_3d, rvec, tvec, cam_mat, dist_vec, 2,
                                       pts_2d);

  // On-axis point should be at principal point
  BOOST_CHECK_CLOSE(pts_2d[0].x, DS_CX, 0.01);
  BOOST_CHECK_CLOSE(pts_2d[0].y, DS_CY, 0.01);

  // Points right of center should have u > cx
  BOOST_CHECK_GT(pts_2d[1].x, DS_CX);
  // Points below center should have v > cy
  BOOST_CHECK_GT(pts_2d[2].y, DS_CY);

  // Symmetric points should be equidistant from center
  float dist_right = pts_2d[1].x - static_cast<float>(DS_CX);
  float dist_down = pts_2d[2].y - static_cast<float>(DS_CY);
  // fx ~= fy, so horizontal and vertical offsets should be similar
  BOOST_CHECK_CLOSE(dist_right, dist_down, 1.0);

  // Round-trip all points
  std::vector<cv::Point3f> rays;
  cam->dsUnproject(pts_2d, rays);
  for (size_t i = 0; i < pts_3d.size(); i++) {
    float norm =
        std::sqrt(pts_3d[i].x * pts_3d[i].x + pts_3d[i].y * pts_3d[i].y +
                  pts_3d[i].z * pts_3d[i].z);
    BOOST_CHECK_CLOSE(double(rays[i].x), double(pts_3d[i].x / norm), 0.5);
    BOOST_CHECK_CLOSE(double(rays[i].y), double(pts_3d[i].y / norm), 0.5);
    BOOST_CHECK_CLOSE(double(rays[i].z), double(pts_3d[i].z / norm), 0.5);
  }
}

// ============================================================
// Test 8: DS model vs Brown model on same point (sanity check)
// ============================================================
BOOST_AUTO_TEST_CASE(DSvsBrownDifferentProjection) {
  auto ds_cam = makeDSCamera();
  cv::Mat ds_cam_mat, ds_dist;
  getCameraMatAndDist(ds_cam, ds_cam_mat, ds_dist);

  // Brown camera with same focal length but no distortion
  cv::Mat brown_cam_mat =
      (cv::Mat_<double>(3, 3) << DS_FX, 0, DS_CX, 0, DS_FY, DS_CY, 0, 0, 1);
  cv::Mat brown_dist = cv::Mat::zeros(1, 5, CV_64F);

  // Off-axis point
  std::vector<cv::Point3f> pts_3d = {cv::Point3f(0.5f, 0.3f, 1.0f)};
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

  std::vector<cv::Point2f> ds_pts, brown_pts;
  McCalib::projectPointsWithDistortion(pts_3d, rvec, tvec, ds_cam_mat, ds_dist,
                                       2, ds_pts);
  McCalib::projectPointsWithDistortion(pts_3d, rvec, tvec, brown_cam_mat,
                                       brown_dist, 0, brown_pts);

  // DS and Brown should give DIFFERENT results (DS has additional distortion)
  double diff = std::sqrt(std::pow(ds_pts[0].x - brown_pts[0].x, 2) +
                          std::pow(ds_pts[0].y - brown_pts[0].y, 2));
  BOOST_CHECK_GT(diff, 1.0); // Should differ by more than 1 pixel
}

// ============================================================
// Test 9: DS distortion model type preserved in Camera
// ============================================================
BOOST_AUTO_TEST_CASE(DSDistortionModelType) {
  McCalib::Camera cam_brown(0, 0);
  McCalib::Camera cam_kannala(1, 1);
  McCalib::Camera cam_ds(2, 2);

  BOOST_CHECK_EQUAL(cam_brown.distortion_model_, 0);
  BOOST_CHECK_EQUAL(cam_kannala.distortion_model_, 1);
  BOOST_CHECK_EQUAL(cam_ds.distortion_model_, 2);
}

// ============================================================
// Test 10: Batch unprojection produces correct number of rays
// ============================================================
BOOST_AUTO_TEST_CASE(DSUnprojectBatchSize) {
  auto cam = makeDSCamera();

  // Generate a grid of 2D points
  std::vector<cv::Point2f> pts_2d;
  for (int r = 0; r < 10; r++) {
    for (int c = 0; c < 10; c++) {
      pts_2d.emplace_back(400.0f + c * 100.0f, 200.0f + r * 60.0f);
    }
  }

  std::vector<cv::Point3f> rays;
  cam->dsUnproject(pts_2d, rays);

  BOOST_CHECK_EQUAL(rays.size(), pts_2d.size());

  // All rays should be unit vectors
  for (const auto &ray : rays) {
    double norm = std::sqrt(ray.x * ray.x + ray.y * ray.y + ray.z * ray.z);
    BOOST_CHECK_CLOSE(norm, 1.0, 0.1);
  }
}

// ============================================================
// Test 11: Extreme DS parameters (boundary values)
// ============================================================
BOOST_AUTO_TEST_CASE(DSExtremParameterBounds) {
  // Camera with xi=0, alpha=0 should behave like standard perspective
  auto cam = std::make_shared<McCalib::Camera>(0, 2);
  cam->im_cols_ = 1920;
  cam->im_rows_ = 1080;
  cam->intrinsics_[0] = 500.0;
  cam->intrinsics_[1] = 500.0;
  cam->intrinsics_[2] = 960.0;
  cam->intrinsics_[3] = 540.0;
  cam->intrinsics_[4] = 0.0; // xi = 0
  cam->intrinsics_[5] = 0.0; // alpha = 0

  cv::Mat cam_mat, dist_vec;
  getCameraMatAndDist(cam, cam_mat, dist_vec);

  // With xi=0, alpha=0, DS reduces to standard pinhole:
  // d1 = ||P||, z1 = Z + 0 = Z, d2 = ||P||, den = 0 + 1*Z = Z
  // u = fx * X/Z + cx (standard pinhole)
  std::vector<cv::Point3f> pts_3d = {cv::Point3f(0.1f, -0.05f, 1.0f)};
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

  std::vector<cv::Point2f> ds_pts;
  McCalib::projectPointsWithDistortion(pts_3d, rvec, tvec, cam_mat, dist_vec, 2,
                                       ds_pts);

  // Expected pinhole projection: u = fx*X/Z + cx, v = fy*Y/Z + cy
  double expected_u = 500.0 * 0.1 / 1.0 + 960.0;     // 1010
  double expected_v = 500.0 * (-0.05) / 1.0 + 540.0; // 515

  BOOST_CHECK_CLOSE(ds_pts[0].x, expected_u, 0.01);
  BOOST_CHECK_CLOSE(ds_pts[0].y, expected_v, 0.01);
}

BOOST_AUTO_TEST_SUITE_END()
