#include <boost/test/unit_test.hpp>

#include <geometrytools.hpp>

BOOST_AUTO_TEST_SUITE(CheckGeometryTools)

double INTRINSICS_TOLERANCE = 1.0; // in percentage

BOOST_AUTO_TEST_CASE(CheckRotationMatrixToQuaternionConversion1) {
  std::array<double, 9> rot_matrix_data = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const cv::Mat rot_matrix = cv::Mat(3, 3, CV_64F, rot_matrix_data.data());
  const cv::Mat quaternion_pred =
      McCalib::convertRotationMatrixToQuaternion(rot_matrix);

  std::array<double, 4> quaternion_gt_data = {0, 0, 0, 1};
  const cv::Mat quaternion_gt =
      cv::Mat(1, 4, CV_64F, quaternion_gt_data.data());

  BOOST_REQUIRE_EQUAL(quaternion_pred.size[0], quaternion_gt.size[0]);
  BOOST_REQUIRE_EQUAL(quaternion_pred.size[1], quaternion_gt.size[1]);
  for (int i = 0; i < quaternion_pred.size[1]; ++i) {
    BOOST_CHECK_CLOSE(quaternion_pred.at<double>(i),
                      quaternion_gt.at<double>(i), INTRINSICS_TOLERANCE);
  }
}

BOOST_AUTO_TEST_CASE(CheckRotationMatrixToQuaternionConversion2) {
  std::array<double, 9> rot_matrix_data = {-0.9999, -0.1998, -0.3996,
                                           0.1998,  0.6000,  -0.8000,
                                           0.3996,  -0.8000, -0.5999};
  const cv::Mat rot_matrix = cv::Mat(3, 3, CV_64F, rot_matrix_data.data());
  const cv::Mat quaternion_pred =
      McCalib::convertRotationMatrixToQuaternion(rot_matrix);

  std::array<double, 4> quaternion_gt_data = {0, 0.8944, -0.4472,
                                              -0.2234}; // x y z w
  const cv::Mat quaternion_gt =
      cv::Mat(1, 4, CV_64F, quaternion_gt_data.data());

  BOOST_REQUIRE_EQUAL(quaternion_pred.size[0], quaternion_gt.size[0]);
  BOOST_REQUIRE_EQUAL(quaternion_pred.size[1], quaternion_gt.size[1]);
  for (int i = 0; i < quaternion_pred.size[1]; ++i) {
    BOOST_CHECK_CLOSE(quaternion_pred.at<double>(i),
                      quaternion_gt.at<double>(i), INTRINSICS_TOLERANCE);
  }
}

BOOST_AUTO_TEST_CASE(CheckQuaternionToRotationMatrixConversion) {
  std::array<double, 4> quaternion_data = {0.2809946, 0.8377387, -0.4188693,
                                           -0.2092473}; // x y z w
  const cv::Mat quaternion = cv::Mat(1, 4, CV_64F, quaternion_data.data());

  cv::Mat rot_matrix_pred =
      McCalib::convertQuaternionToRotationMatrix(quaternion);

  std::array<double, 9> rot_matrix_gt_data = {
      -0.7545152, 0.2955056, -0.5859892, 0.6460947, 0.4911810,
      -0.5842113, 0.1151891, -0.8194008, -0.5615281};
  const cv::Mat rot_matrix_gt =
      cv::Mat(3, 3, CV_64F, rot_matrix_gt_data.data());

  BOOST_REQUIRE_EQUAL(rot_matrix_pred.size[0], rot_matrix_gt.size[0]);
  BOOST_REQUIRE_EQUAL(rot_matrix_pred.size[1], rot_matrix_gt.size[1]);
  for (int i = 0; i < rot_matrix_pred.size[0]; ++i) {
    for (int j = 0; j < rot_matrix_pred.size[1]; ++j) {
      BOOST_CHECK_CLOSE(rot_matrix_pred.at<double>(i, j),
                        rot_matrix_gt.at<double>(i, j), INTRINSICS_TOLERANCE);
    }
  }
}

BOOST_AUTO_TEST_CASE(CheckRotationMatrixToQuaternionAndBackConversion) {
  for (int angle_x = -20; angle_x <= 20; ++angle_x) {
    for (int angle_y = -45; angle_y <= 90; ++angle_y) {
      for (int angle_z = -270; angle_z <= -180; ++angle_z) {
        std::array<double, 3> angles = {static_cast<double>(angle_x),
                                        static_cast<double>(angle_y),
                                        static_cast<double>(angle_z)};
        const cv::Mat rot_vec = cv::Mat(1, 3, CV_64F, angles.data());
        cv::Mat rot_matrix_before;
        cv::Rodrigues(rot_vec, rot_matrix_before);
        const cv::Mat quaternion =
            McCalib::convertRotationMatrixToQuaternion(rot_matrix_before);
        cv::Mat rot_matrix_after =
            McCalib::convertQuaternionToRotationMatrix(quaternion);

        BOOST_REQUIRE_EQUAL(rot_matrix_after.size[0],
                            rot_matrix_before.size[0]);
        BOOST_REQUIRE_EQUAL(rot_matrix_after.size[1],
                            rot_matrix_before.size[1]);
        for (int i = 0; i < rot_matrix_after.size[0]; ++i) {
          for (int j = 0; j < rot_matrix_after.size[1]; ++j) {
            BOOST_CHECK_CLOSE(rot_matrix_after.at<double>(i, j),
                              rot_matrix_before.at<double>(i, j),
                              INTRINSICS_TOLERANCE);
          }
        }
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(CheckRVecTProjAndProj2RTRoundTrip) {
  cv::Mat rvec(3, 1, CV_64F);
  rvec.at<double>(0) = 0.1;
  rvec.at<double>(1) = -0.2;
  rvec.at<double>(2) = 0.3;
  cv::Mat tvec(3, 1, CV_64F);
  tvec.at<double>(0) = 1.0;
  tvec.at<double>(1) = 2.0;
  tvec.at<double>(2) = 3.0;

  cv::Mat proj = McCalib::RVecT2Proj(rvec, tvec);

  cv::Mat recovered_rvec;
  cv::Mat recovered_tvec;
  McCalib::Proj2RT(proj, recovered_rvec, recovered_tvec);

  cv::Mat reproj = McCalib::RVecT2Proj(recovered_rvec, recovered_tvec);

  BOOST_REQUIRE_EQUAL(proj.rows, reproj.rows);
  BOOST_REQUIRE_EQUAL(proj.cols, reproj.cols);
  for (int i = 0; i < proj.rows; ++i) {
    for (int j = 0; j < proj.cols; ++j) {
      BOOST_CHECK_SMALL(std::abs(proj.at<double>(i, j) - reproj.at<double>(i, j)),
                        1e-9);
    }
  }
}

BOOST_AUTO_TEST_CASE(CheckInvertRvecTOverloadsAreConsistent) {
  cv::Mat rvec(3, 1, CV_64F);
  rvec.at<double>(0) = 0.2;
  rvec.at<double>(1) = 0.1;
  rvec.at<double>(2) = -0.15;
  cv::Mat tvec(3, 1, CV_64F);
  tvec.at<double>(0) = 0.5;
  tvec.at<double>(1) = -0.4;
  tvec.at<double>(2) = 2.0;

  cv::Mat inv_rvec_ref;
  cv::Mat inv_tvec_ref;
  McCalib::invertRvecT(rvec, tvec, inv_rvec_ref, inv_tvec_ref);

  cv::Mat inv_rvec_inplace = rvec.clone();
  cv::Mat inv_tvec_inplace = tvec.clone();
  McCalib::invertRvecT(inv_rvec_inplace, inv_tvec_inplace);

  for (int i = 0; i < 3; ++i) {
    BOOST_CHECK_SMALL(std::abs(inv_tvec_ref.at<double>(i) -
                               inv_tvec_inplace.at<double>(i)),
                      1e-9);
  }

  cv::Mat proj_ref = McCalib::RVecT2Proj(inv_rvec_ref, inv_tvec_ref);
  cv::Mat proj_inplace =
      McCalib::RVecT2Proj(inv_rvec_inplace, inv_tvec_inplace);
  for (int i = 0; i < proj_ref.rows; ++i) {
    for (int j = 0; j < proj_ref.cols; ++j) {
      BOOST_CHECK_SMALL(
          std::abs(proj_ref.at<double>(i, j) - proj_inplace.at<double>(i, j)),
          1e-9);
    }
  }
}

BOOST_AUTO_TEST_CASE(CheckTransform3DPtsIdentity) {
  std::vector<cv::Point3f> points = {{1.0f, 2.0f, 3.0f},
                                     {-4.0f, 0.5f, 2.25f}};
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

  std::vector<cv::Point3f> transformed =
      McCalib::transform3DPts(points, rvec, tvec);

  BOOST_REQUIRE_EQUAL(points.size(), transformed.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    BOOST_CHECK_SMALL(static_cast<double>(std::abs(points[i].x - transformed[i].x)), 1e-6);
    BOOST_CHECK_SMALL(static_cast<double>(std::abs(points[i].y - transformed[i].y)), 1e-6);
    BOOST_CHECK_SMALL(static_cast<double>(std::abs(points[i].z - transformed[i].z)), 1e-6);
  }
}

BOOST_AUTO_TEST_CASE(CheckMedianForEmptyOddAndEvenSizedVectors) {
  std::vector<double> empty;
  std::vector<double> odd = {3.0, 1.0, 2.0};
  std::vector<double> even = {10.0, 2.0, 6.0, 14.0};

  BOOST_CHECK_SMALL(std::abs(McCalib::median(empty) - 0.0), 1e-12);
  BOOST_CHECK_SMALL(std::abs(McCalib::median(odd) - 2.0), 1e-12);
  BOOST_CHECK_SMALL(std::abs(McCalib::median(even) - 8.0), 1e-12);
}

BOOST_AUTO_TEST_CASE(CheckGetAverageRotationMedianBranch) {
  std::vector<double> r1 = {0.1, 0.4, -0.2};
  std::vector<double> r2 = {1.0, 2.0, 3.0};
  std::vector<double> r3 = {-5.0, -2.0, -3.0};

  cv::Mat average_rotation =
      McCalib::getAverageRotation(r1, r2, r3, false);

  BOOST_REQUIRE_EQUAL(average_rotation.rows, 3);
  BOOST_REQUIRE_EQUAL(average_rotation.cols, 1);
  BOOST_CHECK_SMALL(std::abs(average_rotation.at<double>(0) - 0.1), 1e-12);
  BOOST_CHECK_SMALL(std::abs(average_rotation.at<double>(1) - 2.0), 1e-12);
  BOOST_CHECK_SMALL(std::abs(average_rotation.at<double>(2) - -3.0), 1e-12);
}

BOOST_AUTO_TEST_CASE(CheckProjectPointsWithDistortionPerspectiveBranch) {
  std::vector<cv::Point3f> object_pts = {{0.0f, 0.0f, 5.0f},
                                         {0.5f, -0.2f, 4.0f}};
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat camera_matrix(3, 3, CV_64F);
  camera_matrix.at<double>(0, 0) = 500.0;
  camera_matrix.at<double>(0, 1) = 0.0;
  camera_matrix.at<double>(0, 2) = 320.0;
  camera_matrix.at<double>(1, 0) = 0.0;
  camera_matrix.at<double>(1, 1) = 500.0;
  camera_matrix.at<double>(1, 2) = 240.0;
  camera_matrix.at<double>(2, 0) = 0.0;
  camera_matrix.at<double>(2, 1) = 0.0;
  camera_matrix.at<double>(2, 2) = 1.0;
  cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);

  std::vector<cv::Point2f> repro_custom;
  std::vector<cv::Point2f> repro_cv;

  McCalib::projectPointsWithDistortion(object_pts, rvec, tvec, camera_matrix,
                                       dist, 0, repro_custom);
  cv::projectPoints(object_pts, rvec, tvec, camera_matrix, dist, repro_cv);

  BOOST_REQUIRE_EQUAL(repro_custom.size(), repro_cv.size());
  for (std::size_t i = 0; i < repro_cv.size(); ++i) {
    BOOST_CHECK_SMALL(
        static_cast<double>(std::abs(repro_custom[i].x - repro_cv[i].x)),
        1e-6);
    BOOST_CHECK_SMALL(
        static_cast<double>(std::abs(repro_custom[i].y - repro_cv[i].y)),
        1e-6);
  }
}

BOOST_AUTO_TEST_CASE(CheckProjectPointsWithDistortionFisheyeBranch) {
  std::vector<cv::Point3f> object_pts = {{0.0f, 0.0f, 5.0f},
                                         {-0.2f, 0.3f, 3.5f}};
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat camera_matrix(3, 3, CV_64F);
  camera_matrix.at<double>(0, 0) = 450.0;
  camera_matrix.at<double>(0, 1) = 0.0;
  camera_matrix.at<double>(0, 2) = 320.0;
  camera_matrix.at<double>(1, 0) = 0.0;
  camera_matrix.at<double>(1, 1) = 450.0;
  camera_matrix.at<double>(1, 2) = 240.0;
  camera_matrix.at<double>(2, 0) = 0.0;
  camera_matrix.at<double>(2, 1) = 0.0;
  camera_matrix.at<double>(2, 2) = 1.0;
  cv::Mat dist = cv::Mat::zeros(1, 4, CV_64F);

  std::vector<cv::Point2f> repro_custom;
  std::vector<cv::Point2f> repro_cv;

  McCalib::projectPointsWithDistortion(object_pts, rvec, tvec, camera_matrix,
                                       dist, 1, repro_custom);
  cv::fisheye::projectPoints(object_pts, repro_cv, rvec, tvec, camera_matrix,
                             dist, 0.0);

  BOOST_REQUIRE_EQUAL(repro_custom.size(), repro_cv.size());
  for (std::size_t i = 0; i < repro_cv.size(); ++i) {
    BOOST_CHECK_SMALL(
        static_cast<double>(std::abs(repro_custom[i].x - repro_cv[i].x)),
        1e-6);
    BOOST_CHECK_SMALL(
        static_cast<double>(std::abs(repro_custom[i].y - repro_cv[i].y)),
        1e-6);
  }
}

BOOST_AUTO_TEST_SUITE_END()