#include <boost/test/unit_test.hpp>

#include <geometrytools.hpp>

BOOST_AUTO_TEST_SUITE(CheckGeometryTools)

double INTRINSICS_TOLERANCE = 1.0; // in percentage

BOOST_AUTO_TEST_CASE(CheckRotationMatrixToQuaternionConversion1) {
  std::array<double, 9> rot_matrix_data = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const cv::Mat rot_matrix = cv::Mat(3, 3, CV_64F, rot_matrix_data.data());
  const cv::Mat quaternion_pred = McCalib::convertRotationMatrixToQuaternion(rot_matrix);

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
  const cv::Mat quaternion_pred = McCalib::convertRotationMatrixToQuaternion(rot_matrix);

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

  cv::Mat rot_matrix_pred = McCalib::convertQuaternionToRotationMatrix(quaternion);

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

BOOST_AUTO_TEST_SUITE_END()