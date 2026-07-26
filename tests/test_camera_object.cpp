#include <boost/test/unit_test.hpp>

#include <BoardObs.hpp>
#include <Camera.hpp>
#include <Frame.hpp>
#include <Object3D.hpp>
#include <Object3DObs.hpp>
#include <geometrytools.hpp>

BOOST_AUTO_TEST_SUITE(CheckCameraAndObject)

BOOST_AUTO_TEST_CASE(CheckCameraIntrinsicsPerspectiveModelRoundTrip) {
  McCalib::Camera camera(0, 0);

  cv::Mat camera_matrix(3, 3, CV_64F);
  camera_matrix.at<double>(0, 0) = 800.0;
  camera_matrix.at<double>(0, 1) = 0.0;
  camera_matrix.at<double>(0, 2) = 320.0;
  camera_matrix.at<double>(1, 0) = 0.0;
  camera_matrix.at<double>(1, 1) = 810.0;
  camera_matrix.at<double>(1, 2) = 240.0;
  camera_matrix.at<double>(2, 0) = 0.0;
  camera_matrix.at<double>(2, 1) = 0.0;
  camera_matrix.at<double>(2, 2) = 1.0;

  cv::Mat distortion_vector(1, 5, CV_64F);
  distortion_vector.at<double>(0) = 0.1;
  distortion_vector.at<double>(1) = -0.2;
  distortion_vector.at<double>(2) = 0.01;
  distortion_vector.at<double>(3) = 0.02;
  distortion_vector.at<double>(4) = 0.03;

  camera.setIntrinsics(camera_matrix, distortion_vector);

  cv::Mat out_k;
  cv::Mat out_dist;
  camera.getIntrinsics(out_k, out_dist);

  BOOST_CHECK_SMALL(std::abs(out_k.at<double>(0, 0) - 800.0), 1e-12);
  BOOST_CHECK_SMALL(std::abs(out_k.at<double>(1, 1) - 810.0), 1e-12);
  BOOST_CHECK_SMALL(std::abs(out_k.at<double>(0, 2) - 320.0), 1e-12);
  BOOST_CHECK_SMALL(std::abs(out_k.at<double>(1, 2) - 240.0), 1e-12);

  BOOST_REQUIRE_EQUAL(out_dist.rows, 1);
  BOOST_REQUIRE_EQUAL(out_dist.cols, 5);
  BOOST_CHECK_SMALL(std::abs(out_dist.at<double>(0) - 0.1), 1e-12);
  BOOST_CHECK_SMALL(std::abs(out_dist.at<double>(1) + 0.2), 1e-12);
  BOOST_CHECK_SMALL(std::abs(out_dist.at<double>(2) - 0.01), 1e-12);
  BOOST_CHECK_SMALL(std::abs(out_dist.at<double>(3) - 0.02), 1e-12);
  BOOST_CHECK_SMALL(std::abs(out_dist.at<double>(4) - 0.03), 1e-12);
}

BOOST_AUTO_TEST_CASE(CheckCameraDistortionVectorFisheyeBranch) {
  McCalib::Camera camera(1, 1);

  cv::Mat distortion_vector(1, 4, CV_64F);
  distortion_vector.at<double>(0) = 0.01;
  distortion_vector.at<double>(1) = -0.02;
  distortion_vector.at<double>(2) = 0.03;
  distortion_vector.at<double>(3) = -0.04;
  camera.setDistortionVector(distortion_vector);

  const cv::Mat out_dist = camera.getDistortionVectorVector();
  BOOST_REQUIRE_EQUAL(out_dist.rows, 1);
  BOOST_REQUIRE_EQUAL(out_dist.cols, 4);
  BOOST_CHECK_SMALL(std::abs(out_dist.at<double>(0) - 0.01), 1e-12);
  BOOST_CHECK_SMALL(std::abs(out_dist.at<double>(1) + 0.02), 1e-12);
  BOOST_CHECK_SMALL(std::abs(out_dist.at<double>(2) - 0.03), 1e-12);
  BOOST_CHECK_SMALL(std::abs(out_dist.at<double>(3) + 0.04), 1e-12);
}

BOOST_AUTO_TEST_CASE(CheckCameraDistortionVectorUnsupportedBranch) {
  McCalib::Camera camera(2, 99);
  const cv::Mat out_dist = camera.getDistortionVectorVector();

  BOOST_REQUIRE_EQUAL(out_dist.rows, 1);
  BOOST_REQUIRE_EQUAL(out_dist.cols, 5);
  for (int i = 0; i < out_dist.cols; ++i) {
    BOOST_CHECK_SMALL(std::abs(out_dist.at<double>(i)), 1e-12);
  }
}

BOOST_AUTO_TEST_CASE(CheckObject3DBoardPoseSetGetViaMatrix) {
  const std::array<int, 3> color = {255, 0, 0};
  McCalib::Object3D object_3d(2, 0, 1, color);

  cv::Mat r_vec(3, 1, CV_64F);
  r_vec.at<double>(0) = 0.2;
  r_vec.at<double>(1) = -0.1;
  r_vec.at<double>(2) = 0.05;
  cv::Mat t_vec(3, 1, CV_64F);
  t_vec.at<double>(0) = 1.0;
  t_vec.at<double>(1) = 2.0;
  t_vec.at<double>(2) = 3.0;

  const cv::Mat pose = McCalib::RVecT2Proj(r_vec, t_vec);
  object_3d.setBoardPoseMat(7, pose);

  const cv::Mat pose_out = object_3d.getBoardPoseMat(7);
  BOOST_REQUIRE_EQUAL(pose_out.rows, 4);
  BOOST_REQUIRE_EQUAL(pose_out.cols, 4);
  for (int i = 0; i < pose.rows; ++i) {
    for (int j = 0; j < pose.cols; ++j) {
      BOOST_CHECK_SMALL(
          std::abs(pose.at<double>(i, j) - pose_out.at<double>(i, j)), 1e-9);
    }
  }
}

BOOST_AUTO_TEST_CASE(CheckObject3DBoardPoseSetGetViaVectors) {
  const std::array<int, 3> color = {0, 255, 0};
  McCalib::Object3D object_3d(3, 1, 2, color);

  cv::Mat r_vec(3, 1, CV_64F);
  r_vec.at<double>(0) = -0.3;
  r_vec.at<double>(1) = 0.25;
  r_vec.at<double>(2) = 0.1;
  cv::Mat t_vec(3, 1, CV_64F);
  t_vec.at<double>(0) = -1.5;
  t_vec.at<double>(1) = 0.25;
  t_vec.at<double>(2) = 4.0;

  object_3d.setBoardPoseVec(3, r_vec, t_vec);

  cv::Mat r_out;
  cv::Mat t_out;
  object_3d.getBoardPoseVec(3, r_out, t_out);

  for (int i = 0; i < 3; ++i) {
    BOOST_CHECK_SMALL(std::abs(r_vec.at<double>(i) - r_out.at<double>(i)),
                      1e-12);
    BOOST_CHECK_SMALL(std::abs(t_vec.at<double>(i) - t_out.at<double>(i)),
                      1e-12);
  }
}

BOOST_AUTO_TEST_CASE(CheckObject3DInsertFrame) {
  const std::array<int, 3> color = {0, 0, 255};
  McCalib::Object3D object_3d(1, 0, 10, color);

  const std::shared_ptr<McCalib::Frame> frame =
      std::make_shared<McCalib::Frame>(12, 4, "cam_004/frame_0012.png");

  object_3d.insertNewFrame(frame);

  BOOST_REQUIRE_EQUAL(object_3d.frames_.size(), 1);
  BOOST_REQUIRE(object_3d.frames_.count(12) == 1);
}

BOOST_AUTO_TEST_CASE(CheckCameraBorderToleranceFisheyeValidPoints) {
  const std::shared_ptr<McCalib::Camera> camera =
      std::make_shared<McCalib::Camera>(0, 1);
  camera->im_cols_ = 1000;
  camera->im_rows_ = 500;

  const std::vector<cv::Point2f> pts_2d = {{100.0f, 100.0f}, {900.0f, 400.0f}};
  const std::vector<int> charuco_id = {0, 1};
  const std::shared_ptr<McCalib::Board> board_3d = nullptr;
  const std::shared_ptr<McCalib::BoardObs> board_obs =
      std::make_shared<McCalib::BoardObs>(0, 1, 2, pts_2d, charuco_id, camera,
                                          board_3d);

  BOOST_CHECK_EQUAL(camera->checkBorderToleranceFisheye(board_obs), true);
}

BOOST_AUTO_TEST_CASE(CheckCameraBorderToleranceFisheyeInvalidAtBorder) {
  const std::shared_ptr<McCalib::Camera> camera =
      std::make_shared<McCalib::Camera>(0, 1);
  camera->im_cols_ = 1000;
  camera->im_rows_ = 500;

  // Border threshold is 5%: x=50 and y=25 are considered invalid.
  const std::vector<cv::Point2f> pts_2d = {{50.0f, 100.0f}, {400.0f, 25.0f}};
  const std::vector<int> charuco_id = {0, 1};
  const std::shared_ptr<McCalib::Board> board_3d = nullptr;
  const std::shared_ptr<McCalib::BoardObs> board_obs =
      std::make_shared<McCalib::BoardObs>(0, 1, 2, pts_2d, charuco_id, camera,
                                          board_3d);

  BOOST_CHECK_EQUAL(camera->checkBorderToleranceFisheye(board_obs), false);
}

BOOST_AUTO_TEST_CASE(CheckBoardObsReprojectionErrorWithoutCamera) {
  const std::shared_ptr<McCalib::Camera> camera = nullptr;
  const std::shared_ptr<McCalib::Board> board_3d = nullptr;
  const std::vector<cv::Point2f> pts_2d = {{100.0f, 120.0f}, {140.0f, 160.0f}};
  const std::vector<int> charuco_id = {0, 1};

  McCalib::BoardObs board_obs(0, 1, 2, pts_2d, charuco_id, camera, board_3d);
  board_obs.pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  BOOST_CHECK_SMALL(
      static_cast<double>(std::abs(board_obs.computeReprojectionError())),
      1e-12);
}

BOOST_AUTO_TEST_CASE(CheckObject3DObsInsertBoardObsWithNullObject) {
  const std::shared_ptr<McCalib::Object3D> null_object = nullptr;
  McCalib::Object3DObs object_obs(null_object, 12);

  const std::shared_ptr<McCalib::Camera> camera =
      std::make_shared<McCalib::Camera>(3, 0);
  const std::shared_ptr<McCalib::Board> board_3d = nullptr;
  const std::vector<cv::Point2f> pts_2d = {{15.0f, 25.0f}, {35.0f, 45.0f}};
  const std::vector<int> charuco_id = {1, 2};
  const std::shared_ptr<McCalib::BoardObs> board_obs =
      std::make_shared<McCalib::BoardObs>(3, 7, 4, pts_2d, charuco_id, camera,
                                          board_3d);

  object_obs.insertNewBoardObs(board_obs);

  BOOST_REQUIRE_EQUAL(object_obs.board_id_.size(), 1);
  BOOST_CHECK_EQUAL(object_obs.board_id_[0], 4);
  BOOST_CHECK_EQUAL(object_obs.camera_id_, 3);
  BOOST_CHECK_EQUAL(object_obs.frame_id_, 7);
  BOOST_CHECK_EQUAL(object_obs.pts_2d_.size(), 0);
  BOOST_CHECK_EQUAL(object_obs.pts_id_.size(), 0);
}

BOOST_AUTO_TEST_CASE(CheckObject3DObsPoseInGroupSetGetRoundTrip) {
  const std::array<int, 3> color = {128, 64, 255};
  const std::shared_ptr<McCalib::Object3D> object_3d =
      std::make_shared<McCalib::Object3D>(1, 0, 3, color);
  McCalib::Object3DObs object_obs(object_3d, 3);

  cv::Mat r_vec(3, 1, CV_64F);
  r_vec.at<double>(0) = 0.12;
  r_vec.at<double>(1) = -0.08;
  r_vec.at<double>(2) = 0.04;
  cv::Mat t_vec(3, 1, CV_64F);
  t_vec.at<double>(0) = 0.7;
  t_vec.at<double>(1) = -1.2;
  t_vec.at<double>(2) = 3.4;

  object_obs.setPoseInGroupVec(r_vec, t_vec);

  cv::Mat r_out;
  cv::Mat t_out;
  object_obs.getPoseInGroupVec(r_out, t_out);
  for (int i = 0; i < 3; ++i) {
    BOOST_CHECK_SMALL(std::abs(r_vec.at<double>(i) - r_out.at<double>(i)),
                      1e-12);
    BOOST_CHECK_SMALL(std::abs(t_vec.at<double>(i) - t_out.at<double>(i)),
                      1e-12);
  }

  const cv::Mat pose_group = object_obs.getPoseInGroupMat();
  BOOST_REQUIRE_EQUAL(pose_group.rows, 4);
  BOOST_REQUIRE_EQUAL(pose_group.cols, 4);
}

BOOST_AUTO_TEST_CASE(CheckObject3DObsReprojectionErrorWithoutCamera) {
  const std::shared_ptr<McCalib::Object3D> object_3d = nullptr;
  McCalib::Object3DObs object_obs(object_3d, 5);

  object_obs.pts_2d_ = {{10.0f, 10.0f}};
  object_obs.pts_id_ = {0};
  object_obs.pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  BOOST_CHECK_SMALL(
      static_cast<double>(std::abs(object_obs.computeReprojectionError())),
      1e-12);
}

BOOST_AUTO_TEST_SUITE_END()
