#define BOOST_TEST_DYN_LINK

#include "opencv2/opencv.hpp"
#include <boost/test/unit_test.hpp>

#include <geometrytools.hpp>

BOOST_AUTO_TEST_SUITE(CheckGeometryTools)

BOOST_AUTO_TEST_CASE(ProjToVecAllZeros) {
  cv::Mat proj_matrix = cv::Mat::eye(4, 4, CV_64F);
  std::array<float, 6> output = McCalib::ProjToVec(proj_matrix);

  std::array<float, 6> answer = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  BOOST_REQUIRE_EQUAL_COLLECTIONS(output.begin(), output.end(), answer.begin(),
                                  answer.end());
}

BOOST_AUTO_TEST_SUITE_END()