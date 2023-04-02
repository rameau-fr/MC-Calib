#define BOOST_TEST_DYN_LINK

#include "opencv2/opencv.hpp"
#include <boost/test/unit_test.hpp>

#include <geometrytools.hpp>

BOOST_AUTO_TEST_SUITE(CheckGeometryTools)

BOOST_AUTO_TEST_CASE(ProjToVecAllZeros) {
  cv::Mat proj_matrix = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                         1, 0, 0, 0, 0, 1);
  std::array<float, 6> output = ProjToVec(proj_matrix);

  std::array<float, 6> answer = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  BOOST_REQUIRE_EQUAL_COLLECTIONS(output.begin(), output.end(), answer.begin(),
                                  answer.end());
}

BOOST_AUTO_TEST_SUITE_END()