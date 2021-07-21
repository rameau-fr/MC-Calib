#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "opencv2/opencv.hpp"

#include <../src/geometrytools.hpp>

BOOST_AUTO_TEST_SUITE(CheckGeometryTools)

BOOST_AUTO_TEST_CASE(ProjToVecAllZeros) {
  cv::Mat proj_matrix = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                         1, 0, 0, 0, 0, 1);
  std::vector<float> output = ProjToVec(proj_matrix);

  std::vector<float> answer = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  BOOST_REQUIRE_EQUAL_COLLECTIONS(output.begin(), output.end(), answer.begin(),
                                  answer.end());
}

BOOST_AUTO_TEST_SUITE_END()