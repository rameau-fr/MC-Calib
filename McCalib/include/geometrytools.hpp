#pragma once

#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <stdio.h>

namespace McCalib {

cv::Mat RT2Proj(const cv::Mat &R, const cv::Mat &T);
cv::Mat RVecT2Proj(const cv::Mat &RVec, const cv::Mat &T);
cv::Mat RVecT2ProjInt(const cv::Mat &RVec, const cv::Mat &T, const cv::Mat &K);
void Proj2RT(const cv::Mat &Proj, cv::Mat &R, cv::Mat &T);
cv::Mat vectorProj(const std::vector<float> &ProjV);
std::array<float, 6> ProjToVec(const cv::Mat &Proj);
void invertRvecT(const cv::Mat &Rvec, const cv::Mat &T, cv::Mat &iR,
                 cv::Mat &iT);
void invertRvecT(cv::Mat &Rvec, cv::Mat &T);
cv::Point3f
triangulateNViewLinearEigen(const std::vector<cv::Point2f> &Pts2D,
                            const std::vector<cv::Mat> &RotationVec,
                            const std::vector<cv::Mat> &TranslationVec,
                            const cv::Mat &Intrinsic);
void calcLinePara(const std::vector<cv::Point2f> &pts, double &a, double &b,
                  double &c, double &res);
void ransacTriangulation(const std::vector<cv::Point2f> &point2d,
                         const std::vector<cv::Mat> &rotation_vec,
                         const std::vector<cv::Mat> &translation_Vec,
                         const cv::Mat &intrinsic,
                         const cv::Mat &disortion_vector, const double thresh,
                         const double p, const int it,
                         cv::Point3f &best_point3d);
cv::Mat ransacP3P(const std::vector<cv::Point3f> &scene_points,
                  const std::vector<cv::Point2f> &image_points,
                  const cv::Mat &intrinsic, const cv::Mat &distortion_vector,
                  cv::Mat &best_R, cv::Mat &best_T, const double thresh,
                  const int it, const double p = 0.99,
                  const bool refine = true);
std::vector<cv::Point3f> transform3DPts(const std::vector<cv::Point3f> &pts3D,
                                        const cv::Mat &rot,
                                        const cv::Mat &trans);
cv::Mat handeyeCalibration(const std::vector<cv::Mat> &pose_abs_1,
                           const std::vector<cv::Mat> &pose_abs_2);
cv::Mat
handeyeBootstraptTranslationCalibration(const unsigned int nb_cluster,
                                        const unsigned int nb_it,
                                        const std::vector<cv::Mat> &pose_abs_1,
                                        const std::vector<cv::Mat> &pose_abs_2);
double median(std::vector<double> &v);
cv::Mat ransacP3PDistortion(const std::vector<cv::Point3f> &scene_points,
                            const std::vector<cv::Point2f> &image_points,
                            const cv::Mat &intrinsic,
                            const cv::Mat &distortion_vector, cv::Mat &best_R,
                            cv::Mat &best_T, const float thresh, const int it,
                            const int distortion_type, const double p = 0.99,
                            const bool refine = true);
void projectPointsWithDistortion(const std::vector<cv::Point3f> &object_pts,
                                 const cv::Mat &rot, const cv::Mat &trans,
                                 const cv::Mat &camera_matrix,
                                 const cv::Mat &distortion_vector,
                                 const int distortion_type,
                                 std::vector<cv::Point2f> &repro_pts);
cv::Mat convertRotationMatrixToQuaternion(const cv::Mat &R);
cv::Mat convertQuaternionToRotationMatrix(const std::array<double, 4> &q);
cv::Mat getAverageRotation(std::vector<double> &r1, std::vector<double> &r2,
                           std::vector<double> &r3,
                           const bool use_quaternion_averaging = true);

} // namespace McCalib