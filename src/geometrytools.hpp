#pragma once

#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <stdio.h>

cv::Mat RT2Proj(cv::Mat R, cv::Mat T);
cv::Mat RVecT2Proj(cv::Mat RVec, cv::Mat T);
cv::Mat RVecT2ProjInt(cv::Mat RVec, cv::Mat T, cv::Mat K);
void Proj2RT(cv::Mat Proj, cv::Mat &R, cv::Mat &T);
cv::Mat vectorProj(std::vector<float> ProjV);
std::vector<float> ProjToVec(cv::Mat Proj);
void invertRvecT(cv::Mat Rvec, cv::Mat T, cv::Mat &iR, cv::Mat &iT);
void invertRvecT(cv::Mat &Rvec, cv::Mat &T);
cv::Point3f triangulateNViewLinearEigen(std::vector<cv::Point2f> Pts2D,
                                        std::vector<cv::Mat> RotationVec,
                                        std::vector<cv::Mat> TranslationVec,
                                        cv::Mat Intrinsic);
void calcLinePara(std::vector<cv::Point2f> pts, double &a, double &b, double &c,
                  double &res);
void ransacTriangulation(std::vector<cv::Point2f> point2d,
                         std::vector<cv::Mat> rotation_vec,
                         std::vector<cv::Mat> translation_Vec,
                         cv::Mat intrinsic, cv::Mat disortion_vector,
                         cv::Point3f &best_point3d, double thresh, double p,
                         int it);
cv::Mat ransacP3P(std::vector<cv::Point3f> scene_points,
                  std::vector<cv::Point2f> image_points, cv::Mat intrinsic,
                  cv::Mat distortion_vector, cv::Mat &best_R, cv::Mat &best_T,
                  double thresh, double p, int it, bool refine);
std::vector<cv::Point3f> transform3DPts(std::vector<cv::Point3f> pts3D,
                                        cv::Mat rot, cv::Mat trans);
cv::Mat handeyeCalibration(std::vector<cv::Mat> pose_abs_1,
                           std::vector<cv::Mat> pose_abs_2);
cv::Mat handeyeBootstratpTranslationCalibration(
    unsigned int nb_cluster, unsigned int nb_it,
    std::vector<cv::Mat> pose_abs_1, std::vector<cv::Mat> pose_abs_2);
double median(std::vector<double> &v);
cv::Mat ransacP3PDistortion(std::vector<cv::Point3f> scene_points,
                            std::vector<cv::Point2f> image_points,
                            cv::Mat intrinsic, cv::Mat distortion_vector,
                            cv::Mat &best_R, cv::Mat &best_T, const float thresh,
                            double p, int it, bool refine, int distortion_type);
void projectPointsWithDistortion(std::vector<cv::Point3f> object_pts,
                                 cv::Mat rot, cv::Mat trans,
                                 cv::Mat camera_matrix,
                                 cv::Mat distortion_vector,
                                 std::vector<cv::Point2f> &repro_pts,
                                 int distortion_type);
