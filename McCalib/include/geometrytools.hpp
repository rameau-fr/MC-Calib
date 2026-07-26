#pragma once

#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <stdio.h>

namespace McCalib {

/** @brief Build a 4x4 projection matrix from rotation matrix and translation.
 */
cv::Mat RT2Proj(const cv::Mat &R, const cv::Mat &T);

/** @brief Build a 4x4 projection matrix from Rodrigues rotation and
 * translation. */
cv::Mat RVecT2Proj(const cv::Mat &RVec, const cv::Mat &T);

/** @brief Build a 4x4 transform whose top block contains K*R and K*T. */
cv::Mat RVecT2ProjInt(const cv::Mat &RVec, const cv::Mat &T, const cv::Mat &K);

/** @brief Decompose a projection matrix into Rodrigues rotation and
 * translation. */
void Proj2RT(const cv::Mat &Proj, cv::Mat &R, cv::Mat &T);

/** @brief Convert a 6-value pose vector [rvec, tvec] into a 4x4 transform. */
cv::Mat vectorProj(const std::vector<float> &ProjV);

/** @brief Convert a projection matrix into a 6-value pose vector [rvec, tvec].
 */
std::array<float, 6> ProjToVec(const cv::Mat &Proj);

/** @brief Invert a pose represented by Rodrigues rotation and translation. */
void invertRvecT(const cv::Mat &Rvec, const cv::Mat &T, cv::Mat &iR,
                 cv::Mat &iT);

/** @brief In-place inversion of a Rodrigues+translation pose. */
void invertRvecT(cv::Mat &Rvec, cv::Mat &T);

/** @brief Triangulate one 3D point from N views using a linear least-squares
 * solver. */
cv::Point3f
triangulateNViewLinearEigen(const std::vector<cv::Point2f> &Pts2D,
                            const std::vector<cv::Mat> &RotationVec,
                            const std::vector<cv::Mat> &TranslationVec,
                            const cv::Mat &Intrinsic);

/** @brief Fit line parameters ax + by + c = 0 to 2D points and return residual.
 */
void calcLinePara(const std::vector<cv::Point2f> &pts, double &a, double &b,
                  double &c, double &res);

/** @brief Robustly triangulate a 3D point with RANSAC over multi-view
 * correspondences. */
void ransacTriangulation(const std::vector<cv::Point2f> &point2d,
                         const std::vector<cv::Mat> &rotation_vec,
                         const std::vector<cv::Mat> &translation_Vec,
                         const cv::Mat &intrinsic,
                         const cv::Mat &disortion_vector, const double thresh,
                         const double p, const int it,
                         cv::Point3f &best_point3d);

/** @brief Solve P3P with RANSAC and optional refinement. Returns inlier mask.
 */
cv::Mat ransacP3P(const std::vector<cv::Point3f> &scene_points,
                  const std::vector<cv::Point2f> &image_points,
                  const cv::Mat &intrinsic, const cv::Mat &distortion_vector,
                  cv::Mat &best_R, cv::Mat &best_T, const double thresh,
                  const int it, const double p = 0.99,
                  const bool refine = true);

/** @brief Transform a set of 3D points by Rodrigues rotation and translation.
 */
std::vector<cv::Point3f> transform3DPts(const std::vector<cv::Point3f> &pts3D,
                                        const cv::Mat &rot,
                                        const cv::Mat &trans);

/** @brief Estimate the rigid transform between two absolute-pose trajectories.
 */
cv::Mat handeyeCalibration(const std::vector<cv::Mat> &pose_abs_1,
                           const std::vector<cv::Mat> &pose_abs_2);

/** @brief Bootstrap hand-eye translation estimation with clustering and
 * resampling. */
cv::Mat
handeyeBootstraptTranslationCalibration(const unsigned int nb_cluster,
                                        const unsigned int nb_it,
                                        const std::vector<cv::Mat> &pose_abs_1,
                                        const std::vector<cv::Mat> &pose_abs_2);

/** @brief Compute median value of a vector of doubles. */
double median(std::vector<double> &v);

/**
 * @brief Solve robust P3P with explicit distortion model selection.
 *
 * distortion_type: 0 for Brown perspective, 1 for fisheye/Kannala style.
 */
cv::Mat ransacP3PDistortion(const std::vector<cv::Point3f> &scene_points,
                            const std::vector<cv::Point2f> &image_points,
                            const cv::Mat &intrinsic,
                            const cv::Mat &distortion_vector, cv::Mat &best_R,
                            cv::Mat &best_T, const float thresh, const int it,
                            const int distortion_type, const double p = 0.99,
                            const bool refine = true);

/** @brief Project 3D points to image coordinates using selected distortion
 * model. */
void projectPointsWithDistortion(const std::vector<cv::Point3f> &object_pts,
                                 const cv::Mat &rot, const cv::Mat &trans,
                                 const cv::Mat &camera_matrix,
                                 const cv::Mat &distortion_vector,
                                 const int distortion_type,
                                 std::vector<cv::Point2f> &repro_pts);

/** @brief Convert a rotation matrix to quaternion [w, x, y, z]. */
cv::Mat convertRotationMatrixToQuaternion(const cv::Mat &R);

/** @brief Convert quaternion [w, x, y, z] to a rotation matrix. */
cv::Mat convertQuaternionToRotationMatrix(const std::array<double, 4> &q);

/** @brief Average rotations from Rodrigues vectors using quaternion mean or
 * median fallback. */
cv::Mat getAverageRotation(std::vector<double> &r1, std::vector<double> &r2,
                           std::vector<double> &r3,
                           const bool use_quaternion_averaging = true);

} // namespace McCalib