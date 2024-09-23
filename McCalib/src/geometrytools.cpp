#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "geometrytools.hpp"
#include "logger.h"

namespace McCalib {

// Tools for rotation and projection matrix
cv::Mat RT2Proj(const cv::Mat &R, const cv::Mat &T) {
  cv::Mat Proj = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
                  0, 0, 0, 1);
  R.copyTo(Proj(cv::Range(0, 3), cv::Range(0, 3)));
  T.copyTo(Proj(cv::Range(0, 3), cv::Range(3, 4)));
  return Proj;
}

cv::Mat RVecT2Proj(const cv::Mat &RVec, const cv::Mat &T) {
  cv::Mat R;
  cv::Rodrigues(RVec, R);
  cv::Mat Proj = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
                  0, 0, 0, 1);
  R.copyTo(Proj(cv::Range(0, 3), cv::Range(0, 3)));
  T.copyTo(Proj(cv::Range(0, 3), cv::Range(3, 4)));
  return Proj;
}

cv::Mat RVecT2ProjInt(const cv::Mat &RVec, const cv::Mat &T, const cv::Mat &K) {
  cv::Mat R;
  cv::Rodrigues(RVec, R);
  cv::Mat Proj = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
                  0, 0, 0, 1);
  cv::Mat KR = K * R;
  cv::Mat KT = K * T;
  KR.copyTo(Proj(cv::Range(0, 3), cv::Range(0, 3)));
  KT.copyTo(Proj(cv::Range(0, 3), cv::Range(3, 4)));
  return Proj;
}

void Proj2RT(const cv::Mat &Proj, cv::Mat &R,
             cv::Mat &T) // Rodrigues and translation vector
{
  cv::Rodrigues(Proj(cv::Range(0, 3), cv::Range(0, 3)), R);
  T = Proj(cv::Range(0, 3), cv::Range(3, 4));
}

cv::Mat vectorProj(const std::vector<float> &ProjV) // R Rodrigues | T vector
{
  cv::Mat RV(1, 3, CV_64F);
  cv::Mat TV(3, 1, CV_64F);
  RV.at<double>(0) = (double)ProjV[0];
  RV.at<double>(1) = (double)ProjV[1];
  RV.at<double>(2) = (double)ProjV[2];
  cv::Mat R;
  cv::Rodrigues(RV, R);
  TV.at<double>(0) = (double)ProjV[3];
  TV.at<double>(1) = (double)ProjV[4];
  TV.at<double>(2) = (double)ProjV[5];
  cv::Mat Proj = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
                  0, 0, 0, 1);
  R.copyTo(Proj(cv::Range(0, 3), cv::Range(0, 3)));
  TV.copyTo(Proj(cv::Range(0, 3), cv::Range(3, 4)));
  return Proj;
}

// Projection matrix to float array
std::array<float, 6> ProjToVec(const cv::Mat &Proj) {
  cv::Mat R(1, 3, CV_64F);
  cv::Mat T(3, 1, CV_64F);
  cv::Rodrigues(Proj(cv::Range(0, 3), cv::Range(0, 3)), R);
  T = Proj(cv::Range(0, 3), cv::Range(3, 4));
  std::array<float, 6> output = {
      (float)R.at<double>(0), (float)R.at<double>(1), (float)R.at<double>(2),
      (float)T.at<double>(0), (float)T.at<double>(1), (float)T.at<double>(2)};
  return output;
}

// Invert vector representation
void invertRvecT(const cv::Mat &Rvec, const cv::Mat &T, cv::Mat &iR,
                 cv::Mat &iT) {
  cv::Mat Proj = RVecT2Proj(Rvec, T);
  Proj = Proj.inv();
  Proj2RT(Proj, iR, iT);
}

void invertRvecT(cv::Mat &Rvec, cv::Mat &T) {
  cv::Mat Proj = RVecT2Proj(Rvec, T);
  Proj = Proj.inv();
  Proj2RT(Proj, Rvec, T);
}

// My SVD triangulation
// Triangulate a 3D point from multiple observations
cv::Point3f
triangulateNViewLinearEigen(const std::vector<cv::Point2f> &Pts2D,
                            const std::vector<cv::Mat> &RotationVec,
                            const std::vector<cv::Mat> &TranslationVec,
                            const cv::Mat &Intrinsic) {
  cv::Mat A; // Projection matrix
  for (std::size_t i = 0; i < RotationVec.size(); i++) {
    cv::Mat cam_temp =
        RVecT2ProjInt(RotationVec[i], TranslationVec[i], Intrinsic);
    float px = Pts2D[i].x;
    float py = Pts2D[i].y;
    cv::Mat M1 = cam_temp.row(0);
    cv::Mat M2 = cam_temp.row(1);
    cv::Mat M3 = cam_temp.row(2);
    A.push_back(px * M3 - M1);
    A.push_back(py * M3 - M2);
  }
  cv::Mat S, U, VT;
  SVDecomp(A, S, U, VT);
  cv::Mat PtsTri = VT.row(3);
  PtsTri.convertTo(PtsTri, CV_32F);
  cv::Point3f PtsTrip;
  PtsTrip.x = PtsTri.at<float>(0) / PtsTri.at<float>(3);
  PtsTrip.y = PtsTri.at<float>(1) / PtsTri.at<float>(3);
  PtsTrip.z = PtsTri.at<float>(2) / PtsTri.at<float>(3);
  return PtsTrip;
}

// Fit the line according to the point set ax + by + c = 0, res is the residual
void calcLinePara(const std::vector<cv::Point2f> &pts, double &a, double &b,
                  double &c, double &res) {
  res = 0;
  cv::Vec4f line;
  std::vector<cv::Point2f> ptsF;
  ptsF.reserve(pts.size());
  for (const auto &pt : pts)
    ptsF.emplace_back(pt);

  cv::fitLine(ptsF, line, cv::DistanceTypes::DIST_L2, 0, 1e-2, 1e-2);
  a = line[1];
  b = -line[0];
  c = line[0] * line[3] - line[1] * line[2];

  for (const auto &pt : pts) {
    double resid_ = fabs(pt.x * a + pt.y * b + c);
    res += resid_;
  }
  res /= pts.size();
}

// RANSAC algorithm
// Return Inliers, p = proba (typical = 0.99), Output : Best Pts3D maximizing
// inliers, Thresh = reprojection tolerance in pixels, it = max iteration
void ransacTriangulation(const std::vector<cv::Point2f> &point2d,
                         const std::vector<cv::Mat> &rotation_vec,
                         const std::vector<cv::Mat> &translation_vec,
                         const cv::Mat &intrinsic,
                         const cv::Mat &distortion_vector, const double thresh,
                         const double p, const int it,
                         cv::Point3f &best_points3d) {
  // Init parameters
  int N = it;
  int trialcount = 0;
  cv::Mat InliersR;
  int countit = 0;
  unsigned int BestInNb = 0;
  double myepsilon = 0.00001; // small value for numerical problem

  // Vector of index to shuffle
  std::srand(unsigned(std::time(0)));
  std::vector<int> myvector(point2d.size());
  std::iota(myvector.begin(), myvector.end(), 0);

  // Ransac iterations
  while (N > trialcount && countit < it) {
    // pick 2 points
    std::random_shuffle(myvector.begin(), myvector.end());
    std::array<int, 4> idx = {myvector[0], myvector[1], myvector[2],
                              myvector[3]};

    // Select the corresponding sample of 2D pts and corresponding rot and trans
    std::vector<cv::Point2f> image2Pts = {point2d[idx[0]], point2d[idx[1]]};
    std::vector<cv::Mat> Rot2Pts = {rotation_vec[idx[0]], rotation_vec[idx[1]]};
    std::vector<cv::Mat> Trans2Pts = {translation_vec[idx[0]],
                                      translation_vec[idx[1]]};

    // Triangulate with these 2 pts
    cv::Point3f PtsTrip =
        triangulateNViewLinearEigen(image2Pts, Rot2Pts, Trans2Pts, intrinsic);

    // compute inliers
    cv::Mat Index;
    unsigned int num_inliers = 0u;
    for (std::size_t k = 0; k < rotation_vec.size(); k++) {
      // Reproject points
      std::vector<cv::Point2f> reprojected_pts;
      std::vector<cv::Point3f> point3d_tmp = {PtsTrip};
      projectPoints(point3d_tmp, rotation_vec[k], translation_vec[k], intrinsic,
                    distortion_vector, reprojected_pts);
      if (std::sqrt(std::pow((point2d[k].x - reprojected_pts[0].x), 2) +
                    std::pow((point2d[k].y - reprojected_pts[0].y), 2)) <
          thresh) {
        Index.push_back(static_cast<int>(k));
        num_inliers++;
      }
    }
    trialcount++;

    // keep the best one
    if (num_inliers > BestInNb) {
      Index.copyTo(InliersR);
      BestInNb = num_inliers;
      best_points3d = PtsTrip;

      // with probability p, a data set with no outliers.
      double totalPts = point2d.size();
      double fracinliers = BestInNb / totalPts;
      double pNoOutliers = 1 - pow(fracinliers, 3);
      if (pNoOutliers == 0)
        pNoOutliers = myepsilon; // Avoid division by Inf
      if (pNoOutliers > (1 - myepsilon))
        pNoOutliers = 1 - myepsilon; // Avoid division by zero
      double tempest = log(1 - p) / log(pNoOutliers);
      N = int(round(tempest));
      trialcount = 0;
    }
    countit++;
  }
}

// RANSAC algorithm
// Return Inliers, p = proba (typical = 0.99), Output : Rot and Trans Mat,
// Thresh = reprojection tolerance in pixels, it = max iteration
cv::Mat ransacP3P(const std::vector<cv::Point3f> &scenePoints,
                  const std::vector<cv::Point2f> &imagePoints,
                  const cv::Mat Intrinsic, const cv::Mat Disto, cv::Mat &BestR,
                  cv::Mat &BestT, const float thresh, const int it,
                  const double p, const bool refine) {

  // Init parameters
  int N = it;
  int trialcount = 0;
  cv::Mat InliersR;
  int countit = 0;
  int BestInNb = 0;
  double myepsilon = 0.00001; // small value for numerical problem
  cv::Mat Rot(1, 3, CV_64F);
  cv::Mat Trans(1, 3, CV_64F);

  // Vector of index to shuffle
  std::srand(unsigned(std::time(0)));
  std::vector<int> myvector(imagePoints.size());
  std::iota(myvector.begin(), myvector.end(), 0);

  // Ransac iterations
  while (N > trialcount && countit < it) {

    // pick 4 points
    std::random_shuffle(myvector.begin(), myvector.end());
    std::array<int, 4> idx = {myvector[0], myvector[1], myvector[2],
                              myvector[3]};

    // Select the corresponding sample
    std::vector<cv::Point3f> scenePoints3Pts = {
        scenePoints[idx[0]], scenePoints[idx[1]], scenePoints[idx[2]],
        scenePoints[idx[3]]};
    std::vector<cv::Point2f> imagePoints3Pts = {
        imagePoints[idx[0]], imagePoints[idx[1]], imagePoints[idx[2]],
        imagePoints[idx[3]]};

    // Apply P3P (fourth point for disambiguation)
    cv::solvePnP(scenePoints3Pts, imagePoints3Pts, Intrinsic, Disto, Rot, Trans,
                 false, 2); // CV_P3P = 2

    // Reproject points
    std::vector<cv::Point2f> reprojected_pts;
    projectPoints(scenePoints, Rot, Trans, Intrinsic, Disto, reprojected_pts);

    // compute inliers
    cv::Mat Index;
    int NbInliers = 0;
    for (int k = 0; k < (int)scenePoints.size(); k++) {
      if (std::sqrt(std::pow(imagePoints[k].x - reprojected_pts[k].x, 2) +
                    std::pow(imagePoints[k].y - reprojected_pts[k].y, 2)) <
          thresh) {
        Index.push_back(k);
        NbInliers++;
      }
    }
    trialcount++;

    // keep the best one
    if (NbInliers > BestInNb) {
      Index.copyTo(InliersR);
      BestInNb = NbInliers;
      Trans.copyTo(BestT);
      Rot.copyTo(BestR);

      // with probability p, a data set with no outliers.
      double totalPts = scenePoints.size();
      double fracinliers = BestInNb / totalPts;
      double pNoOutliers = 1 - pow(fracinliers, 3);
      if (pNoOutliers == 0)
        pNoOutliers = myepsilon; // Avoid division by Inf
      if (pNoOutliers > (1 - myepsilon))
        pNoOutliers = 1 - myepsilon; // Avoid division by zero
      double tempest = log(1 - p) / log(pNoOutliers);
      N = int(round(tempest));
      trialcount = 0;
    }
    countit++;
  }

  if (refine == true && BestInNb >= 4) {
    std::vector<cv::Point3f> scenePointsInliers(BestInNb);
    std::vector<cv::Point2f> imagePointsInliers(BestInNb);

    for (int j = 0; j < BestInNb; j++) {
      imagePointsInliers[j] = imagePoints[InliersR.at<int>(j)];
      scenePointsInliers[j] = scenePoints[InliersR.at<int>(j)];
    }
    cv::solvePnP(scenePointsInliers, imagePointsInliers, Intrinsic, Disto,
                 BestR, BestT, true, 0); // CV_ITERATIVE = 0 non linear
  }
  return InliersR;
}

std::vector<cv::Point3f> transform3DPts(const std::vector<cv::Point3f> &pts3D,
                                        const cv::Mat &Rot,
                                        const cv::Mat &Trans) {
  cv::Mat RotM;
  cv::Rodrigues(Rot, RotM);
  double r11 = RotM.at<double>(0, 0);
  double r12 = RotM.at<double>(0, 1);
  double r13 = RotM.at<double>(0, 2);
  double r21 = RotM.at<double>(1, 0);
  double r22 = RotM.at<double>(1, 1);
  double r23 = RotM.at<double>(1, 2);
  double r31 = RotM.at<double>(2, 0);
  double r32 = RotM.at<double>(2, 1);
  double r33 = RotM.at<double>(2, 2);
  double tx = Trans.at<double>(0);
  double ty = Trans.at<double>(1);
  double tz = Trans.at<double>(2);

  std::vector<cv::Point3f> pts3D_transformed;
  for (const cv::Point3f &pt3D : pts3D) {
    float x = tx + r11 * pt3D.x + r12 * pt3D.y + r13 * pt3D.z;
    float y = ty + r21 * pt3D.x + r22 * pt3D.y + r23 * pt3D.z;
    float z = tz + r31 * pt3D.x + r32 * pt3D.y + r33 * pt3D.z;
    pts3D_transformed.emplace_back(x, y, z);
  }

  return pts3D_transformed;
}

/**
 * @brief Calibrate 2 cameras with handeye calibration
 *

 */
cv::Mat handeyeCalibration(const std::vector<cv::Mat> &pose_abs_1,
                           const std::vector<cv::Mat> &pose_abs_2) {

  // Prepare the poses for handeye calibration
  const size_t num_poses = pose_abs_1.size();
  std::vector<cv::Mat> r_cam_group_1(num_poses), t_cam_group_1(num_poses),
      r_cam_group_2(num_poses), t_cam_group_2(num_poses);
  for (size_t i = 0; i < num_poses; i++) {
    // get the poses
    cv::Mat pose_cam_group_1 = pose_abs_1[i].inv();
    cv::Mat pose_cam_group_2 = pose_abs_2[i];

    // save in datastruct
    cv::Mat r_1, r_2, t_1, t_2;
    Proj2RT(pose_cam_group_1, r_1, t_1);
    Proj2RT(pose_cam_group_2, r_2, t_2);
    cv::Mat r_1_mat, r_2_mat;
    Rodrigues(r_1, r_1_mat);
    Rodrigues(r_2, r_2_mat);
    r_cam_group_1[i] = r_1_mat;
    t_cam_group_1[i] = t_1;
    r_cam_group_2[i] = r_2_mat;
    t_cam_group_2[i] = t_2;
  }

  // Hand-eye calibration
  cv::Mat r_g1_g2, t_g1_g2;
  cv::calibrateHandEye(r_cam_group_1, t_cam_group_1, r_cam_group_2,
                       t_cam_group_2, r_g1_g2, t_g1_g2,
                       cv::CALIB_HAND_EYE_HORAUD);
  cv::Mat pose_g1_g2 = RT2Proj(r_g1_g2, t_g1_g2);

  return pose_g1_g2;
}

/**
 * @brief Calibrate 2 cameras with handeye calibration
 *
 * In this function only N pairs of images are used
 * These pair of images are selected with a clustering technique
 * The clustering is achieved via the translation of cameras
 * The process is repeated multiple time on subset of the poses
 * A test of consistency is performed, all potentially valid poses are saved
 * The mean value of valid poses is returned
 */
cv::Mat handeyeBootstratpTranslationCalibration(
    const unsigned int nb_clusters, const unsigned int nb_it,
    const std::vector<cv::Mat> &pose_abs_1,
    const std::vector<cv::Mat> &pose_abs_2) {
  // N clusters but less if less images available
  const unsigned int nb_cluster =
      (pose_abs_1.size() < nb_clusters) ? pose_abs_1.size() : nb_clusters;

  // Prepare the translational component of the cameras to be clustered
  cv::Mat position_1_2; // concatenation of the translation of pose 1 and 2 for
                        // clustering
  for (unsigned int i = 0; i < pose_abs_1.size(); i++) {
    cv::Mat trans_1, trans_2, rot_1, rot_2;
    Proj2RT(pose_abs_1[i], rot_1, trans_1);
    Proj2RT(pose_abs_2[i], rot_2, trans_2);
    cv::Mat concat_trans_1_2;
    cv::hconcat(trans_1.t(), trans_2.t(), concat_trans_1_2);
    position_1_2.push_back(concat_trans_1_2);
  }
  position_1_2.convertTo(position_1_2, CV_32F);

  // Cluster the observation to select the most diverse poses
  cv::Mat labels;
  cv::Mat centers;
  int nb_kmean_iterations = 5;
  std::ignore =
      cv::kmeans(position_1_2, nb_cluster, labels,
                 cv::TermCriteria(
                     cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 0.01),
                 nb_kmean_iterations, cv::KMEANS_PP_CENTERS, centers);
  labels.convertTo(labels, CV_32S);

  // Iterate n times the
  std::vector<double> r1_he, r2_he, r3_he; // structure to save valid rot
  std::vector<double> t1_he, t2_he, t3_he; // structure to save valid trans
  unsigned int nb_clust_pick = 6;
  unsigned int nb_success = 0;
  for (unsigned int iter = 0; iter < nb_it; iter++) {

    // pick from n of these clusters randomly
    std::vector<unsigned int> shuffled_ind(nb_cluster);
    std::iota(shuffled_ind.begin(), shuffled_ind.end(), 0);
    std::random_device rd; // initialize random number generator
    std::mt19937 g(rd());
    std::shuffle(shuffled_ind.begin(), shuffled_ind.end(), g);
    std::vector<unsigned int> cluster_select;
    cluster_select.reserve(nb_clust_pick);
    for (unsigned int k = 0; k < nb_clust_pick; ++k) {
      cluster_select.push_back(shuffled_ind[k]);
    }

    // Select one pair of pose for each cluster
    std::vector<unsigned int> pose_ind;
    pose_ind.reserve(cluster_select.size());
    for (const unsigned int &clust_ind : cluster_select) {
      std::vector<unsigned int> idx;
      for (unsigned int j = 0; j < pose_abs_2.size(); j++) {
        if (labels.at<unsigned int>(j) == clust_ind) {
          idx.push_back(j);
        }
      }
      // randomly select an index in the occurrences of the cluster
      srand(time(NULL));
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> dis(0, idx.size() - 1);
      unsigned int cluster_idx = dis(gen);
      pose_ind.push_back(idx[cluster_idx]);
    }

    // Prepare the poses for handeye calibration
    std::vector<cv::Mat> r_cam_group_1, t_cam_group_1, r_cam_group_2,
        t_cam_group_2;
    r_cam_group_1.reserve(pose_ind.size());
    t_cam_group_1.reserve(pose_ind.size());
    r_cam_group_2.reserve(pose_ind.size());
    t_cam_group_2.reserve(pose_ind.size());
    for (const auto &pose_ind_i : pose_ind) {
      // get the poses
      cv::Mat pose_cam_group_1 = pose_abs_1[pose_ind_i].inv();
      cv::Mat pose_cam_group_2 = pose_abs_2[pose_ind_i];

      // save in datastruct
      cv::Mat r_1, r_2, t_1, t_2;
      Proj2RT(pose_cam_group_1, r_1, t_1);
      Proj2RT(pose_cam_group_2, r_2, t_2);
      cv::Mat r_1_mat, r_2_mat;
      cv::Rodrigues(r_1, r_1_mat);
      cv::Rodrigues(r_2, r_2_mat);
      r_cam_group_1.push_back(r_1_mat);
      t_cam_group_1.push_back(t_1);
      r_cam_group_2.push_back(r_2_mat);
      t_cam_group_2.push_back(t_2);
    }

    // Hand-eye calibration
    cv::Mat r_g1_g2, t_g1_g2;
    cv::calibrateHandEye(r_cam_group_1, t_cam_group_1, r_cam_group_2,
                         t_cam_group_2, r_g1_g2, t_g1_g2,
                         cv::CALIB_HAND_EYE_TSAI);
    // cv::CALIB_HAN
    cv::Mat pose_g1_g2 = RT2Proj(r_g1_g2, t_g1_g2);

    // Check the consistency of the set
    double max_error = 0;
    for (unsigned int i = 0; i < pose_ind.size(); i++) {
      cv::Mat pose_cam_group_1_1 = pose_abs_1[pose_ind[i]];
      cv::Mat pose_cam_group_2_1 = pose_abs_2[pose_ind[i]];
      for (unsigned int j = 0; j < pose_ind.size(); j++) {
        if (i != j) {
          cv::Mat pose_cam_group_1_2 = pose_abs_1[pose_ind[i]];
          cv::Mat pose_cam_group_2_2 = pose_abs_2[pose_ind[i]];
          cv::Mat PP1 = pose_cam_group_1_2.inv() * pose_cam_group_1_1;
          cv::Mat PP2 = pose_cam_group_2_1.inv() * pose_cam_group_2_2;
          cv::Mat ErrMat = PP2.inv() * pose_g1_g2 * PP1 * pose_g1_g2;
          cv::Mat ErrRot, ErrTrans;
          Proj2RT(ErrMat, ErrRot, ErrTrans);
          cv::Mat ErrRotMat;
          cv::Rodrigues(ErrRot, ErrRotMat);
          double traceRot =
              cv::trace(ErrRotMat)[0] - std::numeric_limits<double>::epsilon();
          double err_degree = std::acos(0.5 * (traceRot - 1.0)) * 180.0 / M_PI;
          if (err_degree > max_error)
            max_error = err_degree;
        }
      }
    }
    if (max_error < 15) {
      nb_success++;
      // if it is a sucess then add to our valid pose evector
      cv::Mat rot_temp, trans_temp;
      Proj2RT(pose_g1_g2, rot_temp, trans_temp);
      r1_he.push_back(rot_temp.at<double>(0));
      r2_he.push_back(rot_temp.at<double>(1));
      r3_he.push_back(rot_temp.at<double>(2));
      t1_he.push_back(trans_temp.at<double>(0));
      t2_he.push_back(trans_temp.at<double>(1));
      t3_he.push_back(trans_temp.at<double>(2));
    }
  }

  // if enough sucess (at least 3) then compute median value
  if (nb_success > 3) {
    cv::Mat r_he = getAverageRotation(r1_he, r2_he, r3_he);
    cv::Mat t_he = cv::Mat::zeros(3, 1, CV_64F);
    t_he.at<double>(0) = median(t1_he);
    t_he.at<double>(1) = median(t2_he);
    t_he.at<double>(2) = median(t3_he);
    cv::Mat pose_g1_g2 = RVecT2Proj(r_he, t_he);
    return pose_g1_g2;
  } else // else run the normal handeye calibration on all the samples
  {
    cv::Mat pose_g1_g2 = handeyeCalibration(pose_abs_1, pose_abs_1);
    return pose_g1_g2;
  }
}

// median of the vector but modifies original vector
double median(std::vector<double> &v) {
  size_t n = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + n, v.end());
  return v[n];
}

// RANSAC algorithm
// Return Inliers, p = proba (typical = 0.99), Output : Rot and Trans Mat,
// Thresh = reprojection tolerance in pixels, it = max iteration
// distortion_type: 0 (perspective), 1 (fisheye)
cv::Mat ransacP3PDistortion(const std::vector<cv::Point3f> &scene_points,
                            const std::vector<cv::Point2f> &image_points,
                            const cv::Mat &intrinsic,
                            const cv::Mat &distortion_vector, cv::Mat &best_R,
                            cv::Mat &best_T, const float thresh, const int it,
                            const int distortion_type, const double p,
                            const bool refine) {
  cv::Mat Inliers;
  // P3P for perspective (Brown model)
  if (distortion_type == 0) {
    Inliers =
        ransacP3P(scene_points, image_points, intrinsic, distortion_vector,
                  best_R, best_T, thresh, it, p, refine);
  }

  // P3P for fisheye
  if (distortion_type == 1) {
    // undistord the point
    std::vector<cv::Point2f> imagePointsUndis;
    cv::Mat New_Intrinsic = intrinsic.clone();
    cv::fisheye::undistortPoints(image_points, imagePointsUndis, intrinsic,
                                 distortion_vector);

    // multiply by K to go into cam ref (because the OpenCV function is so
    // broken ...)
    for (auto &imagePointUndis : imagePointsUndis) {
      imagePointUndis.x =
          imagePointUndis.x * float(intrinsic.at<double>(0, 0)) +
          float(intrinsic.at<double>(0, 2));
      imagePointUndis.y =
          imagePointUndis.y * float(intrinsic.at<double>(1, 1)) +
          float(intrinsic.at<double>(1, 2));
    }
    // Run p3p
    const cv::Mat zero_distortion_vector =
        (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
    Inliers = ransacP3P(scene_points, imagePointsUndis, New_Intrinsic,
                        zero_distortion_vector, best_R, best_T, thresh, it, p,
                        refine);
  }

  return Inliers;
}

// Project point for fisheye and perspective ()
// distortion_type: 0 (perspective), 1 (fisheye)
void projectPointsWithDistortion(const std::vector<cv::Point3f> &object_pts,
                                 const cv::Mat &rot, const cv::Mat &trans,
                                 const cv::Mat &camera_matrix,
                                 const cv::Mat &distortion_vector,
                                 const int distortion_type,
                                 std::vector<cv::Point2f> &repro_pts) {
  if (distortion_type == 0) // perspective (Brown)
  {
    cv::projectPoints(object_pts, rot, trans, camera_matrix, distortion_vector,
                      repro_pts);
  }
  if (distortion_type == 1) // fisheye (Kannala)
  {
    cv::fisheye::projectPoints(object_pts, repro_pts, rot, trans, camera_matrix,
                               distortion_vector, 0.0);
  }
}

cv::Mat convertRotationMatrixToQuaternion(const cv::Mat &R) {
  // code is adapted from
  // https://gist.github.com/shubh-agrawal/76754b9bfb0f4143819dbd146d15d4c8

  cv::Mat Q(1, 4, CV_64F); // x y z w

  double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);

  if (trace > 0.0) {
    double s = std::sqrt(trace + 1.0);
    Q.at<double>(0, 3) = (s * 0.5);
    s = 0.5 / s;
    Q.at<double>(0, 0) = ((R.at<double>(2, 1) - R.at<double>(1, 2)) * s);
    Q.at<double>(0, 1) = ((R.at<double>(0, 2) - R.at<double>(2, 0)) * s);
    Q.at<double>(0, 2) = ((R.at<double>(1, 0) - R.at<double>(0, 1)) * s);
  }

  else {
    int i = R.at<double>(0, 0) < R.at<double>(1, 1)
                ? (R.at<double>(1, 1) < R.at<double>(2, 2) ? 2 : 1)
                : (R.at<double>(0, 0) < R.at<double>(2, 2) ? 2 : 0);
    int j = (i + 1) % 3;
    int k = (i + 2) % 3;

    double s = std::sqrt(R.at<double>(i, i) - R.at<double>(j, j) -
                         R.at<double>(k, k) + 1.0);
    Q.at<double>(0, i) = s * 0.5;
    s = 0.5 / s;

    Q.at<double>(0, 3) = (R.at<double>(k, j) - R.at<double>(j, k)) * s;
    Q.at<double>(0, j) = (R.at<double>(j, i) + R.at<double>(i, j)) * s;
    Q.at<double>(0, k) = (R.at<double>(k, i) + R.at<double>(i, k)) * s;
  }

  return Q;
}

cv::Mat convertQuaternionToRotationMatrix(const std::array<double, 4> &q) {
  // code adapted from
  // https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/

  const double q0 = q[3];
  const double q1 = q[0];
  const double q2 = q[1];
  const double q3 = q[2];

  cv::Mat rot_matrix(3, 3, CV_64F);
  rot_matrix.at<double>(0, 0) = 2 * (q0 * q0 + q1 * q1) - 1;
  rot_matrix.at<double>(0, 1) = 2 * (q1 * q2 - q0 * q3);
  rot_matrix.at<double>(0, 2) = 2 * (q1 * q3 + q0 * q2);

  rot_matrix.at<double>(1, 0) = 2 * (q1 * q2 + q0 * q3);
  rot_matrix.at<double>(1, 1) = 2 * (q0 * q0 + q2 * q2) - 1;
  rot_matrix.at<double>(1, 2) = 2 * (q2 * q3 - q0 * q1);

  rot_matrix.at<double>(2, 0) = 2 * (q1 * q3 - q0 * q2);
  rot_matrix.at<double>(2, 1) = 2 * (q2 * q3 + q0 * q1);
  rot_matrix.at<double>(2, 2) = 2 * (q0 * q0 + q3 * q3) - 1;

  return rot_matrix;
}

cv::Mat getAverageRotation(std::vector<double> &r1, std::vector<double> &r2,
                           std::vector<double> &r3,
                           const bool use_quaternion_averaging) {
  cv::Mat average_rotation = cv::Mat::zeros(3, 1, CV_64F);
  if (use_quaternion_averaging) {
    // The Quaternion Averaging algorithm is described in
    // https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
    // implementaion references:
    //  -
    //  https://gist.github.com/PeteBlackerThe3rd/f73e9d569e29f23e8bd828d7886636a0
    //  -
    //  https://github.com/tolgabirdal/averaging_quaternions/blob/master/avg_quaternion_markley.m

    assert(r1.size() == r2.size() && r2.size() == r3.size());

    std::vector<cv::Mat> quaternions;
    // convert rotation vector to quaternion through rotation matrix
    for (unsigned int angle_idx = 0u; angle_idx < r1.size(); ++angle_idx) {
      std::array<double, 3> angles = {r1[angle_idx], r2[angle_idx],
                                      r3[angle_idx]};
      const cv::Mat rot_vec = cv::Mat(1, 3, CV_64F, angles.data());
      cv::Mat rot_matrix;
      cv::Rodrigues(rot_vec, rot_matrix);
      const cv::Mat quaternion = convertRotationMatrixToQuaternion(rot_matrix);
      quaternions.push_back(quaternion);
    }

    cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
    for (cv::Mat &q : quaternions) {
      if (q.at<double>(0, 3) < 0) {
        // handle the antipodal configurations
        q = -q;
      }
      A += q.t() * q;
    }
    A /= quaternions.size();

    cv::SVD svd(A, cv::SVD::FULL_UV);
    cv::Mat U = svd.u;
    cv::Mat singularValues = svd.w;

    const unsigned int largestEigenValueIndex = 0u;
    std::array<double, 4> average_quaternion = {
        svd.u.at<double>(0, largestEigenValueIndex),
        svd.u.at<double>(1, largestEigenValueIndex),
        svd.u.at<double>(2, largestEigenValueIndex),
        svd.u.at<double>(3, largestEigenValueIndex)};

    cv::Mat rot_matrix = convertQuaternionToRotationMatrix(average_quaternion);
    cv::Rodrigues(rot_matrix, average_rotation);
  } else {
    average_rotation.at<double>(0) = median(r1);
    average_rotation.at<double>(1) = median(r2);
    average_rotation.at<double>(2) = median(r3);
  }

  return average_rotation;
}

} // namespace McCalib